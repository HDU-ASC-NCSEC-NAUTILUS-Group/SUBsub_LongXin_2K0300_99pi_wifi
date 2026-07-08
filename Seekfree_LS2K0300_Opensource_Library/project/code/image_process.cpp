/*******************************************************************************
* UVC摄像头设备进程文件
*******************************************************************************/
#include "zf_common_headfile.h"
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>
#include <string>
#include <vector>
#include <time.h>       // clock_gettime

#define BEEP    "/dev/zf_driver_gpio_beep"

/**********************************************************/
/*[S] 图像本体定义 [S]--------------------------------------*/
/**********************************************************/

#define SCREEN_WIDTH  240
#define SCREEN_HEIGHT 320

// 启用后图像处理使用 1/2 分辨率 (320x240)，减少约60-70% OpenCV 运算开销
// 注释掉即恢复全分辨率处理 (640x480)
#define UVC_HALF_RESOLUTION

#ifdef UVC_HALF_RESOLUTION
    #define PROC_WIDTH   (UVC_WIDTH / 2)
    #define PROC_HEIGHT  (UVC_HEIGHT / 2)
#else
    #define PROC_WIDTH   UVC_WIDTH
    #define PROC_HEIGHT  UVC_HEIGHT
#endif

//ips图像显示外部变量引用
extern cv::Mat frame_rgay;

/**********************************************************/
/*--------------------------------------[E] 图像本体定义 [E]*/
/**********************************************************/


/**********************************************************/
/*[S] 二维码处理 [S]----------------------------------------*/
/**********************************************************/

//二维码相关数据初始化
cv::QRCodeDetector qrDecoder;

/*
 * 二维码解码处理
 *
 * 调用示例：
 *   const char* qr = QR_process();
 *   if (qr != nullptr) {
 *       // 识别成功，qr 为解码字符串，例如 "01"
 *       printf("QR: %s\n", qr);
 *   } else {
 *       // 暂无识别结果（无帧、跳帧或未检测到二维码）
 *   }
 *
 * 返回值：
 *   const char*  — 识别成功时返回解码字符串（静态缓冲区，下次调用会覆盖）
 *   nullptr      — 无帧、跳帧或未检测到二维码
 */
const char* QR_process(void)
{
    int result = wait_image_refresh();       // 1=有帧, 0=暂无, <0=错误
    if (result <= 0) {
        return nullptr;
    }

    if (frame_rgay.empty() || rgay_image == nullptr) {
        return nullptr;
    }

    // 跳帧：每隔 ~500ms 处理一次二维码，节省算力
    static int64_t last_qr_ms = 0;
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    int64_t now_ms = (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
    if (now_ms - last_qr_ms < 500) {
        return nullptr;
    }
    last_qr_ms = now_ms;

    cv::Mat frame_gray_proc;
#ifdef UVC_HALF_RESOLUTION
    cv::resize(frame_rgay, frame_gray_proc, cv::Size(PROC_WIDTH, PROC_HEIGHT), 0, 0, cv::INTER_NEAREST);
#else
    frame_gray_proc = frame_rgay;
#endif

    cv::Mat frame_rotated;
    cv::rotate(frame_gray_proc, frame_rotated, cv::ROTATE_90_CLOCKWISE);

#ifdef UVC_HALF_RESOLUTION
    ips200_show_gray_image(0, 0, frame_rotated.ptr(0), SCREEN_WIDTH, SCREEN_HEIGHT);
#else
    cv::Mat frame_gray_display;
    cv::resize(frame_rotated, frame_gray_display, cv::Size(SCREEN_WIDTH, SCREEN_HEIGHT), 0, 0, cv::INTER_NEAREST);
    ips200_show_gray_image(0, 0, frame_gray_display.ptr(0), SCREEN_WIDTH, SCREEN_HEIGHT);
#endif

    std::string qr_data = qrDecoder.detectAndDecode(frame_gray_proc);

    static char qr_result[128];

    if (!qr_data.empty()) {
        char buf[64];
        snprintf(buf, sizeof(buf), "QR: %.40s", qr_data.c_str());
        ips200_show_string(0, SCREEN_HEIGHT - 16, buf);
        //gpio_set_level(BEEP, 0x1);

        snprintf(qr_result, sizeof(qr_result), "%s", qr_data.c_str());
        return qr_result;
    } else {
        ips200_show_string(0, SCREEN_HEIGHT - 16, "No QR code");
        gpio_set_level(BEEP, 0x0);
        return nullptr;
    }
}
/**********************************************************/
/*----------------------------------------[E] 二维码处理 [E]*/
/**********************************************************/


/**********************************************************/
/*[S] 物块跟踪 [S]-----------------------------------------*/
/**********************************************************/

// 红色物块检测函数：输入 BGR 图像，返回质心坐标（若未检测到则返回 (-1,-1)）
static cv::Point2i detect_red_object(const cv::Mat &frame_bgr)
{
    // 预分配：Mat 内存只分配一次，后续调用复用
    static cv::Mat hsv, mask1, mask2, mask;
    static const cv::Scalar kRedLow1(0,   50, 50);
    static const cv::Scalar kRedHigh1(10,  255, 255);
    static const cv::Scalar kRedLow2(160,  50, 50);
    static const cv::Scalar kRedHigh2(180, 255, 255);
    static const cv::Mat    kKernel3x3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

    // 转换为 HSV 颜色空间
    cv::cvtColor(frame_bgr, hsv, cv::COLOR_BGR2HSV);

    // 红色在 HSV 中有两个区间：低区间 (0~10) 和高区间 (160~180)
    cv::inRange(hsv, kRedLow1,  kRedHigh1, mask1);
    cv::inRange(hsv, kRedLow2,  kRedHigh2, mask2);
    mask = mask1 | mask2;

    // 形态学操作，去除噪声
    cv::erode(mask, mask, kKernel3x3);
    cv::dilate(mask, mask, kKernel3x3);

    // 寻找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        return cv::Point2i(-1, -1);  // 未检测到红色物体
    }

    // 单次遍历找最大轮廓：每个轮廓只算一次面积
    float best_area = -1.0f;
    size_t best_idx = 0;
    for (size_t i = 0; i < contours.size(); i++) {
        float a = (float)cv::contourArea(contours[i]);
        if (a > best_area) {
            best_area = a;
            best_idx  = i;
        }
    }

    // 计算质心（一阶矩）
    cv::Moments m = cv::moments(contours[best_idx]);
    if (m.m00 == 0) {
        return cv::Point2i(-1, -1);
    }
    int cx = static_cast<int>(m.m10 / m.m00);
    int cy = static_cast<int>(m.m01 / m.m00);

    return cv::Point2i(cx, cy);
}

// 2. 检测红色物体（使用彩色图 frame_rgb）
//cv::Point2i red_center = detect_red_object(frame_rgb);

int16_t coordinate_x = 0;
int16_t coordinate_y = 0;

//红色物块跟踪函数
int object_tracking(void)
{
    int result = wait_image_refresh_rgb();   // 1=有帧, 0=暂无, <0=错误
    if (result <= 0) {
        return 0;
    }

    // 跳帧：每隔 ~200ms 处理一帧，其余帧丢弃（减轻 LS2K0300 处理压力）
    static int64_t last_process_ms = 0;
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    int64_t now_ms = (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
    if (now_ms - last_process_ms < 200) {
        return 0;
    }
    last_process_ms = now_ms;

    cv::Mat frame_proc;
#ifdef UVC_HALF_RESOLUTION
    cv::resize(frame_rgb, frame_proc, cv::Size(PROC_WIDTH, PROC_HEIGHT), 0, 0, cv::INTER_NEAREST);
#else
    frame_proc = frame_rgb;
#endif

    cv::Point2i red_center = detect_red_object(frame_proc);

#ifdef UVC_HALF_RESOLUTION
    if (red_center.x != -1 && red_center.y != -1) {
        red_center.x *= 2;
        red_center.y *= 2;
    }
#endif

    cv::Mat frame_rotated;
    cv::rotate(frame_proc, frame_rotated, cv::ROTATE_90_CLOCKWISE);

#ifdef UVC_HALF_RESOLUTION
    ips200_show_rgb_image(0, 0, frame_rotated.ptr(0), SCREEN_WIDTH, SCREEN_HEIGHT);
#else
    cv::Mat frame_display;
    cv::resize(frame_rotated, frame_display, cv::Size(SCREEN_WIDTH, SCREEN_HEIGHT), 0, 0, cv::INTER_NEAREST);
    ips200_show_rgb_image(0, 0, frame_display.ptr(0), SCREEN_WIDTH, SCREEN_HEIGHT);
#endif

    char display_buf[64];
    if (red_center.x != -1 && red_center.y != -1) {
        snprintf(display_buf, sizeof(display_buf), "Red:(%3d,%3d)", red_center.x, red_center.y);
    } else {
        snprintf(display_buf, sizeof(display_buf), "No red object");
    }
    ips200_show_string(0, SCREEN_HEIGHT - 16, display_buf);

    coordinate_x = red_center.x;
    coordinate_y = red_center.y;

    // -1: 已处理但未检测到红色物体, 1: 成功追踪, 0: 跳帧/无帧
    if (red_center.x == -1 || red_center.y == -1) {
        return -1;
    }
    return 1;
}
/**********************************************************/
/*-----------------------------------------[E] 物块跟踪 [E]*/
/**********************************************************/


/**********************************************************/
/*[S] 坐标 [S]---------------------------------------------*/
/**********************************************************/

float real_x, real_y;

void coordinate_transformation(void)
{
    // ---------- 1. 定义标定点（像素坐标 -> 物理坐标）----------
    static const cv::Point2f src_pts[3] = {
        cv::Point2f(458, 272),   // 对应物理 (x1,y1)
        cv::Point2f(364, 170),   // 对应物理 (x2,y2)
        cv::Point2f(280, 374)    // 对应物理 (x3,y3)
    };
    static const cv::Point2f dst_pts[3] = {
        cv::Point2f(0, 13),
        cv::Point2f(4.5f, 16.8f),
        cv::Point2f(-3.6f, 19.6f)
    };

    // 计算仿射变换矩阵（2x3），只计算一次并缓存
    static cv::Mat affine_matrix;
    static bool matrix_initialized = false;
    if (!matrix_initialized) {
        affine_matrix = cv::getAffineTransform(src_pts, dst_pts);
        matrix_initialized = true;
        // 可选：打印矩阵以便调试
        // std::cout << "Affine matrix:\n" << affine_matrix << std::endl;
    }

    // ---------- 2. 获取当前红色物块的像素坐标 ----------
    // coordinate_x, coordinate_y 在文件顶部定义，object_tracking() 中赋值
    
    if (coordinate_x == -1 || coordinate_y == -1) {
        real_x = -1000.0f;
        real_y = -1000.0f;
        return;
    }

    // ---------- 3. 应用仿射变换，计算物理坐标 ----------
    cv::Point2f pixel_point(coordinate_x, coordinate_y);
    cv::Point2f physical_point;
    physical_point.x = affine_matrix.at<double>(0,0) * pixel_point.x +
                       affine_matrix.at<double>(0,1) * pixel_point.y +
                       affine_matrix.at<double>(0,2);
    physical_point.y = affine_matrix.at<double>(1,0) * pixel_point.x +
                       affine_matrix.at<double>(1,1) * pixel_point.y +
                       affine_matrix.at<double>(1,2);

    // ---------- 4. 存储物理坐标到全局变量 ----------
    real_x = physical_point.x;
    real_y = physical_point.y;

    // ---------- 5. （可选）在屏幕文本区显示物理坐标 ----------
    char buf[48];
    snprintf(buf, sizeof(buf), "Real:(%.2f,%.2f)cm", real_x, real_y);
    ips200_show_string(0, SCREEN_HEIGHT - 32, buf);

    // 可选：控制台输出（便于调试）
    // printf("Pixel(%d,%d) -> Real(%.2f,%.2f) cm\n", coordinate_x, coordinate_y, real_x, real_y);
}
/**********************************************************/
/*---------------------------------------------[E] 坐标 [E]*/
/**********************************************************/