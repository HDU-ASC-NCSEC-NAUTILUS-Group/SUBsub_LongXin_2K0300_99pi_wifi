#include "zf_common_headfile.h"
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>
#include <string>
#include <vector>

#define BEEP    "/dev/zf_driver_gpio_beep"

#define SCREEN_WIDTH  240
#define SCREEN_HEIGHT 320

//ips图像显示外部变量引用
extern uint16 ips200_pencolor;
extern cv::Mat frame_rgay;

//二维码相关数据初始化
cv::QRCodeDetector qrDecoder;

// 文字叠加区域（图像底部）
const int text_x = 0;
const int text_y = SCREEN_HEIGHT - 16;

float real_x, real_y;

// 二维码解码处理
void QR_process(void)
{

    if(wait_image_refresh() < 0)
    {
        return;
    }

    if (frame_rgay.empty() || rgay_image == nullptr) {
        return;
    }

    cv::Mat frame_rotated;
    cv::rotate(frame_rgay, frame_rotated, cv::ROTATE_90_CLOCKWISE);
    cv::Mat frame_gray_display;
    cv::resize(frame_rotated, frame_gray_display, cv::Size(SCREEN_WIDTH, SCREEN_HEIGHT), 0, 0, cv::INTER_NEAREST);
    ips200_show_gray_image(0, 0, frame_gray_display.ptr(0), SCREEN_WIDTH, SCREEN_HEIGHT);

    std::string qr_data = qrDecoder.detectAndDecode(frame_rgay);

    if (!qr_data.empty()) {
        char buf[64];
        snprintf(buf, sizeof(buf), "QR: %.40s", qr_data.c_str());
        ips200_show_string(text_x, text_y, buf);
        gpio_set_level(BEEP, 0x1);
    } else {
        ips200_show_string(text_x, text_y, "No QR code");
        gpio_set_level(BEEP, 0x0);
    }
}

// 红色物块检测函数：输入 BGR 图像，返回质心坐标（若未检测到则返回 (-1,-1)）
static cv::Point2i detect_red_object(const cv::Mat &frame_bgr)
{
    // 转换为 HSV 颜色空间
    cv::Mat hsv;
    cv::cvtColor(frame_bgr, hsv, cv::COLOR_BGR2HSV);

    // 红色在 HSV 中有两个区间：低区间 (0~10) 和高区间 (160~180)
    cv::Mat mask1, mask2, mask;
    cv::inRange(hsv, cv::Scalar(0, 50, 50), cv::Scalar(10, 255, 255), mask1);
    cv::inRange(hsv, cv::Scalar(160, 50, 50), cv::Scalar(180, 255, 255), mask2);
    mask = mask1 | mask2;

    // 可选：形态学操作，去除噪声
    cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 1);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

    // 寻找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        return cv::Point2i(-1, -1);  // 未检测到红色物体
    }

    // 选择面积最大的轮廓
    auto largest = std::max_element(contours.begin(), contours.end(),
                                    [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b) {
                                        return cv::contourArea(a) < cv::contourArea(b);
                                    });

    // 计算质心（一阶矩）
    cv::Moments m = cv::moments(*largest);
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
void object_tracking(void)
{
    if (wait_image_refresh_rgb() < 0) {
        printf("摄像头采集失败，退出\n");
        exit(0);
    }

    cv::Point2i red_center = detect_red_object(frame_rgb);

    cv::Mat frame_rotated;
    cv::rotate(frame_rgb, frame_rotated, cv::ROTATE_90_CLOCKWISE);
    cv::Mat frame_display;
    cv::resize(frame_rotated, frame_display, cv::Size(SCREEN_WIDTH, SCREEN_HEIGHT), 0, 0, cv::INTER_NEAREST);
    ips200_show_rgb_image(0, 0, frame_display.ptr(0), SCREEN_WIDTH, SCREEN_HEIGHT);

    char display_buf[64];
    if (red_center.x != -1 && red_center.y != -1) {
        snprintf(display_buf, sizeof(display_buf), "Red: (%3d,%3d)", red_center.x, red_center.y);
    } else {
        snprintf(display_buf, sizeof(display_buf), "No red object");
    }
    ips200_show_string(text_x, text_y, display_buf);

    coordinate_x = red_center.x;
    coordinate_y = red_center.y;
}

void coordinate_transformation(void)
{
    // ---------- 1. 定义标定点（像素坐标 -> 物理坐标）----------
    static const cv::Point2f src_pts[3] = {
        cv::Point2f(98, 306),   // 对应物理 (0, 28.5)
        cv::Point2f(143, 156),   // 对应物理 (3.5, 27)
        cv::Point2f(131, 393)    // 对应物理 (-2.5, 27.6)
    };
    static const cv::Point2f dst_pts[3] = {
        cv::Point2f(0, 28.5),
        cv::Point2f(3.5f, 27.0f),
        cv::Point2f(-2.5f, 27.6f)
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
    // 注意：coordinate_x, coordinate_y 是在 object_tracking() 中赋值的全局变量
    extern int16_t coordinate_x, coordinate_y;
    
    // 若未检测到红色物体（坐标无效），则物理坐标也设为无效值（例如 -1000）
    if (coordinate_x == -1 || coordinate_y == -1) {
        extern float real_x, real_y;
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
    extern float real_x, real_y;
    real_x = physical_point.x;
    real_y = physical_point.y;

    // ---------- 5. （可选）在屏幕文本区显示物理坐标 ----------
    char buf[48];
    snprintf(buf, sizeof(buf), "Real: (%.2f, %.2f) cm", real_x, real_y);
    ips200_show_string(0, SCREEN_HEIGHT - 32, buf);

    // 可选：控制台输出（便于调试）
    // printf("Pixel(%d,%d) -> Real(%.2f,%.2f) cm\n", coordinate_x, coordinate_y, real_x, real_y);
}