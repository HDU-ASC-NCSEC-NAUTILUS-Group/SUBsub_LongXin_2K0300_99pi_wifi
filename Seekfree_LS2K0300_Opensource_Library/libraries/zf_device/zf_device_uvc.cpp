#include "zf_device_uvc.h"


#include <opencv2/imgproc/imgproc.hpp>  // for cv::cvtColor
#include <opencv2/highgui/highgui.hpp> // for cv::VideoCapture
#include <opencv2/opencv.hpp>

#include <iostream> // for std::cerr
#include <fstream>  // for std::ofstream
#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>

#include <fcntl.h>       // open
#include <sys/select.h>  // select
#include <unistd.h>      // close

using namespace cv;

cv::Mat frame_rgb;      // 构建opencv对象 彩色
cv::Mat frame_rgay;     // 构建opencv对象 灰度

uint8_t *rgay_image;    // 灰度图像数组指针
uint8_t *rgb_image;     // 新增：彩色图像数组指针（BGR24格式）

VideoCapture cap;
static const char *g_uvc_path = NULL;   // 记录设备路径，用于 select() 超时保护

int8 uvc_camera_init(const char *path)
{
    g_uvc_path = path;

    cap.open(path);

    if(!cap.isOpened())
    {
        printf("find uvc camera error.\r\n");
        return -1;
    } 
    else 
    {
        printf("find uvc camera Successfully.\r\n");
    }

    cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('Y', 'U', 'Y', 'V'));  // 设置格式为YUYV（摄像头原生）
    cap.set(CAP_PROP_FRAME_WIDTH, UVC_WIDTH);                           // 设置摄像头宽度
    cap.set(CAP_PROP_FRAME_HEIGHT, UVC_HEIGHT);                         // 设置摄像头高度
    cap.set(CAP_PROP_FPS, UVC_FPS);                                     // 显示屏幕帧率
    cap.set(CAP_PROP_BUFFERSIZE, 1);                                    // 最小缓冲，降低延迟

    printf("get uvc width = %f.\r\n",  cap.get(CAP_PROP_FRAME_WIDTH));
    printf("get uvc height = %f.\r\n", cap.get(CAP_PROP_FRAME_HEIGHT));
    printf("get uvc fps = %f.\r\n",    cap.get(CAP_PROP_FPS));

    return 0;
}


/*
 * 非阻塞检查：V4L2 设备上是否有帧数据可读
 *
 * 使用 select() 轮询，超时 50ms
 * 摄像头正常时 ~1ms 内返回，断连时最多等 50ms
 */
static bool uvc_frame_ready(void)
{
    if (!g_uvc_path) return true;   // 无路径则回退到阻塞模式

    int fd = open(g_uvc_path, O_RDONLY | O_NONBLOCK);
    if (fd < 0) return true;

    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    struct timeval tv = {0, 50000};  // 50ms 超时

    int ret = select(fd + 1, &fds, NULL, NULL, &tv);
    close(fd);
    return (ret > 0);
}


int8 wait_image_refresh()
{
    if (!uvc_frame_ready()) {
        return 0;
    }

    try 
    {
        cap >> frame_rgb;
        if (frame_rgb.empty()) 
        {
            std::cerr << "未获取到有效图像帧" << std::endl;
            return -1;
        }
    } 
    catch (const cv::Exception& e) 
    {
        std::cerr << "OpenCV 异常: " << e.what() << std::endl;
        return -1;
    }

    // rgb转灰度
    cv::cvtColor(frame_rgb, frame_rgay, cv::COLOR_BGR2GRAY);

    // cv对象转指针
    rgay_image = reinterpret_cast<uint8_t *>(frame_rgay.ptr(0));

    return 1;
}

// 新增函数：获取彩色图像，不进行灰度转换
//
// 返回值: 1=成功获取帧, 0=暂无帧(50ms内未等到), -1=错误
//         调用方应循环调直到返回 1 或 -1
int8 wait_image_refresh_rgb()
{
    // 非阻塞检查：帧未就绪则立即返回，不阻塞主循环
    if (!uvc_frame_ready()) {
        return 0;
    }

    try 
    {
        // 帧已就绪，阻塞读极快（通常 <1ms）
        cap >> frame_rgb;
        if (frame_rgb.empty()) 
        {
            std::cerr << "未获取到有效图像帧" << std::endl;
            return -1;
        }
    } 
    catch (const cv::Exception& e) 
    {
        std::cerr << "OpenCV 异常: " << e.what() << std::endl;
        return -1;
    }

    // 直接指向彩色图像数据（BGR24格式）
    rgb_image = frame_rgb.data;   // frame_rgb.data 是 uchar*，兼容 uint8_t*

    return 1;
}

