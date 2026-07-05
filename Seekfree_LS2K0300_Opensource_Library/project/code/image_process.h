/*******************************************************************************
* UVC摄像头设备进程文件
*******************************************************************************/
#ifndef __image_process_H__
#define __image_process_H__

#define BEEP    "/dev/zf_driver_gpio_beep"

//UVC摄像头识别相关函数
// 返回1表示识别到物体，返回-1表示处理了但未识别，返回0表示跳帧/无帧
int QR_process(void);
int object_tracking(void);
void coordinate_transformation(void);

// 物块跟踪的像素坐标值
extern int16_t coordinate_x, coordinate_y;
// 映射完成的物理坐标值
extern float real_x, real_y;

// 二维码解码结果：QR_process() 识别成功时写入，transport() 消费后清零
extern char g_qr_data[128];
extern volatile bool g_qr_data_ready;


#endif