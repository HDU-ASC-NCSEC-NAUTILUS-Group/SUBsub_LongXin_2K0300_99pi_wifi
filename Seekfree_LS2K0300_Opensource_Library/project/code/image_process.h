/*******************************************************************************
* UVC摄像头设备进程文件
*******************************************************************************/
#ifndef __image_process_H__
#define __image_process_H__


//UVC摄像头识别相关函数
// 二维码解码：识别成功返回解码字符串，无结果返回 NULL
const char* QR_process(void);

int object_tracking(void);
void coordinate_transformation(void);

// 物块跟踪的像素坐标值
extern int16_t coordinate_x, coordinate_y;
// 映射完成的物理坐标值
extern float real_x, real_y;


#endif