/*******************************************************************************
* UVC摄像头设备进程文件
*******************************************************************************/
#ifndef __image_process_H__
#define __image_process_H__


//UVC摄像头识别相关函数
// 返回1表示本帧处理了图像, 返回0表示跳过或错误
int QR_process(void);
int object_tracking(void);
void coordinate_transformation(void);


#endif