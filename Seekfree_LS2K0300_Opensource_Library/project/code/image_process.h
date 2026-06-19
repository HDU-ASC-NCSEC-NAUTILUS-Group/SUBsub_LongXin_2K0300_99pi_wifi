/*******************************************************************************
* UVC摄像头设备进程文件
*******************************************************************************/
#ifndef __image_process_H__
#define __image_process_H__


//UVC摄像头识别相关函数
// 返回1表示识别到物体，返回-1表示处理了但未识别，返回0表示跳帧/无帧
int QR_process(void);
int object_tracking(void);
void coordinate_transformation(void);

// 映射完成的物理坐标值
extern float real_x, real_y;


#endif