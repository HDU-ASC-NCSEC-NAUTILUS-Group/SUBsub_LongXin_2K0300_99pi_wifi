/*******************************************************************************
* 全功能非阻塞式按键
* 原代码来自江协科技
*******************************************************************************/
#ifndef __KEY_H__
#define __KEY_H__


// 按键宏定义
// 一级命名
#define KEY_DEFINE_1            "/dev/zf_driver_gpio_key_3"    //P16;S3
#define KEY_DEFINE_2            "/dev/zf_driver_gpio_key_2"    //P15;S4
#define KEY_DEFINE_3            "/dev/zf_driver_gpio_key_1"    //P14;S5
#define KEY_DEFINE_4            "/dev/zf_driver_gpio_key_0"    //P13;S6

// 二级映射
#define KEY_DEFINE_UP           KEY_DEFINE_1
#define KEY_DEFINE_DOWN         KEY_DEFINE_2
#define KEY_DEFINE_CONFIRM      KEY_DEFINE_3
#define KEY_DEFINE_BACK         KEY_DEFINE_4

//用宏定义替换1/0，便于理解Key_GetState()函数
#define KEY_PRESSED 			1
#define	KEY_UNPRESSED			0

//宏定义替换时间阈值
/*
说明：按键检测由 Key_Tick() 驱动（如定时中断 10ms 调用），
但状态机每 2 次调用才推进一次（Key.cpp: Count >= 2），
实际状态机周期 = 2 × 中断周期（如 20ms）。
下方的各时间阈值以该实际周期为单位递减，
所以应设为实际周期（如 20ms）的倍数。
*/
#define KEY_Time_DOUBLE 		0
#define KEY_Time_LONG			200
#define KEY_Time_REPEAT			100

//宏定义调换按键数量
#define KEY_COUNT				4

//用宏定义替换按键索引号
// 一级定义
#define KEY_1				    0	
#define KEY_2			        1	
#define KEY_3		            2	
#define KEY_4			        3	

#define KEY_NAME_UP				KEY_1
#define KEY_NAME_DOWN			KEY_2
#define KEY_NAME_CONFIRM		KEY_3
#define KEY_NAME_BACK			KEY_4

//用宏定义替换按键标志位的位掩码，使程序的意义更清晰
#define KEY_HOLD				0x01
#define KEY_DOWN				0x02
#define KEY_UP					0x04
#define KEY_SINGLE				0X08
#define KEY_DOUBLE				0x10
#define KEY_LONG				0x20
#define KEY_REPEAT				0x40

uint8_t Key_Check(uint8_t n, uint8_t Flag);
void Key_Tick(void);


#endif