/*******************************************************************************
* 进程控制文件
*******************************************************************************/

#include "zf_common_headfile.h"
#include <time.h>       // clock_gettime

#define GRASP_KEEP_DELAY_SEC    5   // 收到 DONE 后延迟 N 秒再进入下一步
#define GRASP_CONTROL_DELAY_SEC 1   // 舵机到位后延迟 N 秒再闭合夹爪

typedef enum{
    PROCESS_IDLE = 0,   // 进程 不进行/空闲
    GRASP_RESET,        // 正在 复位 机械臂
    QR_SCANNING,        // 正在 扫描/寻找 二维码
    QR_SEND,            // 正在 发送 数据
    TRACK_SCANNING,     // 正在 寻找 物块
    GRASP_CONTROL,      // 正在 控制 机械臂 
    GRASP_UP,           // 正在 抬起 机械臂 （在这一步结尾或者下一步，将发送信息给下板）
    GRASP_KEEP,         // 正在 保持 机械臂
    GRASP_RELEASE,      // 正在 松开 机械臂
    PROCESS_DONE,       // 进程 完成
}Sub_B_PROCESS;

Sub_B_PROCESS Cur_STATE = PROCESS_IDLE; // 初始化为 进程 不进行/空闲

// 二维码识别结果
static char QR_Result[128];

// 物块识别相关
// 上一次识别得到的坐标
int16_t pre_coordinate_x = 9999;
int16_t pre_coordinate_y = 9999;
float sum_real_x = 0.0f;
float sum_real_y = 0.0f;
uint8_t coordinate_stable_count = 0;

int Sub_Board_Process(void)
{

    ips200_clear();
    ips200_show_string(8  ,0  , "IDLE");

    while(1)
    {
        /* 按键处理*/
        if (Key_Check(KEY_NAME_UP,KEY_SINGLE) || Key_Check(KEY_NAME_DOWN,KEY_SINGLE))
        {
            // 状态重置为 进程 不进行/空闲
            Cur_STATE = PROCESS_IDLE;
               
            ips200_clear();
            ips200_show_string(8  ,0  , "IDLE");

            //机械臂控制关闭
            servo_move_sync(0);
            servo_reset_init(0);
            Stop_Servo_All();
        }
        else if (Key_Check(KEY_NAME_CONFIRM,KEY_SINGLE))
        {
            // 进程开始，第一步为复位机械臂
            servo_reset_init(1);
            printf("GRASP_RESET\n");
            Cur_STATE = GRASP_RESET;        
        }
        else if(Key_Check(KEY_NAME_BACK,KEY_SINGLE))
        {
            servo_move_sync(0);
            servo_reset_init(0);
            Stop_Servo_All();
            // 返回上一级界面
            return 0;
        }


        // 进程状态机
        switch(Cur_STATE)
        {
            // 进程 不进行/空闲
            case PROCESS_IDLE:{

                
                break;
            }

            // 正在 复位 机械臂
            case GRASP_RESET:{

                // 推进机械臂复位状态机
                if (servo_reset_init(2))
                {
                    // 进程进入下一步
                    printf("QR_SCANNING\n");
                    Cur_STATE = QR_SCANNING;                   
                }
                break;
            }

            // 正在 扫描/寻找 二维码
            case QR_SCANNING:{

                const char* qr = QR_process();
                // 二维码有返回值
                if (qr != nullptr) {
                    strncpy(QR_Result, qr, sizeof(QR_Result) - 1);
                    QR_Result[sizeof(QR_Result) - 1] = '\0';
                    
                    // 进程进入下一步
                    printf("QR_SEND\n");
                    Cur_STATE = QR_SEND;                    
                }

                break;
            }

            // 正在 发送 数据
            case QR_SEND:{      
            
                // 清理屏幕，顺便显示一些东西
                ips200_clear();
                ips200_show_string(8  ,0  , "QR_SEND");

                if (strcmp(QR_Result, "01") == 0)
                {
                    uart1_printf("[01]\n");
                }
                else if (strcmp(QR_Result, "02") == 0)
                {
                    uart1_printf("[02]\n");
                }
                else if (strcmp(QR_Result, "03") == 0)
                {
                    uart1_printf("[03]\n");
                }

                // 进程进入下一步
                printf("TRACK_SCANNING\n");
                Cur_STATE = TRACK_SCANNING;

                break;
            }

            // 正在 寻找 物块
            case TRACK_SCANNING:{     
                
                if (object_tracking() == 1) // 有识别结果的帧
                {
                    if ((abs(pre_coordinate_x - coordinate_x) <= 2) &&
                    (abs(pre_coordinate_y - coordinate_y) <= 2))
                    {
                        coordinate_stable_count ++;
                    }
                    else
                    {
                        // 实际值累计数据重置
                        sum_real_x = 0.0f;
                        sum_real_y = 0.0f;
                        // 稳定计数重置
                        coordinate_stable_count = 0;
                    }

                    // 更新历史值
                    pre_coordinate_x = coordinate_x;
                    pre_coordinate_y = coordinate_y;

                    if (coordinate_stable_count > (20 - 10))
                    {
                        coordinate_transformation();

                        sum_real_x += real_x;
                        sum_real_y += real_y;
                        if (coordinate_stable_count >= 20)
                        {
                            x_result = sum_real_x / 10.0f + 0.0f;
                            y_result = sum_real_y / 10.0f + 0.0f;
                            sum_real_x    = 0.0f;
                            sum_real_y    = 0.0f;
                            if (grasp_compute_angles())
                            {
                                // printf("B:%.1f ,1:%.1f ,2:%.1f\n", target_angle[0], target_angle[1], target_angle[2]);
                                coordinate_stable_count = 0;

                                // 进程进入下一步
                                printf("GRASP_CONTROL\n");
                                Cur_STATE = GRASP_CONTROL;
                            }
                            else // 解析失败，重新开始识别物块位置
                            {
                                // 实际值累计数据重置
                                sum_real_x = 0.0f;
                                sum_real_y = 0.0f;
                                // 稳定计数重置
                                coordinate_stable_count = 0;
                            }
                        }
                    }
                }

                break;
            }

            // 正在 控制 机械臂
            case GRASP_CONTROL:{
                static uint64_t deadline_us = 0;   // 0=无延迟

                // 延迟等待中：检查是否到期
                if (deadline_us != 0) {
                    struct timespec ts;
                    clock_gettime(CLOCK_MONOTONIC, &ts);
                    uint64_t now_us = (uint64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
                    if (now_us >= deadline_us) {
                        deadline_us = 0;
                        // 夹爪闭合
                        Servo_Set_Angle(DEFINE_JOINT_GRIPPER, 85.0f);
                        current_angle[NAME_JOINT_GRIPPER] = 85.0f;
                        target_angle[NAME_JOINT_GRIPPER] = 85.0f;
                        printf("GRASP_UP\n");
                        Cur_STATE = GRASP_UP;
                    }
                    break;
                }

                object_tracking();
                if (servo_move_sync(1)) // 等待底座，一大臂，二大臂到达设置位置
                {
                    // 启动延迟，GRASP_CONTROL_DELAY_SEC 秒后再闭合夹爪
                    struct timespec ts;
                    clock_gettime(CLOCK_MONOTONIC, &ts);
                    deadline_us = (uint64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000
                                + (uint64_t)GRASP_CONTROL_DELAY_SEC * 1000000UL;
                }

                break;
            }

            // 正在 抬起 机械臂 在这一步结尾或者下一步，将发送信息给下板）
            case GRASP_UP:{       
                
                object_tracking();
                target_angle[NAME_JOINT_BASE]    = 90.0f;
                target_angle[NAME_JOINT_ARM_1]   = 90.0f;
                target_angle[NAME_JOINT_ARM_2]   = 90.0f;
                if (servo_move_sync(1)) //等待果底座，一大臂，二大臂到达设置位置
                {
                    // 进程进入下一步
                    printf("GRASP_KEEP\n");
                    uart1_printf("[DONE]\n");
                    Cur_STATE = GRASP_KEEP;
                }

                break;
            }

            // 正在 保持 机械臂
            case GRASP_KEEP:{
                static uint64_t deadline_us = 0;   // 0=无延迟, 非0=到期时刻(微秒)

                // 延迟等待中：检查是否到期
                if (deadline_us != 0) {
                    struct timespec ts;
                    clock_gettime(CLOCK_MONOTONIC, &ts);
                    uint64_t now_us = (uint64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
                    if (now_us >= deadline_us) {
                        deadline_us = 0;

                        // 进程进入下一步
                        printf("GRASP_RELEASE\n");
                        Cur_STATE = GRASP_RELEASE;
                    }
                    break;
                }

                /* UART1 接收 */
                char *cmd = uart1_recv_frame();
                if (cmd) // 如果拿到了完整的包体数据（已经去除包头包尾）
                {
                    if (strcmp(cmd, "DONE") == 0)        // 收到 [DONE]\n
                    {
                        // 启动延迟，GRASP_KEEP_DELAY_SEC 秒后再进入下一步
                        struct timespec ts;
                        clock_gettime(CLOCK_MONOTONIC, &ts);
                        deadline_us = (uint64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000
                                    + (uint64_t)GRASP_KEEP_DELAY_SEC * 1000000UL;
                    }
                }

                break;
            }

            // 正在 松开 机械臂
            case GRASP_RELEASE:{  
            
                // 夹爪设置在135°
                Servo_Set_Angle(DEFINE_JOINT_GRIPPER, 170.0f);
                current_angle[NAME_JOINT_GRIPPER] = 170.0f;
                target_angle[NAME_JOINT_GRIPPER] = 170.0f;

                // 进程进入下一步
                printf("PROCESS_DONE\n");
                Cur_STATE = PROCESS_DONE;

                break;
            }

            // 进程 完成
            case PROCESS_DONE:{       

                break;
            }
        }

    }
}