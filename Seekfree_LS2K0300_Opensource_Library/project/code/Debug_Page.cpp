/*******************************************************************************
* Debug模式菜单文件
*******************************************************************************/
#include "zf_common_headfile.h"
#include <time.h>       // clock_gettime

#define ARM_MOVE_DONE_DELAY_SEC 1   // 舵机到位后延迟 N 秒再闭合夹爪

/**********************************************************/
/*[S] 菜单样式 [S]-----------------------------------------*/
/**********************************************************/

// [二级界面]Debug模式界面
void Debug_Page_Menu_UI(uint8_t Page)
{
	switch(Page)
	{
		// 第一页
		case 1:
			ips200_show_string(8  ,0  , "[Debug]");
			ips200_show_string(0  ,16 , "==============================");
			ips200_show_string(10 ,32 , "UART1");
            ips200_show_string(10 ,48 , "UVC-QR");
            ips200_show_string(10 ,64 , "UVC-TRACK");
            ips200_show_string(10 ,80 , "Servo(PCA9685)");
            ips200_show_string(10 ,96 , "ARM-Grasp");
		
			break;
	}
}

// [三级界面]UART1调试界面
void Debug_UART1_UI(void)
{
    ips200_show_string(8  ,0  , "[DEBUG]-UART1");
    ips200_show_string(0  ,16 , "==============================");
    ips200_show_string(0  ,32 , "  [Press CONFIRM to send TX]");
    ips200_show_string(0  ,48 , "TX:");
    // 占位(必要时上一行溢出的字符会切割到这一行显示)
    ips200_show_string(0  ,80 , "RX:");
    // 占位(必要时上一行溢出的字符会切割到这一行显示)
    // 占位
    ips200_show_string(0  ,128, "CH1:"); // 切分到的第一个字符
    ips200_show_string(0  ,144, "CH2:"); // 切分到的第二个字符(如果有)
    ips200_show_string(0  ,160, "CH3:"); // 切分到的第三个字符(如果有)
}

// [三级界面]UVC摄像头的二维码识别调试界面
void Debug_UVC_QR_UI(void)
{
}

// [三级界面]UVC摄像头的跟踪调试界面
void Debug_UVC_TRACK_UI(void)
{
}

// [三级界面]舵机(PCA9685驱动)调试界面
void Debug_Servo_UI(void)
{
    ips200_show_string(8  ,0  , "[DEBUG]-Servo(PCA9685)");
    ips200_show_string(0  ,16 , "==============================");
    ips200_show_string(0  ,32 , "[Set_Angle]");
    ips200_show_string(10 ,48 , "Servo_1:##");
    ips200_show_string(10 ,64 , "Servo_2:##");
    ips200_show_string(10 ,80 , "Servo_3:##");
    ips200_show_string(10 ,96 , "Servo_4:##");
    ips200_show_string(10 ,112, "Reset(OFF)");
}

// [三级界面]机械臂抓取调试界面
void ARM_Grasp_UI(void)
{
    ips200_show_string(8  ,0  , "[DEBUG]-ARM-Grasp");
    ips200_show_string(0  ,16 , "==============================");
}

/**********************************************************/
/*-----------------------------------------[E] 菜单样式 [E]*/
/**********************************************************/


/**********************************************************/
/*[S] 界面逻辑 [S]-----------------------------------------*/
/**********************************************************/

// 相关函数提前声明
int Debug_UART1         (void);
int Debug_UVC_QR        (void);
int Debug_UVC_TRACK     (void);
int Debug_Servo         (void);
int Debug_ARM_Grasp     (void);


// [二级界面]Debug模式界面
int Debug_Page_Menu(void)
{
    // Debug模式选项 标志位
    uint8_t Debug_Page_flag = 1;

    Debug_Page_Menu_UI(1);
    ips200_show_string(0  ,32 , ">");

    while(1)
    {
        // 存储确认键被按下时Debug_Page_flag的值的临时变量，默认为无效值0
		uint8_t Debug_Page_flag_temp = 0;
		// 上/下按键是否被按下过
		uint8_t key_pressed = 0;


        /* 按键处理*/
        if (Key_Check(KEY_NAME_UP,KEY_SINGLE))
        {
            key_pressed = 1;
            Debug_Page_flag --;
            if (Debug_Page_flag < 1)Debug_Page_flag = 5;
        }
        else if (Key_Check(KEY_NAME_DOWN,KEY_SINGLE))
        {
            key_pressed = 1;
            Debug_Page_flag ++;
            if (Debug_Page_flag > 5)Debug_Page_flag = 1;
        }
        else if (Key_Check(KEY_NAME_CONFIRM,KEY_SINGLE))
        {
            Debug_Page_flag_temp = Debug_Page_flag;
        }
        else if (Key_Check(KEY_NAME_BACK,KEY_SINGLE))    
        {
            // 返回上一级界面
            return 0;   
        }


        /* 模式跳转*/
        if (Debug_Page_flag_temp == 1)
        {
            ips200_clear();
            Debug_UART1();
            
            // 从子界面返回后
            ips200_clear();
            Debug_Page_Menu_UI(1);
            ips200_show_string(0  ,32 , ">");
        }
        else if (Debug_Page_flag_temp == 2)
        {
            ips200_clear();
            Debug_UVC_QR();
            
            // 从子界面返回后
            ips200_clear();
            Debug_Page_Menu_UI(1);
            ips200_show_string(0  ,48 , ">");
        }
        else if (Debug_Page_flag_temp == 3)
        {
            ips200_clear();
            Debug_UVC_TRACK();
            
            // 从子界面返回后
            ips200_clear();
            Debug_Page_Menu_UI(1);
            ips200_show_string(0  ,64 , ">");
        }
        else if (Debug_Page_flag_temp == 4)
        {
            ips200_clear();
            Debug_Servo();
            
            // 从子界面返回后
            ips200_clear();
            Debug_Page_Menu_UI(1);
            ips200_show_string(0  ,80 , ">");
        }
        else if (Debug_Page_flag_temp == 5)
        {
            ips200_clear();
            Debug_ARM_Grasp();
            
            // 从子界面返回后
            ips200_clear();
            Debug_Page_Menu_UI(1);
            ips200_show_string(0  ,96 , ">");
        }
        

        /* 显示更新*/
        if (key_pressed)
        {
            switch (Debug_Page_flag)
            {
                case 1:
                    ips200_clear();
                    Debug_Page_Menu_UI(1);
                    ips200_show_string(0  ,32 , ">");

                    break;
                case 2:
                    ips200_clear();
                    Debug_Page_Menu_UI(1);
                    ips200_show_string(0  ,48 , ">");

                    break;
                case 3:
                    ips200_clear();
                    Debug_Page_Menu_UI(1);
                    ips200_show_string(0  ,64 , ">");

                    break;
                case 4:
                    ips200_clear();
                    Debug_Page_Menu_UI(1);
                    ips200_show_string(0  ,80 , ">");

                    break;
                case 5:
                    ips200_clear();
                    Debug_Page_Menu_UI(1);
                    ips200_show_string(0  ,96 , ">");

                    break;
            }
        }
    }
}

//  #   #   ###   ####   #####    #    
//  #   #  #   #  #   #    #    ###    
//  #   #  #####  ####     #      #    
//  #   #  #   #  #  #     #      #    
//   ###   #   #  #   #    #    #####  
//
// [三级界面]UART1调试
int Debug_UART1(void)
{
    Debug_UART1_UI();

    const char *words[] = {"AAA", "BBB", "CCC", "DDD", "EEE", "FFF"};
    const int num_words = 6;
    int window_start = 0;

    #define PARSE_BUF_SIZE 128
    uint8_t rx_ring[PARSE_BUF_SIZE];
    int rx_pos = 0;

    while(1)
    {
        // 消费掉按键判定
        Key_Check(KEY_NAME_UP,KEY_SINGLE);
        Key_Check(KEY_NAME_DOWN,KEY_SINGLE);

        if (Key_Check(KEY_NAME_CONFIRM, KEY_SINGLE))
        {
            char send_buf[64];
            int len = snprintf(send_buf, sizeof(send_buf), "[%s,%s,%s]\n",
                     words[window_start % num_words],
                     words[(window_start + 1) % num_words],
                     words[(window_start + 2) % num_words]);
            uart1_send((uint8*)send_buf, len);

            ips200_Printf(30, 48, "[%s,%s,%s]\\n  ",
                     words[window_start % num_words],
                     words[(window_start + 1) % num_words],
                     words[(window_start + 2) % num_words]);

            window_start = (window_start + 1) % num_words;
        }
        else if (Key_Check(KEY_NAME_BACK, KEY_SINGLE))
        {
            return 0;
        }

        {
            int n = uart1_recv(rx_ring + rx_pos, sizeof(rx_ring) - rx_pos - 1);
            if (n > 0)
            {
                rx_pos += n;
                rx_ring[rx_pos] = 0;

                {
                    char rx_display[64];
                    int j = 0;
                    for (int i = 0; rx_ring[i] && j < (int)sizeof(rx_display) - 2; i++)
                    {
                        if (rx_ring[i] == '\n')
                        {
                            rx_display[j++] = '\\';
                            rx_display[j++] = 'n';
                        }
                        else if (rx_ring[i] == '\r')
                        {
                            rx_display[j++] = '\\';
                            rx_display[j++] = 'r';
                        }
                        else
                        {
                            rx_display[j++] = rx_ring[i];
                        }
                    }
                    rx_display[j] = '\0';
                    ips200_Printf(30, 80, "%-20s", rx_display);
                }

                char *end = strchr((char*)rx_ring, '\n');
                if (end)
                {
                    *end = 0;

                    char *start = strchr((char*)rx_ring, '[');
                    char *stop  = strchr((char*)rx_ring, ']');
                    if (start && stop && stop > start)
                    {
                        char parse_buf[PARSE_BUF_SIZE];
                        int len = stop - start - 1;
                        if (len >= PARSE_BUF_SIZE) len = PARSE_BUF_SIZE - 1;
                        memcpy(parse_buf, start + 1, len);
                        parse_buf[len] = '\0';

                        char *ch1 = strtok(parse_buf, ",");
                        char *ch2 = strtok(NULL, ",");
                        char *ch3 = strtok(NULL, ",");

                        ips200_Printf(40, 128, "%-10s", ch1 ? ch1 : "");
                        ips200_Printf(40, 144, "%-10s", ch2 ? ch2 : "");
                        ips200_Printf(40, 160, "%-10s", ch3 ? ch3 : "");
                    }

                    int remain = rx_pos - (end + 1 - (char*)rx_ring);
                    if (remain > 0)
                    {
                        memmove(rx_ring, end + 1, remain);
                        rx_pos = remain;
                    }
                    else
                    {
                        rx_pos = 0;
                    }
                }
            }
        }

        system_delay_ms(10);
    }
}

// #   #  #   #   ####         ###   ####   
// #   #  #   #  #            #   #  #   #  
// #   #  #   #  #      ###   #   #  ####   
// #   #   # #   #            #  ##  #  #   
//  ###     #     ####         ####  #   #  
//
// [三级界面]二维码调试
int Debug_UVC_QR(void)
{
    Debug_UVC_QR_UI();

    while(1)
    {
        // 消费掉按键判定
        Key_Check(KEY_NAME_UP,KEY_SINGLE);
        Key_Check(KEY_NAME_DOWN,KEY_SINGLE);
        Key_Check(KEY_NAME_CONFIRM,KEY_SINGLE);

        if (Key_Check(KEY_NAME_BACK, KEY_SINGLE))
        {
            return 0;
        }

        QR_process();
    }
}

// #   #  #   #   ####         #####  ####    ###    ####  #   #
// #   #  #   #  #               #    #   #  #   #  #      #  #
// #   #  #   #  #       ###     #    ####   #####  #      ###
// #   #   # #   #               #    #  #   #   #  #      #  #
//  ###     #     ####           #    #   #  #   #   ####  #   #
//
// [三级界面]物块跟踪调试
int Debug_UVC_TRACK(void)
{
    Debug_UVC_TRACK_UI();

    while(1)
    {
        // 消费掉按键判定
        Key_Check(KEY_NAME_UP,KEY_SINGLE);
        Key_Check(KEY_NAME_DOWN,KEY_SINGLE);
        Key_Check(KEY_NAME_CONFIRM,KEY_SINGLE);

        if (Key_Check(KEY_NAME_BACK, KEY_SINGLE))
        {
            return 0;
        }
        
        if (object_tracking() == 1)
        {
            coordinate_transformation();
        }
    }
}

// #####  #####  ####   #   #   ###   
// #      #      #   #  #   #  #   #  
// #####  #####  ####   #   #  #   #  
//     #  #      #  #    # #   #   #  
// #####  #####  #   #    #     ###   
//
// [三级界面]舵机(PCA9685驱动)调试
int Debug_Servo(void)
{
    // 舵机调试 标志位
    uint8_t Debug_Servo_flag = 1;
    // 存储确认键被按下时Debug_Servo_flag的值的临时变量，默认为无效值0
    uint8_t Debug_Servo_flag_temp = 0;

    Debug_Servo_UI();
    ips200_show_string(0 ,48 , ">");

    // 存储设置的舵机角度,实际考虑更改数值时才做实际设置PWM
    uint8_t Angle[4] = {90 ,90 ,135 ,145};

    // 停止所有的舵机控制
    Stop_Servo_All();

    ips200_Printf(74 ,48 , "%d    ", Angle[0]);
    ips200_Printf(74 ,64 , "%d    ", Angle[1]);
    ips200_Printf(74 ,80 , "%d    ", Angle[2]);
    ips200_Printf(74 ,96 , "%d    ", Angle[3]);

        while(1)
    {
        // 上/下按键是否被按下过
        uint8_t key_pressed = 0;

        /*======================================================*/
		/*[按键处理]**********************************************/
		/*======================================================*/
        // 选择模式（无选中目标）
        if (Debug_Servo_flag_temp == 0)
        {                   
            if (Key_Check(KEY_NAME_UP,KEY_SINGLE)) 
            {
                Debug_Servo_flag --;
                if (Debug_Servo_flag < 1)Debug_Servo_flag = 5;
                key_pressed = 1;
            }
            else if (Key_Check(KEY_NAME_DOWN,KEY_SINGLE)) 
            {
                Debug_Servo_flag ++;
                if (Debug_Servo_flag > 5)Debug_Servo_flag = 1;
                key_pressed = 1;
            }
            else if (Key_Check(KEY_NAME_CONFIRM,KEY_SINGLE))
            {
                Debug_Servo_flag_temp = Debug_Servo_flag;
            }
            else if (Key_Check(KEY_NAME_BACK,KEY_SINGLE))
            {
                // 返回上一级界面
                Stop_Servo_All();
                return 0;   
            }

            // 刚刚选中了目标，光标更新
            if (Debug_Servo_flag_temp != 0)
            {
                ips200_show_string(0  ,32 + Debug_Servo_flag * 16, "=");

                // "重置"选项
                if (Debug_Servo_flag_temp == 5)
                {
                    key_pressed = 2;
                    Debug_Servo_flag_temp = 0;

                    Angle[0] = Angle[1] = Angle[2] = Angle[3] = 90;
                    Stop_Servo_All();

                    ips200_show_string(0  ,32 + Debug_Servo_flag * 16, ">");
                }
            }
        }

        // 更改模式（有选中目标）
        else if (Debug_Servo_flag_temp != 0)
        {   
            if (Key_Check(KEY_NAME_UP,KEY_SINGLE)) 
            {
                key_pressed = 2;

                if ( 1 <= Debug_Servo_flag_temp && Debug_Servo_flag_temp <= 4)
                {
                    // 中间计算变量temp
                    int16_t temp = Angle[Debug_Servo_flag_temp - 1];
                    temp += 5;
                    // 边界处理
                    if (temp > 180)
                    {
                        temp = 180;
                    }
                    else if (temp < 0)
                    {
                        temp = 0;
                    }
                    Angle[Debug_Servo_flag_temp - 1] = temp;

                    Servo_Set_Angle(Debug_Servo_flag_temp - 1, Angle[Debug_Servo_flag_temp - 1]); 
                }                     
            }
            else if (Key_Check(KEY_NAME_DOWN,KEY_SINGLE)) 
            {
                key_pressed = 2;

                if ( 1 <= Debug_Servo_flag_temp && Debug_Servo_flag_temp <= 4)
                {
                    // 中间计算变量temp
                    int16_t temp = Angle[Debug_Servo_flag_temp - 1];
                    temp -= 5;
                    // 边界处理
                    if (temp > 180)
                    {
                        temp = 180;
                    }
                    else if (temp < 0)
                    {
                        temp = 0;
                    }
                    Angle[Debug_Servo_flag_temp - 1] = temp;

                    Servo_Set_Angle(Debug_Servo_flag_temp - 1, Angle[Debug_Servo_flag_temp - 1]); 
                }                            
            }
            else if ((Key_Check(KEY_NAME_CONFIRM,KEY_SINGLE)) || (Key_Check(KEY_NAME_BACK,KEY_SINGLE)))
            {
                // 光标更新
                ips200_show_string(0  ,32 + Debug_Servo_flag * 16, ">");
                Debug_Servo_flag_temp = 0;
            }
        }
        /*======================================================*/
		/**********************************************[按键处理]*/
		/*======================================================*/


        /* 光标显示更新*/
        if (key_pressed == 1)
        {
            // 清除光标，暂时用这个方法
            ips200_show_string(0  ,48 , " ");
            ips200_show_string(0  ,64 , " ");
            ips200_show_string(0  ,80 , " ");
            ips200_show_string(0  ,96 , " ");
            ips200_show_string(0  ,112, " ");
            ips200_show_string(0  ,32 + Debug_Servo_flag * 16, ">");
        }


        /* 舵机角度设置的显示更新*/
        if (key_pressed == 2)
        {
            ips200_Printf(74 ,48 , "%d    ", Angle[0]);
            ips200_Printf(74 ,64 , "%d    ", Angle[1]);
            ips200_Printf(74 ,80 , "%d    ", Angle[2]);
            ips200_Printf(74 ,96 , "%d    ", Angle[3]);
        }
    }
}

//  ###   ####   #   #          ###   ####    ###   #####  #####  
// #   #  #   #  ## ##         #      #   #  #   #  #      #   #  
// #####  ####   # # #   ###   #  ##  ####   #####  #####  #####  
// #   #  #  #   #   #         #   #  #  #   #   #      #  #      
// #   #  #   #  #   #          ###   #   #  #   #  #####  #      
//
// [三级界面]机械臂抓取调试
int Debug_ARM_Grasp(void)
{
    ARM_Grasp_UI();

    uint8_t Track_Success = 0;
    uint8_t Track_Count = 0;
    float x_sum = 0.0f, y_sum = 0.0f;

    // 机械臂工作状态标志位
    // 0 停止
    // 1 接收视觉返回
    // 2 开始舵机动作
    // 3 复位中
    // 4 夹爪闭合
    // 5 机械臂抬起
    uint8_t arm_work_state = 3;

    servo_reset_init(1);   // 启动复位，穿透执行第一个关节

    while(1)
    {
        if (Key_Check(KEY_NAME_UP,KEY_SINGLE) || Key_Check(KEY_NAME_DOWN,KEY_SINGLE))
        {
            // 复位中不重复触发
            if (arm_work_state != 3) {
                arm_work_state = 3;
                servo_reset_init(1);
            }
        }
        else if (Key_Check(KEY_NAME_CONFIRM,KEY_SINGLE) && arm_work_state != 3)
        {
            // 开始接收视觉返回（复位中忽略）
            arm_work_state = 1;
        }
        else if (Key_Check(KEY_NAME_BACK,KEY_SINGLE))
        {
            // 返回上一级界面
            arm_work_state = 0;
            x_sum       = 0.0f;
            y_sum       = 0.0f;
            Track_Count = 0;

            servo_move_sync(0);
            servo_reset_init(0);
            Stop_Servo_All();
            return 0;
        }

        // 复位中：逐帧推进，忽略其他逻辑
        if (arm_work_state == 3)
        {
            if (servo_reset_init(2))
            {
                arm_work_state = 0;
                printf("复位完成\n");
            }
            continue;
        }

        // 每次循环重置为0,识别成功后置1
        Track_Success = 0;

        // 视觉识别
        if (object_tracking() == 1)
        {
            coordinate_transformation();
            Track_Success = 1;
        }

        // 机械臂抓取解析
        if (Track_Success == 1 && arm_work_state == 1)
        {
            x_sum += real_x;
            y_sum += real_y;
            Track_Count += 1;
            printf("ARM:TRACK-%d\n",Track_Count);

            if (Track_Count == 10)
            {
                x_result = x_sum / 10.0f + 0.0f;
                y_result = y_sum / 10.0f + 0.0f;
                x_sum    = 0.0f;
                y_sum    = 0.0f;
                if (grasp_compute_angles())
                {
                    arm_work_state = 2;
                    printf("B:%.1f ,1:%.1f ,2:%.1f\n", target_angle[0], target_angle[1], target_angle[2]);
                    printf("ARM:MOVE-START!\n");
                }
                Track_Count = 0;
            }
        }

        // 舵机运动中
        if (arm_work_state == 2)
        {
            static uint64_t deadline_us = 0;   // 0=无延迟

            // 延迟等待中：检查是否到期
            if (deadline_us != 0) {
                struct timespec ts;
                clock_gettime(CLOCK_MONOTONIC, &ts);
                uint64_t now_us = (uint64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
                if (now_us >= deadline_us) {
                    deadline_us = 0;
                    arm_work_state = 4;
                    printf("ARM:MOVE-DONE!\n");
                }
            } else if (servo_move_sync(1)) {
                struct timespec ts;
                clock_gettime(CLOCK_MONOTONIC, &ts);
                deadline_us = (uint64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000
                            + (uint64_t)ARM_MOVE_DONE_DELAY_SEC * 1000000UL;
            }
        }

        // 夹爪闭合（一步到位）
        if (arm_work_state == 4)
        {
            Servo_Set_Angle(DEFINE_JOINT_GRIPPER, 85.0f);
            current_angle[NAME_JOINT_GRIPPER] = 85.0f;
            // 设定抬起目标角度（夹爪保持在85°不张开）
            target_angle[NAME_JOINT_GRIPPER] = 85.0f;
            target_angle[NAME_JOINT_BASE]    = 90.0f;
            target_angle[NAME_JOINT_ARM_1]   = 90.0f;
            target_angle[NAME_JOINT_ARM_2]   = 90.0f;
            arm_work_state = 5;
            printf("ARM:GRIP-CLOSE!\n");
        }

        // 机械臂抬起
        if (arm_work_state == 5)
        {
            if (servo_move_sync(1))
            {
                arm_work_state = 0;
                printf("ARM:LIFT-DONE!\n");
            }
        }
    }
}
/**********************************************************/
/*-----------------------------------------[E] 界面逻辑 [E]*/
/**********************************************************/