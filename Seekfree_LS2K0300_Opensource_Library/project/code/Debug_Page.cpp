/********************************************************************************************************************
* Debug模式菜单文件
********************************************************************************************************************/
#include "zf_common_headfile.h"

#include "image_process.h"



/*******************************************************************************************************************/
/*[S] 菜单样式 [S]--------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

// [二级界面]Debug模式界面
void Debug_Page_Menu_UI(uint8_t Page)
{
	switch(Page)
	{
		// 第一页
		case 1:
			ips200_show_string(8  ,0  , "[Debug]");
			ips200_show_string(0  ,16 , "==============================");
			ips200_show_string(10 ,32 , "BUZ");
            ips200_show_string(10 ,48 , "UVC-QR");
            ips200_show_string(10 ,64 , "UVC-TRACK");
		
			break;
	}
}

// [三级界面]蜂鸣器调试界面
void Debug_BUZ_UI(void)
{
    ips200_show_string(8  ,0  , "[DEBUG]-BUZ");
    ips200_show_string(0  ,16 , "==============================");
    ips200_show_string(10 ,32 , "OFF");
}

// [三级界面]UVC摄像头的二维码识别调试界面
void Debug_UVC_QR_UI(void)
{
    ips200_show_string(8  ,0  , "[DEBUG]-UVC-QR");
    ips200_show_string(0  ,16 , "==============================");
    // 下面的区域将被UVC回传的图像覆盖
}

// [三级界面]UVC摄像头的跟踪调试界面
void Debug_UVC_TRACK_UI(void)
{
    ips200_show_string(8  ,0  , "[DEBUG]-UVC-TRACK");
    ips200_show_string(0  ,16 , "==============================");
    // 下面的区域将被UVC回传的图像覆盖
}
/*******************************************************************************************************************/
/*--------------------------------------------------------------------------------------------------[E] 菜单样式 [E]*/
/*******************************************************************************************************************/


/*******************************************************************************************************************/
/*[S] 界面逻辑 [S]--------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

// 相关函数提前声明
int Debug_BUZ           (void);
int Debug_UVC_QR        (void);
int Debug_UVC_TRACK     (void);


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
            if (Debug_Page_flag < 1)Debug_Page_flag = 3;
        }
        else if (Key_Check(KEY_NAME_DOWN,KEY_SINGLE))
        {
            key_pressed = 1;
            Debug_Page_flag ++;
            if (Debug_Page_flag > 3)Debug_Page_flag = 1;
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
            Debug_BUZ();
            
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
            }
        }
    }
}


//	####   #   #  #####
//	#   #  #   #     #  
//	####   #   #    #  
//	#   #  #   #   #   
//	####    ###   ##### 
//
// [三级界面]蜂鸣器调试
int Debug_BUZ(void)
{
    Debug_BUZ_UI();
    uint8_t BUZ_flag = 0;

    while(1)
    {
        /* 按键处理*/   
        if (Key_Check(KEY_NAME_CONFIRM,KEY_SINGLE)) 
        {
            BUZ_flag = 1 - BUZ_flag;
            if (BUZ_flag)
            {
                // 开启蜂鸣器
                gpio_set_level(BEEP_DEFINE, 0x1);
                ips200_show_string(10 ,32 , "ON ");
            }
            else
            {
                // 关闭蜂鸣器
                gpio_set_level(BEEP_DEFINE, 0x0);
                ips200_show_string(10 ,32 , "OFF");
            }
        }
        else if (Key_Check(KEY_NAME_BACK,KEY_SINGLE))
        {
            // 返回上一级界面
            return 0;   
        }
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
        if (Key_Check(KEY_NAME_BACK,KEY_SINGLE))
        {
            // 恢复默认颜色
            ips200_set_pen_color(RGB565_RED);
            // 返回上一级界面
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
        if (Key_Check(KEY_NAME_BACK,KEY_SINGLE))
        {
            // 恢复默认颜色
            ips200_set_pen_color(RGB565_RED);
            // 返回上一级界面
            return 0;   
        }
        object_tracking();  // 红色物块跟踪显示
//        coordinate_transformation();  // 坐标转换显示
    }
}
/*******************************************************************************************************************/
/*--------------------------------------------------------------------------------------------------[E] 界面逻辑 [E]*/
/*******************************************************************************************************************/
