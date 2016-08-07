																			   /**
  ******************************************************************************
  * @file    main.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    11/20/2009
  * @brief   This file contains all the functions prototypes for the main.c 
  *          file.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
 extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f107.h"


/* Exported function prototypes ----------------------------------------------*/
void Time_Update(void);
void Delay(uint32_t nCount);

//MYSELFBOARD定义了表示使用开发板。不定义表示使用我自己做的那板子
//框架一样但是1820的IO口和SD卡的CS引脚不一样。
#define MYSELFBOARD
//使用SMTP
//#define USED_SMTP


//定义了则使用MODBUS RTU TX/RX底层收发 (注意应用层没有使用。因为应用层打算交给服务器做，这边仅仅做RTU透传)
//#define USED_MODBUS		1



//定义后允许使用SD卡文件系统
//#define USED_FILESYSTEM

//定义后允许使用HTTP 
//#define USED_HTTP

//定义后允许使用 DS18B20 采集温度
#define USED_DS18B20

//定义后允许对采集的温度进行PID运算
#define USING_PID

//定义后允许使用网络
#define  USING_NET

//定义后允许使用显示屏
#define USING_DISPLAY

//定义后允许使用按键
#define USING_KEY



#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/

