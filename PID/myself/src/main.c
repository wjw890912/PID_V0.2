/**
  ******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    11/20/2009
  * @brief   Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32_eth.h"
#include "netconf.h"
#include "main.h"
#include "filesystem.h"
#include "ds18b20.h"
#include "pidctrl.h" 
#include "ds12864.h" 
#include "key.h"


#ifdef USED_MODBUS
#include "stm32f10x_it.h"
#include "usart.h"
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#endif

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SYSTEMTICK_PERIOD_MS  10

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint32_t LocalTime = 0; /* this variable is used to create a time reference incremented by 10ms */
uint32_t timingdelay;

/* Private function prototypes -----------------------------------------------*/
void System_Periodic_Handle(void);
void InitTIM(void);
void TIM2_IRQHandler(void);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */

extern void ModbusSenttset(uint32_t Time);
extern void UDPSendData(uint32_t Time);
extern void SendDataToSever(uint32_t Time);  
extern void tcp_sever_test(void);
extern void FileSystemInit(void);
extern void FileSystemThread(void); 
extern void httpd_init(void);
char poll_net_data_lock=0;
extern void LCD_init(void);
int main(void)

{
  



	 /* Setup STM32 system (clocks, Ethernet, GPIO, NVIC) */
		  System_Setup();
		 	 
	      InitTIM();//systick 服务 由TIM代替 systick 专用于DS18B20的US延时服务


		  #ifdef USING_KEY
		  ConfigHWkey();
		  #endif

		  #ifdef USING_DISPLAY
		  LCD_init();
		  #endif

		  #ifdef USING_PID
		  InitSSR();	 
		  init_PID();
		  #endif

	      #ifdef USED_FILESYSTEM
		   FileSystemInit(); 
	       FileSystemThread();    
		  #endif 
		    
		  #ifdef USING_NET
		  LwIP_Init();
		  netbios_init();
		  tftpd_init();
		  tcp_sever_test();
	      #endif

		  #ifdef USED_FILESYSTEM && USED_HTTP
		  httpd_init();
		  #endif

		  #ifdef USED_SMTP
		  my_smtp_test("The Board is Power up...");
		  #endif

		


		 

		  while (1)
		  {   
		   	 

			  #ifdef USING_NET
			  TcpTestThread(LocalTime);
		      SendDataToSever(LocalTime);
		    //  UDPSendData(LocalTime);
		      LwIP_Periodic_Handle(LocalTime);
			  #endif

			  #ifdef USED_DS18B20
			  TemperatureThread(LocalTime);
			  #endif

			  #ifdef USING_PID
			  PidThread(LocalTime);
			  #endif

			  #ifdef USED_MODBUS
			  ModbusSenttset(LocalTime); 
			 (void)eMBMasterPoll();
			  #endif

		      #ifdef USING_KEY
		      KeyScane(LocalTime);
		      #endif

		      #ifdef USING_DISPLAY
		      Display_Task(LocalTime);
		      #endif


	
		  }
}

/**
  * @brief  Inserts a delay time.
  * @param  nCount: number of 10ms periods to wait for.
  * @retval None
  */
void Delay(uint32_t nCount)
{
  /* Capture the current local time */
  timingdelay = LocalTime + nCount;  

  /* wait until the desired delay finish */  
  while(timingdelay > LocalTime)
  {     
  }
}

/**
  * @brief  Updates the system local time
  * @param  None
  * @retval None
  */
void Time_Update(void)
{
 // LocalTime += SYSTEMTICK_PERIOD_MS
}

 /*
		Systick 用于DS18B20的精确延时上。
		TIM2用于本地的时间服务
		20151020

  */
void TIM2_IRQHandler(void)
{
	
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		
		TIM_ClearFlag(TIM2, TIM_FLAG_Update);	     //清中断标记
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);	 //清除定时器TIM2溢出中断标志位

		LocalTime += SYSTEMTICK_PERIOD_MS;
  	
	}
	
}	


void InitTIM(void)
{	
    uint16_t 	usPrescalerValue;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	//====================================时钟初始化===========================
	//使能定时器2时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	//====================================定时器初始化===========================
	//定时器时间基配置说明
	//HCLK为72MHz，APB1经过2分频为36MHz
	//TIM2的时钟倍频后为72MHz（硬件自动倍频,达到最大）
	//TIM2的分频系数为3599，时间基频率为72 / (1 + Prescaler) = 20KHz,基准为50us
	//TIM最大计数值为usTim1Timerout50u	
	usPrescalerValue = (uint16_t) (72000000 / 20000) - 1;
	//预装载使能
	TIM_ARRPreloadConfig(TIM2, ENABLE);
	//====================================中断初始化===========================
	//设置NVIC优先级分组为Group2：0-3抢占式优先级，0-3的响应式优先级
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//清除溢出中断标志位
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	//定时器2溢出中断关闭
	TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
	//定时器3禁能
	TIM_Cmd(TIM2, DISABLE);

	TIM_TimeBaseStructure.TIM_Prescaler = usPrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = (uint16_t)(10 * 1000 / 50);//10ms
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_SetCounter(TIM2, 0);
	TIM_Cmd(TIM2, ENABLE);	


}



#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif


 





/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
