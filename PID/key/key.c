

#include <stdio.h>
#include "stm32f10x.h"
#include "ds12864.h"
#include "main.h"
#include "key.h"

#define KEY_CREATTRM_INTERVAL  	(250*1)  /*250*1 ms --- KEY scane time that again and again*/

#ifdef USING_KEY


/*
		  key button interface


		  PD1  -- SET_TMEMP	 button
		  PD0  -- SET_TIME	 button
		  PC12 -- UP		 button
		  PC11 -- DOWN		 button
		  PC10 -- OK		 button
*/


#define  GET_BUTTON1  	   (GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_1)  & 0x01)
#define  GET_BUTTON2  	   (GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_0)  & 0x01)
#define  GET_BUTTON3  	   (GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_12) & 0x01)
#define  GET_BUTTON4  	   (GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_11) & 0x01)
#define  GET_BUTTON5  	   (GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_10) & 0x01)
#define  GET_BUTTON6  	   (1/*Get GPIO pin status*/ & 0x01)
#define  GET_BUTTON7  	   (1/*Get GPIO pin status*/ & 0x01)
#define  GET_BUTTON8  	   (1/*Get GPIO pin status*/ & 0x01)


#define  KEY1 		        0x01 
#define  KEY2 		        0x02
#define  KEY3 		        0x04 
#define  KEY4 		        0x08

#define  KEY5 		        0x10
#define  KEY6 		        0x20
#define  KEY7 		        0x40
#define  KEY8 		        0x80    



uint8_t MenuMachine = MAIN_MENU;
uint32_t SetTemp=TYPE_TEMP_VALUE; //fixed value 40℃	that be initialize 
uint32_t SetTime=TYPE_TIME_VALUE; //fixed value 20 minute that be initialize 
uint32_t RunStatus=STOP;
float    NowTemp;
/*system timer on task */
uint32_t KeyCreatTrm;//task execute time 




 void ConfigHWkey(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD , ENABLE);
  /* Configure button pins */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1 ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC , ENABLE);
  /* Configure button pins */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12 ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOD, &GPIO_InitStructure);




} 

/* SET_TMEMP  进入恒温值设定界面
	SET_TIME   进入定时时间设置界面
	UP		   增加数值
	DOWN 	   减少数值
	OK		   确认（子界面）/运行/停止（主界面）
*/


void KeyConfigSetTemp()
{
	if(RunStatus==STOP)// not allow into START status var configration parameter .
	{
   MenuMachine = SET_TEMP_MENU;
	}
}
void KeyConfigSetTime()
{
	if(RunStatus==STOP)
	{
   MenuMachine = SET_TIME_MENU;
    }
}
void KeyConfigOk()
{
   static char fkey=0;

   if(MenuMachine!=MAIN_MENU)
   {
   MenuMachine = MAIN_MENU;
   }
   else
   {
	     if(fkey)
	     {
		  fkey=0;
	   	  RunStatus=STOP;

		 }
		 else
		 {
	 	   fkey=1;
		   RunStatus=START;
	     }
   //do  nothing 
   }
}

void KeyConfigUp()
{
	switch(MenuMachine)
	{
		 case SET_TEMP_MENU:
		 {
		 
		   if(SetTemp<MAX_TEMP_VALUE)
		   {
		   SetTemp++;
		   }
	
		 break;
		 }
		 case SET_TIME_MENU:
		 {
		 
		   if(SetTime<MAX_TIME_VALUE)
		   {
		   SetTemp++;
		   }
	
		  break;
		 }
	}


}

void KeyConfigDown()
{
	  switch(MenuMachine)
	{
		 case SET_TEMP_MENU:
		 {
		 

		   if(SetTemp>MIN_TEMP_VALUE)
		   {
		   SetTemp--;
		   }
	
		 break;
		 }
		 case SET_TIME_MENU:
		 {
		 
		   if(SetTime>MIN_TIME_VALUE)
		   {
		   SetTemp--;
		   }
	
		  break;
		 }
	}


}










unsigned char Trg;//触发
unsigned char Cont;//连续
static char flage=0;

void KeyRead( void )
{
    unsigned char ReadData = 255;

	 ReadData =255;

	if(!GET_BUTTON1){ReadData=(ReadData&0xfe);}
	if(!GET_BUTTON2){ReadData=(ReadData&0xfd);}
	if(!GET_BUTTON3){ReadData=(ReadData&0xfb);}
	if(!GET_BUTTON4){ReadData=(ReadData&0xf7);}
	if(!GET_BUTTON5){ReadData=(ReadData&0xef);}
	if(!GET_BUTTON6){ReadData=(ReadData&0xdf);}
	if(!GET_BUTTON7){ReadData=(ReadData&0xbf);}
	if(!GET_BUTTON8){ReadData=(ReadData&0x7f);}


	ReadData =ReadData ^0xff;
    Trg = ReadData & (ReadData ^ Cont); 
    Cont = ReadData; 

}

void KeyProc(void)
{
  
	if(Cont|Trg )
	{	
	}
	/*SET_TEMP进入恒温值设定界面*/
	if (Trg  & KEY1){ KeyConfigSetTemp();   } 
	if (Cont & KEY1){                       }
	/*SET_TIME 进入定时时间设置界面*/
    if (Trg  & KEY2){ KeyConfigSetTime();   }
	if (Cont & KEY2){                       }
	/*UP	   增加数值	 */
    if (Trg  & KEY3){ KeyConfigUp();        }
    if (Cont & KEY3){                       }
	/*DOWN 	   减少数值*/
	if (Trg  & KEY4){ KeyConfigDown();      }
    if (Cont & KEY4){                       }
    /*OK		   确认（子界面）/运行/停止（主界面）  */
	if (Trg  & KEY5){ KeyConfigOk();        }
    if (Cont & KEY5){                       }

 }

void KeyScane(uint32_t Time)
{
			 
	if (Time - KeyCreatTrm >= KEY_CREATTRM_INTERVAL)
   {
   	   KeyCreatTrm  =  Time;
		 
	KeyRead();
	KeyProc();

   }
}	


#endif








