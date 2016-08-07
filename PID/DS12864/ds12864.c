#include<math.h>  
#include <stdio.h>
#include "stm32f10x.h"
#include "ds12864.h"
#include "main.h"
#include "key.h"

#define DISPLAY_CREATTRM_INTERVAL  	(250*2)  /*250*2 ms --- display updata time that again and again*/

#ifdef USING_DISPLAY



	/*	lcd interface
		  PD5--CLK
		  PD6--DTA
		  STM32F107板子上
   */
  

#define SID_UP		 GPIO_SetBits(GPIOD, GPIO_Pin_6)
#define SID_DOWN	 GPIO_ResetBits(GPIOD, GPIO_Pin_6)
#define SCK_UP		 GPIO_SetBits(GPIOD, GPIO_Pin_5)
#define SCK_DOWN	 GPIO_ResetBits(GPIOD, GPIO_Pin_5)


#define  clear   0x01  //清屏
#define  reset_DDRAM  0x02 //DDRAM地址归位
#define  left_move  0x04 //游标左移
#define  right_move  0x06 //游标右移
#define  all_left_move 0x05 //画面整体左移
#define  all_right_move 0x07  //画面整体右移
#define  display_left_move  0x10  //显示游标左移
#define  display_right_move  0x14  //显示游标右移
#define  set_function1   0x30  //基本指令集动作
#define  set_CGRAM    0x40  //设定CGRAM地址
#define  set_DDRAM    0x80  //设定DDRAM地址
#define  set_function2   0x34  //扩充指令集动作
#define  fanbai            0x04   //反白第一行(扩充指令集)
#define  set_GDRAM    0x80  //设定GDRAM地址(扩充指令集)
#define  ON_G     0x36  //开绘图显示(扩充指令集)
#define  set_function2   0x34  //关绘图显示(扩充指令集)

/**************************************/
#define  uchar    unsigned char
#define  uint     unsigned int 
extern uint32_t LocalTime ;
 uchar  a[]={"QC12864"};      //定义要显示的字符串

 



 void InitHW12864(void)
 {
   
	  //配置IO口为输出模式。推动液晶屏的CLK 和 DATA  俩引脚

  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD , ENABLE);
  /* Configure PD6 pins */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

 }
 





/**************************************/
//延时函数
/**************************************/
void Delay_nms(uchar n)
{
 	uint32_t trm=1000;//LocalTime;//get now systick times
		trm=n*trm;
	 do
	 {
	  //while(LocalTime -trm < 10);	//延时
	 }while(trm--); 

}
/**************************************/
//串行发送一个字节
/**************************************/
void LCD_sendbyte(uchar byte)
{
   uchar i;
   for(i=0;i<8;i++)
    {
   SCK_DOWN;               //拉低时钟线

  // DrvSYS_Delay(1);//10US
   if(byte&0x80)
   {
   	SID_UP;
   
   }
   else
   {
   	SID_DOWN;

   }
   SCK_UP;               //上升沿发送数据
   byte=byte<<1;                  //左移一位
 }
}

/****************************************/
//写指令
/****************************************/
void LCD_write_com(uchar com)
{
        
   LCD_sendbyte(0xf8);       //送入5个连续的“1“，启动一个周期,11111,RW(0),RS(0),0
   LCD_sendbyte(0xf0&com);       //取高四位，数据分两次传送，
                                 //每个字节的内容被送入两个字节
               //高四位放在第一个字节的高四位
   LCD_sendbyte(0xf0&(com<<4));        //低四位放在第二个字节的高四位 
           
  Delay_nms(1);             //串行不支持读操作，不可检测忙操作，这里用延时替代   
}

void LCD_write_com_clr(uchar com)
{
        
   LCD_sendbyte(0xf8);       //送入5个连续的“1“，启动一个周期,11111,RW(0),RS(0),0
   LCD_sendbyte(0xf0&com);       //取高四位，数据分两次传送，
                                 //每个字节的内容被送入两个字节
               //高四位放在第一个字节的高四位
   LCD_sendbyte(0xf0&(com<<4));        //低四位放在第二个字节的高四位 
           

}

/******************************************/
//写数据
/******************************************/
void LCD_write_dat(uchar dat)
{
            
   LCD_sendbyte(0xfa);        //送入5个连续的“1“，启动一个周期,11111,RW(0),RS(1),0
   LCD_sendbyte(0xf0&dat);       //取高四位，数据分两次传送，
                                 //每个字节的内容被送入两个字节
               //高四位放在第一个字节的高四位
   LCD_sendbyte(0xf0&(dat<<4));        //低四位放在第二个字节
           
   Delay_nms(10);
}
/********************************************/
//LCD初始化
/********************************************/
void LCD_init(void)
{  
  InitHW12864();     
 LCD_write_com(0x30);             //选择基本指令集   
 LCD_write_com(0x0c);       //开显示，无游标，不反白
 LCD_write_com(0x01);       //清除显示屏幕，把DDRAM位址计数器调整为00H
 Delay_nms(5);         //清屏操作时间较长1.6ms 因此加此延时
 LCD_write_com(0x02);       //清DDRAM位址归位,此处貌似与清屏重复
 LCD_write_com(0x06);       //设定光标右移，整体显示不移动
}
/*************************************************/
//显示字符串
/*************************************************/
void print(uchar *s)
{ 
    while(*s!='\0')
    {  
       LCD_write_dat(*s);
       s++;     
    }
}
/***************************************************/
//设置显示地址
/***************************************************/
void LCD_Setaddress(uchar x,uchar y)
{              //地址从第1行第1列开始不从0开始
  uchar addr;
  switch(x)
  {
   case 1: addr=0x80+y-1;
           break;
   case 2: addr=0x90+y-1;
           break;
   case 3: addr=0x88+y-1;
           break;
   case 4: addr=0x98+y-1;
           break;
   default : break;
  }
   LCD_write_com(addr);    //字符显示开始地址
}
/*****************************************************/
//让字符串显示在固定位置
/*****************************************************/
void LCD_Putstring( uchar x, uchar y, uchar *pData )
{
 LCD_Setaddress(x,y);
 while( *pData != '\0' )
 {
   LCD_write_dat( *pData++ );
 }
}

/*---------------------------------------------------------------------------------------------------------------------- */
//打点绘图，适用于在屏幕上打稀疏的几个点，不能用于横行连续打点
void LCD_draw_point(uchar x, uchar y) 
 {
   uchar x_byte, x_bit;         //在横坐标的哪一个字节，哪一个位
   uchar y_byte, y_bit;    //在纵坐标的哪一个字节，哪一个位
   x_byte=x/16;                   //算出它在哪一个字节(地址)
                                      //注意一个地址是16位的
   x_bit=x%16;                    //(取模)算出它在哪一个位
   y_byte=y/32;                    //y是没在哪个字节这个说法
                             //这里只是确定它在上半屏(32行为一屏)还是下半屏
                                     //0:上半屏 1:下半屏
   y_bit=y%32;                    //y_bit确定它是在第几行
   LCD_write_com(0x34);         //打开扩展指令集
  
 LCD_write_com(0x80+y_bit);       //垂直地址(上)   貌似与说明书正好相反
   LCD_write_com(0x80+x_byte+8*y_byte);  //先写水平坐标（下）   貌似与说明书正好相反    ???????
 
                                      //具体参照数据手册 
                                               //下半屏的水平坐标起始地址为0x88
                                     //(+8*y_byte)就是用来确定在上半屏还是下半屏
   if(x_bit<8)                              //如果x_bit位数小于8
   {
       LCD_write_dat(0x01<<(7-x_bit));    //写高字节。因为坐标是从左向右的
                                                //而GDRAM高位在左，低位在右
    LCD_write_dat(0x00);                   //低字节全部填0
   }
   else
   {
       LCD_write_dat(0x00);                   //高字节全部填0
       LCD_write_dat(0x01<<(15-x_bit));
   } 
   LCD_write_com(0x36);                     //打开绘图显示
   LCD_write_com(0x30);               //回到基本指令集
 }

/************************************/
 //打点绘图  一次打水平一行     可以避免断点现象
 //x表示数组的首地址，y表示纵坐标的值，也即是表示第多少行
//即对一个数组中的数进行这样的处理：
//检测数组，并默认数组为一行数的记录即128字节，只要数组中有数等于y，就把第y行的数全部打出
/************************************/
void LCD_draw_word(uchar *x, uchar y) 
 {
  uchar i,j,k,m,n,count=0;
   uchar hdat, ldat;         
   uchar y_byte, y_bit;    //在纵坐标的哪一个字节，哪一个位
    uchar   a[16];
   LCD_write_com(0x34);         //打开扩展指令集
    y_byte=y/32;                    //y是没在哪个字节这个说法
    y_bit=y%32;                    //y_bit确定它是在第几行
   for(j=0;j<8;j++)
 {
  hdat=0, ldat=0;     //此处清零是很必要的
  n=j*16;
  for(k=n;k<n+16;k++)
   {
   if(x[k]==y)
      {
    a[count]=k;
    count++;
    }
    } 
 for(m=0;m<count;m++)
   {
    i=a[m]-n;
       if(i<8)                              //如果x_bit位数小于8
          hdat=hdat|(0x01<<(7-i));    //写高字节。因为坐标是从左向右的
       else
        ldat=ldat|(0x01<<(15-i));
      }
   
   LCD_write_com(0x80+y_bit);       //垂直地址(上)   貌似与说明书正好相反
     LCD_write_com(0x80+j+8*y_byte);  //水平坐标（下）   貌似与说明书正好相反   
   LCD_write_dat(hdat);
    LCD_write_dat(ldat);
  }
   LCD_write_com(0x36);                     //打开绘图显示
   LCD_write_com(0x30);               //回到基本指令集
 }
/**********************************************************/
//清图形程序
/**********************************************************/
void LCD_draw_clr(void)
{ 
uchar i,j;
     LCD_write_com(0x34);      //8Bit扩充指令集,即使是36H也要写两次
    LCD_write_com(0x36);      //绘图ON,基本指令集里面36H不能开绘图 
     for(i=0;i<32;i++)            //12864实际为256x32
     {
           LCD_write_com(0x80+i);      //行位置    貌似与说明书正好相反 （上）
           LCD_write_com(0x80);      //列位置     貌似与说明书正好相反    （下）
           for(j=0;j<32;j++)            //256/8=32 byte
     LCD_write_dat(0);
  }
  LCD_write_com(0x30);     //开基本指令集
}

/*----------------------------------------------------------------------------------------------------------------------*/
/******************************************************/
//画正弦波的波形
/******************************************************/
void print_sinx(void)
{
 uchar i;
 uchar y_sin[128];   //定义屏幕上要打的正弦波的纵坐标
   uchar  v[128]={32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,
32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,
32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,
32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,};
 float y;
 for(i=0;i<128;i++)
 {
  y=31*sin(0.09*i)+0.5;   //此处系数为31比较好
   y_sin[i]=32-y;
 }
  for(i=0;i<64;i++)     
 LCD_draw_word(y_sin, i);   //绘图  一行一行绘
  LCD_draw_word(v, 32);
  
}

/******************************************************/
//主函数
//用于观看显示效果
/******************************************************/
void lcd_main(void)
{ 

 LCD_init();  
 LCD_Setaddress(2,3);
 print("您在使用");   
 LCD_Putstring(3,3,a);
// LCD_write_dat(0x35);  
// LCD_draw_clr();
 print_sinx(); 
}


/*

 	当前温度：xxx.xx℃	 （实时显示当前传感器采集的温度）
	设定温度：xxx℃		（显示当前的恒温温度）
	恒温时间：xxx分钟 （实时显示当前所剩余的时间）
	加热状态：（停止）、（运行）

	恒温温度设置
	设置温度：XXX℃	  [居中显示]	

	恒温时间设置
	恒温时间：xxx分钟 
	



*/

static char NowTemp_ds12864[22]  = "当前温度：xxx.xx";
static char SetTemp_ds12864[22]  = "设定温度：xxx";
static char SetTime_ds12864[22]  = "恒温时间：xxx";
static char RunStat_ds12864[22]  = "运行状态：xxx";

char *line1,*line2,*line3,*line4;

#define Start 9
#define End 10
#define Start_len 2
#define End_len 1


extern uint8_t MenuMachine;
extern uint32_t SetTemp; 
extern uint32_t SetTime;
extern uint32_t RunStatus;
extern float    NowTemp;



void Set_NowTemp2Display(int NowTemp)
{
	  static char d=0,b=0;
  d=sprintf(&NowTemp_ds12864[Start],"%d",NowTemp);  	
   if(d!=b)
   {	 b=d;
   		LCD_write_com(1); 
   }

}
void SetTemp2Display(int SetTemp)
{	 static char d=0,b=0;
   d=sprintf(&SetTemp_ds12864[Start],"%d",SetTemp);
   if(d!=b)
   {	 b=d;
   		LCD_write_com(1); 
   }
}
void SetTime2Display(int SetTime)
{	 static char d=0,b=0;

   d=sprintf(&SetTime_ds12864[Start],"%d",SetTime);
   if(d!=b)
   {	 b=d;
   		LCD_write_com(1); 
   }
}

void RunStat2Display(int RunStatus)
{	 static char d=0,b=0;


	  if(RunStatus)
	  {
	   memcpy(&RunStat_ds12864[Start],"运行",6);
	  }
	  else
	  {
	   memcpy(&RunStat_ds12864[Start],"停止",6);
	  }
   		LCD_write_com(1); 

}

void MainDisplayInterface(uint32_t NowTemp,uint32_t SetTemp,uint32_t SetTime,uint32_t RunStatus)
{

  Set_NowTemp2Display(NowTemp); //re-time temperature
  SetTemp2Display(SetTemp);// setting temperature
  SetTime2Display(SetTime); //setting run time 
  RunStat2Display(RunStatus);  //system run status 
  line1 =  NowTemp_ds12864;
  line2 =  SetTemp_ds12864;
  line3	=  SetTime_ds12864;
  line4	=  RunStat_ds12864;
}

void SetTempDisplayInterface(uint32_t temp)
{

  SetTemp2Display(temp);// setting temperature
  line1 =  "温度设置";
  line2 =  SetTemp_ds12864;
  line3	= " ";
  line4	= " ";


}
void SetTimeDisplayInterface(uint32_t time)
{

  SetTime2Display(time); //setting run time 
  line1 =  "时间设置";
  line2 =  SetTime_ds12864;
  line3	= " ";
  line4	= " ";


}

/*
The task must keep some "ms" updata once again and again ..

   			100-500MS
*/
uint32_t DisplayCreatTrm;
void Display_Task(uint32_t Time)
{
  if (Time - DisplayCreatTrm >= DISPLAY_CREATTRM_INTERVAL)
  {
   	   DisplayCreatTrm  =  Time;

	   switch(MenuMachine)
	   {
		 case  MAIN_MENU:
		 {
		   MainDisplayInterface(NowTemp,SetTemp,SetTime,RunStatus);
		   break;
		 }
		 case SET_TEMP_MENU:
		 {
		   SetTempDisplayInterface(SetTemp);
		   break;
		 }
		 case SET_TIME_MENU:
		 {
           SetTimeDisplayInterface(SetTime);
		   break;
		 }
       }

		   //public display code 
	       LCD_Setaddress(1,2);
		   print(line1); 
		   LCD_Setaddress(2,2);
		   print(line2); 
		   LCD_Setaddress(3,2);
		   print(line3);
		   LCD_Setaddress(4,2);
		   print(line4);

  }

}



#endif