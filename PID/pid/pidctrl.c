

 
/**




**/
#include <math.h>
#include <string.h>
#include "stm32f10x.h"
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "pidctrl.h"
#include "ds18b20.h" 
#include "main.h"
#ifndef ABS
#define ABS(a)	   (((a) < 0) ? -(a) : (a))
#endif

#define WINDUP_ON(_pid)         (_pid->features & PID_ENABLE_WINDUP)
#define DEBUG_ON(_pid)          (_pid->features & PID_DEBUG)
#define SAT_MIN_ON(_pid)        (_pid->features & PID_OUTPUT_SAT_MIN)
#define SAT_MAX_ON(_pid)        (_pid->features & PID_OUTPUT_SAT_MAX)
#define HIST_ON(_pid)           (_pid->features & PID_INPUT_HIST)


#define PID_CREATTRM_INTERVAL  	(250*4)  /*250*2 ms ---  pid once that again and again*/

#ifdef USING_PID





/*****external SSR ctrl module*********/
 
 uint8_t Interrupt_Extern=0;
 uint16_t Adj_Power_Time=0;
 uint16_t Power_Adj=0;


 void initBta16TMER()
 {
 //使用了一个100US的定时器，
 //初始化
 uint16_t 	usPrescalerValue;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	//====================================时钟初始化===========================
	//使能定时器2时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	//====================================定时器初始化===========================
	//定时器时间基配置说明
	//HCLK为72MHz，APB1经过2分频为36MHz
	//TIM2的时钟倍频后为72MHz（硬件自动倍频,达到最大）
	//TIM2的分频系数为3599，时间基频率为72 / (1 + Prescaler) = 100KHz,基准为10us
	//TIM最大计数值为usTim1Timerout50u	
	usPrescalerValue = (uint16_t) (72000000 / 25000) - 1;
	//预装载使能
	TIM_ARRPreloadConfig(TIM4, ENABLE);
	//====================================中断初始化===========================
	//设置NVIC优先级分组为Group2：0-3抢占式优先级，0-2的响应式优先级
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//清除溢出中断标志位
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	//定时器2溢出中断关闭
	TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
	//定时器3禁能
	TIM_Cmd(TIM4, DISABLE);

	TIM_TimeBaseStructure.TIM_Prescaler = usPrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = (uint16_t)( 1);//100us
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	TIM_SetCounter(TIM4, 0);
	TIM_Cmd(TIM4, DISABLE);
 
 }
 void TIM4_IRQHandler(void)
{
	
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
	{
		
		TIM_ClearFlag(TIM4, TIM_FLAG_Update);	     //清中断标记
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);	 //清除定时器TIM3溢出中断标志位

	    Adj_Power_Time++;//increase time 
  	   AutoRunPowerAdjTask();
	}
	
}	
void EXTI4_IRQHandler(void)
{
     
	 	Interrupt_Extern=1;
		Adj_Power_Time=0;
    /* Clear the Key Button EXTI line pending bit */
    EXTI_ClearITPendingBit(EXTI_Line4);

   
}
void InitzeroGPIO()
 {
 
//DrvGPIO_SetIntCallback(pfP0P1Callback, pfP2P3P4Callback);
//DrvGPIO_EnableInt(E_PORT2, E_PIN7, E_IO_RISING, E_MODE_EDGE);
	   //配置引脚为外部中断下降沿有效

	 NVIC_InitTypeDef NVIC_InitStructure;
	 GPIO_InitTypeDef GPIO_InitStructure;
     EXTI_InitTypeDef EXTI_InitStructure;

	  /* Enable GPIOC and GPIOB clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
  
  /* Enable AFIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

 	/* Enable the EXTI3 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

	/* configure PB4 as SSR ctrl pin */

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	  GPIO_SetBits(GPIOE,GPIO_Pin_2);
	  GPIO_ResetBits(GPIOE,GPIO_Pin_2);
	  	/* configure PB3 as external interrupt */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

     GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource4);
	 
	    /* Configure  EXTI Line to generate an interrupt on falling edge */
    EXTI_InitStructure.EXTI_Line = EXTI_Line4;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

	/* Clear the Key Button EXTI line pending bit */
	EXTI_ClearITPendingBit(EXTI_Line4); 
	  
 }


/*
设置功率百分比

 Power 范围是0-100%
 0%：最小功率
 100%：最高功率
*/
void Set_Power(char Power)
{

	//	Power_Adj=0	最亮功率最高
	//	Power_Adj=100最暗

	 Power_Adj=100-Power;



}

void Trdelay()
{

	  uint32_t i;
	  for(i=500;i;i--);


}
 void Trigger_SSR_Task()
 {

	 
   #if SSR_ULN2004

	GPIO_ResetBits(GPIOE,GPIO_Pin_2);	

	Trdelay();

	GPIO_SetBits(GPIOE,GPIO_Pin_2);

	#else

	GPIO_ResetBits(GPIOE,GPIO_Pin_2);	

	Trdelay();

	GPIO_SetBits(GPIOE,GPIO_Pin_2);


	#endif
	 
				   
 }

 /*----------Power Task Auto  Run---------------------------------------------------------------------*/
 void AutoRunPowerAdjTask()
 {
		   if(Interrupt_Extern==1)
		 {	
				 if( Adj_Power_Time>=Power_Adj)
				   {
				   
				   	Trigger_SSR_Task();		 
		 		   				 
				   	 
					 Interrupt_Extern=0;

				   }
		 		  	
			
		 }	


 }


 void InitSSR(void)
 {
 	InitzeroGPIO();
 
   initBta16TMER();
 }

 /*start of PID*/

 extern float Temp_pid[1];
 uint32_t PidCreatTrm=0;

 	PID_t pid;
	float result,kj;
	



extern void DS1820main(void); 		
//PID

void init_PID(void)
{
	/*初始化PID结构*/
		/*
	  	 比例因子=1；
		 积分因子=0；
		 微分因子=0；

	   */	
   	 pid_init(&pid, 1, 0, 0);
	  /*使能或者禁能PID计算的功能*/	
   //pid_enable_feature(&pid, , 0)		
	/*目标值设定*/		 
     pid_set(&pid,40);

}

extern uint8_t Mb2TcpBuff[256];
extern uint32_t Mb2TcpLenth;
extern    char TcpRecvBuf[1500];
extern   uint32_t TcpRecvLenth;


float ProcessTcpSrting(char *buf,char*str)
{
	   char *location=0;
	   uint8_t val[50],i,j,k;
	   float finteger=0,fdecimal=0,fresult=0;

	  location=	strstr(buf,str);//查找制定字符,返回字符位置。
	
	   if(*(location+2)=='=') //定位准确？yes into   kp=12.54\r\n				 2,i=1;	10^0+10^1
	   {
	   	  for(i=0;i<50;i++)	//开始遍历
	   	   {
			    val[i] = *(location+2+i);//从"="开始向后找

				if(val[i]=='.')//以上为整数部分
				{
				   	    k=i-1;//记录小数有效的字符最后一个位置
						j=0;
						finteger=0;
					while(1)
					{
					
						finteger+=(float)((val[k]-0x30)*pow(10,j));//进行整数部分的运算
						  j++;//平方增加
						  k--;//移位数据
						 if(val[k]=='=')break;//如果找到开始的位置那么结束此次搜索
					}


				}

				if(val[i]==',')//搜索到结束字符
				{
					   	k=i-1;//记录小数有效的字符最后一个位置
						j=0;
					  fdecimal=0;
					  while(1)
					  {
						 
						  
					  fdecimal+=(float)((val[k]-0x30)*pow(10,j));//进行整数部分的运算
					  k--;//移位数据
					  j++;//平方增加
						  if(val[k]=='.')
						  {
						  fdecimal=fdecimal/pow(10,j);	//转换为小数
						  fresult=fdecimal+finteger;
					      return  fresult;
						  }
						 
					  
					  }

					//计算小数并最终得出计算值返回
				  
				} 

			}
			//如果执行到此处表面没有设置任何参数，返回错误即可
	   		 return -1;
	   }


}

 /*
                要求发送数据为：【pidset:kp=1.01,ki=3.1,kd=1.5,sp=20.01,】逗号必须加。 
				全部带小数点，不限制小数位数
 */
 extern float    NowTemp;
void PidThread(uint32_t Time)
{  
    float kp,ki,kd,sp; 
	float temppid;
	static char Timeonce=0;
     			 
if (Time - PidCreatTrm >= PID_CREATTRM_INTERVAL)
  {
		
        PidCreatTrm =  Time;
		if(TcpRecvLenth>0)
		{

		   kp =  ProcessTcpSrting(TcpRecvBuf,"kp");
		   ki =  ProcessTcpSrting(TcpRecvBuf,"ki");
		   kd =  ProcessTcpSrting(TcpRecvBuf,"kd");
		   sp =	 ProcessTcpSrting(TcpRecvBuf,"sp");

		 pid_init(&pid, kp, ki, kd);
		 pid_set(&pid,sp);
		 TcpRecvLenth=0;


			Mb2TcpBuff[0] ='p';
			Mb2TcpBuff[1] ='i';
			Mb2TcpBuff[2] ='d';
			Mb2TcpBuff[3] ='o';
			Mb2TcpBuff[4] ='k';
			Mb2TcpBuff[5] ='\r';
			Mb2TcpBuff[6] ='\n';

			Mb2TcpLenth   =	 7;
			return;
		}

	  

	   DS1820main();//采集当前温度  
	    
	temppid = Temp_pid[0]/100;
    NowTemp	= temppid ;

     result  =  pid_calculate(&pid,temppid, 1) ;


	Mb2TcpBuff[0] = '\t';
	Mb2TcpBuff[1] = (uint8_t)temppid/10 +0x30;//当前温度
	Mb2TcpBuff[2] = (uint8_t)temppid%10 +0x30;//当前温度
	Mb2TcpBuff[3] = '.';//小数点

	temppid=temppid-(((Mb2TcpBuff[1]-0x30)*10)+(Mb2TcpBuff[2]-0x30));
	temppid=temppid*100;//保留2位小数



	Mb2TcpBuff[4] = (uint8_t)temppid/10 +0x30;//当前温度
	Mb2TcpBuff[5] = (uint8_t)temppid%10 +0x30;//当前温度

	Mb2TcpBuff[6] = '\t';


		
			  
			
		   if((result>100))/*0-100区间*/
		   {
				 result=100;
		   	  Set_Power(result);
			  	Mb2TcpBuff[7] = (uint8_t) result/100+0x30 ;//当前输出百分比
	            Mb2TcpBuff[8] = (uint8_t) result%100/10+0x30 ;//当前输出百分比
	            Mb2TcpBuff[9] = (uint8_t) result%100%10+0x30 ;//当前输出百分比


				Mb2TcpBuff[10] = '\r';
				Mb2TcpBuff[11] = '\n';
				
				Mb2TcpLenth   =	 12;
		   }
		   else
		   if((result<0))/*0-100区间*/
		   {
				 result=0;
		   	  Set_Power(result);
			  	Mb2TcpBuff[7] = (uint8_t) result/100+0x30 ;//当前输出百分比
	            Mb2TcpBuff[8] = (uint8_t) result%100/10+0x30 ;//当前输出百分比
	            Mb2TcpBuff[9] = (uint8_t) result%100%10+0x30 ;//当前输出百分比


				Mb2TcpBuff[10] = '\r';
				Mb2TcpBuff[11] = '\n';
				
				Mb2TcpLenth   =	 12;
		   }
		   else
		   {
		   
		   	 Set_Power(result);
			 	Mb2TcpBuff[7] = (uint8_t) result/100+0x30 ;//当前输出百分比
	            Mb2TcpBuff[8] = (uint8_t) result%100/10+0x30 ;//当前输出百分比
	            Mb2TcpBuff[9] = (uint8_t) result%100%10+0x30 ;//当前输出百分比


				Mb2TcpBuff[10] = '\r';
				Mb2TcpBuff[11] = '\n';
				
				Mb2TcpLenth   =	 12;
		   }

		if(Timeonce==0)
		{
		TIM_Cmd(TIM4, ENABLE);//just run once
		Timeonce=1;
		}

  }
	
	
}


void pid_enable_feature(PID_t *pid, unsigned int feature, float value)
{
    pid->features |= feature;

    switch (feature) {
        case PID_ENABLE_WINDUP:
            /* integral windup is in absolute output units, so scale to input units */
            pid->intmax = ABS(value / pid->ki);
            break;
        case PID_DEBUG:
            break;
        case PID_OUTPUT_SAT_MIN:
            pid->sat_min = value;
            break;
        case PID_OUTPUT_SAT_MAX:
            pid->sat_max = value;
            break;
        case PID_INPUT_HIST:
            break;
    }
}

/**
 *
 * @param pid
 * @param kp
 * @param ki
 * @param kd
 */
void pid_init(PID_t *pid, float kp, float ki, float kd)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;

	pid->sp = 0;
	pid->error_previous = 0;
	pid->integral = 0;

    pid->features = 0;

    if (DEBUG_ON(pid))
        printf("setpoint,value,P,I,D,error,i_total,int_windup\n");
}

void pid_set(PID_t *pid, float sp)
{
	pid->sp = sp;
	pid->error_previous = 0;
	pid->integral = 0;
}

/**
 *
 * @param pid
 * @param val  	now pv value
 * @param dt   
 * @return
 *			
 *     PID  = [ Kp*e(t)] + [Ki*∫(0-t)e(τ)dτ] + [Kd*de(t)/dt]
 *
比例项：[ Kp*e(t)]		   误差同比例线性函数			 --正比于误差的差值
积分项：[Ki*∫(0-t)e(τ)dτ]，区间0-t上的误差曲线的积分	 --由于积分的求和作用所以代表了误差的过去
微分项：[Kd*de(t)/dt]，    区间0-t上的误差曲线的微分	 --由于微分的求导（斜率）作用所以代表了误差的未来的走向.

The PID control scheme is named after its three correcting terms,
whose sum constitutes the manipulated variable (MV). 
The proportional, integral, and derivative terms are summed to calculate the output of the PID controller. 
Defining u(t) as the controller output, the final form of the PID algorithm is:

   	u(t) =MV(t) = [ Kp*e(t)] + [Ki*∫(0-t)e(τ)dτ] + [Kd*de(t)/dt]

where

Kp: Proportional gain, a tuning parameter
Ki: Integral gain, a tuning parameter
Kd: Derivative gain, a tuning parameter
e(t): Error=SP-PV(t)
SP: Set Point
PV(t): Process Variable
t: Time or instantaneous time (the present)
τ: Variable of integration; takes on values from time 0 to the present t.
Equivalently, the transfer function in the Laplace Domain of the PID controller is

L(s)=Kp+Ki/s+Kds

where

{ s} s: complex number frequency



Proportional term

Plot of PV vs time, for three values of Kp (Ki and Kd held constant)
The proportional term produces an output value that is proportional to the current error value.
 The proportional response can be adjusted by multiplying the error by a constant Kp, called the proportional gain constant.

The proportional term is given by:

 Pout =Kp*e(t)

A high proportional gain results in a large change in the output for a given change in the error.
If the proportional gain is too high, 
the system can become unstable (see the section on loop tuning). 
In contrast, a small gain results in a small output response to a large input error,
and a less responsive or less sensitive controller. 
If the proportional gain is too low, the control action may be too small when responding to system disturbances. 
Tuning theory and industrial practice indicate that the proportional term should contribute the bulk of the output change.


Steady-state error
Because a non-zero error is required to drive it, 
a proportional controller generally operates with a so-called steady-state error.[a] 
Steady-state error (SSE) is proportional to the process gain and inversely proportional to proportional gain. 
SSE may be mitigated by adding a compensating bias term to the setpoint or output, 
or corrected dynamically by adding an integral term.



Integral term[edit]

Plot of PV vs time, for three values of Ki (Kp and Kd held constant)
The contribution from the integral term is proportional to both the magnitude of the error and the duration of the error. 
The integral in a PID controller is the sum of the instantaneous error over time and gives the accumulated offset that should have been corrected previously. 
The accumulated error is then multiplied by the integral gain ( Ki) and added to the controller output.

The integral term is given by:

Iout =[Ki*∫(0-t)e(τ)dτ]
The integral term accelerates the movement of the process towards setpoint and eliminates the residual steady-state error that occurs with a pure proportional controller. 
However, since the integral term responds to accumulated errors from the past, it can cause the present value to overshoot the setpoint value (see the section on loop tuning).

Derivative term[edit]

Plot of PV vs time, for three values of Kd (Kp and Ki held constant)
The derivative of the process error is calculated by determining the slope of the error over time and multiplying this rate of change by the derivative gain Kd. 
The magnitude of the contribution of the derivative term to the overall control action is termed the derivative gain, Kd.

The derivative term is given by:

Dout = [Kd*de(t)/dt]

Derivative action predicts system behavior and thus improves settling time and stability of the system.[12][13]
 An ideal derivative is not causal, so that implementations of PID controllers include an additional low pass filtering for the derivative term,
  to limit the high frequency gain and noise.[14] Derivative action is seldom used in practice though - by one estimate in only 25% of deployed controllers[14] 
  - because of its variable impact on system stability in real-world applications.[14]

 */
float pid_calculate(PID_t *pid, float pv, float dt)
{
	float i,d, error, total;

	error = pid->sp - pv;
	i = pid->integral + (error * dt);
	d = (error - pid->error_previous) / dt;

    total = (error * pid->kp) + (i * pid->ki) + (d * pid->kd);

    if (DEBUG_ON(pid))
        printf("%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d\n", 
                pid->sp,pv,
                (error * pid->kp), (i * pid->ki), (d * pid->kd),
                error, pid->integral, ABS(i) == pid->intmax);

    if ( WINDUP_ON(pid) ) {
        if ( i < 0 )
            i = ( i < -pid->intmax ? -pid->intmax : i );
        else
   		    i = ( i < pid->intmax ? i : pid->intmax );
    }
    pid->integral = i;

    if ( SAT_MIN_ON(pid) && (total < pid->sat_min) )
        return pid->sat_min;
    if ( SAT_MAX_ON(pid) && (total > pid->sat_max) )
        return pid->sat_max;

	pid->error_previous = error;
	return total;
}


 /*end of PID*/



 #endif
 
 /*
1、限幅滤波法（又称程序判断滤波法）
 A、方法：
 根据经验判断，确定两次采样允许的最大偏差值（设为A）
 每次检测到新值时判断：
 如果本次值与上次值之差<=A,则本次值有效
 如果本次值与上次值之差>A,则本次值无效,放弃本次值,用上次值代替本次值
 B、优点：
 能有效克服因偶然因素引起的脉冲干扰
 C、缺点
 无法抑制那种周期性的干扰
 平滑度差
 
 
#define A 10
char value;
char filter()
{
   char  new_value;
   new_value = get_ad();
   if ( ( new_value - value > A ) || ( value - new_value > A )
      return value;
   return new_value;
}
 
2、中位值滤波法
 A、方法：
 连续采样N次（N取奇数）
 把N次采样值按大小排列
 取中间值为本次有效值
 B、优点：
 能有效克服因偶然因素引起的波动干扰
 对温度、液位的变化缓慢的被测参数有良好的滤波效果
 C、缺点：
 对流量、速度等快速变化的参数不宜


#define N  11
char filter()
{
   char value_buf[N];
   char count,i,j,temp;
   for ( count=0;count<N;count++)
   {
      value_buf[count] = get_ad();
      delay();
   }
   for (j=0;j<N-1;j++)
   {
      for (i=0;i<N-j;i++)
      {
         if ( value_buf[i]>value_buf[i+1] )
         {
            temp = value_buf[i];
            value_buf[i] = value_buf[i+1]; 
             value_buf[i+1] = temp;
         }
      }
   }
   return value_buf[(N-1)/2];
}     

3、算术平均滤波法
 A、方法：
 连续取N个采样值进行算术平均运算
 N值较大时：信号平滑度较高，但灵敏度较低
 N值较小时：信号平滑度较低，但灵敏度较高
 N值的选取：一般流量，N=12；压力：N=4
 B、优点：
 适用于对一般具有随机干扰的信号进行滤波
 这样信号的特点是有一个平均值，信号在某一数值范围附近上下波动
 C、缺点：
 对于测量速度较慢或要求数据计算速度较快的实时控制不适用
 比较浪费RAM

#define N 12
char filter()
{
   int  sum = 0;
   for ( count=0;count<N;count++)
   {
      sum + = get_ad();
      delay();
   }
   return (char)(sum/N);
}

4、递推平均滤波法（又称滑动平均滤波法）
 A、方法：
 把连续取N个采样值看成一个队列
 队列的长度固定为N
 每次采样到一个新数据放入队尾,并扔掉原来队首的一次数据.(先进先出原则)
 把队列中的N个数据进行算术平均运算,就可获得新的滤波结果
 N值的选取：流量，N=12；压力：N=4；液面，N=4~12；温度，N=1~4
 B、优点：
 对周期性干扰有良好的抑制作用，平滑度高
 适用于高频振荡的系统
 C、缺点：
 灵敏度低
 对偶然出现的脉冲性干扰的抑制作用较差
 不易消除由于脉冲干扰所引起的采样值偏差
 不适用于脉冲干扰比较严重的场合
 比较浪费RAM
 
#define N 12 
char value_buf[N];
char i=0;
char filter()
{
   char count;
   int  sum=0;
   value_buf[i++] = get_ad();
   if ( i == N )   i = 0;
   for ( count=0;count<N,count++)
      sum = value_buf[count];
   return (char)(sum/N);
}

5、中位值平均滤波法（又称防脉冲干扰平均滤波法）
 A、方法：
 相当于“中位值滤波法”+“算术平均滤波法”
 连续采样N个数据，去掉一个最大值和一个最小值
 然后计算N-2个数据的算术平均值
 N值的选取：3~14
 B、优点：
 融合了两种滤波法的优点
 对于偶然出现的脉冲性干扰，可消除由于脉冲干扰所引起的采样值偏差
 C、缺点：
 测量速度较慢，和算术平均滤波法一样
 比较浪费RAM
 
#define N 12
char filter()
{
   char count,i,j;
   char value_buf[N];
   int  sum=0;
   for  (count=0;count<N;count++)
   {
      value_buf[count] = get_ad();
      delay();
   }
   for (j=0;j<N-1;j++)
   {
      for (i=0;i<N-j;i++)
      {
         if ( value_buf[i]>value_buf[i+1] )
         {
            temp = value_buf[i];
            value_buf[i] = value_buf[i+1]; 
             value_buf[i+1] = temp;
         }
      }
   }
   for(count=1;count<N-1;count++)
      sum += value[count];
   return (char)(sum/(N-2));
}

6、限幅平均滤波法
 A、方法：
 相当于“限幅滤波法”+“递推平均滤波法”
 每次采样到的新数据先进行限幅处理，
 再送入队列进行递推平均滤波处理
 B、优点：
 融合了两种滤波法的优点
 对于偶然出现的脉冲性干扰，可消除由于脉冲干扰所引起的采样值偏差
 C、缺点：
 比较浪费RAM
  
略 参考子程序1、3
7、一阶滞后滤波法
 A、方法：
 取a=0~1
 本次滤波结果=（1-a）*本次采样值+a*上次滤波结果
 B、优点：
 对周期性干扰具有良好的抑制作用
 适用于波动频率较高的场合
 C、缺点：
 相位滞后，灵敏度低
 滞后程度取决于a值大小
 不能消除滤波频率高于采样频率的1/2的干扰信号
 
#define a 50
char value;
char filter()
{
   char  new_value;
   new_value = get_ad();
   return (100-a)*value + a*new_value; 
}

8、加权递推平均滤波法
 A、方法：
 是对递推平均滤波法的改进，即不同时刻的数据加以不同的权
 通常是，越接近现时刻的数据，权取得越大。
 给予新采样值的权系数越大，则灵敏度越高，但信号平滑度越低
 B、优点：
 适用于有较大纯滞后时间常数的对象
 和采样周期较短的系统
 C、缺点：
 对于纯滞后时间常数较小，采样周期较长，变化缓慢的信号
 不能迅速反应系统当前所受干扰的严重程度，滤波效果差
 
#define N 12
char code coe[N] = {1,2,3,4,5,6,7,8,9,10,11,12};
char code sum_coe = 1+2+3+4+5+6+7+8+9+10+11+12;
char filter()
{
   char count;
   char value_buf[N];
   int  sum=0;
   for (count=0,count<N;count++)
   {
      value_buf[count] = get_ad();
      delay();
   }
   for (count=0,count<N;count++)
      sum += value_buf[count]*coe[count];
   return (char)(sum/sum_coe);
}

9、消抖滤波法
 A、方法：
 设置一个滤波计数器
 将每次采样值与当前有效值比较：
 如果采样值＝当前有效值，则计数器清零
 如果采样值<>当前有效值，则计数器+1，并判断计数器是否>=上限N(溢出)
 如果计数器溢出,则将本次值替换当前有效值,并清计数器
 B、优点：
 对于变化缓慢的被测参数有较好的滤波效果,
 可避免在临界值附近控制器的反复开/关跳动或显示器上数值抖动
 C、缺点：
 对于快速变化的参数不宜
 如果在计数器溢出的那一次采样到的值恰好是干扰值,则会将干扰值当作有效值导入系统
#define N 12
char filter()
{
   char count=0;
   char new_value;
   new_value = get_ad();
   while (value !=new_value);
   {
      count++;
      if (count>=N)   return new_value;
       delay();
      new_value = get_ad();
   }
   return value;    
}

10、限幅消抖滤波法
 A、方法：
 相当于“限幅滤波法”+“消抖滤波法”
 先限幅,后消抖
 B、优点：
 继承了“限幅”和“消抖”的优点
 改进了“消抖滤波法”中的某些缺陷,避免将干扰值导入系统
 C、缺点：
  
略 参考子程序1、9 */

/*
float SetPoint;       //设定温度
float Proportion;     //比例系数
float  Integral;      //积分系数 
float Derivative;     //微分系数 
float LastError;      // Error[-1] 
float PrevError;      // Error[-2] 
float SumError;       // Sums of Errors 

 // PID计算程序（通用PID）

float PIDCalc( float NextPoint ) 
{ 
	
  float dError=0, Error=0;  
 Error = SetPoint - NextPoint;         
 SumError += Error;                      
 dError = Error - LastError;            
 PrevError = LastError; 
 LastError = Error; 
 return ((Proportion * Error) + (Integral * SumError) + (Derivative * dError) ); 
 
} 

//初始化PID计算程
void Init_Pid(void)
{



		 SetPoint =90;       //设定温度90摄氏度
		 Proportion =1;     //比例系数     可以用温升决定这个系数的大小 增大减慢调整，减小增快调整
		 Integral =0;      //积分系数 
		 Derivative =0;     //微分系数 
		 LastError =0;      // Error[-1] 
		 PrevError =0;      // Error[-2] 
		 SumError =0;       // Sums of Errors 




}
  */
 
/*
20160713
float ProcessTcpSrting(char *buf,char*str)
{
	   char *location;
	   uint8_t val[50],i,j,k;
	   float finteger=0,fdecimal=0,fresult=0;

	  location=	strstr(buf,str);
	
	   if(*(location+2)=='=') 
	   {
	   	  for(i=0;i<50;i++)
	   	   {
			    val[i] = *(location+2+i);

				if(val[i]=='.')
				{
				   	    k=i-1;
						j=0;
						finteger=0;
					while(1)
					{
					
						finteger+=(float)((val[k]-0x30)*pow(10,j));
						  j++;
						  k--;
						 if(val[k]=='=')break;
					}
				}

				if(val[i]==0x20)
				{
					   	k=i-1;
						j=0;
					  fdecimal=0;
					  while(1)
					  {	  
					  fdecimal+=(float)((val[k]-0x30)*pow(10,j));
					  k--;
					  j++;
						  if(val[k]=='.')
						  {
						  fdecimal=fdecimal/pow(10,j);	
						  fresult=fdecimal+finteger;
					      return  fresult;
						  }
					  
					  } 
				} 
			}	
	   		 return -1;
	   }







}
*/