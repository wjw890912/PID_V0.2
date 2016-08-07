
#include "stm32f10x_it.h"
#include "usart.h"
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "mbframe.h"



 uint32_t Tick;
 void Master_Send(char Slave_addr,char *Data,int len);
/* ----------------------- Start implementation -----------------------------*/

  #define T32_INTERVAL  (250*4)*1

	  static  uint32_t t32=0;

extern char TcpRecvBuf[1500];
extern uint32_t TcpRecvLenth;
 void ModbusSenttset(uint32_t Time)
 {
 		   

		 if(TcpRecvLenth>0)
		 {
				   {
				   uint32_t  len;
				   len=	TcpRecvLenth; //save
				 	 TcpRecvLenth=0; //restart data lenth 
				   	Master_Send(6,TcpRecvBuf,len);
					 }
		 }


 			   #if 0
 	 		   char  usedata[10] = "stm32f107";
		if (Time - t32 >= T32_INTERVAL)
		  {
		    t32 =  Time;
		
		    
			 	Master_Send(6,(char *)&usedata[0],9);
			//Master_Send(6,"rs485 master sent data ",sizeof("rs485 master sent data "));
		
		
		
		  }
 		   #endif
 
 }


int
modbus_main( void )
{
  char  usedata[10] = { 3, 0, 0, 0,0xa};
    eMBErrorCode    eStatus;
	// char *p;

	/* init system setting */
//	SystemInit();
	/*NVIC init*/
//	NVIC_Configuration();
	/* init sysTick*/
  // SysTick_Configuration();
	 /* init USART*/
	 Hw_Usart_init();
	
	 while(1)
	 {

    eStatus = eMBMasterInit( MB_RTU, 2, 9600,MB_PAR_NONE );

   // eStatus = eMBSetSlaveID( 0x34, TRUE, ucSlaveID, 3 );
 
    /* Enable the Modbus Protocol Stack. */
    eStatus = eMBMasterEnable();

	/*	vMBMasterSetDestAddress(6);
		 vMBMasterGetPDUSndBuf( &p );
		 *p=3;
		 p++;
		 *p=0;
		  p++;
		 *p=0x0;
		  p++;
		 *p=0;
		  p++;
		 *p=5;


		  */
		    for( ;; )
		    {
				 	if(Tick>=20)
			 	{
					Master_Send(6,(char *)&usedata[0],5);
			/*	vMBMasterGetPDUSndBuf( &p );
				 eMBMasterRTUSend( 6, p,5 );  */
				 Tick=0;
				}
		
		        (void)eMBMasterPoll();
		    
		    }
      }

 }


void Master_Send(char Slave_addr,char *Data,int len)
{ 

         char *p ;
		 int i;

		  if(len>MB_PDU_SIZE_MAX)return ;// if the data lenth up the PDU  lenth we need stop it .

		  //vMBMasterSetDestAddress(Slave_addr);// we dont need slave addrs
		  vMBMasterGetPDUSndBuf( &p );   //The 1st  we get the pdu buffer point
			 for(i=0;i<len;i++)
			 {
			 	 *p=*Data;
				 p++;				     //The 2st we fill in data to the buffer .
				 Data++;
			 }

		 	vMBMasterGetPDUSndBuf( &p ); // The 3st we reget the pdu snetBuffer point
		    eMBMasterRTUSend( Slave_addr, p, len );	// 	start up snet status machin and excution sent data on RTU 
}



