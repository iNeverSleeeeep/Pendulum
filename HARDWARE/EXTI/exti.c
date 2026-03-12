#include "exti.h"
#include "key.h"
#include "control.h"
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
/**************************************************************************
函数功能：外部中断初始化
入口参数：无
返回  值：无 
**************************************************************************/
void EXTI_Init(void)
{
	KEY_Init();	
	Ex_NVIC_Config(GPIO_A,7,FTIR);		//下降沿触发
	Ex_NVIC_Config(GPIO_A,5,FTIR);		//下降沿触发
	Ex_NVIC_Config(GPIO_A,11,FTIR);		//下降沿触发
	Ex_NVIC_Config(GPIO_A,12,FTIR);		//下降沿触发

	MY_NVIC_Init(2,2,EXTI9_5_IRQn,2);  	//抢占2，子优先级2，组2
	MY_NVIC_Init(2,2,EXTI15_10_IRQn,2);	//抢占2，子优先级2，组2	  
}


//外部中断9~5服务程序
void EXTI9_5_IRQHandler(void)
{			
	delay_ms(5);   //消抖			 
   if(KEY5==0)	// 
	{
		if (ErrorCode == 0) ErrorCode = 1;
		else ErrorCode = 0;
		if(ErrorCode==0) Swing_up=0;
	}		
	if(KEY7==0)	///Menu 
	{
		if(Menu++==4) Menu=1;
	}		
 		EXTI->PR=1<<5;     //清除LINE5上的中断标志位  
		EXTI->PR=1<<7;     //清除LINE7上的中断标志位
}
//外部中断15~10服务程序
void EXTI15_10_IRQHandler(void)
{			
	delay_ms(5);   //消抖

  if(KEY12==0)	//PID-
	{
		if(Menu==1)        k0_add-=1;
	  else	if(Menu==2)  k1_add-=1;
		else  if(Menu==3)  k2_add-=1;
		else  if(Menu==4)  k3_add-=1;
	}		
	 if(KEY11==0)	//PID+ 
	{
			    if(Menu==1)  k0_add+=1;
	  else	if(Menu==2)  k1_add+=1;
		else  if(Menu==3)  k2_add+=1;
		else  if(Menu==4)  k3_add+=1;
	}		
	
	
	/*
  if(KEY12==0)	//PID-
	{
		if(Menu==1)        Moto-=0.2;
	  else	if(Menu==2)  Moto-=0.2;
		else  if(Menu==3)  Moto-=0.2;
		else  if(Menu==4)  Moto-=0.2;
	}		
	 if(KEY11==0)	//PID+ 
	{
			    if(Menu==1)  Moto+=0.2;
	  else	if(Menu==2)  Moto+=0.2;
		else  if(Menu==3)  Moto+=0.2;
		else  if(Menu==4)  Moto+=0.2;
	}		
	*/
  EXTI->PR=1<<11; //清除LINE11上的中断标志位  
	EXTI->PR=1<<12; //清除LINE12上的中断标志位 
}





