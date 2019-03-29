#include "timer.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F429开发板
//定时器中断驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2016/1/6
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 

extern vu16 USART2_RX_STA;
TIM_HandleTypeDef TIM3_Handler;      //定时器句柄 

//通用定时器3中断初始化
//arr：自动重装值。
//psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz
//这里使用的是定时器3!(定时器3挂在APB1上，时钟为HCLK/2)
void TIM3_Init(u16 arr,u16 psc)
{  
	__HAL_RCC_TIM3_CLK_ENABLE();            //使能TIM3时钟
	
	TIM3_Handler.Instance=TIM3;                          //通用定时器3
	TIM3_Handler.Init.Prescaler=psc;                     //分频系数
	TIM3_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //向上计数器
	TIM3_Handler.Init.Period=arr;                        //自动装载值
	TIM3_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//时钟分频因子
	HAL_TIM_Base_Init(&TIM3_Handler);
    	
	HAL_NVIC_SetPriority(TIM3_IRQn,1,3);    //设置中断优先级，抢占优先级1，子优先级3
	HAL_NVIC_EnableIRQ(TIM3_IRQn);          //开启ITM3中断   
  HAL_TIM_Base_Start_IT(&TIM3_Handler); //使能定时器3和定时器3更新中断：TIM_IT_UPDATE   
}

//定时器3中断服务函数
void TIM3_IRQHandler(void)
{
  if(__HAL_TIM_GET_FLAG(&TIM3_Handler, TIM_FLAG_UPDATE) != RESET)
  {
    if(__HAL_TIM_GET_IT_SOURCE(&TIM3_Handler, TIM_IT_UPDATE) !=RESET)
    {
      __HAL_TIM_CLEAR_IT(&TIM3_Handler, TIM_IT_UPDATE);
      USART3_RX_STA|=1<<15;	//标记接收完成
			__HAL_TIM_DISABLE(&TIM3_Handler);//关闭定时器3
    }
  }
}



