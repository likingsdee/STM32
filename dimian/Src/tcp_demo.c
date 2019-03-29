/**
************************************************************************************************
* @file   		tcp_demo.c
* @author  		WIZnet Software Team 
* @version 		V1.0
* @date    		2015-02-14
* @brief   		TCP 演示函数
* @attention  
************************************************************************************************
**/

#include <stdio.h>
#include <string.h>
#include "tcp_demo.h"
#include "W5500_conf.h"
#include "w5500.h"
#include "socket.h"
#include "stdlib.h"

uint8 buff[2048];				                              	         /*定义一个2KB的缓存*/
extern uint8_t   Com1_Rxbuff[1024];  //串口接收数组
extern uint16_t  Com1_Counter;     //接收计数
extern uint16_t  Com1_TxCounter;   //发送计数
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
/**
*@brief		TCP Server回环演示函数。
*@param		无
*@return	无
*/
void do_tcp_server(void)
{	
	uint16 len=0;  
	switch(getSn_SR(SOCK_TCPS))											            	/*获取socket的状态*/
	{
		case SOCK_CLOSED:													                  /*socket处于关闭状态*/
			socket(SOCK_TCPS ,Sn_MR_TCP,local_port,Sn_MR_ND);	        /*打开socket*/
		  break;     
    
		case SOCK_INIT:														                  /*socket已初始化状态*/
			listen(SOCK_TCPS);												                /*socket建立监听*/
		  break;
		
		case SOCK_ESTABLISHED:												              /*socket处于连接建立状态*/
		
			if(getSn_IR(SOCK_TCPS) & Sn_IR_CON)
			{
				setSn_IR(SOCK_TCPS, Sn_IR_CON);								          /*清除接收中断标志位*/
			}
			len=getSn_RX_RSR(SOCK_TCPS);									            /*定义len为已接收数据的长度*/
			if(len>0)
			{
				recv(SOCK_TCPS,buff,len);								              	/*接收来自Client的数据*/
				buff[len]=0x00; 											                  /*添加字符串结束符*/
				printf("%s\r\n",buff);
				send(SOCK_TCPS,buff,len);									              /*向Client发送数据*/
		  }
		  break;
		
		case SOCK_CLOSE_WAIT:												                /*socket处于等待关闭状态*/
			close(SOCK_TCPS);
		  break;
	}
}

/**
*@brief		TCP Client回环演示函数。
*@param		无
*@return	无
*/
void do_tcp_client(void)
{	
   uint16 len=0;	
	switch(getSn_SR(SOCK_TCPC))								  				         /*获取socket的状态*/
	{
		case SOCK_CLOSED:											        		         /*socket处于关闭状态*/
			socket(SOCK_TCPC,Sn_MR_TCP,local_port++,Sn_MR_ND);
		  break;
		
		case SOCK_INIT:													        	         /*socket处于初始化状态*/
			connect(SOCK_TCPC,remote_ip,remote_port);                /*socket连接服务器*/ 
		  break;
		
		case SOCK_ESTABLISHED: 												             /*socket处于连接建立状态*/
			if(getSn_IR(SOCK_TCPC) & Sn_IR_CON)
			{
				setSn_IR(SOCK_TCPC, Sn_IR_CON); 							         /*清除接收中断标志位*/
			}
		
			len=getSn_RX_RSR(SOCK_TCPC); 								  	         /*定义len为已接收数据的长度*/
			if(len>0)
			{
				recv(SOCK_TCPC,buff,len); 							   		         /*接收来自Server的数据*/
				buff[len]=0x00;  											                 /*添加字符串结束符*/
				//printf("%s\r\n",buff);
				//send(SOCK_TCPC,buff,len);
                                 HAL_UART_Transmit_IT(&huart1 ,(uint8_t*)buff,len);	//接受到w5500的数据后发送给串口/*向Server发送数据*/

			}
                                                      if(Com1_Counter>0)  //表示串口接收缓冲内有内容，需要经网口发送
				    {
					    
                                            send(SOCK_TCPC,Com1_Rxbuff,Com1_Counter);
                                            HAL_UART_Transmit_IT(&huart2 ,Com1_Rxbuff,Com1_Counter);
                                          //  HAL_Delay(50);
                                            memset(Com1_Rxbuff,0,sizeof(Com1_Rxbuff));
					    Com1_Counter = 0;  				    
				    }
		  break;
			
		case SOCK_CLOSE_WAIT: 											    	         /*socket处于等待关闭状态*/
			close(SOCK_TCPC);
		  break;

	}
}

void tcp_sendbuf(const uint8 *buff)
{
 //  uint16 len=0;
      int i=0;
	switch(getSn_SR(SOCK_TCPC))								  				         /*获取socket的状态*/
	{
		case SOCK_CLOSED:											        		         /*socket处于关闭状态*/
			socket(SOCK_TCPC,Sn_MR_TCP,local_port++,Sn_MR_ND);
		  break;
		
		case SOCK_INIT:													        	         /*socket处于初始化状态*/
			connect(SOCK_TCPC,remote_ip,remote_port);                /*socket连接服务器*/ 
                        i++;
                        if(i>3)
                        {
                          close(SOCK_TCPC);//如果3次没有连上，更换端口号重连 
                          i=0;
                        }
		  break;
		
		case SOCK_ESTABLISHED: 
                  
                  
                  if(getSn_IR(SOCK_TCPC) & Sn_IR_CON)
			{
				setSn_IR(SOCK_TCPC, Sn_IR_CON); 							         /*清除接收中断标志位*/
			}
		
		//	len=getSn_RX_RSR(SOCK_TCPC); 								  	         /*定义len为已接收数据的长度*/
//			if(len>0)
//			{
//				recv(SOCK_TCPC,buff,len); 							   		         /*接收来自Server的数据*/
//				buff[len]=0x00;  											                 /*添加字符串结束符*/
////				printf("%s\r\n",buff);
////				send(SOCK_TCPC,buff,len);								     	         /*向Server发送数据*/
//			}		
        
		send(SOCK_TCPC,buff,strlen(buff));								     	         /*向Server发送数据*/
					  
		  break;
			
		case SOCK_CLOSE_WAIT: 											    	         /*socket处于等待关闭状态*/
			close(SOCK_TCPC);
		  break;

	}





}



/**
*@brief		UDP测试程序
*@param		无
*@return	无
*/
void do_udp(void)
{                                                              
	uint16 len=0;
	uint8 buff[512];                                                          /*定义一个2KB的缓存*/	
	switch(getSn_SR(SOCK_UDPS))                                                /*获取socket的状态*/
	{
		case SOCK_CLOSED:                                                        /*socket处于关闭状态*/
			socket(SOCK_UDPS,Sn_MR_UDP,local_port,0);                              /*初始化socket*/
		  break;
		
		case SOCK_UDP:                                                           /*socket初始化完成*/
			HAL_Delay(10);
			if(getSn_IR(SOCK_UDPS) & Sn_IR_RECV)
			{
				setSn_IR(SOCK_UDPS, Sn_IR_RECV);                                     /*清接收中断*/
			}
			if((len=getSn_RX_RSR(SOCK_UDPS))>0)                                    /*接收到数据*/
			{
				recvfrom(SOCK_UDPS,buff, len, remote_ip,&remote_port);               /*W5500接收计算机发送来的数据*/
				buff[len-8]=0x00;                                                    /*添加字符串结束符*/
				printf("%s\r\n",buff);                                               /*打印接收缓存*/ 
				sendto(SOCK_UDPS,buff,len-8, remote_ip, remote_port);                /*W5500把接收到的数据发送给Remote*/
			}
			break;
	}

}

