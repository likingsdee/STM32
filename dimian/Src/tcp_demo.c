/**
************************************************************************************************
* @file   		tcp_demo.c
* @author  		WIZnet Software Team 
* @version 		V1.0
* @date    		2015-02-14
* @brief   		TCP ��ʾ����
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

uint8 buff[2048];				                              	         /*����һ��2KB�Ļ���*/
extern uint8_t   Com1_Rxbuff[1024];  //���ڽ�������
extern uint16_t  Com1_Counter;     //���ռ���
extern uint16_t  Com1_TxCounter;   //���ͼ���
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
/**
*@brief		TCP Server�ػ���ʾ������
*@param		��
*@return	��
*/
void do_tcp_server(void)
{	
	uint16 len=0;  
	switch(getSn_SR(SOCK_TCPS))											            	/*��ȡsocket��״̬*/
	{
		case SOCK_CLOSED:													                  /*socket���ڹر�״̬*/
			socket(SOCK_TCPS ,Sn_MR_TCP,local_port,Sn_MR_ND);	        /*��socket*/
		  break;     
    
		case SOCK_INIT:														                  /*socket�ѳ�ʼ��״̬*/
			listen(SOCK_TCPS);												                /*socket��������*/
		  break;
		
		case SOCK_ESTABLISHED:												              /*socket�������ӽ���״̬*/
		
			if(getSn_IR(SOCK_TCPS) & Sn_IR_CON)
			{
				setSn_IR(SOCK_TCPS, Sn_IR_CON);								          /*��������жϱ�־λ*/
			}
			len=getSn_RX_RSR(SOCK_TCPS);									            /*����lenΪ�ѽ������ݵĳ���*/
			if(len>0)
			{
				recv(SOCK_TCPS,buff,len);								              	/*��������Client������*/
				buff[len]=0x00; 											                  /*����ַ���������*/
				printf("%s\r\n",buff);
				send(SOCK_TCPS,buff,len);									              /*��Client��������*/
		  }
		  break;
		
		case SOCK_CLOSE_WAIT:												                /*socket���ڵȴ��ر�״̬*/
			close(SOCK_TCPS);
		  break;
	}
}

/**
*@brief		TCP Client�ػ���ʾ������
*@param		��
*@return	��
*/
void do_tcp_client(void)
{	
   uint16 len=0;	
	switch(getSn_SR(SOCK_TCPC))								  				         /*��ȡsocket��״̬*/
	{
		case SOCK_CLOSED:											        		         /*socket���ڹر�״̬*/
			socket(SOCK_TCPC,Sn_MR_TCP,local_port++,Sn_MR_ND);
		  break;
		
		case SOCK_INIT:													        	         /*socket���ڳ�ʼ��״̬*/
			connect(SOCK_TCPC,remote_ip,remote_port);                /*socket���ӷ�����*/ 
		  break;
		
		case SOCK_ESTABLISHED: 												             /*socket�������ӽ���״̬*/
			if(getSn_IR(SOCK_TCPC) & Sn_IR_CON)
			{
				setSn_IR(SOCK_TCPC, Sn_IR_CON); 							         /*��������жϱ�־λ*/
			}
		
			len=getSn_RX_RSR(SOCK_TCPC); 								  	         /*����lenΪ�ѽ������ݵĳ���*/
			if(len>0)
			{
				recv(SOCK_TCPC,buff,len); 							   		         /*��������Server������*/
				buff[len]=0x00;  											                 /*����ַ���������*/
				//printf("%s\r\n",buff);
				//send(SOCK_TCPC,buff,len);
                                 HAL_UART_Transmit_IT(&huart1 ,(uint8_t*)buff,len);	//���ܵ�w5500�����ݺ��͸�����/*��Server��������*/

			}
                                                      if(Com1_Counter>0)  //��ʾ���ڽ��ջ����������ݣ���Ҫ�����ڷ���
				    {
					    
                                            send(SOCK_TCPC,Com1_Rxbuff,Com1_Counter);
                                            HAL_UART_Transmit_IT(&huart2 ,Com1_Rxbuff,Com1_Counter);
                                          //  HAL_Delay(50);
                                            memset(Com1_Rxbuff,0,sizeof(Com1_Rxbuff));
					    Com1_Counter = 0;  				    
				    }
		  break;
			
		case SOCK_CLOSE_WAIT: 											    	         /*socket���ڵȴ��ر�״̬*/
			close(SOCK_TCPC);
		  break;

	}
}

void tcp_sendbuf(const uint8 *buff)
{
 //  uint16 len=0;
      int i=0;
	switch(getSn_SR(SOCK_TCPC))								  				         /*��ȡsocket��״̬*/
	{
		case SOCK_CLOSED:											        		         /*socket���ڹر�״̬*/
			socket(SOCK_TCPC,Sn_MR_TCP,local_port++,Sn_MR_ND);
		  break;
		
		case SOCK_INIT:													        	         /*socket���ڳ�ʼ��״̬*/
			connect(SOCK_TCPC,remote_ip,remote_port);                /*socket���ӷ�����*/ 
                        i++;
                        if(i>3)
                        {
                          close(SOCK_TCPC);//���3��û�����ϣ������˿ں����� 
                          i=0;
                        }
		  break;
		
		case SOCK_ESTABLISHED: 
                  
                  
                  if(getSn_IR(SOCK_TCPC) & Sn_IR_CON)
			{
				setSn_IR(SOCK_TCPC, Sn_IR_CON); 							         /*��������жϱ�־λ*/
			}
		
		//	len=getSn_RX_RSR(SOCK_TCPC); 								  	         /*����lenΪ�ѽ������ݵĳ���*/
//			if(len>0)
//			{
//				recv(SOCK_TCPC,buff,len); 							   		         /*��������Server������*/
//				buff[len]=0x00;  											                 /*����ַ���������*/
////				printf("%s\r\n",buff);
////				send(SOCK_TCPC,buff,len);								     	         /*��Server��������*/
//			}		
        
		send(SOCK_TCPC,buff,strlen(buff));								     	         /*��Server��������*/
					  
		  break;
			
		case SOCK_CLOSE_WAIT: 											    	         /*socket���ڵȴ��ر�״̬*/
			close(SOCK_TCPC);
		  break;

	}





}



/**
*@brief		UDP���Գ���
*@param		��
*@return	��
*/
void do_udp(void)
{                                                              
	uint16 len=0;
	uint8 buff[512];                                                          /*����һ��2KB�Ļ���*/	
	switch(getSn_SR(SOCK_UDPS))                                                /*��ȡsocket��״̬*/
	{
		case SOCK_CLOSED:                                                        /*socket���ڹر�״̬*/
			socket(SOCK_UDPS,Sn_MR_UDP,local_port,0);                              /*��ʼ��socket*/
		  break;
		
		case SOCK_UDP:                                                           /*socket��ʼ�����*/
			HAL_Delay(10);
			if(getSn_IR(SOCK_UDPS) & Sn_IR_RECV)
			{
				setSn_IR(SOCK_UDPS, Sn_IR_RECV);                                     /*������ж�*/
			}
			if((len=getSn_RX_RSR(SOCK_UDPS))>0)                                    /*���յ�����*/
			{
				recvfrom(SOCK_UDPS,buff, len, remote_ip,&remote_port);               /*W5500���ռ����������������*/
				buff[len-8]=0x00;                                                    /*����ַ���������*/
				printf("%s\r\n",buff);                                               /*��ӡ���ջ���*/ 
				sendto(SOCK_UDPS,buff,len-8, remote_ip, remote_port);                /*W5500�ѽ��յ������ݷ��͸�Remote*/
			}
			break;
	}

}

