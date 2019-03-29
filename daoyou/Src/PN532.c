#include "stm32f1xx_hal.h"
#include "usart.h"
#include "PN532.h" 

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
#define UART_BUFF_SIZE      1024
/* 私有变量 ------------------------------------------------------------------*/
uint8_t data1[]={0x55,0x55,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x03,0xFD,0xD4,0x14,0x01,0x17,0x00};  
uint8_t data2[]={0x00,0x00,0xFF,0x04,0xFC,0xD4,0x4A,0x01,0x00,0xE1,0x00};  

uint8_t aRxBuffer;
uint8_t UID[4]; //存储 UID
uint8_t UID_backup[4];//UID备份  用于处理 不连续写同一卡
__IO  uint16_t uart_p = 0;
uint8_t uart_buff[UART_BUFF_SIZE];
extern __IO uint8_t flag_nfc_status; 

/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/

/**
  * 函数功能: 唤醒NFC模块
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void nfc_WakeUp(void)
{



    HAL_UART_Transmit(&husartx_HMI,&data1[0],24,0xffff);//往USART2，发送 length长度的数据data
  	while(__HAL_UART_GET_FLAG(&husartx_HMI,UART_FLAG_TXE)==0); //循环发送,直到发送完毕
    
    HAL_Delay(180); 


}

/**
  * 函数功能: 获取ID值
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void  nfc_InListPassiveTarget(void)
{
  uint8_t i;
  uint8_t temp=0;
  uint8_t CheckCode=0; //数据校验码
	uint16_t len; 
  while(1)
  {   
    HAL_UART_Transmit(&husartx_HMI,&data2[0],11,0xffff);//往USART2，发送 length长度的数据data
  	while(__HAL_UART_GET_FLAG(&husartx_HMI,UART_FLAG_TXE)==0); //循环发送,直到发送完毕
    
    HAL_Delay(180); 
    /*获取数据*/
    //00 00 FF 04 FC D4 4A 01 00 E1 00    
    if((len!=0)&&(uart_buff[9]!=0))
    {
  
      for(i=11;i<23;i++)
      {
          temp+=uart_buff[i];
      }
      CheckCode=0x100-temp;
      if(CheckCode==uart_buff[23])
      {
          UID[0]=uart_buff[19];
          UID[1]=uart_buff[20];
          UID[2]=uart_buff[21];
          UID[3]=uart_buff[22];  
//          if((UID[0]!=0)||(UID[1]!=0)||(UID[2]!=0)||(UID[3]!=0))
//          {
//            printf("UID为:%x %x %x %x\n",UID[0],UID[1],UID[2],UID[3]);                       
//          }
          break;
      }
    }

  }
}


/**
  * 函数功能: 读02区的16个字节
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void nfc_read(void)
{
  uint8_t i,data[12];
  uint8_t temp=0;
  data[0]=0x00;
  data[1]=0x00;
  data[2]=0xFF;
  
  data[3]=0x05; //包 长度
  data[4]=0xFB; //包 长度 校验  0x100-data[3]
  
  data[5]=0xD4; //命令标识码
  data[6]=0x40; //命令标识码
  
  data[7]=0x01;
  data[8]=0x30;
  data[9]=0x02; //读第二块的16字节数据 
  
  temp=0;
  for(i=5;i<10;i++)
  {
      temp+=data[i];
  }
  data[10]=0x100-temp; 
  data[11]=0x00;  
  HAL_UART_Transmit(&husartx_HMI,&data[0],12,0xffff);//往USART2，发送 length长度的数据data
  while(__HAL_UART_GET_FLAG(&husartx_HMI,UART_FLAG_TXE)==0); //循环发送,直到发送完毕
  
  HAL_Delay(180); 

}



