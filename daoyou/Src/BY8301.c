#include "BY8301.h"
#include "usart.h"   
uint8_t BY8301_Send_Data[20];

void BY8301_Send_Cmd(uint8_t cmd)
{
  uint8_t BY8301_Send_Cmd_Data[10];
  
  BY8301_Send_Cmd_Data[0]=0x7E;
  BY8301_Send_Cmd_Data[1]=0X03;
  BY8301_Send_Cmd_Data[2]=cmd;
  BY8301_Send_Cmd_Data[3]=0;
    for (uint8_t i = 1;i<BY8301_Send_Cmd_Data[1]-1;i++)
    {
        BY8301_Send_Cmd_Data[3] = BY8301_Send_Cmd_Data[3] ^ BY8301_Send_Cmd_Data[i]; //进行异或交验取值 
    }
  BY8301_Send_Cmd_Data[3]=0x0d;
  BY8301_Send_Cmd_Data[4]=0xEF;
  HAL_UART_Transmit(&huart3,BY8301_Send_Cmd_Data,5,1000);
}

void BY8301_Play_Voice(uint16_t voice_num)
{
  //if(Flag_playvoivce == 0)
   { 
   // Audio_SW_OPEN;                    
    BY8301_Send_Data[0]=0x7E;
    BY8301_Send_Data[1]=0X05;
    BY8301_Send_Data[2]=0x41;
    BY8301_Send_Data[3]=(uint8_t)(voice_num&0xFF00)>>8;
    BY8301_Send_Data[4]=(uint8_t)(voice_num&0xFF);
    BY8301_Send_Data[5]=0;
    for (uint8_t i = 1;i<BY8301_Send_Data[1];i++)
    {
        BY8301_Send_Data[5] = BY8301_Send_Data[5] ^ BY8301_Send_Data[i]; //进行异或交验取值 
    }
     BY8301_Send_Data[6]=0xEF;
    // HAL_UART_Transmit(&huart4,BY8301_Send_Data,7,1000);
     HAL_UART_Transmit(&huart3,BY8301_Send_Data,7,1000);
     //HAL_Delay(5000);
    // Audio_SW_CLOSE;
    }  
}