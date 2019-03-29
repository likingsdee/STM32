#include "stm32f1xx_hal.h"
#include "usart.h"
#include "PN532.h" 

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
#define UART_BUFF_SIZE      1024
/* ˽�б��� ------------------------------------------------------------------*/
uint8_t data1[]={0x55,0x55,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x03,0xFD,0xD4,0x14,0x01,0x17,0x00};  
uint8_t data2[]={0x00,0x00,0xFF,0x04,0xFC,0xD4,0x4A,0x01,0x00,0xE1,0x00};  

uint8_t aRxBuffer;
uint8_t UID[4]; //�洢 UID
uint8_t UID_backup[4];//UID����  ���ڴ��� ������дͬһ��
__IO  uint16_t uart_p = 0;
uint8_t uart_buff[UART_BUFF_SIZE];
extern __IO uint8_t flag_nfc_status; 

/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/

/**
  * ��������: ����NFCģ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void nfc_WakeUp(void)
{



    HAL_UART_Transmit(&husartx_HMI,&data1[0],24,0xffff);//��USART2������ length���ȵ�����data
  	while(__HAL_UART_GET_FLAG(&husartx_HMI,UART_FLAG_TXE)==0); //ѭ������,ֱ���������
    
    HAL_Delay(180); 


}

/**
  * ��������: ��ȡIDֵ
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void  nfc_InListPassiveTarget(void)
{
  uint8_t i;
  uint8_t temp=0;
  uint8_t CheckCode=0; //����У����
	uint16_t len; 
  while(1)
  {   
    HAL_UART_Transmit(&husartx_HMI,&data2[0],11,0xffff);//��USART2������ length���ȵ�����data
  	while(__HAL_UART_GET_FLAG(&husartx_HMI,UART_FLAG_TXE)==0); //ѭ������,ֱ���������
    
    HAL_Delay(180); 
    /*��ȡ����*/
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
//            printf("UIDΪ:%x %x %x %x\n",UID[0],UID[1],UID[2],UID[3]);                       
//          }
          break;
      }
    }

  }
}


/**
  * ��������: ��02����16���ֽ�
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void nfc_read(void)
{
  uint8_t i,data[12];
  uint8_t temp=0;
  data[0]=0x00;
  data[1]=0x00;
  data[2]=0xFF;
  
  data[3]=0x05; //�� ����
  data[4]=0xFB; //�� ���� У��  0x100-data[3]
  
  data[5]=0xD4; //�����ʶ��
  data[6]=0x40; //�����ʶ��
  
  data[7]=0x01;
  data[8]=0x30;
  data[9]=0x02; //���ڶ����16�ֽ����� 
  
  temp=0;
  for(i=5;i<10;i++)
  {
      temp+=data[i];
  }
  data[10]=0x100-temp; 
  data[11]=0x00;  
  HAL_UART_Transmit(&husartx_HMI,&data[0],12,0xffff);//��USART2������ length���ȵ�����data
  while(__HAL_UART_GET_FLAG(&husartx_HMI,UART_FLAG_TXE)==0); //ѭ������,ֱ���������
  
  HAL_Delay(180); 

}



