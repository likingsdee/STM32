#include "as608.h"


  uint8_t check[]={0XEF,0X01,0XFF,0XFF,0XFF,0XFF,0X01,0X00,0X00};
  uint8_t PS_GetImage_buf[]={0XEF,0X01,0XFF,0XFF,0XFF,0XFF,0X01,0X00,0X03,0X01,0X00,0X05};
  uint8_t PS_GenChar_buf[]={0XEF,0X01,0XFF,0XFF,0XFF,0XFF,0X01,0X00,0X04,0X02,0X01,0X00,0X08};
  uint8_t PS_Match_buf[] = {0XEF,0X01,0XFF,0XFF,0XFF,0XFF,0X01,0X00,0X03,0X03,0X00,0X07};
  uint8_t PS_RegModel_buf[] ={0XEF,0X01,0XFF,0XFF,0XFF,0XFF,0X01,0X00,0X03,0X05,0X00,0X09};
  uint8_t PS_StoreChar_buf[]={0XEF,0X01,0XFF,0XFF,0XFF,0XFF,0X01,0X00,0X06,0X06,0X01,0X00,0X00,0X00,0X0E};
  uint8_t PS_HighSpeedSearch_buf[]={0XEF,0X01,0XFF,0XFF,0XFF,0XFF,0X01,0X00,0X08,0X1B,0X01,0X00,0X00,0X01,0X00,0X26};
  uint8_t PS_ReadSysPara_buf[]={0XEF,0X01,0XFF,0XFF,0XFF,0XFF,0X01,0X00,0X03,0X0F,0X00,0X13};
  uint8_t PS_ValidTempleteNum_buf[]={0XEF,0X01,0XFF,0XFF,0XFF,0XFF,0X01,0X00,0X03,0X1D,0X00,0X21};


//��ʾȷ���������Ϣ
void ShowErrMessage(uint8_t ensure)
{
	//LCD_Fill(0,120,lcddev.width,160,WHITE);
	//Show_Str_Mid(0,120,(uint8_t*)EnsureMessage(ensure),16,240);
}
//¼��ͼ�� PS_GetImage
//����:̽����ָ��̽�⵽��¼��ָ��ͼ�����ImageBuffer�� 
//ģ�鷵��ȷ����
uint8_t PS_GetImage(void)
{
  uint8_t  ensure;
  HAL_UART_Transmit(&huart3,PS_GetImage_buf,12,0xffff);
 // HAL_Delay(1);
//	if(UART3_TYPE.receive_flag ==  1 )
//        {
//		ensure=UART3_TYPE.usartDMA_rxBuf[9];
//                UART3_TYPE.receive_flag=  0; 
//        }
//	else
//		ensure=0xff;
       while(UART3_TYPE.receive_flag !=  1 );
       // {
		ensure=UART3_TYPE.usartDMA_rxBuf[9];
                UART3_TYPE.receive_flag=  0; 
        //}
	//else
	//	ensure=0xff;
	return ensure;
}
//�������� PS_GenChar
//����:��ImageBuffer�е�ԭʼͼ������ָ�������ļ�����CharBuffer1��CharBuffer2			 
//����:BufferID --> charBuffer1:0x01	charBuffer1:0x02												
//ģ�鷵��ȷ����
uint8_t PS_GenChar(uint8_t BufferID)
{
        uint8_t  ensure;
        if(BufferID == 0X01)
        {
          PS_GenChar_buf[10] = 0X01;
          PS_GenChar_buf[12]= 0X08;
          HAL_UART_Transmit(&huart3,PS_GenChar_buf,13,0xffff);
        //    HAL_Delay(200);
        }
        else {
          PS_GenChar_buf[10] = 0X02;
          PS_GenChar_buf[12]= 0X09;
          HAL_UART_Transmit(&huart3,PS_GenChar_buf,13,0xffff);
       //     HAL_Delay(200);
        }
//	if(UART3_TYPE.receive_flag ==  1 )
//        {
//		ensure=UART3_TYPE.usartDMA_rxBuf[9];
//                UART3_TYPE.receive_flag=  0;
//        }
//	else
//		ensure=0xff;
         while(UART3_TYPE.receive_flag !=  1 );
       		ensure=UART3_TYPE.usartDMA_rxBuf[9];
               UART3_TYPE.receive_flag=  0;
	return ensure;
}
//��ȷ�ȶ���öָ������ PS_Match
//����:��ȷ�ȶ�CharBuffer1 ��CharBuffer2 �е������ļ� 
//ģ�鷵��ȷ����
uint8_t PS_Match(void)
{
        uint8_t  ensure;
        HAL_UART_Transmit(&huart3,PS_Match_buf,12,0xffff);
          HAL_Delay(200);
	if(UART3_TYPE.receive_flag ==  1)
        {
		ensure=UART3_TYPE.usartDMA_rxBuf[9];
                UART3_TYPE.receive_flag=  0;
        }
	else
		ensure=0xff;
	return ensure;
}
//�ϲ�����������ģ�壩PS_RegModel
//����:��CharBuffer1��CharBuffer2�е������ļ��ϲ����� ģ��,�������CharBuffer1��CharBuffer2	
//˵��:  ģ�鷵��ȷ����
uint8_t PS_RegModel(void)
{
        uint8_t  ensure;
	HAL_UART_Transmit(&huart3,PS_RegModel_buf,12,0xffff);
        HAL_Delay(200);
	if(UART3_TYPE.receive_flag ==  1)
        {
		ensure=UART3_TYPE.usartDMA_rxBuf[9];
                UART3_TYPE.receive_flag=  0;
        }
	else
		ensure=0xff;
	return ensure;		
}
//����ģ�� PS_StoreChar
//����:�� CharBuffer1 �� CharBuffer2 �е�ģ���ļ��浽 PageID ��flash���ݿ�λ�á�			
//����:  BufferID @ref charBuffer1:0x01	charBuffer1:0x02
//       PageID��ָ�ƿ�λ�úţ�
//˵��:  ģ�鷵��ȷ����
//uint8_t PS_StoreChar(uint8_t BufferID,uint16_t PageID)
uint8_t PS_StoreChar(void)
{
        uint8_t  ensure;
	HAL_UART_Transmit(&huart3,PS_StoreChar_buf,15,0xffff);
        HAL_Delay(200);
	if(UART3_TYPE.receive_flag ==  1)
        {
		ensure=UART3_TYPE.usartDMA_rxBuf[9];
                UART3_TYPE.receive_flag=  0;
        }
	else
		ensure=0xff;
	return ensure;	
}
//��ϵͳ�������� PS_ReadSysPara
//����:  ��ȡģ��Ļ��������������ʣ�����С��)
//����:  ��
//˵��:  ģ�鷵��ȷ���� + ����������16bytes��
uint8_t PS_ReadSysPara(SysPara *p)
{
         uint8_t  ensure;
        HAL_UART_Transmit(&huart3,PS_ReadSysPara_buf,12,0xffff);
        HAL_Delay(200);
	if(UART3_TYPE.receive_flag ==  1)
	{
		ensure = UART3_TYPE.usartDMA_rxBuf[9];
		p->PS_max = (UART3_TYPE.usartDMA_rxBuf[14]<<8)+UART3_TYPE.usartDMA_rxBuf[15];
		p->PS_level = UART3_TYPE.usartDMA_rxBuf[17];
		p->PS_addr=(UART3_TYPE.usartDMA_rxBuf[18]<<24)+(UART3_TYPE.usartDMA_rxBuf[19]<<16)+(UART3_TYPE.usartDMA_rxBuf[20]<<8)+UART3_TYPE.usartDMA_rxBuf[21];
		p->PS_size = UART3_TYPE.usartDMA_rxBuf[23];
		p->PS_N = UART3_TYPE.usartDMA_rxBuf[25];
                UART3_TYPE.receive_flag=  0;
	}		
	else
		ensure=0xff;
	if(ensure==0x00)
	{
		printf("\r\nģ�����ָ������=%d",p->PS_max);
		printf("\r\n�Աȵȼ�=%d",p->PS_level);
		printf("\r\n��ַ=%x",p->PS_addr);
		printf("\r\n������=%d",p->PS_N*9600);
	}
//	else 
	//		printf("\r\n%s",EnsureMessage(ensure));
	return ensure;
}
//��������PS_HighSpeedSearch
//���ܣ��� CharBuffer1��CharBuffer2�е������ļ��������������򲿷�ָ�ƿ⡣
//		  �����������򷵻�ҳ��,��ָ����ڵ�ȷ������ָ�ƿ��� ���ҵ�¼ʱ����
//		  �ܺõ�ָ�ƣ���ܿ�������������
//����:  BufferID�� StartPage(��ʼҳ)��PageNum��ҳ����
//˵��:  ģ�鷵��ȷ����+ҳ�루����ָ��ģ�壩
//uint8_t PS_HighSpeedSearch(uint8_t BufferID,u16 StartPage,u16 PageNum,SearchResult *p)
uint8_t PS_HighSpeedSearch(void)
{
  uint8_t  ensure;
	HAL_UART_Transmit(&huart3,PS_HighSpeedSearch_buf,16,0xffff);
        HAL_Delay(500);
	if(UART3_TYPE.receive_flag ==  1)
        {
		ensure=UART3_TYPE.usartDMA_rxBuf[9];
                UART3_TYPE.receive_flag=  0;
        }
	else
		ensure=0xff;
	return ensure;
}
//����Чģ����� PS_ValidTempleteNum
//���ܣ�����Чģ�����
//����: ��
//˵��: ģ�鷵��ȷ����+��Чģ�����ValidN
uint8_t PS_ValidTempleteNum(uint16_t *ValidN)
{
        uint8_t  ensure;
        HAL_UART_Transmit(&huart3,PS_ValidTempleteNum_buf,12,0xffff);
        HAL_Delay(200);
	if(UART3_TYPE.receive_flag ==  1)
	{
		ensure=UART3_TYPE.usartDMA_rxBuf[9];
		*ValidN = (UART3_TYPE.usartDMA_rxBuf[10]) +UART3_TYPE.usartDMA_rxBuf[11];
	}		
	else
		ensure=0xff;
	
	if(ensure==0x00)
	{
		printf("\r\n��Чָ�Ƹ���=%d",(UART3_TYPE.usartDMA_rxBuf[10])+UART3_TYPE.usartDMA_rxBuf[11]);
	}
//	else
//		printf("\r\n%s",EnsureMessage(ensure));
	return ensure;
}
//ˢָ��
void press_FR(void)
{
	//SearchResult seach;
	uint8_t ensure;
	//char *str;
	ensure=PS_GetImage();
	if(ensure==0x00)//��ȡͼ��ɹ� 
	{	
		ensure=PS_GenChar(0X01);
		if(ensure==0x00) //���������ɹ�
		{		

			ensure=PS_HighSpeedSearch();
			if(ensure==0x00)//�����ɹ�
			{				
			
                           printf("1");
			}
			else 
				ShowErrMessage(ensure);					
	  }
		else
			ShowErrMessage(ensure);
	}
		
}
void Add_FR(void)
{
	uint8_t i,ensure ,processnum=0;
	//u16 ID;
	while(1)
	{
		switch (processnum)
		{
			case 0:
				i++;
				ensure=PS_GetImage();
				if(ensure==0x00) 
				{
					ensure=PS_GenChar(0X01);//��������
					if(ensure==0x00)
					{
						i=0;
						processnum=1;//�����ڶ���						
					}else ShowErrMessage(ensure);				
				}else ShowErrMessage(ensure);						
				break;
			
			case 1:
				i++;
				ensure=PS_GetImage();
				if(ensure==0x00) 
				{
					ensure=PS_GenChar(0X02);//��������
					if(ensure==0x00)
					{

						i=0;
						processnum=2;//����������
					}else ShowErrMessage(ensure);	
				}else ShowErrMessage(ensure);		
				break;

			case 2:
          //                PS_RegModel();
          //                PS_StoreChar();
				ensure=PS_Match();
				if(ensure==0x00) 
				{
					processnum=3;//�������Ĳ�
				}
				else 
				{
					ShowErrMessage(ensure);
					i=0;
					processnum=0;//���ص�һ��		
				}
				HAL_Delay(1200);
				break;
                }
        }
}