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


//显示确认码错误信息
void ShowErrMessage(uint8_t ensure)
{
	//LCD_Fill(0,120,lcddev.width,160,WHITE);
	//Show_Str_Mid(0,120,(uint8_t*)EnsureMessage(ensure),16,240);
}
//录入图像 PS_GetImage
//功能:探测手指，探测到后录入指纹图像存于ImageBuffer。 
//模块返回确认字
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
//生成特征 PS_GenChar
//功能:将ImageBuffer中的原始图像生成指纹特征文件存于CharBuffer1或CharBuffer2			 
//参数:BufferID --> charBuffer1:0x01	charBuffer1:0x02												
//模块返回确认字
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
//精确比对两枚指纹特征 PS_Match
//功能:精确比对CharBuffer1 与CharBuffer2 中的特征文件 
//模块返回确认字
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
//合并特征（生成模板）PS_RegModel
//功能:将CharBuffer1与CharBuffer2中的特征文件合并生成 模板,结果存于CharBuffer1与CharBuffer2	
//说明:  模块返回确认字
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
//储存模板 PS_StoreChar
//功能:将 CharBuffer1 或 CharBuffer2 中的模板文件存到 PageID 号flash数据库位置。			
//参数:  BufferID @ref charBuffer1:0x01	charBuffer1:0x02
//       PageID（指纹库位置号）
//说明:  模块返回确认字
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
//读系统基本参数 PS_ReadSysPara
//功能:  读取模块的基本参数（波特率，包大小等)
//参数:  无
//说明:  模块返回确认字 + 基本参数（16bytes）
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
		printf("\r\n模块最大指纹容量=%d",p->PS_max);
		printf("\r\n对比等级=%d",p->PS_level);
		printf("\r\n地址=%x",p->PS_addr);
		printf("\r\n波特率=%d",p->PS_N*9600);
	}
//	else 
	//		printf("\r\n%s",EnsureMessage(ensure));
	return ensure;
}
//高速搜索PS_HighSpeedSearch
//功能：以 CharBuffer1或CharBuffer2中的特征文件高速搜索整个或部分指纹库。
//		  若搜索到，则返回页码,该指令对于的确存在于指纹库中 ，且登录时质量
//		  很好的指纹，会很快给出搜索结果。
//参数:  BufferID， StartPage(起始页)，PageNum（页数）
//说明:  模块返回确认字+页码（相配指纹模板）
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
//读有效模板个数 PS_ValidTempleteNum
//功能：读有效模板个数
//参数: 无
//说明: 模块返回确认字+有效模板个数ValidN
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
		printf("\r\n有效指纹个数=%d",(UART3_TYPE.usartDMA_rxBuf[10])+UART3_TYPE.usartDMA_rxBuf[11]);
	}
//	else
//		printf("\r\n%s",EnsureMessage(ensure));
	return ensure;
}
//刷指纹
void press_FR(void)
{
	//SearchResult seach;
	uint8_t ensure;
	//char *str;
	ensure=PS_GetImage();
	if(ensure==0x00)//获取图像成功 
	{	
		ensure=PS_GenChar(0X01);
		if(ensure==0x00) //生成特征成功
		{		

			ensure=PS_HighSpeedSearch();
			if(ensure==0x00)//搜索成功
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
					ensure=PS_GenChar(0X01);//生成特征
					if(ensure==0x00)
					{
						i=0;
						processnum=1;//跳到第二步						
					}else ShowErrMessage(ensure);				
				}else ShowErrMessage(ensure);						
				break;
			
			case 1:
				i++;
				ensure=PS_GetImage();
				if(ensure==0x00) 
				{
					ensure=PS_GenChar(0X02);//生成特征
					if(ensure==0x00)
					{

						i=0;
						processnum=2;//跳到第三步
					}else ShowErrMessage(ensure);	
				}else ShowErrMessage(ensure);		
				break;

			case 2:
          //                PS_RegModel();
          //                PS_StoreChar();
				ensure=PS_Match();
				if(ensure==0x00) 
				{
					processnum=3;//跳到第四步
				}
				else 
				{
					ShowErrMessage(ensure);
					i=0;
					processnum=0;//跳回第一步		
				}
				HAL_Delay(1200);
				break;
                }
        }
}