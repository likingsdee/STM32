#include "gps.h" 		
#include "stdio.h"	 
#include "stdarg.h"	 
#include "string.h"	 
#include "math.h"
#include "stm32f1xx_hal.h"
#include "usart.h"
///////////////////////////////////////////////////////////////////////////////// 	   

//��buf����õ���cx���������ڵ�λ��
//����ֵ:0~0XFE,����������λ�õ�ƫ��.
//       0XFF,�������ڵ�cx������							  
uint8_t NMEA_Comma_Pos(uint8_t *buf,uint8_t cx)
{	 		    
	uint8_t *p=buf;
	while(cx)
	{		 
		if(*buf=='*'||*buf<' '||*buf>'z')return 0XFF;//����'*'���߷Ƿ��ַ�,�򲻴��ڵ�cx������
		if(*buf==',')cx--;
		buf++;
	}
	return buf-p;	 
}
//m^n����
//����ֵ:m^n�η�.
uint32_t NMEA_Pow(uint8_t m,uint8_t n)
{
	uint32_t result=1;	 
	while(n--)result*=m;    
	return result;
}
//strת��Ϊ����,��','����'*'����
//buf:���ִ洢��
//dx:С����λ��,���ظ����ú���
//����ֵ:ת�������ֵ
int NMEA_Str2num(uint8_t *buf,uint8_t*dx)
{
	uint8_t *p=buf;
	uint32_t ires=0,fres=0;
	uint8_t ilen=0,flen=0,i;
	uint8_t mask=0;
	int res;
	while(1) //�õ�������С���ĳ���
	{
		if(*p=='-'){mask|=0X02;p++;}//�Ǹ���
		if(*p==','||(*p=='*'))break;//����������
		if(*p=='.'){mask|=0X01;p++;}//����С������
		else if(*p>'9'||(*p<'0'))	//�зǷ��ַ�
		{	
			ilen=0;
			flen=0;
			break;
		}	
		if(mask&0X01)flen++;
		else ilen++;
		p++;
	}
	if(mask&0X02)buf++;	//ȥ������
	for(i=0;i<ilen;i++)	//�õ�������������
	{  
		ires+=NMEA_Pow(10,ilen-1-i)*(buf[i]-'0');
	}
	if(flen>5)flen=5;	//���ȡ5λС��
	*dx=flen;	 		//С����λ��
	for(i=0;i<flen;i++)	//�õ�С����������
	{  
		fres+=NMEA_Pow(10,flen-1-i)*(buf[ilen+1+i]-'0');
	} 
	res=ires*NMEA_Pow(10,flen)+fres;
	if(mask&0X02)res=-res;		   
	return res;
}	  							 

//����GNGSA��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�?
void NMEA_GNGGA_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p1,dx;			 
	uint8_t posx; 
  int temp;  
	p1=(uint8_t*)strstr((const char *)buf,"GGA");//$GNGGA
	posx=NMEA_Comma_Pos(p1,6);								//�õ�GPS״̬
	if(p1)
	{
		if(posx!=0XFF)gpsx->gpssta=NMEA_Str2num(p1+posx,&dx);	
		posx=NMEA_Comma_Pos(p1,7);								//�õ����ڶ�λ��������
		if(posx!=0XFF)gpsx->posslnum=NMEA_Str2num(p1+posx,&dx); 
		posx=NMEA_Comma_Pos(p1,9);								//�õ����θ߶�
		if(posx!=0XFF)//gpsx->altitude=NMEA_Str2num(p1+posx,&dx); 
		{
			temp=NMEA_Str2num(p1+posx,&dx);
			gpsx->altitude=temp;//����10��
			gpsx->falt=(double)temp/10;
			
		}
	}
}
//����GNRMC��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void NMEA_GNRMC_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p1,dx;			 
	uint8_t posx;     
	int64_t temp;	   
	double rs;  
	p1=(uint8_t*)strstr((const char *)buf,"RMC");//"$GNRMC",������&��GNRMC�ֿ������,��ֻ�ж�GPRMC.
	if(p1)
	{
		posx=NMEA_Comma_Pos(p1,1);								//�õ�UTCʱ��
		if(posx!=0XFF)
		{
			temp=NMEA_Str2num(p1+posx,&dx);	 	//�õ�UTCʱ��,ȥ��ms
			gpsx->utc.hour=temp/1000000;
			gpsx->utc.min=(temp/10000)%100;
			gpsx->utc.sec=(temp/100)%100;
			gpsx->utc.hsec=(temp%100)*10;	 	 
		}	
		posx=NMEA_Comma_Pos(p1,2);                //��λ״̬
		gpsx->locationsta=*(p1+posx);
		posx=NMEA_Comma_Pos(p1,3);								//�õ�γ��
		if(posx!=0XFF)
		{
			temp=NMEA_Str2num(p1+posx,&dx);		 	 
			gpsx->latitude=temp/NMEA_Pow(10,dx+2);	//�õ���
			rs=temp%NMEA_Pow(10,dx+2);				//�õ�'		 
			gpsx->latitude=(uint32_t)(gpsx->latitude*NMEA_Pow(10,7)+(rs*NMEA_Pow(10,7-dx))/60);//ת��Ϊ�� 
			gpsx->flat=(double)gpsx->latitude/10000000;
		}
		posx=NMEA_Comma_Pos(p1,4);								//��γ���Ǳ�γ 
		if(posx!=0XFF)gpsx->nshemi=*(p1+posx);					 
		posx=NMEA_Comma_Pos(p1,5);								//�õ�����
		if(posx!=0XFF)
		{												  
			temp=NMEA_Str2num(p1+posx,&dx);	
		//	printf("temp=%lld\r\n",temp);			
			gpsx->longitude=temp/NMEA_Pow(10,dx+2);	//�õ���
			rs=temp%NMEA_Pow(10,dx+2);				//�õ�'		 
			gpsx->longitude=(uint32_t)(gpsx->longitude*NMEA_Pow(10,7)+(rs*NMEA_Pow(10,7-dx))/60);//ת��Ϊ��
			gpsx->flon=(double)gpsx->longitude*0.0000001;		
		}
		posx=NMEA_Comma_Pos(p1,6);								//������������
		if(posx!=0XFF)gpsx->ewhemi=*(p1+posx);	
		posx=NMEA_Comma_Pos(p1,7);								//�õ���������
		if(posx!=0XFF)
		{
			temp=NMEA_Str2num(p1+posx,&dx);
			//gpsx->speed=temp/NMEA_Pow(10,3-dx)*1.852;	 	 		//ȷ������1000������1000�õ�KM/H
			gpsx->speed=(uint16_t)(temp/(3.6*NMEA_Pow(10,dx-3))*1.852);	 	//ȷ������1000������ת����CM/S
		}	
		posx=NMEA_Comma_Pos(p1,8);								//�õ����溽��
		if(posx!=0XFF)
	        {
		 temp=NMEA_Str2num(p1+posx,&dx);
		 gpsx->direction=temp/NMEA_Pow(10,dx);
	        }
		posx=NMEA_Comma_Pos(p1,9);								//�õ�UTC����
		if(posx!=0XFF)
		{
			temp=NMEA_Str2num(p1+posx,&dx);		 				//�õ�UTC����
			gpsx->utc.date=temp/10000;
			gpsx->utc.month=(temp/100)%100;
			gpsx->utc.year=2000+temp%100;	
			//gpsx->utc.year=temp%100;		
		} 
	}
}

  
//��ȡNMEA-0183��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void GPS_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	NMEA_GNGGA_Analysis(gpsx,buf);	//GNGGA���� 	
	NMEA_GNRMC_Analysis(gpsx,buf);	//GPNMC����
}

//GPSУ��ͼ���
//buf:���ݻ������׵�ַ
//len:���ݳ���
//cka,ckb:����У����.
void Ublox_CheckSum(uint8_t *buf,uint16_t len,uint8_t* cka,uint8_t*ckb)
{
	uint16_t i;
	*cka=0;*ckb=0;
	for(i=0;i<len;i++)
	{
		*cka=*cka+buf[i];
		*ckb=*ckb+*cka;
	}
}


/**
  * ��������: �ж�����(�������2000�Ժ�����) 
  * �������: iYear    ��λ����
  * �� �� ֵ: uint8_t        1:Ϊ����    0:Ϊƽ�� 
  * ˵    ������
  */
static uint8_t IsLeapYear(uint8_t iYear) 
{ 
    uint16_t    Year; 
    Year    =   iYear; 
    if((Year%4)==0) 
    { 
        return ((Year%400==0) || (Year%100!=0)); 
    } 
     return 0; 
} 

/**
  * ��������: ��������ʱ�任�������ʱ��ʱ�� 
  * �������: *DT:��ʾ����ʱ������� ��ʽ YY,MM,DD,HH,MM,SS
  * �� �� ֵ: ��
  * ˵    ����AREA:1(+)���� W0(-)����   GMT:ʱ���� 
  */
void    GMTconvert(nmea_time *SourceTime, nmea_time *ConvertTime, uint8_t GMT,uint8_t AREA) 
{ 
    uint32_t    YY,MM,DD,hh,mm,ss,ms;        //������ʱ�����ݴ���� 
     
    if(GMT==0)    return;                //�������0ʱ��ֱ�ӷ��� 
    if(GMT>12)    return;                //ʱ�����Ϊ12 �����򷵻�         

    YY    =    SourceTime->year;                //��ȡ�� 
    MM    =    SourceTime->month;                 //��ȡ�� 
    DD    =    SourceTime->date;                 //��ȡ�� 
    hh    =    SourceTime->hour;                //��ȡʱ 
    mm    =    SourceTime->min;                 //��ȡ�� 
    ss    =    SourceTime->sec;                 //��ȡ�� 
    ms    =    SourceTime->hsec;               //���� 
	
    if(AREA)                        //��(+)ʱ������ 
    { 
        if(hh+GMT<24)    hh    +=    GMT;//������������ʱ�䴦��ͬһ�������Сʱ���� 
        else                        //����Ѿ����ڸ�������ʱ��1����������ڴ��� 
        { 
            hh    =    hh+GMT-24;        //�ȵó�ʱ�� 
            if(MM==1 || MM==3 || MM==5 || MM==7 || MM==8 || MM==10)    //���·�(12�µ�������) 
            { 
                if(DD<31)    DD++; 
                else 
                { 
                    DD    =    1; 
                    MM    ++; 
                } 
            } 
            else if(MM==4 || MM==6 || MM==9 || MM==11)                //С�·�2�µ�������) 
            { 
                if(DD<30)    DD++; 
                else 
                { 
                    DD    =    1; 
                    MM    ++; 
                } 
            } 
            else if(MM==2)    //����2�·� 
            { 
                if((DD==29) || (DD==28 && IsLeapYear(YY)==0))        //��������������2��29�� ���߲�����������2��28�� 
                { 
                    DD    =    1; 
                    MM    ++; 
                } 
                else    DD++; 
            } 
            else if(MM==12)    //����12�·� 
            { 
                if(DD<31)    DD++; 
                else        //�������һ�� 
                {               
                    DD    =    1; 
                    MM    =    1; 
                    YY    ++; 
                } 
            } 
        } 
    } 
    else 
    {     
        if(hh>=GMT)    hh    -=    GMT;    //������������ʱ�䴦��ͬһ�������Сʱ���� 
        else                        //����Ѿ����ڸ�������ʱ��1����������ڴ��� 
        { 
            hh    =    hh+24-GMT;        //�ȵó�ʱ�� 
            if(MM==2 || MM==4 || MM==6 || MM==8 || MM==9 || MM==11)    //�����Ǵ��·�(1�µ�������) 
            { 
                if(DD>1)    DD--; 
                else 
                { 
                    DD    =    31; 
                    MM    --; 
                } 
            } 
            else if(MM==5 || MM==7 || MM==10 || MM==12)                //������С�·�2�µ�������) 
            { 
                if(DD>1)    DD--; 
                else 
                { 
                    DD    =    30; 
                    MM    --; 
                } 
            } 
            else if(MM==3)    //�����ϸ�����2�·� 
            { 
                if((DD==1) && IsLeapYear(YY)==0)                    //�������� 
                { 
                    DD    =    28; 
                    MM    --; 
                } 
                else    DD--; 
            } 
            else if(MM==1)    //����1�·� 
            { 
                if(DD>1)    DD--; 
                else        //�����һ�� 
                {               
                    DD    =    31; 
                    MM    =    12; 
                    YY    --; 
                } 
            } 
        } 
    }         

    ConvertTime->year   =    YY;                //������ 
    ConvertTime->month  =    MM;                //������ 
    ConvertTime->date   =    DD;                //������ 
    ConvertTime->hour   =    hh;                //����ʱ 
    ConvertTime->min    =    mm;                //���·� 
    ConvertTime->sec    =    ss;                //������ 
    ConvertTime->hsec    =   ms;                //������ 

}  
uint64_t GetTimestamp(nmea_time *Time)
{
  const uint8_t mon_table[12]={31,28,31,30,31,30,31,31,30,31,30,31};//ƽ��ƽ��������
  const uint8_t leap_mon_table[12]={31,29,31,30,31,30,31,31,30,31,30,31};//��������������
  uint16_t i= 0;
  uint64_t time_tamp_ms,time_tamp_ss;
  uint32_t    YY,MM,DD,hh,mm,ss,ms,days_sum=0;        //������ʱ�����ݴ���� 
       
  YY    =    Time->year;                //��ȡ�� 
  MM    =    Time->month;                 //��ȡ�� 
  DD    =    Time->date;                 //��ȡ�� 
  hh    =    Time->hour;                //��ȡʱ 
  mm    =    Time->min;                 //��ȡ�� 
  ss    =    Time->sec;                 //��ȡ�� 
  ms    =    Time->hsec;               //���� 
	    for(i = 0;i < (YY - 1970);i++)
        {
            if(IsLeapYear(1970+i) == 1)
            {
                days_sum += 366;
            }
            else
            {
                days_sum += 365;
            }
        }
			if(IsLeapYear(YY) != 0)//
        {
            for(i = 0; i < (MM-1);i++)
            {
                days_sum += leap_mon_table[i];
            }
        }
        else
        {
            for(int i = 0; i < (MM-1);i++)
            {
                days_sum +=  mon_table[i];
            }
        }
				
	days_sum +=DD;//---------------��֪���Բ���(uint64_t)
	time_tamp_ss=days_sum*24*3600+hh*3600+mm*60+ss;
	time_tamp_ms=time_tamp_ss*1000+ms;
        return time_tamp_ms;
}

//void GPS_Update(void)
//{ 
//  uint16_t len=0,i=0,j=0;
//  
//  if(UART2_TYPE.receive_flag == 1)
//  
////  
////  Amail_TypeDef GPS_PVT;
////Amail_TypeDef GPS_ODO;
////Amail_TypeDef GPS_NEMA;
//  {
//    if(UART2_TYPE.usartDMA_rxBuf[0]==0xB5&&UART2_TYPE.usartDMA_rxBuf[1]==0x62&&UART2_TYPE.usartDMA_rxBuf[2]==0x01&&UART2_TYPE.usartDMA_rxBuf[3]==0x07)
//    {
//    
//        for(j=0;j<100;j++,i++)
//          {
//            GPS_PVT.udp_to_send[j]=UART2_TYPE.usartDMA_rxBuf[i];
//          }
//      GPS_PVT.ucMessage_len=100;
//      osMailPut(mailQ01Handle,&GPS_PVT);
////        osMutexWait(UDP_Send_MutexHandle,osWaitForever);
////	do_udp((char*)PVT_RX_BUFF.rxBuf,100);	//100PVT + ODO 28
//       // osMutexRelease(UDP_Send_MutexHandle);      
//    }
////    for(i=0;i<UART2_TYPE.rx_len;i++)
////    {
////      if(UART2_TYPE.usartDMA_rxBuf[i+0]==0xB5&&UART2_TYPE.usartDMA_rxBuf[i+1]==0x62&&UART2_TYPE.usartDMA_rxBuf[i+2]==0x01&&UART2_TYPE.usartDMA_rxBuf[i+3]==0x09)
////	{
////          for(j=0;j<28;j++,i++)
////          {
////            ODO_RX_BUFF->udp_to_send[j]=UART2_TYPE.usartDMA_rxBuf[i];
////          }	
////          ODO_RX_BUFF->ucMessage_len=28;
////          osMutexWait(UDP_Send_MutexHandle,osWaitForever);
////	//do_udp((char*)ODO_RX_BUFF.udp_to_send,28);	//100PVT + ODO 28
////        osMutexRelease(UDP_Send_MutexHandle);
////            
////         }
////    }
////				 
////    for(i=0;i<UART2_TYPE.rx_len;i++)
////    {
////	if(UART2_TYPE.usartDMA_rxBuf[i+0]=='$'&&UART2_TYPE.usartDMA_rxBuf[i+1]=='G'&&UART2_TYPE.usartDMA_rxBuf[i+2]=='N')
////	{
////          NEMA_RX_BUFF->ucMessage_len=UART2_TYPE.rx_len-i;
////          for(j=0;j<len;j++,i++)
////          {
////            NEMA_RX_BUFF->udp_to_send[j]=UART2_TYPE.usartDMA_rxBuf[i];
////          }
////          
////          break;
////	}
////      }	
////			GPS_Analysis(&gpsx,(uint8_t*)GPS_RX_BUFF);
////			if(gpsx.locationsta=='A')
////			{	
////				printf("��λ�ɹ�\r\n");				
////			}else
////					{
////					 printf("��λʧ��\r\n");
////					}
//          memset(UART2_TYPE.usartDMA_rxBuf,0,UART2_TYPE.rx_len);
//          UART2_TYPE.rx_len = 0;
//          UART2_TYPE.receive_flag = 0;
//          HAL_UART_Receive_DMA(&huart2,UART2_TYPE.usartDMA_rxBuf,1024);
//	}
//}
//      if(pvt_msg_send->gnssFixOK==1)
//      {  
//        if(pvt_aver_first==1)
//        {
//          pvt_aver_first=0;
//          aver_lon=pvt_msg_send->lon;
//          aver_lat=pvt_msg_send->lat;
//          aver_height=pvt_msg_send->height;
//        }
//        else
//        {
//          aver_lon=(pvt_msg_send->lon+aver_lon)/2;
//          aver_lat=(pvt_msg_send->lat+aver_lat)/2;
//          aver_height=(pvt_msg_send->height+aver_height)/2;
//        }
//      }  
