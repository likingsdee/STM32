#include "gps.h" 		
#include "stdio.h"	 
#include "stdarg.h"	 
#include "string.h"	 
#include "math.h"
#include "stm32f1xx_hal.h"
#include "usart.h"
///////////////////////////////////////////////////////////////////////////////// 	   

//从buf里面得到第cx个逗号所在的位置
//返回值:0~0XFE,代表逗号所在位置的偏移.
//       0XFF,代表不存在第cx个逗号							  
uint8_t NMEA_Comma_Pos(uint8_t *buf,uint8_t cx)
{	 		    
	uint8_t *p=buf;
	while(cx)
	{		 
		if(*buf=='*'||*buf<' '||*buf>'z')return 0XFF;//遇到'*'或者非法字符,则不存在第cx个逗号
		if(*buf==',')cx--;
		buf++;
	}
	return buf-p;	 
}
//m^n函数
//返回值:m^n次方.
uint32_t NMEA_Pow(uint8_t m,uint8_t n)
{
	uint32_t result=1;	 
	while(n--)result*=m;    
	return result;
}
//str转换为数字,以','或者'*'结束
//buf:数字存储区
//dx:小数点位数,返回给调用函数
//返回值:转换后的数值
int NMEA_Str2num(uint8_t *buf,uint8_t*dx)
{
	uint8_t *p=buf;
	uint32_t ires=0,fres=0;
	uint8_t ilen=0,flen=0,i;
	uint8_t mask=0;
	int res;
	while(1) //得到整数和小数的长度
	{
		if(*p=='-'){mask|=0X02;p++;}//是负数
		if(*p==','||(*p=='*'))break;//遇到结束了
		if(*p=='.'){mask|=0X01;p++;}//遇到小数点了
		else if(*p>'9'||(*p<'0'))	//有非法字符
		{	
			ilen=0;
			flen=0;
			break;
		}	
		if(mask&0X01)flen++;
		else ilen++;
		p++;
	}
	if(mask&0X02)buf++;	//去掉负号
	for(i=0;i<ilen;i++)	//得到整数部分数据
	{  
		ires+=NMEA_Pow(10,ilen-1-i)*(buf[i]-'0');
	}
	if(flen>5)flen=5;	//最多取5位小数
	*dx=flen;	 		//小数点位数
	for(i=0;i<flen;i++)	//得到小数部分数据
	{  
		fres+=NMEA_Pow(10,flen-1-i)*(buf[ilen+1+i]-'0');
	} 
	res=ires*NMEA_Pow(10,flen)+fres;
	if(mask&0X02)res=-res;		   
	return res;
}	  							 

//分析GNGSA信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地?
void NMEA_GNGGA_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p1,dx;			 
	uint8_t posx; 
  int temp;  
	p1=(uint8_t*)strstr((const char *)buf,"GGA");//$GNGGA
	posx=NMEA_Comma_Pos(p1,6);								//得到GPS状态
	if(p1)
	{
		if(posx!=0XFF)gpsx->gpssta=NMEA_Str2num(p1+posx,&dx);	
		posx=NMEA_Comma_Pos(p1,7);								//得到用于定位的卫星数
		if(posx!=0XFF)gpsx->posslnum=NMEA_Str2num(p1+posx,&dx); 
		posx=NMEA_Comma_Pos(p1,9);								//得到海拔高度
		if(posx!=0XFF)//gpsx->altitude=NMEA_Str2num(p1+posx,&dx); 
		{
			temp=NMEA_Str2num(p1+posx,&dx);
			gpsx->altitude=temp;//扩大10倍
			gpsx->falt=(double)temp/10;
			
		}
	}
}
//分析GNRMC信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void NMEA_GNRMC_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	uint8_t *p1,dx;			 
	uint8_t posx;     
	int64_t temp;	   
	double rs;  
	p1=(uint8_t*)strstr((const char *)buf,"RMC");//"$GNRMC",经常有&和GNRMC分开的情况,故只判断GPRMC.
	if(p1)
	{
		posx=NMEA_Comma_Pos(p1,1);								//得到UTC时间
		if(posx!=0XFF)
		{
			temp=NMEA_Str2num(p1+posx,&dx);	 	//得到UTC时间,去掉ms
			gpsx->utc.hour=temp/1000000;
			gpsx->utc.min=(temp/10000)%100;
			gpsx->utc.sec=(temp/100)%100;
			gpsx->utc.hsec=(temp%100)*10;	 	 
		}	
		posx=NMEA_Comma_Pos(p1,2);                //定位状态
		gpsx->locationsta=*(p1+posx);
		posx=NMEA_Comma_Pos(p1,3);								//得到纬度
		if(posx!=0XFF)
		{
			temp=NMEA_Str2num(p1+posx,&dx);		 	 
			gpsx->latitude=temp/NMEA_Pow(10,dx+2);	//得到°
			rs=temp%NMEA_Pow(10,dx+2);				//得到'		 
			gpsx->latitude=(uint32_t)(gpsx->latitude*NMEA_Pow(10,7)+(rs*NMEA_Pow(10,7-dx))/60);//转换为° 
			gpsx->flat=(double)gpsx->latitude/10000000;
		}
		posx=NMEA_Comma_Pos(p1,4);								//南纬还是北纬 
		if(posx!=0XFF)gpsx->nshemi=*(p1+posx);					 
		posx=NMEA_Comma_Pos(p1,5);								//得到经度
		if(posx!=0XFF)
		{												  
			temp=NMEA_Str2num(p1+posx,&dx);	
		//	printf("temp=%lld\r\n",temp);			
			gpsx->longitude=temp/NMEA_Pow(10,dx+2);	//得到°
			rs=temp%NMEA_Pow(10,dx+2);				//得到'		 
			gpsx->longitude=(uint32_t)(gpsx->longitude*NMEA_Pow(10,7)+(rs*NMEA_Pow(10,7-dx))/60);//转换为°
			gpsx->flon=(double)gpsx->longitude*0.0000001;		
		}
		posx=NMEA_Comma_Pos(p1,6);								//东经还是西经
		if(posx!=0XFF)gpsx->ewhemi=*(p1+posx);	
		posx=NMEA_Comma_Pos(p1,7);								//得到地面速率
		if(posx!=0XFF)
		{
			temp=NMEA_Str2num(p1+posx,&dx);
			//gpsx->speed=temp/NMEA_Pow(10,3-dx)*1.852;	 	 		//确保扩大1000倍除以1000得到KM/H
			gpsx->speed=(uint16_t)(temp/(3.6*NMEA_Pow(10,dx-3))*1.852);	 	//确保扩大1000倍，节转换成CM/S
		}	
		posx=NMEA_Comma_Pos(p1,8);								//得到地面航向
		if(posx!=0XFF)
	        {
		 temp=NMEA_Str2num(p1+posx,&dx);
		 gpsx->direction=temp/NMEA_Pow(10,dx);
	        }
		posx=NMEA_Comma_Pos(p1,9);								//得到UTC日期
		if(posx!=0XFF)
		{
			temp=NMEA_Str2num(p1+posx,&dx);		 				//得到UTC日期
			gpsx->utc.date=temp/10000;
			gpsx->utc.month=(temp/100)%100;
			gpsx->utc.year=2000+temp%100;	
			//gpsx->utc.year=temp%100;		
		} 
	}
}

  
//提取NMEA-0183信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void GPS_Analysis(nmea_msg *gpsx,uint8_t *buf)
{
	NMEA_GNGGA_Analysis(gpsx,buf);	//GNGGA解析 	
	NMEA_GNRMC_Analysis(gpsx,buf);	//GPNMC解析
}

//GPS校验和计算
//buf:数据缓存区首地址
//len:数据长度
//cka,ckb:两个校验结果.
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
  * 函数功能: 判断闰年(仅针对于2000以后的年份) 
  * 输入参数: iYear    两位年数
  * 返 回 值: uint8_t        1:为闰年    0:为平年 
  * 说    明：无
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
  * 函数功能: 格林尼治时间换算世界各时区时间 
  * 输入参数: *DT:表示日期时间的数组 格式 YY,MM,DD,HH,MM,SS
  * 返 回 值: 无
  * 说    明：AREA:1(+)东区 W0(-)西区   GMT:时区数 
  */
void    GMTconvert(nmea_time *SourceTime, nmea_time *ConvertTime, uint8_t GMT,uint8_t AREA) 
{ 
    uint32_t    YY,MM,DD,hh,mm,ss,ms;        //年月日时分秒暂存变量 
     
    if(GMT==0)    return;                //如果处于0时区直接返回 
    if(GMT>12)    return;                //时区最大为12 超过则返回         

    YY    =    SourceTime->year;                //获取年 
    MM    =    SourceTime->month;                 //获取月 
    DD    =    SourceTime->date;                 //获取日 
    hh    =    SourceTime->hour;                //获取时 
    mm    =    SourceTime->min;                 //获取分 
    ss    =    SourceTime->sec;                 //获取秒 
    ms    =    SourceTime->hsec;               //毫秒 
	
    if(AREA)                        //东(+)时区处理 
    { 
        if(hh+GMT<24)    hh    +=    GMT;//如果与格林尼治时间处于同一天则仅加小时即可 
        else                        //如果已经晚于格林尼治时间1天则进行日期处理 
        { 
            hh    =    hh+GMT-24;        //先得出时间 
            if(MM==1 || MM==3 || MM==5 || MM==7 || MM==8 || MM==10)    //大月份(12月单独处理) 
            { 
                if(DD<31)    DD++; 
                else 
                { 
                    DD    =    1; 
                    MM    ++; 
                } 
            } 
            else if(MM==4 || MM==6 || MM==9 || MM==11)                //小月份2月单独处理) 
            { 
                if(DD<30)    DD++; 
                else 
                { 
                    DD    =    1; 
                    MM    ++; 
                } 
            } 
            else if(MM==2)    //处理2月份 
            { 
                if((DD==29) || (DD==28 && IsLeapYear(YY)==0))        //本来是闰年且是2月29日 或者不是闰年且是2月28日 
                { 
                    DD    =    1; 
                    MM    ++; 
                } 
                else    DD++; 
            } 
            else if(MM==12)    //处理12月份 
            { 
                if(DD<31)    DD++; 
                else        //跨年最后一天 
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
        if(hh>=GMT)    hh    -=    GMT;    //如果与格林尼治时间处于同一天则仅减小时即可 
        else                        //如果已经早于格林尼治时间1天则进行日期处理 
        { 
            hh    =    hh+24-GMT;        //先得出时间 
            if(MM==2 || MM==4 || MM==6 || MM==8 || MM==9 || MM==11)    //上月是大月份(1月单独处理) 
            { 
                if(DD>1)    DD--; 
                else 
                { 
                    DD    =    31; 
                    MM    --; 
                } 
            } 
            else if(MM==5 || MM==7 || MM==10 || MM==12)                //上月是小月份2月单独处理) 
            { 
                if(DD>1)    DD--; 
                else 
                { 
                    DD    =    30; 
                    MM    --; 
                } 
            } 
            else if(MM==3)    //处理上个月是2月份 
            { 
                if((DD==1) && IsLeapYear(YY)==0)                    //不是闰年 
                { 
                    DD    =    28; 
                    MM    --; 
                } 
                else    DD--; 
            } 
            else if(MM==1)    //处理1月份 
            { 
                if(DD>1)    DD--; 
                else        //新年第一天 
                {               
                    DD    =    31; 
                    MM    =    12; 
                    YY    --; 
                } 
            } 
        } 
    }         

    ConvertTime->year   =    YY;                //更新年 
    ConvertTime->month  =    MM;                //更新月 
    ConvertTime->date   =    DD;                //更新日 
    ConvertTime->hour   =    hh;                //更新时 
    ConvertTime->min    =    mm;                //更新分 
    ConvertTime->sec    =    ss;                //更新秒 
    ConvertTime->hsec    =   ms;                //更新秒 

}  
uint64_t GetTimestamp(nmea_time *Time)
{
  const uint8_t mon_table[12]={31,28,31,30,31,30,31,31,30,31,30,31};//平年平月天数表
  const uint8_t leap_mon_table[12]={31,29,31,30,31,30,31,31,30,31,30,31};//闰年闰月天数表
  uint16_t i= 0;
  uint64_t time_tamp_ms,time_tamp_ss;
  uint32_t    YY,MM,DD,hh,mm,ss,ms,days_sum=0;        //年月日时分秒暂存变量 
       
  YY    =    Time->year;                //获取年 
  MM    =    Time->month;                 //获取月 
  DD    =    Time->date;                 //获取日 
  hh    =    Time->hour;                //获取时 
  mm    =    Time->min;                 //获取分 
  ss    =    Time->sec;                 //获取秒 
  ms    =    Time->hsec;               //毫秒 
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
				
	days_sum +=DD;//---------------不知道对不对(uint64_t)
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
////				printf("定位成功\r\n");				
////			}else
////					{
////					 printf("定位失败\r\n");
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
