#ifndef __GPS_H_
#define __GPS_H_
#include "stm32f1xx_hal.h"   

//GPS NMEA-0183协议重要参数结构体定义 
//卫星信息


__packed typedef struct  
{										    
 	uint8_t num;		//卫星编号
	uint8_t eledeg;	//卫星仰角
	uint16_t azideg;	//卫星方位角
	uint8_t sn;		//信噪比		   
}nmea_slmsg;  
//UTC时间信息
__packed typedef struct  
{										    
 	uint16_t year;	//年份
	uint8_t  month;	//月份
	uint8_t  date;	//日期
	uint8_t  hour; 	//小时
	uint8_t  min; 	//分钟
	uint8_t  sec; 	//秒钟
	uint16_t  hsec;  //毫秒
}nmea_time;  

__packed typedef struct  
{										    
        uint32_t iTOW;
        uint16_t year;
        uint8_t  month;
        uint8_t  day;
        uint8_t  hour;
        uint8_t  min;
        uint8_t  sec;
        uint8_t  valid;
        uint32_t tAcc;
        int32_t  nano;
}pvt_time; 

//NMEA 0183 协议解析后数据存放结构体
__packed typedef struct  
{										    
 	uint8_t svnum;					//可见卫星数
	nmea_slmsg slmsg[12];		//最多12颗卫星
	nmea_time utc;			//UTC时间
	uint32_t latitude;				//纬度 分扩大1000000倍,实际要除以1000000
	uint8_t nshemi;					//北纬/南纬,N:北纬;S:南纬				  
	uint32_t longitude;			    //经度 分扩大1000000倍,实际要除以1000000
	uint8_t ewhemi;					//东经/西经,E:东经;W:西经
	uint8_t gpssta;					//GPS状态:0,未定位;1,非差分定位;2,差分定位;6,正在估算.	
	uint8_t locationsta;     //Location state A有效定位 V无效定位
 	uint8_t posslnum;				//用于定位的卫星数,0~12.
 	uint8_t possl[12];				//用于定位的卫星编号
	uint8_t fixmode;					//定位类型:1,没有定位;2,2D定位;3,3D定位
	uint16_t pdop;					//位置精度因子 0~500,对应实际值0~50.0
	uint16_t hdop;					//水平精度因子 0~500,对应实际值0~50.0
	uint16_t vdop;					//垂直精度因子 0~500,对应实际值0~50.0 
        uint16_t direction;    //地面航向0-359.9度，0正北
	int altitude;			 	//海拔高度,放大了10倍,实际除以10.单位:0.1m	 
	uint16_t speed;					//地面速率,放大了1000倍,实际除以1000.单位:0.001公里/小时	Location state
	double flat;
	double flon;
	double falt;
}nmea_msg; 
 
__packed typedef struct  
{										    
 	uint16_t header;//6F79					
	uint16_t classid;//0107						
	uint16_t dlength;//96	

        pvt_time utc;

        uint8_t  fixType;

        uint8_t  gnssFixOK:1;  //uint8_t  flags;
        uint8_t  diffSoln:1;
        uint8_t  psmState:3;
        uint8_t  headVehValid:1;
        uint8_t  carrSoln:2;
        
        uint8_t  flags2;
        uint8_t  numSV;
        int32_t  lon;
        int32_t  lat;
        int32_t  height;
        int32_t  hMSL;
        uint32_t hAcc;
        uint32_t vAcc;
        int32_t  velN;
        int32_t  velE;
        int32_t  velD;
        int32_t  gSpeed;
        int32_t  headMot;
        uint32_t sAcc;
        uint32_t headAcc;
        uint16_t pDOP;
        uint8_t  reserved1[6];
        int32_t  headVeh;
        uint8_t  reserved2[4];
        
        uint8_t  id;						                //设备ID
        int32_t  xAngRate;
        int32_t  yAngRate;
        int32_t  zAngRate;     
        int32_t  xAccel;
        int32_t  yAccel;
        int32_t  zAccel;                                               
        
	uint8_t cka;		 									 	 
	uint8_t ckb;	 			
}_pvt_msg; 










			 
int NMEA_Str2num(uint8_t *buf,uint8_t*dx);
void GPS_Analysis(nmea_msg *gpsx,uint8_t *buf);
void NMEA_GPGSV_Analysis(nmea_msg *gpsx,uint8_t *buf);
void NMEA_GPGGA_Analysis(nmea_msg *gpsx,uint8_t *buf);
void NMEA_GNGGA_Analysis(nmea_msg *gpsx,uint8_t *buf);
void NMEA_GPGSA_Analysis(nmea_msg *gpsx,uint8_t *buf);
void NMEA_GPGSA_Analysis(nmea_msg *gpsx,uint8_t *buf);
void NMEA_GPRMC_Analysis(nmea_msg *gpsx,uint8_t *buf);
void NMEA_GNRMC_Analysis(nmea_msg *gpsx,uint8_t *buf);
void NMEA_GPVTG_Analysis(nmea_msg *gpsx,uint8_t *buf);

void GPS_Update(void);
void GMTconvert(nmea_time *SourceTime, nmea_time *ConvertTime, uint8_t GMT,uint8_t AREA);
static uint8_t IsLeapYear(uint8_t iYear);
uint64_t GetTimestamp(nmea_time *Time);

void Ublox_CheckSum(uint8_t *buf,uint16_t len,uint8_t* cka,uint8_t*ckb);
extern nmea_msg gpsx;	
#endif  

 



