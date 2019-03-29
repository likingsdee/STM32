#ifndef __GPS_H_
#define __GPS_H_
#include "stm32f1xx_hal.h"   

//GPS NMEA-0183Э����Ҫ�����ṹ�嶨�� 
//������Ϣ


__packed typedef struct  
{										    
 	uint8_t num;		//���Ǳ��
	uint8_t eledeg;	//��������
	uint16_t azideg;	//���Ƿ�λ��
	uint8_t sn;		//�����		   
}nmea_slmsg;  
//UTCʱ����Ϣ
__packed typedef struct  
{										    
 	uint16_t year;	//���
	uint8_t  month;	//�·�
	uint8_t  date;	//����
	uint8_t  hour; 	//Сʱ
	uint8_t  min; 	//����
	uint8_t  sec; 	//����
	uint16_t  hsec;  //����
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

//NMEA 0183 Э����������ݴ�Žṹ��
__packed typedef struct  
{										    
 	uint8_t svnum;					//�ɼ�������
	nmea_slmsg slmsg[12];		//���12������
	nmea_time utc;			//UTCʱ��
	uint32_t latitude;				//γ�� ������1000000��,ʵ��Ҫ����1000000
	uint8_t nshemi;					//��γ/��γ,N:��γ;S:��γ				  
	uint32_t longitude;			    //���� ������1000000��,ʵ��Ҫ����1000000
	uint8_t ewhemi;					//����/����,E:����;W:����
	uint8_t gpssta;					//GPS״̬:0,δ��λ;1,�ǲ�ֶ�λ;2,��ֶ�λ;6,���ڹ���.	
	uint8_t locationsta;     //Location state A��Ч��λ V��Ч��λ
 	uint8_t posslnum;				//���ڶ�λ��������,0~12.
 	uint8_t possl[12];				//���ڶ�λ�����Ǳ��
	uint8_t fixmode;					//��λ����:1,û�ж�λ;2,2D��λ;3,3D��λ
	uint16_t pdop;					//λ�þ������� 0~500,��Ӧʵ��ֵ0~50.0
	uint16_t hdop;					//ˮƽ�������� 0~500,��Ӧʵ��ֵ0~50.0
	uint16_t vdop;					//��ֱ�������� 0~500,��Ӧʵ��ֵ0~50.0 
        uint16_t direction;    //���溽��0-359.9�ȣ�0����
	int altitude;			 	//���θ߶�,�Ŵ���10��,ʵ�ʳ���10.��λ:0.1m	 
	uint16_t speed;					//��������,�Ŵ���1000��,ʵ�ʳ���1000.��λ:0.001����/Сʱ	Location state
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
        
        uint8_t  id;						                //�豸ID
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

 



