#ifndef __GENFENCE_H
#define __GENFENCE_H
#include "stm32f1xx_hal.h" 

#define PI                      3.1415926
#define EARTH_RADIUS            6378.137        //µØÇò½üËÆ°ë¾¶

#define GENFENCE_1_RADII        200             //Ô²ÐÎÎ§À¸°ë¾¶£¬µ¥Î»cm

__packed typedef struct  
{										    
 	uint16_t header;                   //0x69 0x79					
	uint16_t classid;                  //0x06 0x69						
	uint16_t dlength;	

        uint8_t  version;
        uint8_t  numFences;
        uint8_t  confLvl;
        uint8_t  reserved1;
        uint8_t  pioEnabled;
        uint8_t  pinPolarity;
        uint8_t  pin;
        uint8_t  reserved2;

        int32_t lat;
        int32_t lon;
        int32_t radius;
        uint8_t id;
       
	uint8_t cka;		 									 	 
	uint8_t ckb;	 			
}_to_ser_geofence_msg;

__packed typedef struct  
{										    
 	uint16_t header;                   //0x69 0x79				
	uint16_t classid;                  //0x01 0x08					
	uint16_t dlength;	

        uint8_t  id;
        uint8_t  reserved1;       
        uint8_t  info;
       
	uint8_t cka;		 									 	 
	uint8_t ckb;	 			
}_to_ser_worning_msg;

typedef struct tagST_POINT {
    int64_t x;
    int64_t y;
} ST_POINT;

extern int PtInPolygon(ST_POINT p, ST_POINT pt[], int nCount);
extern double get_distance(double lat1, double lng1, double lat2, double lng2);

#endif