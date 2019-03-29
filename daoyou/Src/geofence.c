#include "stdio.h"	 
#include "stdarg.h"	 
#include "string.h"	 
#include "math.h"
#include "geofence.h"
#include "stm32f1xx_hal.h"



uint32_t max(uint32_t a,uint32_t b)
{
  if(a>b)
  return a;
  else
  return b;
}

uint32_t min(uint32_t a,uint32_t b)
{
  if(a<b)
  return a;
  else
  return b;
}
//���Ƿ��ڶ������
int PtInPolygon(ST_POINT p, ST_POINT* ptPolygon, int nCount) 
{ 
    int nCross = 0, i;
    double x;
    ST_POINT p1, p2;
    
    for (i = 0; i < nCount; i++) 
    { 
        p1 = ptPolygon[i]; 
        p2 = ptPolygon[(i + 1) % nCount];
        // ��� y=p.y �� p1p2 �Ľ���
        if ( p1.y == p2.y ) // p1p2 �� y=p.yƽ�� 
            continue;
        if ( p.y < min(p1.y, p2.y) ) // ������p1p2�ӳ����� 
            continue; 
        if ( p.y >= max(p1.y, p2.y) ) // ������p1p2�ӳ����� 
            continue;
        // �󽻵�� X ���� -------------------------------------------------------------- 
        x = (double)(p.y - p1.y) * (double)(p2.x - p1.x) / (double)(p2.y - p1.y) + p1.x;
        if ( x > p.x ) 
        {
            nCross++; // ֻͳ�Ƶ��߽��� 
        }
    }
    // ���߽���Ϊż�������ڶ����֮�� --- 
    return (nCross % 2 == 1); 

}

// �󻡶�
double radian(double d)
{
    return d * PI / 180.0;   //�Ƕ�1? = �� / 180
}

//�������
double get_distance(double lat1, double lng1, double lat2, double lng2)
{
    double radLat1 = radian(lat1);
    double radLat2 = radian(lat2);
    double a = radLat1 - radLat2;
    double b = radian(lng1) - radian(lng2);
    
    double dst = 2 * asin((sqrt(pow(sin(a / 2), 2) + cos(radLat1) * cos(radLat2) * pow(sin(b / 2), 2) )));
    
    dst = dst * EARTH_RADIUS;
    dst= round(dst * 10000) / 10000;
    return dst;
}