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
//求是否在多边形内
int PtInPolygon(ST_POINT p, ST_POINT* ptPolygon, int nCount) 
{ 
    int nCross = 0, i;
    double x;
    ST_POINT p1, p2;
    
    for (i = 0; i < nCount; i++) 
    { 
        p1 = ptPolygon[i]; 
        p2 = ptPolygon[(i + 1) % nCount];
        // 求解 y=p.y 与 p1p2 的交点
        if ( p1.y == p2.y ) // p1p2 与 y=p.y平行 
            continue;
        if ( p.y < min(p1.y, p2.y) ) // 交点在p1p2延长线上 
            continue; 
        if ( p.y >= max(p1.y, p2.y) ) // 交点在p1p2延长线上 
            continue;
        // 求交点的 X 坐标 -------------------------------------------------------------- 
        x = (double)(p.y - p1.y) * (double)(p2.x - p1.x) / (double)(p2.y - p1.y) + p1.x;
        if ( x > p.x ) 
        {
            nCross++; // 只统计单边交点 
        }
    }
    // 单边交点为偶数，点在多边形之外 --- 
    return (nCross % 2 == 1); 

}

// 求弧度
double radian(double d)
{
    return d * PI / 180.0;   //角度1? = π / 180
}

//计算距离
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