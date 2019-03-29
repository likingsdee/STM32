#ifndef __BY8301_H
#define __BY8301_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "main.h"

#define Audio_SW_OPEN                      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET)
#define Audio_SW_CLOSE                     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET)
   
#define BY8301_Playing                                             0x01    //播放                       
#define BY8301_Pause                                               0x02    //暂停                      
#define BY8301_Volume_plus                                         0x05     //音量加                    
#define BY8301_Volume_reduction                                    0x06//音量减                     
#define BY8301_Standby_normal_work                                 0x07//待机/正常工作              
#define BY8301_Stop                                                0x0E //停止                      
#define BY8301_Set_volume                                          0x31 //设置音量                  
#define BY8301_Select_play_track                                   0x41 //选择播放曲目               
#define BY8301_Query_playback_status                               0x10 //查询播放状态               
#define BY8301_Query_the_volume_level                              0x11 //查询音量大小               
#define BY8301_Query_the_time_of_the_current_song                  0x1C //查询当前播放歌曲的时间     
#define BY8301_Query_the_total_time_of_the_currently_playing_song  0x1D //查询当前播放歌曲总时间   
   


void BY8301_Send_Cmd(uint8_t cmd);//不带参数指令
void BY8301_Send_Cmd_Parameter(uint8_t cmd,uint16_t parameter);//带参数指令
void BY8301_Play_Voice(uint16_t voice_num);
#ifdef __cplusplus
}
#endif
#endif /*__ pinoutConfig_H */


