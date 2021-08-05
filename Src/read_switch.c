/***********************************************************************************************************
   ___  _    _ _     _                  
  / _ \| |__(_) |_  | |_ ___ __ _ _ __  
 | (_) | '_ \ |  _| |  _/ -_) _` | '  \ 
  \___/|_.__/_|\__|  \__\___\__,_|_|_|_|

Hanoi, Creartive by UP, 20/7/2019
Xbus Rs485 
Using STM8S003F3P6 + Slock AKS
-------------------------------------------------------------------------------------------------------------------------------------------
Code by Chu Tien Thinh  
--------------------------------------------------------------------------------------------------------------------------------------------
tienthinh.obit@gmail.com
chutienthinh.bk@gmail.com
+84981762826
https://www.facebook.com/thinh.chutien                                   
https://www.facebook.com/obit.team/
https://obitvn.wordpress.com/
************************************************************************************************************/
#include "read_switch.h"
#include <stdlib.h>

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

typedef struct
{
  uint16_t ValueReadFromADC;
  uint8_t HexRaw;

}decdoder_value_type;

 decdoder_value_type encoder_table [16] ={
 {0,0x00},
 {2640,0x01},
 {1225,0x02},
 {2827,0x03},
 {518,0x04},
 {2714,0x05},
 {1513,0x06},
 {2883,0x07},
 {190,0x08},
 {2671,0x09},
 {1349,0x0A},
 {2851,0x0B},
 {712,0x0C},
 {2742,0x0D},
 {1614,0x0E},
 {2905,0x0F},
};





uint8_t GetLocationMinValue(uint16_t adc_value)
{
  uint16_t min_value = adc_value;
  uint8_t location_value_in_table=0;
  uint16_t scratch_paper;
  for(int i=0; i<16; i++)
  {
    scratch_paper = abs(adc_value - encoder_table[i].ValueReadFromADC);
    if(min_value > scratch_paper) 
      {
       min_value = scratch_paper;
       location_value_in_table = i;
      }
  }
  return location_value_in_table;
}


uint8_t GetAddressBinaryFromADCValue(uint16_t adc_value)
{
  uint8_t location_value_in_table;
  location_value_in_table = GetLocationMinValue(adc_value);
  return encoder_table[location_value_in_table].HexRaw;
}



uint8_t AddressBoard()  
{
	static uint8_t scan=0;
  uint8_t msb,lsb, get;
	static uint16_t adc_value[2];
  if(scan==0)
	{
  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_value,2);
	scan=1;
	}
  msb = GetAddressBinaryFromADCValue(adc_value[0]);
  lsb  = GetAddressBinaryFromADCValue(adc_value[1]);

  msb = msb<<4;
  msb= msb&0xf0;
  get= msb|lsb;
  return get;
}
