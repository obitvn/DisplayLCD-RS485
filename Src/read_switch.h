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
#ifndef __READ_SWITCH_H
#define __READ_SWITCH_H

#include "stm32f1xx_hal.h"


#define VREF 3300







uint8_t GetLocationMinValue(uint16_t adc_value);
uint8_t GetAddressBinaryFromADCValue(uint16_t adc_value);
uint8_t AddressBoard();
#endif