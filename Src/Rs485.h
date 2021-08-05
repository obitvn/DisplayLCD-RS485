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
#ifndef __RS485_H
#define __RS485_H

#include "stm32f1xx_hal.h"


#define RS485_UART_BUFFER_MAX_SIZE      128

typedef enum {
  WAIT_FOR_FLAG_START,
  CHECK_B,
  CHECK_U,
  CHECK_S,
  SAVE_ADD_SEND,
  CHECK_ADD_RECEVICE,
  READ_DATA_LEN_MSB,
  READ_DATA_LEN_LSB,
  READ_DATA,
  CHECK_FLAG_STOP_CR,
  CHECK_FLAG_STOP_LF,
} status_BUS;

extern status_BUS UartState;

void UART_Config(void);
void UART_Puts(uint8_t *data, uint16_t len_data);
void UART_CallBack(void);
void UART_PutChar(uint8_t c);
void ProcessBuffInWhileLoop(void);


uint8_t ButtonSendToRs485(uint8_t add_recevice_pack, uint8_t val);

#endif