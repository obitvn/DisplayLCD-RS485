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
#include "Rs485.h"

extern UART_HandleTypeDef huart1;
extern uint8_t rx_data;

status_BUS UartState;

uint8_t is_device, data_correct;

uint32_t timeout_uart, time_last_uart;
volatile uint8_t UartData, AddSendPack;
extern uint8_t AddDevice;
uint16_t DataLength;
uint8_t DataBuff[RS485_UART_BUFFER_MAX_SIZE], data_index;
extern uint8_t ADD_MASTER;

void UART_CallBack()
{
  UartData=0;
  UartData = rx_data;
	rx_data=0;
  //timeout_uart = GetMicroTick() - time_last_uart;
  switch (UartState)
  {
    case WAIT_FOR_FLAG_START:
      if((UartData=='X')) UartState = CHECK_B;
      else UartState = WAIT_FOR_FLAG_START;
      break;
      
    case CHECK_B:
      if((UartData=='B')) UartState = CHECK_U;
      else UartState = WAIT_FOR_FLAG_START;
      break;
      
    case CHECK_U:
      if((UartData=='U')) UartState = CHECK_S;
      else UartState = WAIT_FOR_FLAG_START;
      break;
      
    case CHECK_S:
      if((UartData=='S')) UartState = CHECK_ADD_RECEVICE;
      else UartState = WAIT_FOR_FLAG_START;
      break;;
      
    case CHECK_ADD_RECEVICE:
      if(AddDevice == UartData) is_device=1;
      else is_device=0;
      UartState = SAVE_ADD_SEND;
      break;
      
    case SAVE_ADD_SEND:
      AddSendPack = UartData;
      UartState = READ_DATA_LEN_MSB;
      break;
      
   case READ_DATA_LEN_MSB:
      DataLength = UartData;
      UartState = READ_DATA_LEN_LSB;
      break;
      
   case READ_DATA_LEN_LSB:
      DataLength = (DataLength<<8)|UartData;
      UartState = READ_DATA;
      data_index=0;
	    for(int i=0; i<RS485_UART_BUFFER_MAX_SIZE; i++)
	    {
				DataBuff[i]=0;
	    }
      break;
      
   case READ_DATA:
      if((data_index < DataLength)&&(DataLength < RS485_UART_BUFFER_MAX_SIZE))
      {
				
        DataBuff[data_index++] = UartData;
        
        if(data_index == DataLength) UartState = CHECK_FLAG_STOP_CR;
        else UartState = READ_DATA;
      }
      else UartState = WAIT_FOR_FLAG_START;
      break;
      
   case CHECK_FLAG_STOP_CR:
      if(UartData == 13) // \r
      {
        UartState = CHECK_FLAG_STOP_LF;
      }
      else UartState = WAIT_FOR_FLAG_START;
      break;
      
   case CHECK_FLAG_STOP_LF:
      if((UartData == 10)&&(is_device)) // \n
      {
        data_correct=1;
      }
      else data_correct=0;
      UartState = WAIT_FOR_FLAG_START;
      break;
  }
	HAL_UART_Receive_IT(&huart1,&rx_data,1);
}

void ProcessBuffInWhileLoop()
{
	uint8_t ContentBuff[RS485_UART_BUFFER_MAX_SIZE-2];
	for(int i=0; i<RS485_UART_BUFFER_MAX_SIZE-2; i++)
			  {
					ContentBuff[i]=0;
				}
				
				
	if(data_correct==1)
	{
		for(int i=2; i<RS485_UART_BUFFER_MAX_SIZE; i++)
		{
			ContentBuff[i-2] = DataBuff[i];
		}
		if(DataBuff[0] == 0x11) // write LCD
		{
				uint8_t x,y;
				x = DataBuff[1]&0xF0;
				x= x>>4;
				y = DataBuff[1]&0x0F;
				LCD_Gotoxy(x,y);
		    LCD_Puts((char*)ContentBuff);
		}
		else if (DataBuff[0] == 0x12)
		{
			if(DataBuff[1] == 0xe1) 
			{
				LCD_Gotoxy(0,0);
		    LCD_Puts("                ");
			}
			else if(DataBuff[1] == 0xe2) 
			{
				LCD_Gotoxy(0,1);
		    LCD_Puts("                ");
			}
			else if(DataBuff[1] == 0xe0) 
			{
				LCD_Clear();
			}
		}
		else if (DataBuff[0] == 0x22) // control led
		{
			if(DataBuff[1] == 0x00) 
			{
				HAL_GPIO_WritePin(LED_SOS_GPIO_Port,LED_SOS_Pin,DataBuff[2]);
				HAL_GPIO_WritePin(LED_BUSY_GPIO_Port,LED_BUSY_Pin,DataBuff[3]);
				HAL_GPIO_WritePin(LED_NETWORK_GPIO_Port,LED_NETWORK_Pin,DataBuff[4]);
				HAL_GPIO_WritePin(RELAY_LED1_GPIO_Port,RELAY_LED1_Pin,DataBuff[5]);
        HAL_GPIO_WritePin(RELAY_LED2_GPIO_Port,RELAY_LED2_Pin,DataBuff[6]);
				HAL_GPIO_WritePin(RELAY_LED3_GPIO_Port,RELAY_LED3_Pin,DataBuff[7]);
				HAL_GPIO_WritePin(RELAY_LED4_GPIO_Port,RELAY_LED4_Pin,DataBuff[8]);
				HAL_GPIO_WritePin(SPK_GPIO_Port,SPK_Pin,!DataBuff[9]);		
			}
		}
		for(int i=0; i<RS485_UART_BUFFER_MAX_SIZE; i++)
			  {
					DataBuff[i]=0;
				}
		data_correct=0;
		is_device=0;
	}
	
	if(HAL_GPIO_ReadPin(BUTTON1_GPIO_Port,BUTTON1_Pin)==0)
	{
		while((HAL_GPIO_ReadPin(BUTTON1_GPIO_Port,BUTTON1_Pin)==0)&&(data_correct==0)){};
		uint32_t time_start = HAL_GetTick();
	  while(ButtonSendToRs485(ADD_MASTER,0))
    {
			if((HAL_GetTick() - time_start)>1000) break;
		}
	}
	
	if(HAL_GPIO_ReadPin(BUTTON2_GPIO_Port,BUTTON2_Pin)==0)
	{
		while((HAL_GPIO_ReadPin(BUTTON2_GPIO_Port,BUTTON2_Pin)==0)&&(data_correct==0)){};
		uint32_t time_start = HAL_GetTick();
	  while(ButtonSendToRs485(ADD_MASTER,1))
    {
			if((HAL_GetTick() - time_start)>1000) break;
		}
	}
	
	if(HAL_GPIO_ReadPin(BUTTON3_GPIO_Port,BUTTON3_Pin)==0)
	{
		while((HAL_GPIO_ReadPin(BUTTON3_GPIO_Port,BUTTON3_Pin)==0)&&(data_correct==0)){};
		uint32_t time_start = HAL_GetTick();
	  while(ButtonSendToRs485(ADD_MASTER,2))
    {
			if((HAL_GetTick() - time_start)>1000) break;
		}
	}
	
	if(HAL_GPIO_ReadPin(BUTTON4_GPIO_Port,BUTTON4_Pin)==0)
	{
		while((HAL_GPIO_ReadPin(BUTTON4_GPIO_Port,BUTTON4_Pin)==0)&&(data_correct==0)){};
		uint32_t time_start = HAL_GetTick();
	  while(ButtonSendToRs485(ADD_MASTER,3))
    {
			if((HAL_GetTick() - time_start)>1000) break;
		}
	}
	
	if(HAL_GPIO_ReadPin(BUTTON5_GPIO_Port,BUTTON5_Pin)==0)
	{
		while((HAL_GPIO_ReadPin(BUTTON5_GPIO_Port,BUTTON5_Pin)==0)&&(data_correct==0)){};
		uint32_t time_start = HAL_GetTick();
	  while(ButtonSendToRs485(ADD_MASTER,4))
    {
			if((HAL_GetTick() - time_start)>1000) break;
		}
	}
}

void ProcessRs485(void)
{
	if(data_correct)
	{
		
		data_correct =0;
	}
}


void UART_PutChar(uint8_t c)
{
  
  HAL_UART_Transmit(&huart1, &c, 1, 10);

  
}


void UART_Puts(uint8_t *data, uint16_t len_data)
{
 HAL_UART_Transmit(&huart1, data, len_data, 10);
}

void AddDeviceToText(uint8_t add)
{
	uint8_t adr = add;
  static uint8_t add_lsb, add_msb;
  add_msb = add >> 4;
  if(add_msb < 10) UART_PutChar(add_msb +48);
    else UART_PutChar(add_msb + 55); 
  add_lsb =add<<4;
  add_lsb = add_lsb >> 4;  
  if(add_lsb < 10) UART_PutChar(add_lsb +48);
    else UART_PutChar(add_lsb + 55); 
}

uint8_t ButtonSendToRs485(uint8_t add_recevice_pack, uint8_t data)
{
	uint8_t first_name[] ="XBUS";
  if( UartState == WAIT_FOR_FLAG_START)
  {
		HAL_GPIO_WritePin(RS485_DIR_GPIO_Port,RS485_DIR_Pin,1);
		HAL_Delay(5);
    UART_Puts(first_name,4);
    UART_PutChar(add_recevice_pack);
    UART_PutChar(AddDevice);
    UART_PutChar(0x00);
		UART_PutChar(0x0a);
		UART_Puts("\r\nID:",5);     
		AddDeviceToText(AddDevice+data);
    UART_PutChar('@');
    AddDeviceToText(AddDevice);
    UART_PutChar(13);
    UART_PutChar(10);
		HAL_Delay(5);
		HAL_GPIO_WritePin(RS485_DIR_GPIO_Port,RS485_DIR_Pin,0);
    UartState = WAIT_FOR_FLAG_START;
    return 0;
  }
  else return 1;
}
