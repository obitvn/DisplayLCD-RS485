
#include "stm32f1xx_hal.h"

#define GPIO_PIN_SET    1
#define GPIO_PIN_RESET  0
#define RS_Port GPIOA
#define EN_Port GPIOA
#define D4_Port GPIOA
#define D5_Port GPIOA
#define D6_Port GPIOA
#define D7_Port GPIOA


void LCD_Enable()
{
HAL_GPIO_WritePin(GPIOA,EN_Pin,1);
	for(int i=0; i<300;i++);
//HAL_Delay(1);
HAL_GPIO_WritePin(GPIOA,EN_Pin,0);  
	for(int i=0; i<300;i++);
//HAL_Delay(1); 
}

void LCD_Send4Bit(unsigned char Data)
{
HAL_GPIO_WritePin(GPIOA,D4_Pin,Data&0x01);
HAL_GPIO_WritePin(GPIOA,D5_Pin,(Data>>1)&0x01);
HAL_GPIO_WritePin(GPIOA,D6_Pin,(Data>>2)&0x01);
HAL_GPIO_WritePin(GPIOA,D7_Pin,(Data>>3)&0x01); 
}

void LCD_SendCommand(unsigned char command)
{
  LCD_Send4Bit(command >>4);/* Gui 4 bit cao */
  LCD_Enable();
  LCD_Send4Bit(command);  /* Gui 4 bit thap*/
  LCD_Enable();
}
/* USER CODE END 0 */
void LCD_Clear()
{
  LCD_SendCommand(0x01); 
  for(int i=0; i<300;i++);	
  //HAL_Delay(1); 
}

void LCD_Init()
{
	HAL_GPIO_WritePin(LCD_RW_GPIO_Port,LCD_RW_Pin,0);  
  LCD_Send4Bit(0x00);
  HAL_GPIO_WritePin(GPIOA,RS_Pin,0);
  LCD_Send4Bit(0x03);
  LCD_Enable();
  LCD_Enable();
  LCD_Enable();
  LCD_Send4Bit(0x02);
  LCD_Enable();
  LCD_SendCommand(0x28); // giao thuc 4 bit, hien thi 2 hang, ki tu 5x8
  LCD_SendCommand(0x0C); // cho phep hien thi man hinh
  LCD_SendCommand(0x06); // tang ID, khong dich khung hinh
  LCD_SendCommand(0x01); // xoa toan bo khung hinh
}

void LCD_Gotoxy(unsigned char x, unsigned char y)
{
  unsigned char address;
    if(!y)address=(0x80+x);
    else address=(0xC0+x);
    LCD_SendCommand(address);

}

void LCD_PutChar(unsigned char Data)
{
  HAL_GPIO_WritePin(GPIOA,RS_Pin,1);
  LCD_SendCommand(Data);
  HAL_GPIO_WritePin(GPIOA,RS_Pin,0);
}

void LCD_Puts(char *s)
{
    while (*s){
        LCD_PutChar(*s);
      s++;
    }
}