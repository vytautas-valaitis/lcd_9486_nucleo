#include "stm32f7xx_nucleo_144.h"

int main(void)
{
	BSP_LED_Init(LED1);
	BSP_LED_Init(LED2);
	BSP_LED_Init(LED3);

  lcd_gpio_init();
  
  lcd_write_8(0x00);
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_SET); // CS_IDLE;
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET); // RD_IDLE;
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET); // WR_IDLE;
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_5, GPIO_PIN_SET); // RESET_IDLE;
  for(int i = 0; i < 1000; i++) {};
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_5, GPIO_PIN_RESET); // RESET_ACTIVE;
  for(int i = 0; i < 2000; i++) {};
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_5, GPIO_PIN_SET); // RESET_IDLE;
  for(int i = 0; i < 2000; i++) {};
  
  
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_RESET); //CS_ACTIVE;
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET); //CD_COMMAND;
  lcd_write_8(0x11);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET); // WR_ACTIVE;
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET); // WR_IDLE;
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET); //CD_DATA;
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_SET); //CS_IDLE;
  
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_RESET); //CS_ACTIVE;
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET); //CD_COMMAND; 
  lcd_write_8(0x29);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET); // WR_ACTIVE;
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET); // WR_IDLE;
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET); //CD_DATA;
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_SET); //CS_IDLE;
   
	for(;;)
	{
		BSP_LED_Toggle(LED1);
		for(int i = 0; i < 100000; i++) {};
		BSP_LED_Toggle(LED2);
		for(int i = 0; i < 100000; i++) {};
		BSP_LED_Toggle(LED3);
		for(int i = 0; i < 100000; i++) {};
}
}

void lcd_write_8(uint8_t data) {
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, ((data >> 0 & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET));
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, ((data >> 1 & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET));
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, ((data >> 2 & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET));
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, ((data >> 3 & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET));
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, ((data >> 4 & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET));
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, ((data >> 5 & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET));
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, ((data >> 6 & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET));
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, ((data >> 7 & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET));
}
 
void lcd_gpio_init(void) {

  // RD PA3
  // WR PC0
  // RS/CD PC3
  // CS PF3
  // RST PF5
  
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct;  
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  //GPIO_InitTypeDef  GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
  //GPIO_InitTypeDef  GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
  
  // LCD_D1 PD15
  // LCD_D0 PF12
  // LCD_D7 PF13
  // LCD_D6 PE9
  // LCD_D5 PE11
  // LCD_D4 PF14
  // LCD_D3 PE13
  // LCD_D2 PF15
  
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET); 

  GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET); 
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET); 
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_SET); 
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, GPIO_PIN_SET);
}

