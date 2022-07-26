#include "stm32f7xx_nucleo_144.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_it.h"

#define TFTLCD_DELAY8 0x7f


static void SystemClock_Config(void);
static void CPU_CACHE_Enable(void);
static void Error_Handler(void);

static void delay(uint16_t time) {
  for(int i = 0; i < time * 100; i++) {};
}

static void fill_black() {
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_RESET); //CS_ACTIVE;
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET); //CD_COMMAND;
  lcd_write_8(0x2c);
  for(int i = 0; i < 480*320; i++) {       
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET); //CD_DATA;
    //lcd_write_8(0xff); // 18-bit
    //lcd_write_8(0x00);
    //lcd_write_8(0x00);
    
    lcd_write_8(0xf8); // 16-bit
    lcd_write_8(0x00);
    
    //delay(20);
  }
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_SET); //CS_IDLE;
  
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_RESET); //CS_ACTIVE;
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET); //CD_COMMAND;
  lcd_write_8(0x2c);
  for(int i = 0; i < 480*320; i++) {       
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET); //CD_DATA;
    //lcd_write_8(0xff); // 18-bit
    //lcd_write_8(0x00);
    //lcd_write_8(0x00);
    
    lcd_write_8(0x07); // 16-bit
    lcd_write_8(0xe0);
    
    //delay(20);
  }
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_SET); //CS_IDLE;
  
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_RESET); //CS_ACTIVE;
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET); //CD_COMMAND;
  lcd_write_8(0x2c);
  for(int i = 0; i < 480*320; i++) {       
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET); //CD_DATA;
    //lcd_write_8(0xff); // 18-bit
    //lcd_write_8(0x00);
    //lcd_write_8(0x00);
    
    lcd_write_8(0x00); // 16-bit r 5-bit g 6-bit b 5-bit, 65536 colours 
    lcd_write_8(0x1f);
    
    //delay(20);
  }
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_SET); //CS_IDLE;

  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_RESET); //CS_ACTIVE;
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET); //CD_COMMAND;
  lcd_write_8(0x2c);
  for(int i = 0; i < 480*320; i++) {       
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET); //CD_DATA;
    //lcd_write_8(0xff); // 18-bit
    //lcd_write_8(0x00);
    //lcd_write_8(0x00);
    
    lcd_write_8(0x00); // 16-bit
    lcd_write_8(0x00);
    
    //delay(20);
  }
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_SET); //CS_IDLE;
}

static void write_table(const uint8_t *table, int16_t size) {
  uint8_t *p = table;
  while (size > 0) {
    uint8_t cmd = *(p++);
    uint8_t len = *(p++);
    if (cmd == TFTLCD_DELAY8) {
      delay(len);
      len = 0;
    } else {
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_RESET); //CS_ACTIVE;
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET); //CD_COMMAND;
      lcd_write_8(cmd);
      delay(1);
      for (uint8_t d = 0; d++ < len; ) {
        uint8_t x = *(p++);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET); //CD_DATA;
        lcd_write_8(x);
        delay(20);
      }
         HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_SET); //CS_IDLE;
      }
        size -= len + 2;
    }
}

int main(void)
{
  CPU_CACHE_Enable();
  HAL_Init();
  SystemClock_Config();
  /* Output SYSCLK  / 2 on MCO2 pin(PC.09) */
  //HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_SYSCLK, RCC_MCODIV_2);
  
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
  
  static const uint8_t t0[] = {
    0xC0,   2,  0x0d, 0x0d,              // power control 1
    0xC1,   2,  0x43, 0x00,              // power control 2
    0xC2,   1,  0x00,                    // power control 3
    0xC5,   4,  0x00, 0x48, 0x00, 0x48,  // vcom control 1
    0xB4,   1,  0x00,                    // display inversion control
    0xB6,   3,  0x02, 0x02, 0x3B,        // display function control
    0xE0,  15,  0x0F, 0x21, 0x1C, 0x0B, 0x0E, 0x08, 0x49, 0x98, 0x38, 0x09, 0x11, 0x03, 0x14, 0x10, 0x00, // positive gamma control
    0xE1,  15,  0x0F, 0x2F, 0x2B, 0x0C, 0x0E, 0x06, 0x47, 0x76, 0x37, 0x07, 0x11, 0x04, 0x23, 0x1E, 0x00, // negative gamma control
    0x3a,   1,  0x55,        // interface pixel format, 0x55 - 16bit, 0x66 - 18bit.
    0xB6,   2,  0x00, 0x22,  // display function control
    0x36,   1,  0x08,        // memory access control, rotation, 0x08, 0x68, 0xc8, 0xa8
    0x11,   0,               // sleep out
    0x29,   0                // display on
  };
  
  write_table(&t0, sizeof(t0));
  
  fill_black();
  /*
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_RESET); //CS_ACTIVE;
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET); //CD_COMMAND;
  lcd_write_8(0x11);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET); //CD_DATA;
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_SET); //CS_IDLE;
  */
   
	for(;;)
	{
		BSP_LED_Toggle(LED1);
		//HAL_Delay(100);
		for(int i = 0; i < 300000; i++) {};
		//BSP_LED_Toggle(LED2);
		//for(int i = 0; i < 100000; i++) {};
		//BSP_LED_Toggle(LED3);
		//for(int i = 0; i < 100000; i++) {};
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

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET); // WR_ACTIVE;
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET); // WR_IDLE;
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



                                                                           /**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 216000000
  *            HCLK(Hz)                       = 216000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 432
  *            PLL_P                          = 2
  *            PLL_Q                          = 9
  *            PLL_R                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 7
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 432;  
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    while(1) {};
  }
  
  /* Activate the OverDrive to reach the 216 Mhz Frequency */
  if(HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    while(1) {};
  }
  
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    while(1) {};
  }
}




/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED3 on */
  BSP_LED_On(LED3);

  while (1)
  {
  }
}

/**
* @brief  CPU L1-Cache enable.
* @param  None
* @retval None
*/
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}

