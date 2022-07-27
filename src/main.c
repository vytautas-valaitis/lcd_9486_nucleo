#include "stm32f7xx_nucleo_144.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_it.h"
#include "im.h"

#define TFTLCD_DELAY8 0x7f

#define RST_L GPIOF -> BSRR = 32 << 16
#define RST_H GPIOF -> BSRR = 32

#define RD_L GPIOA -> BSRR = 8 << 16
#define RD_H GPIOA -> BSRR = 8

#define DC_C GPIOC -> BSRR = 8 << 16
#define DC_D GPIOC -> BSRR = 8

#define CS_L GPIOF -> BSRR = 8 << 16
#define CS_H GPIOF -> BSRR = 8

#define WR_L GPIOC -> BSRR = 1 << 16
#define WR_H GPIOC -> BSRR = 1
  
#define WR_TWRL WR_L
#define WR_TWRH WR_H; WR_H
#define WR_STB WR_TWRL; WR_TWRH
            
#define D0_PIN_MASK (1UL << 12) // PF12 PF3
#define D1_PIN_MASK (1UL << 15) // PD15
#define D2_PIN_MASK (1UL << 15) // PF15 PG14
#define D3_PIN_MASK (1UL << 13) // PE13
#define D4_PIN_MASK (1UL << 14) // PF14
#define D5_PIN_MASK (1UL << 11) // PE11
#define D6_PIN_MASK (1UL <<  9) // PE9
#define D7_PIN_MASK (1UL << 13) // PF13 PG12
      
#define D0_BSR_MASK(B) ((D0_PIN_MASK << 16) >> (((B) << 4) & 0x10))
#define D1_BSR_MASK(B) ((D1_PIN_MASK << 16) >> (((B) << 3) & 0x10))
#define D2_BSR_MASK(B) ((D2_PIN_MASK << 16) >> (((B) << 2) & 0x10))
#define D3_BSR_MASK(B) ((D3_PIN_MASK << 16) >> (((B) << 1) & 0x10))
#define D4_BSR_MASK(B) ((D4_PIN_MASK << 16) >> (((B) << 0) & 0x10))
#define D5_BSR_MASK(B) ((D5_PIN_MASK << 16) >> (((B) >> 1) & 0x10))
#define D6_BSR_MASK(B) ((D6_PIN_MASK << 16) >> (((B) >> 2) & 0x10))
#define D7_BSR_MASK(B) ((D7_PIN_MASK << 16) >> (((B) >> 3) & 0x10))
      
#define lcd_write_8(C) GPIOF -> BSRR = D0_BSR_MASK(C) | D2_BSR_MASK(C) | D4_BSR_MASK(C) | D7_BSR_MASK(C); \ 
                       WR_L; \       
                       GPIOD -> BSRR = D1_BSR_MASK(C); \
                       GPIOE -> BSRR = D3_BSR_MASK(C) | D5_BSR_MASK(C) | D6_BSR_MASK(C); \
                       WR_STB;
                                                            
static void SystemClock_Config(void);
static void CPU_CACHE_Enable(void);
static void Error_Handler(void);

static void fill_black(void) {
  HAL_Delay(1000);
  CS_L;
  DC_C;
  lcd_write_8(0x2c);
  for(int i = 0; i < 480*320; i++) {       
    DC_D;
    lcd_write_8(0x00); // 16-bit r 5-bit g 6-bit b 5-bit, 65536 colours 
    lcd_write_8(0x00);
  }
  CS_H;
  HAL_Delay(100);
  CS_L;
  DC_C;
  lcd_write_8(0x2c);
   
  for(int i = 0; i < 480*320*2; i++) {       
    DC_D;
    lcd_write_8(im[i]);
  }
  CS_H;
}

static void write_table(const uint8_t *table, int16_t size) {
  uint8_t *p = table;
  while (size > 0) {
    uint8_t cmd = *(p++);
    uint8_t len = *(p++);
    if (cmd == TFTLCD_DELAY8) {
      HAL_Delay(len);
      len = 0;
    } else {
      CS_L;
      DC_C;
      lcd_write_8(cmd);
      //HAL_Delay(1);
      for (uint8_t d = 0; d++ < len; ) {
        uint8_t x = *(p++);
        DC_D;
        lcd_write_8(x);
  //      HAL_Delay(1);
      }
         CS_H;
      }
        size -= len + 2;
    }
}

void lcd_reset(void) {
  DC_D;
  CS_H;
  RD_H;
  WR_H;
  RST_H;
  HAL_Delay(1);
  RST_L;
  HAL_Delay(1);
  RST_H;
  HAL_Delay(1);
}

int main(void) {
  CPU_CACHE_Enable();
  HAL_Init();
  SystemClock_Config();
  HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_SYSCLK, RCC_MCODIV_2); // output SYSCLK / 2 on MCO2 pin (PC.09)
  
	BSP_LED_Init(LED1);
	BSP_LED_Init(LED2);
	BSP_LED_Init(LED3);

  lcd_gpio_init();
  lcd_reset();  

  static const uint8_t t0[] = {
    0xC0,   2,  0x17, 0x15,              // power control 1
    0xC1,   1,  0x41,                    // power control 2
    0xC2,   1,  0x00,                    // power control 3
    0xC5,   3,  0x00, 0x12, 0x80,        // vcom control 1
    0xB4,   1,  0x02,                    // display inversion control
    0xB6,   3,  0x02, 0x02, 0x3B,        // display function control
    0xE0,  15,  0x0F, 0x21, 0x1C, 0x0B, 0x0E, 0x08, 0x49, 0x98, 0x38, 0x09, 0x11, 0x03, 0x14, 0x10, 0x00, // positive gamma control
    0xE1,  15,  0x0F, 0x2F, 0x2B, 0x0C, 0x0E, 0x06, 0x47, 0x76, 0x37, 0x07, 0x11, 0x04, 0x23, 0x1E, 0x00, // negative gamma control
    0x3a,   1,  0x55,        // interface pixel format, 0x55 - 16bit, 0x66 - 18bit.
    0xB6,   2,  0x00, 0x22,  // display function control
    0x36,   1,  0xc8,        // memory access control, mx, bgr, rotation, 0x08, 0x68, 0xc8, 0xa8
    //0x36,   2,  0x08, 0x20, //ladscape
    0xB0,   1,  0x00, // Interface Mode Control
    0xB1,   1,  0xA0, // Frame Rate Control
    0xB7,   1,  0xC6, // Entry Mode Set
    0xF7,   4,  0xA9, 0x51, 0x2C, 0x82, // Adjust Control 3  
    0x11,   0,               // sleep out
    0x29,   0                // display on
  };
  
  write_table(&t0, sizeof(t0));
  
  fill_black();
   
	for(;;)	{
		BSP_LED_Toggle(LED1);
		HAL_Delay(100);
  }
}

void lcd_gpio_init(void) {
  GPIO_InitTypeDef GPIO_InitStruct;
/*
* RD PA3
* WR PC0
* RS/CD/DC PC3
* CS PF3
* RST PF5
*/  

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
  
/*
* LCD_D1 PD15
* LCD_D0 PF12
* LCD_D7 PF13
* LCD_D6 PE9
* LCD_D5 PE11
* LCD_D4 PF14
* LCD_D3 PE13
* LCD_D2 PF15
*/
  
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
}

/*
* System Clock source            = PLL (HSE)
* SYSCLK(Hz)                     = 216000000
* HCLK(Hz)                       = 216000000
* AHB Prescaler                  = 1
* APB1 Prescaler                 = 4
* APB2 Prescaler                 = 2
* HSE Frequency(Hz)              = 8000000
* PLL_M                          = 8
* PLL_N                          = 432
* PLL_P                          = 2
* PLL_Q                          = 9
* PLL_R                          = 7
* VDD(V)                         = 3.3
* Main regulator output voltage  = Scale1 mode
* Flash Latency(WS)              = 7
*/
void SystemClock_Config(void) {
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  
  // Enable HSE Oscillator and activate PLL with HSE as source
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
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    while(1) {};
  }
  
  // Activate the OverDrive to reach the 216 Mhz Frequency
  if(HAL_PWREx_EnableOverDrive() != HAL_OK) {
    while(1) {};
  }
    
  // Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK) {
    while(1) {};
  }
}

// CPU L1-Cache enable.
static void CPU_CACHE_Enable(void) {
  SCB_EnableICache();
  SCB_EnableDCache();
}

static void Error_Handler(void) {
  BSP_LED_On(LED3);
  while(1) {};
}

