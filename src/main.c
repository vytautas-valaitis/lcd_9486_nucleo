// picocom /dev/ttyACM0 -b 115200 --omap delbs --imap lfcrlf
// sudo dd if=/dev/sda of=ff skip=36144 bs=512 count=1
// https://www.liquisearch.com/fatx/technical_design/directory_table/vfat_long_file_names
// https://www.easeus.com/resource/fat32-disk-structure.htm
// https://www.pjrc.com/tech/8051/ide/fat32.html
// https://codeandlife.com/2012/04/02/simple-fat-and-sd-tutorial-part-1/

#include <stdio.h>
#include <inttypes.h>

#include "stm32f7xx_nucleo_144.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_it.h"

#include "font.h"
#include "fck.h"

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

#define D0_MSK(B) (((1UL << 12) << 16) >> (((B) << 4) & 0x10)) // PF.12
#define D1_MSK(B) (((1UL << 15) << 16) >> (((B) << 3) & 0x10)) // PD.15
#define D2_MSK(B) (((1UL << 15) << 16) >> (((B) << 2) & 0x10)) // PF.15
#define D3_MSK(B) (((1UL << 13) << 16) >> (((B) << 1) & 0x10)) // PE.13
#define D4_MSK(B) (((1UL << 14) << 16) >> (((B) << 0) & 0x10)) // PF.14
#define D5_MSK(B) (((1UL << 11) << 16) >> (((B) >> 1) & 0x10)) // PE.11
#define D6_MSK(B) (((1UL <<  9) << 16) >> (((B) >> 2) & 0x10)) // PE.09
#define D7_MSK(B) (((1UL << 13) << 16) >> (((B) >> 3) & 0x10)) // PF.13

#define lcd_write_8(C) GPIOF -> BSRR = D0_MSK(C) | D2_MSK(C) | D4_MSK(C) | D7_MSK(C); WR_L; GPIOD -> BSRR = D1_MSK(C); GPIOE -> BSRR = D3_MSK(C) | D5_MSK(C) | D6_MSK(C); WR_L; WR_H;

#define SPI_CS_H GPIOD -> BSRR = 16384 << 16
#define SPI_CS_L GPIOD -> BSRR = 16384

#define CMD0     (0x40 + 0)   // GO_IDLE_STATE
#define CMD1     (0x40 + 1)   // SEND_OP_COND
#define CMD8     (0x40 + 8)   // SEND_IF_COND
#define CMD9     (0x40 + 9)   // SEND_CSD
#define CMD10    (0x40 + 10)  // SEND_CID
#define CMD12    (0x40 + 12)  // STOP_TRANSMISSION
#define CMD16    (0x40 + 16)  // SET_BLOCKLEN
#define CMD17    (0x40 + 17)  // READ_SINGLE_BLOCK
#define CMD18    (0x40 + 18)  // READ_MULTIPLE_BLOCK
#define CMD23    (0x40 + 23)  // SET_BLOCK_COUNT
#define CMD24    (0x40 + 24)  // WRITE_BLOCK
#define CMD25    (0x40 + 25)  // WRITE_MULTIPLE_BLOCK
#define CMD41    (0x40 + 41)  // SEND_OP_COND (ACMD)
#define CMD55    (0x40 + 55)  // APP_CMD
#define CMD58    (0x40 + 58)  // READ_OCR

#define CT_MMC    0x01  // MMC ver 3
#define CT_SD1    0x02  // SD ver 1
#define CT_SD2    0x04  // SD ver 2
#define CT_SDC    0x06  // SD
#define CT_BLOCK  0x08  // Block addressing

static char buffer[1];
static char *p_buffer = buffer;

volatile uint8_t uflag = 0;
uint16_t cursor_x;
uint16_t cursor_y;

UART_HandleTypeDef h_uart3;
SPI_HandleTypeDef h_spi1;

static void SystemClock_Config(void);
static void CPU_CACHE_Enable(void);
static void Error_Handler(void);
static void MPU_Config(void);

static void uart_init(void);
static void spi_init(void);
static void lcd_gpio_init(void);
static void lcd_init(void);
static void lcd_reset(void);
static void fill_black(void);
static void fill_frame(void);
static void draw_img(void);
static void draw_char(char, uint16_t*, uint16_t*);
static void write_table(const uint8_t[], int16_t);
static void spi_tx_8(uint8_t);
static void spi_tx_buffer(uint8_t *buffer, uint16_t len);
static uint8_t spi_rx_8(void);
static void spi_rx_ptr(uint8_t *buff);
static void sd_power_on(void);
static uint8_t sd_send_cmd(uint8_t, uint32_t);
static uint8_t sd_disk_read(uint8_t, uint8_t*, uint16_t, uint16_t);
static uint8_t sd_rx_data_block(uint8_t*, uint16_t);
static uint8_t sd_ready_wait(void);


int main(void) {
  MPU_Config();
  CPU_CACHE_Enable();
  HAL_Init();
  SystemClock_Config();
  HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_SYSCLK, RCC_MCODIV_2); // output SYSCLK / 2 on MCO2 pin (PC.09)

  BSP_LED_Init(LED1);
  BSP_LED_Init(LED2);
  BSP_LED_Init(LED3);

  lcd_gpio_init();
  uart_init();
  spi_init();

  lcd_reset();
  HAL_Delay(1);
  lcd_init();
  HAL_Delay(70);

  fill_black();
  draw_img();

  cursor_x = 3;
  cursor_y = 0;
  draw_char('$', &cursor_x, &cursor_y);
  draw_char(' ', &cursor_x, &cursor_y);

  printf("-\n");
  sd_power_on();

  uint8_t n, type, ocr[4];

  SPI_CS_H;

  if (sd_send_cmd(CMD0, 0) == 1) {                 // send GO_IDLE_STATE command
    printf("sd: idle");
    if (sd_send_cmd(CMD8, 0x1aa) == 1) {           // SDC V2+ accept CMD8 command, http://elm-chan.org/docs/mmc/mmc_e.html
      printf(", sdc v2+");
      for (n = 0; n < 4; n++) {                    // operation condition register
        ocr[n] = spi_rx_8();
      }
      if (ocr[2] == 0x01 && ocr[3] == 0xaa) {      // ACMD41 with HCS bit
        printf(", acmd41 with hcs bit");
        do {
          if (sd_send_cmd(CMD55, 0) <= 1 && sd_send_cmd(CMD41, 1UL << 30) == 0) break;
        } while (1);
        printf(", cmd55, cmd41");
        if (sd_send_cmd(CMD58, 0) == 0) {          // check CCS bit
          for (n = 0; n < 4; n++) {
            ocr[n] = spi_rx_8();
          }
          printf(", ccs bit ok, sd card ready.\n");
          //type = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2; // SDv2 (HC or SC)
        }
      }
    }

    /*
    else {                                         // SDC V1 or MMC
      type = (SD_SendCmd(CMD55, 0) <= 1 && SD_SendCmd(CMD41, 0) <= 1) ? CT_SD1 : CT_MMC;
      do {
        if (type == CT_SD1) {
          if (SD_SendCmd(CMD55, 0) <= 1 && SD_SendCmd(CMD41, 0) == 0) break; // ACMD41
        } else {
          if (SD_SendCmd(CMD1, 0) == 0) break;     // CMD1
        }
      } while (1);
      if (SD_SendCmd(CMD16, 512) != 0) type = 0;   // SET_BLOCKLEN
    }
    */
  }

  spi_rx_8();

  SPI_CS_L;

  uint8_t b[512];
  uint8_t b2[512];

  sd_disk_read(0, &b, 0, 1);

  printf("partition 1: ");
  uint8_t u; 
  for (int i = 0; i < 15; i++) {
    u = *((uint8_t *) &b[0x01be + i]);
    printf("0x%02x, ", u);
  }
  u = *((uint8_t *) &b[0x01be + 15]);
  printf("0x%02x.\n", u);

  uint8_t p1_head = *((uint8_t *) &b[0x01be + 5]); 
  uint8_t p1_cylinder_1 = *((uint8_t *) &b[0x01be + 6]);
  uint8_t p1_cylinder_sector = *((uint8_t *) &b[0x01be + 7]);
  uint16_t p1_cylinder;

  p1_cylinder = (p1_cylinder_sector && 0xc0) << 2;
  p1_cylinder = p1_cylinder && p1_cylinder_1;
  uint8_t p1_sector = p1_cylinder_sector && 0x3f;

  printf("cylinder: 0x%02x, ", p1_cylinder);
  printf("head: 0x%02x, ", p1_head);
  printf("sector: 0x%02x.\n", p1_sector);

  uint32_t ss = *((uint32_t *) &b[0x01be + 8]);
  printf("number of sectors: 0x%08x.\n", ss);

  ss = *((uint32_t *) &b[0x01be + 0x0c]);
  printf("number of sectors ip: 0x%08x.\n", ss);

  printf("partition 2: ");
  for (int i = 0; i < 15; i++) {
    u = *((uint8_t *) &b[0x01ce + i]);
    printf("0x%02x, ", u);
  }
  u = *((uint8_t *) &b[0x01ce + 15]);
  printf("0x%02x.\n", u);

  printf("partition 3: ");
  for (int i = 0; i < 15; i++) {
    u = *((uint8_t *) &b[0x01de + i]);
    printf("0x%02x, ", u);
  }
  u = *((uint8_t *) &b[0x01de + 15]);
  printf("0x%02x.\n", u);

  printf("partition 4: ");
  for (int i = 0; i < 15; i++) {
    u = *((uint8_t *) &b[0x01ee + i]);
    printf("0x%02x, ", u);
  }
  u = *((uint8_t *) &b[0x01ee + 15]);
  printf("0x%02x.\n", u);

  uint16_t signature = *((uint16_t *) &b[0x01fe]);
  printf("signature: 0x%04x.\n", signature);

  //uint32_t partition_lba_begin = *((uint32_t *) &b[0x01c6]);
  uint64_t partition_lba_begin = *((uint64_t *) &b[0x01be]);
  //printf("partition at: 0x%016x.\n", partition_lba_begin);

  sd_disk_read(0, &b, partition_lba_begin, 1);

  // uint16_t sector_size = *((uint16_t *) &b[0x0b]);
  uint8_t sectors_per_cluster = *((uint8_t *) &b[0x0d]);
  uint16_t reserved_sectors = *((uint16_t *) &b[0x0e]);
  uint8_t number_of_fats = *((uint8_t *) &b[0x10]);
  //uint16_t sectors_per_fat = *((uint16_t *) &b[0x16]);  // for FAT16
  uint32_t sectors_per_fat = *((uint32_t *) &b[0x24]);  // for FAT32

  uint32_t root_dir_offset = partition_lba_begin + reserved_sectors + (sectors_per_fat * number_of_fats);

  sd_disk_read(0, &b, root_dir_offset, 1);

  printf("root dir at: 0x%08x.\n", root_dir_offset);
  printf("sectors per cluster: 0x%02x.\n", sectors_per_cluster);

  uint8_t fn[11];
  uint8_t sf[] = "IMG";

  for (int i = 0; i < 0x200; i+=0x20) {
    if (b[i] > 0x1f && b[i] < 0x7f && b[i] != 0xe5 && b[i + 0x1a] != 0 && b[i + 0x10] != 0) {
      uint16_t starting_cluster = *((uint16_t *) &b[i + 0x1a]);
      uint32_t file_size = *((uint32_t *) &b[i + 0x1c]);
      uint32_t file_start = root_dir_offset + sectors_per_cluster * (starting_cluster - 2); // clusters are numbered from 2
      for (int j = 0; j < 11; j++) {
        printf("%c", b[i+j]);
        draw_char(b[j+i], &cursor_x, &cursor_y);
        fn[j] = b[i+j];
      }
      printf("  file start: 0x%08x, file size: 0x%08x.\n", file_start, file_size);
      if (memcmp(fn, sf, 3) == 0) {

        static const uint8_t t0[] = {
          0x36,   2,  0x68, 0x20 // ladscape
        };
        write_table(t0, sizeof(t0));

        CS_L;
        DC_C;
        lcd_write_8(0x2a); // set column address
        DC_D;
        lcd_write_8(1);    // SC[15:8]
        lcd_write_8(0);    // SC[7:0]
        lcd_write_8(1);    // EC[15:8]
        lcd_write_8(63);   // EC[7:0]
        CS_H;
        CS_L;
        DC_C;
        lcd_write_8(0x2b); // set page address
        DC_D;
        lcd_write_8(0);    // SP[15:8]
        lcd_write_8(10);   // SP[7:0]
        lcd_write_8(0);    // EP[15:8]
        lcd_write_8(73);   // EP[7:0]
        CS_H;
        CS_L;
        DC_C;
        lcd_write_8(0x2c);
        DC_D;
        for (int k = 0; k < 16; k++) {
          sd_disk_read(0, &b2, (file_start + k), 1);
          for (int l = 0; l < 512; l++) {
            lcd_write_8(b2[l]);
          }
        }
        CS_H;
      }
      draw_char(13, &cursor_x, &cursor_y);
    }
  }

  //for (int i = 0; i < 512; i++) {
  //  printf("%x ", b[i]);
  //}

  fill_frame();

  for (;;) {
    BSP_LED_Toggle(LED1);
    HAL_Delay(100);
  }

}


static uint8_t sd_rx_data_block(uint8_t *buff, uint16_t len) {
  uint8_t token;

  do {
    token = spi_rx_8();
  } while (token == 0xff);

  // invalid response
  if (token != 0xfe) return 0;

  // receive data
  do {
    spi_rx_ptr(buff++);
  } while (len--);

    // discard CRC
    spi_rx_8();
    spi_rx_8();
    return 1;
}


// read sector
static uint8_t sd_disk_read(uint8_t pdrv, uint8_t* buff, uint16_t sector, uint16_t count) {
  // pdrv should be 0
  //if (pdrv || !count) return RES_PARERR;

  // no disk
  //if (Stat & STA_NOINIT) return RES_NOTRDY;

  // convert to byte address
  //if (!(CardType & CT_SD2)) sector *= 512;

  SPI_CS_H;

  if (count == 1) {
    if ((sd_send_cmd(CMD17, sector) == 0) && sd_rx_data_block(buff, 512)) count = 0; // READ_SINGLE_BLOCK
  } else {

    if (sd_send_cmd(CMD18, sector) == 0) {                                           // READ_MULTIPLE_BLOCK

      do {
        if (!sd_rx_data_block(buff, 512)) break;
        buff += 512;
      } while (--count);

      sd_send_cmd(CMD12, 0);                                                         // STOP_TRANSMISSION
    }
  }

  SPI_CS_L;

  spi_rx_8();

  return count ? 0 : 1;
}


static uint8_t sd_ready_wait(void) {
  uint8_t res;

  // if SD goes ready, receives 0xff
  do {
    res = spi_rx_8();
  } while (res != 0xff);

  return res;
}


static uint8_t sd_send_cmd(uint8_t cmd, uint32_t arg) {
  uint8_t crc, res;

  // wait SD ready
  if (sd_ready_wait() != 0xff) return 0xff;

  // transmit command
  spi_tx_8(cmd);                  // command
  spi_tx_8((uint8_t)(arg >> 24)); // argument[31..24]
  spi_tx_8((uint8_t)(arg >> 16)); // argument[23..16]
  spi_tx_8((uint8_t)(arg >> 8));  // argument[15..8]
  spi_tx_8((uint8_t)arg);         // argument[7..0]

  // prepare CRC
  if (cmd == CMD0) crc = 0x95;
  else if (cmd == CMD8) crc = 0x87;
  else crc = 1;

  // transmit CRC
  spi_tx_8(crc);

  // skip a stuff byte when STOP_TRANSMISSION
  if (cmd == CMD12) spi_rx_8();

  // receive response
  uint8_t n = 10;
  do {
    res = spi_rx_8();
  } while ((res & 0x80) && --n);

  return res;
}


static void sd_power_on(void) {
  uint8_t args[6];
  uint32_t cnt = 0x1fff;

  SPI_CS_H;
  for (int i = 0; i < 10; i++) {
    spi_tx_8(0xff);
  }
  SPI_CS_L;

  // make idle state
  args[0] = CMD0;  // CMD0:GO_IDLE_STATE
  args[1] = 0;
  args[2] = 0;
  args[3] = 0;
  args[4] = 0;
  args[5] = 0x95; // CRC

  SPI_CS_H;
  spi_tx_buffer(args, sizeof(args));
  SPI_CS_L;

  SPI_CS_H;
  // wait response
  while ((spi_rx_8() != 0x01)) {}
  SPI_CS_L;

  SPI_CS_H;
  spi_tx_8(0xff);
  SPI_CS_L;
}


static void spi_tx_8(uint8_t data) {
  while (!__HAL_SPI_GET_FLAG(&h_spi1, SPI_FLAG_TXE));
  HAL_SPI_Transmit(&h_spi1, &data, 1, 100);
}


static void spi_tx_buffer(uint8_t *buffer, uint16_t len) {
  while (!__HAL_SPI_GET_FLAG(&h_spi1, SPI_FLAG_TXE));
  HAL_SPI_Transmit(&h_spi1, buffer, len, 100);
}


static uint8_t spi_rx_8(void) {
  uint8_t dummy, data;
  dummy = 0xff;
  while (!__HAL_SPI_GET_FLAG(&h_spi1, SPI_FLAG_TXE));
  HAL_SPI_TransmitReceive(&h_spi1, &dummy, &data, 1, 100);
  return data;
}


static void spi_rx_ptr(uint8_t *buff) {
  *buff = spi_rx_8();
}


static void lcd_init(void) { 
  static const uint8_t t0[] = {
    0xc0,   2,  0x17, 0x15,              // power control 1
    0xc1,   1,  0x41,                    // power control 2
    0xc2,   1,  0x00,                    // power control 3
    0xc5,   3,  0x00, 0x12, 0x80,        // vcom control 1
    0xb4,   1,  0x02,                    // display inversion control
    0xb6,   3,  0x02, 0x02, 0x3b,        // display function control
    0xe0,  15,  0x0f, 0x21, 0x1c, 0x0b, 0x0e, 0x08, 0x49, 0x98, 0x38, 0x09, 0x11, 0x03, 0x14, 0x10, 0x00, // positive gamma control
    0xe1,  15,  0x0f, 0x2f, 0x2b, 0x0c, 0x0e, 0x06, 0x47, 0x76, 0x37, 0x07, 0x11, 0x04, 0x23, 0x1e, 0x00, // negative gamma control
    0x3a,   1,  0x55,        // interface pixel format, 0x55 - 16bit, 0x66 - 18bit.
    0xb6,   2,  0x00, 0x22,  // display function control
  //  0x36,   1,  0x68,        // memory access control, mx, bgr, rotation, 0x08, 0x68, 0xc8, 0xa8
  //  0x36,   2,  0x08, 0x20,  // ladscape
    0x36,   1,  0x02,
    0xb0,   1,  0x00, // Interface Mode Control
    0xb1,   1,  0xa0, // Frame Rate Control
    0xb7,   1,  0xc6, // Entry Mode Set
    0xf7,   4,  0xa9, 0x51, 0x2c, 0x82, // Adjust Control 3
    0x11,   0,               // sleep out
    0x29,   0                // display on
  };

  write_table(t0, sizeof(t0));
}


static void draw_char(char c, uint16_t* cursor_x, uint16_t* cursor_y) {
  uint8_t d;
  uint8_t backspace = 0;

  GFXglyph g = Monospaced_plain_24Glyphs[c - 0x20];
  GFXglyph g2 = Monospaced_plain_24Glyphs[c - 0x20 + 1];

  if (c == 8) {
    backspace = 1;
    if (*cursor_x > 33) {
      *cursor_x -= 15; // only for exact monospaced font
    }
    g.width = 15;
    g.height = 24;
    g.xOffset = 0;
    g.yOffset = -18;
  }

  if (c == 13) {
    *cursor_y += 28;
    *cursor_x = 3;
    draw_char('$', cursor_x, cursor_y);
    draw_char(' ', cursor_x, cursor_y);
    return;
  }

  if (c == 10) {
    return;
  }

  if (c == 0x7e) {
    draw_char(' ', cursor_x, cursor_y);
    return;
  }

  static const uint8_t t0[] = {
      0x36,   2,  0x68, 0x20 // ladscape
  };

  write_table(t0, sizeof(t0));

  CS_L;
  DC_C;
  lcd_write_8(0x2a); // set column address
  DC_D;
  lcd_write_8(((*cursor_x + g.xOffset) >> 8) & 0xff); // SC[15:8]
  lcd_write_8(((*cursor_x + g.xOffset) >> 0) & 0xff); // SC[7:0]
  lcd_write_8(((*cursor_x + g.xOffset + g.width - 1) >> 8) & 0xff); // EC[15:8]
  lcd_write_8(((*cursor_x + g.xOffset + g.width - 1) >> 0) & 0xff); // EC[7:0]
  CS_H;
  CS_L;
  DC_C;
  lcd_write_8(0x2b); // set page address
  DC_D;
  lcd_write_8(((*cursor_y + 22 + g.yOffset) >> 8) & 0xff); // SP[15:8]
  lcd_write_8(((*cursor_y + 22 + g.yOffset) >> 0) & 0xff); // SP[7:0]
  lcd_write_8(((*cursor_y + 22 + g.yOffset + g.height - 1) >> 8) & 0xff); // EP[15:8]
  lcd_write_8(((*cursor_y + 22 + g.yOffset + g.height - 1) >> 0) & 0xff); // EP[7:0]
  CS_H;
  CS_L;
  DC_C;
  lcd_write_8(0x2c);

  if (backspace == 1) {
    DC_D;
    for(int i = 0; i < 15 * 24; i++) {
      lcd_write_8(0x00);
      lcd_write_8(0x00);
    }
    CS_H;
  } else {
    for (int i = g.bitmapOffset; i < g2.bitmapOffset; i++) {
      d = Monospaced_plain_24Bitmaps[i];
      DC_D;
      for(int j = 7; j > -1; j--) {
        if(d & (1 << j)) {
          lcd_write_8(0xff);
          lcd_write_8(0xff);
        } else {
          lcd_write_8(0x00);
          lcd_write_8(0x00);
        }
      }
    }
  }

  if (backspace == 0) {
    *cursor_x += g.xAdvance;
  }

  if (*cursor_x > 460) {
    *cursor_x = 0;
    *cursor_y += 28;
  }

}


static void draw_img(void) {

  static const uint8_t t0[] = {
    0x36,   2,  0x68, 0x20 // ladscape
  };
  write_table(t0, sizeof(t0));

  CS_L;
  DC_C;
  lcd_write_8(0x2a); // set column address
  DC_D;
  lcd_write_8(1);   // SC[15:8]
  lcd_write_8(171); // SC[7:0]
  lcd_write_8(1);   // EC[15:8]
  lcd_write_8(213); // EC[7:0]
  CS_H;
  CS_L;
  DC_C;
  lcd_write_8(0x2b); // set page address
  DC_D;
  lcd_write_8(0);   // SP[15:8]
  lcd_write_8(246); // SP[7:0]
  lcd_write_8(1);   // EP[15:8]
  lcd_write_8(53);  // EP[7:0]
  CS_H;
  CS_L;
  DC_C;
  lcd_write_8(0x2c);
  DC_D;
  for (int i = 0; i < 64 * 43 * 2; i++) {
    lcd_write_8(imf[i]);
  }
  CS_H;

}


static void fill_frame(void) {
  for (int j = 1; j < 360; j++) {

    static const uint8_t t0[] = {
      0x36,   2,  0x08, 0x20 // ladscape
    };
    write_table(t0, sizeof(t0));

    CS_L;
    DC_C;
    lcd_write_8(0x2a); // set column address
    DC_D;
    lcd_write_8(0x00);   // SC[15:8]
    lcd_write_8(0);      // SC[7:0]
    lcd_write_8((j >> 8) & 0xff); // EC[15:8]
    lcd_write_8((j >> 0) & 0xff); // EC[7:0]
    CS_H;
    CS_L;
    DC_C;
    lcd_write_8(0x2b); // set page address
    DC_D;
    lcd_write_8(0x01); // SP[15:8]
    lcd_write_8(221);  // SP[7:0]
    lcd_write_8(0x01); // EP[15:8]
    lcd_write_8(224);  // EP[7:0]
    CS_H;
    CS_L;
    DC_C;
    lcd_write_8(0x2c);
    DC_D;

    for(int i = 0; i < 4 * (j + 1); i++) {
      lcd_write_8(0x90); // 16-bit r 5-bit g 6-bit b 5-bit, 65536 colours
      lcd_write_8(0x00);
    }
    CS_H;
    HAL_Delay(2);

    if (j == 359) {
      fill_black();
      j = 0;
    }
    if (uflag == 1) {
      static const uint8_t t0[] = {
        0x36,   2,  0x08, 0x20 // ladscape
      };
      write_table(t0, sizeof(t0));

      printf("%c", buffer[0]);
      draw_char(buffer[0], &cursor_x, &cursor_y);
      uflag = 0;
    }

    SPI_CS_H;
    spi_rx_8();
    SPI_CS_L;
  }
}


static void fill_black(void) {
  CS_L;
  DC_C;
  lcd_write_8(0x2c);
  DC_D;

  for(int i = 0; i < 480 * 320; i++) {
    lcd_write_8(0x00); // 16-bit r 5-bit g 6-bit b 5-bit, 65536 colours
    lcd_write_8(0x00);
  }

  CS_H;
}


static void write_table(const uint8_t table[], int16_t size) {
  int p = 0;
  while (size > 0) {
    uint8_t cmd = table[p++];
    uint8_t len = table[p++];
    CS_L;
    DC_C;
    lcd_write_8(cmd);
    DC_D;

    for (uint8_t d = 0; d++ < len; ) {
      uint8_t x = table[p++];
      lcd_write_8(x);
    }

    CS_H;
    size -= len + 2;
  }
}


static void lcd_reset(void) {
  DC_D;
  CS_H;
  RD_H;
  WR_H;
  RST_H;
  RST_L;
  RST_H;
}


static void spi_init(void) {
  h_spi1.Instance = SPI1;
  h_spi1.Init.Mode = SPI_MODE_MASTER;
  h_spi1.Init.Direction = SPI_DIRECTION_2LINES;
  h_spi1.Init.DataSize = SPI_DATASIZE_8BIT;
  h_spi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  h_spi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  h_spi1.Init.NSS = SPI_NSS_SOFT;
  h_spi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  h_spi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  h_spi1.Init.TIMode = SPI_TIMODE_DISABLE;
  h_spi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  h_spi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&h_spi1) != HAL_OK) {
    Error_Handler();
  }
}


static void lcd_gpio_init(void) {
  GPIO_InitTypeDef GPIO_InitStruct;

  /*
  RD PA3
  WR PC0
  RS/CD/DC PC3
  CS PF3
  RST PF5
  */

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_USART3_CLK_ENABLE();
  __HAL_RCC_SPI1_CLK_ENABLE();

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
  LCD_D1 PD15
  LCD_D0 PF12
  LCD_D7 PF13
  LCD_D6 PE9
  LCD_D5 PE11
  LCD_D4 PF14
  LCD_D3 PE13
  LCD_D2 PF15
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

  // uart3
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*
  PA5 SPI1_SCK
  PA6 SPI1_MISO
  PA7 SPI1_MOSI (or PB5)
  PD14 SPI1_CS
  */

  // spi1
  GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  //GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}


static void SystemClock_Config(void) {
  /*
  System Clock source            = PLL (HSE)
  SYSCLK(Hz)                     = 216000000
  HCLK(Hz)                       = 216000000
  AHB Prescaler                  = 1
  APB1 Prescaler                 = 4
  APB2 Prescaler                 = 2
  HSE Frequency(Hz)              = 8000000
  PLL_M                          = 8
  PLL_N                          = 432
  PLL_P                          = 2
  PLL_Q                          = 9
  PLL_R                          = 7
  VDD(V)                         = 3.3
  Main regulator output voltage  = Scale1 mode
  Flash Latency(WS)              = 7
  */

  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  // enable HSE Oscillator and activate PLL with HSE as source
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

  // activate the OverDrive to reach the 216 Mhz Frequency
  if(HAL_PWREx_EnableOverDrive() != HAL_OK) {
    while(1) {};
  }

  // select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK) {
    while(1) {};
  }
}


static void uart_init(void) {
  h_uart3.Instance = USART3;
  h_uart3.Init.BaudRate = 115200;
  h_uart3.Init.WordLength = UART_WORDLENGTH_8B;
  h_uart3.Init.StopBits = UART_STOPBITS_1;
  h_uart3.Init.Parity = UART_PARITY_NONE;
  h_uart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  h_uart3.Init.Mode = UART_MODE_TX_RX;
  h_uart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&h_uart3) != HAL_OK) {
    Error_Handler();
  }
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
  __HAL_UART_ENABLE_IT(&h_uart3, UART_IT_RXNE);
}


static void MPU_Config(void) {
  MPU_Region_InitTypeDef MPU_InitStruct;
  HAL_MPU_Disable();
  // configure the MPU as Strongly ordered for not defined regions
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x00;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}


int __io_putchar(int ch) {
  HAL_UART_Transmit(&h_uart3, (uint8_t *)&ch, 1, 0xffff);
  return ch;
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


void USART3_IRQHandler(void)  {
  HAL_UART_Receive(&h_uart3, p_buffer, 1, 1000);
  uflag = 1;
}

