/****************************************************************************
 * boards/arm/stm32/nucleo-l152re/src/stm32_ili9341.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/ili9341.h>

#include "stm32_gpio.h"
#include "stm32_fsmc.h"
#include "arm_internal.h"
#include <arch/board/board.h>
#include "stm32f4devebox.h"

#ifdef CONFIG_LCD_ILI9341

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

static const uint32_t g_lcdpin[] =
{
  /* Address Lines:  A6 only */

  GPIO_FSMC_A6,

  /* Data Lines: D0... D15 */
  GPIO_FSMC_D0,  GPIO_FSMC_D1,  GPIO_FSMC_D2,
  GPIO_FSMC_D3,  GPIO_FSMC_D4,  GPIO_FSMC_D5,
  GPIO_FSMC_D6,  GPIO_FSMC_D7,  GPIO_FSMC_D8,
  GPIO_FSMC_D9,  GPIO_FSMC_D10, GPIO_FSMC_D11,
  GPIO_FSMC_D12, GPIO_FSMC_D13, GPIO_FSMC_D14,
  GPIO_FSMC_D15,

  /* NOE, NWE, NE4 */

  GPIO_FSMC_NOE, GPIO_FSMC_NWE, GPIO_FSMC_NE4,

  /* Backlight GPIO */

  GPIO_LCD_BACKLIGHT,
};

#define LCD_NPINS (sizeof(g_lcdpin) / sizeof(uint32_t))

/* Command and data transmission control */

static void stm32_ili9341_deselect(struct ili9341_lcd_s *lcd);
static void stm32_ili9341_select(struct ili9341_lcd_s *lcd);
static int stm32_ili9341_sendcmd(struct ili9341_lcd_s *lcd,
                                   const uint8_t cmd);
static int stm32_ili9341_sendparam(struct ili9341_lcd_s *lcd,
                                     const uint8_t param);
static int stm32_ili9341_recvparam(struct ili9341_lcd_s *lcd,
                                     uint8_t *param);
static int stm32_ili9341_backlight(struct ili9341_lcd_s *lcd,
                                     int level);
static int stm32_ili9341_sendgram(struct ili9341_lcd_s *lcd,
                                    const uint16_t *wd, uint32_t nwords);
static int stm32_ili9341_recvgram(struct ili9341_lcd_s *lcd,
                                    uint16_t *wd, uint32_t nwords);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* This is the standard, NuttX LCD driver object */
struct lcd_dev_s *g_lcddev;
static struct ili9341_lcd_s g_ili9341_lcddev =
{
  /* Initialize structure */

  .select    = stm32_ili9341_select,
  .deselect  = stm32_ili9341_deselect,
  .sendcmd   = stm32_ili9341_sendcmd,
  .sendparam = stm32_ili9341_sendparam,
  .recvparam = stm32_ili9341_recvparam,
  .sendgram  = stm32_ili9341_sendgram,
  .recvgram  = stm32_ili9341_recvgram,
  .backlight = stm32_ili9341_backlight,
};

static inline void write_data(uint16_t data)
{
	*(volatile uint16_t *)(LCD_DAT_ADDR) = data;
}

static inline void write_cmd(uint16_t cmd)
{
	 *(volatile uint16_t *)(LCD_CMD_ADDR) = cmd;
}

static inline uint16_t read_data(void)
{
	return (*(volatile uint16_t *)(LCD_CMD_ADDR));
}

/****************************************************************************
 * Name: stm32_ili9341_select
 *
 * Description:
 *   Select the LCD
 *
 * Input Parameters:
 *   lcd  - Reference to the public driver structure
 *
 * Returned Value:
 *
 ****************************************************************************/

static void stm32_ili9341_select(struct ili9341_lcd_s *lcd)
{

}

/****************************************************************************
 * Name: stm32_ili9341_deselect
 *
 * Description:
 *   De-select the LCD
 *
 * Input Parameters:
 *   lcd  - Reference to the public driver structure
 *
 * Returned Value:
 *
 ****************************************************************************/

static void stm32_ili9341_deselect(struct ili9341_lcd_s *lcd)
{

}

/****************************************************************************
 * Name: stm32_ili9341_sendparam
 *
 * Description:
 *   Send a parameter to the lcd driver.
 *
 * Input Parameters:
 *   lcd    - Reference to the ili9341_lcd_s driver structure
 *   param  - parameter to send
 *
 * Returned Value:
 *   OK - On Success
 *
 ****************************************************************************/

static int stm32_ili9341_sendparam(struct ili9341_lcd_s *lcd,
                                     const uint8_t param)
{
  lcdinfo("param=%04x\n", param);
  write_data(param);
  return OK;
}

/****************************************************************************
 * Name: stm32_ili9341_sendgram
 *
 * Description:
 *   Send a number of pixel words to the lcd driver gram.
 *
 * Input Parameters:
 *   lcd    - Reference to the ili9341_lcd_s driver structure
 *   wd     - Reference to the words to send
 *   nwords - number of words to send
 *
 * Returned Value:
 *   OK - On Success
 *
 ****************************************************************************/

static int stm32_ili9341_sendgram(struct ili9341_lcd_s *lcd,
                                    const uint16_t *wd, uint32_t nwords)
{
  lcdinfo("wd=%p , wd=0x%x, nwords=%" PRId32 "\n", wd, *wd, nwords);

  const uint16_t *src = wd;
  uint16_t word;
  while (nwords-- > 0)
    {
      word = *src++;
      write_data(word);
    }

  return OK;
};

/****************************************************************************
 * Name: stm32_ili9341_recvparam
 *
 * Description:
 *   Receive a parameter from the lcd driver.
 *
 * Input Parameters:
 *   lcd    - Reference to the ili9341_lcd_s driver structure
 *   param  - Reference to where parameter receive
 *
 * Returned Value:
 *   OK - On Success
 *
 ****************************************************************************/

static int stm32_ili9341_recvparam(struct ili9341_lcd_s *lcd,
                                     uint8_t *param)
{
  *param = read_data();
  lcdinfo("param=%p\n", param);
  return OK;
}

/****************************************************************************
 * Name: stm32_ili9341_recvgram
 *
 * Description:
 *   Receive pixel words from the lcd driver gram.
 *
 * Input Parameters:
 *   lcd    - Reference to the public driver structure
 *   wd     - Reference to where the pixel words receive
 *   nwords - number of pixel words to receive
 *
 * Returned Value:
 *   OK - On Success
 *
 ****************************************************************************/

static int stm32_ili9341_recvgram(struct ili9341_lcd_s *lcd,
                                    uint16_t *wd, uint32_t nwords)
{
  lcdinfo("wd=%p, nwords=%" PRId32 "\n", wd, nwords);
  return ERROR;
}

/****************************************************************************
 * Name: stm32_ili9341_sndcmd
 *
 * Description:
 *   Send a command to the lcd driver.
 *
 * Input Parameters:
 *   lcd  - Reference to the ili9341_lcd_s driver structure
 *   cmd  - command to send
 *
 * Returned Value:
 *   On success - OK
 *
 ****************************************************************************/

static int stm32_ili9341_sendcmd(
    struct ili9341_lcd_s *lcd, const uint8_t cmd)
{

  write_cmd(cmd);
  return OK;
}

/****************************************************************************
 * Name: stm32_ili9341_backlight
 *
 * Description:
 *   Set the backlight level of the connected display.
 *
 * Input Parameters:
 *   lcd   - Reference to the public driver structure
 *   level - backligth level
 *
 * Returned Value:
 *   OK - On Success
 *
 ****************************************************************************/

static int stm32_ili9341_backlight(struct ili9341_lcd_s *lcd,
                                     int level)
{
  return OK;
}

/****************************************************************************
 * Name:  sam_gpio_initialize
 *
 * Description:
 *   Configure LCD GPIO pins
 *
 ****************************************************************************/

static inline void stm32_gpio_initialize(void)
{
  int i;

  /* Configure all LCD pins pins (backlight is initially off) */

  for (i = 0; i < LCD_NPINS; i++)
    {
      stm32_configgpio(g_lcdpin[i]);
    }
}


/****************************************************************************
 * Name: stm32_selectlcd
 *
 * Description:
 *   Initialize to the LCD
 *
 ****************************************************************************/

static void stm32_selectlcd(void)
{  

  /* Configure gpios */

  stm32_gpio_initialize();
  up_mdelay(50);


  /* Enable AHB clocking to the FSMC */

  stm32_fsmc_enable();
  /* Color LCD configuration (LCD configured as follow):
   *
   *   - Data/Address MUX  = Disable   "FSMC_BCR_MUXEN" just not enable it.
   *   - Extended Mode     = Disable   "FSMC_BCR_EXTMOD"
   *   - Memory Type       = SRAM      "FSMC_BCR_SRAM"
   *   - Data Width        = 16bit     "FSMC_BCR_MWID16"
   *   - Write Operation   = Enable    "FSMC_BCR_WREN"
   *   - Asynchronous Wait = Disable
   */

  /* Bank1 NOR/SRAM control register configuration */

  putreg32(FSMC_BCR_SRAM | FSMC_BCR_MWID16 | FSMC_BCR_WREN, STM32_FSMC_BCR4);

  /* Bank1 NOR/SRAM timing register configuration */

  putreg32(FSMC_BTR_ADDSET(5) | FSMC_BTR_ADDHLD(1) |
           FSMC_BTR_DATAST(9) | FSMC_BTR_BUSTURN(1) |
           FSMC_BTR_CLKDIV(1) | FSMC_BTR_DATLAT(2) |
           FSMC_BTR_ACCMODA, STM32_FSMC_BTR4);

  putreg32(0xffffffff, STM32_FSMC_BWTR4);

  /* Enable the bank by setting the MBKEN bit */

  putreg32(FSMC_BCR_MBKEN | FSMC_BCR_SRAM |
           FSMC_BCR_MWID16 | FSMC_BCR_WREN, STM32_FSMC_BCR4);
}

/****************************************************************************
 * Name:  stm32_ili93414ws_initialize
 *
 * Description:
 *   Initialize the device structure to control the LCD Single chip driver.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   On success, this function returns a reference to the LCD control object
 *   for the specified ILI9341 LCD Single chip driver connected as 8-bit
 *   series parallel. NULL is returned on any failure.
 *
 ****************************************************************************/

struct lcd_dev_s *stm32_ili9341_initialize(void)
{
  struct ili9341_lcd_s *lcd = &g_ili9341_lcddev;

  /* Configure gpios */
  stm32_selectlcd();

  /* initialize LCD */

  g_lcddev = ili9341_initialize(lcd, 0);

  /* Select LCD device */

  lcdinfo("Initialize ili9341 lcd driver\n");
  lcd->select(lcd);

#if 1
  /* Reset the lcd display to the default state */

  lcdinfo("ili9341 LCD driver: Software Reset\n");
  lcd->sendcmd(lcd, ILI9341_SOFTWARE_RESET);
  up_mdelay(5);

  lcd->sendcmd(lcd, ILI9341_POWER_CONTROL_B);
	lcd->sendparam(lcd, 0x00); 
	lcd->sendparam(lcd, 0xC9); //C1 
	lcd->sendparam(lcd, 0X30); 

	lcd->sendcmd(lcd, ILI9341_POWER_ON_SEQUENCE_CONTROL);  
	lcd->sendparam(lcd, 0x64); 
	lcd->sendparam(lcd, 0x03); 
	lcd->sendparam(lcd, 0X12); 
	lcd->sendparam(lcd, 0X81); 

	lcd->sendcmd(lcd, ILI9341_DRIVER_TIMING_CTL_A);  
	lcd->sendparam(lcd, 0x85); 
	lcd->sendparam(lcd, 0x10); 
	lcd->sendparam(lcd, 0x7A); 

	lcd->sendcmd(lcd, ILI9341_POWER_CONTROL_A);  
	lcd->sendparam(lcd, 0x39); 
	lcd->sendparam(lcd, 0x2C); 
	lcd->sendparam(lcd, 0x00); 
	lcd->sendparam(lcd, 0x34); 
	lcd->sendparam(lcd, 0x02); 

	lcd->sendcmd(lcd, ILI9341_PUMP_RATIO_CONTROL);  
	lcd->sendparam(lcd, 0x20); 

	lcd->sendcmd(lcd, ILI9341_DRIVER_TIMING_CTL_B);  
	lcd->sendparam(lcd, 0x00); 
	lcd->sendparam(lcd, 0x00);

	lcd->sendcmd(lcd, ILI9341_POWER_CONTROL_1);    //Power control 
	lcd->sendparam(lcd, 0x1B);   //VRH[5:0] 

	lcd->sendcmd(lcd, ILI9341_POWER_CONTROL_2);    //Power control 
	lcd->sendparam(lcd, 0x00);   //SAP[2:0];BT[3:0] 01 

	lcd->sendcmd(lcd, ILI9341_VCOM_CONTROL_1);    //VCM control 
	lcd->sendparam(lcd, 0x30); 	 //3F
	lcd->sendparam(lcd, 0x30); 	 //3C

	lcd->sendcmd(lcd, ILI9341_VCOM_CONTROL_2);    //VCM control2 
	lcd->sendparam(lcd, 0XB7); 

	lcd->sendcmd(lcd, ILI9341_MEMORY_ACCESS_CONTROL);    // Memory Access Control 
  lcd->sendparam(lcd, 0xc8); 

	lcd->sendcmd(lcd, ILI9341_PIXEL_FORMAT_SET);   
	lcd->sendparam(lcd, 0x55); 

	lcd->sendcmd(lcd, ILI9341_FRAME_RATE_CONTROL_NORMAL);   
	lcd->sendparam(lcd, 0x00);   
	lcd->sendparam(lcd, 0x1A);

	lcd->sendcmd(lcd, ILI9341_DISPLAY_FUNCTION_CTL);    // Display Function Control 
	lcd->sendparam(lcd, 0x0A); 
	lcd->sendparam(lcd, 0xA2); 

	lcd->sendcmd(lcd, ILI9341_ENABLE_3_GAMMA_CONTROL);    // 3Gamma Function Disable 
	lcd->sendparam(lcd, 0x00);

	lcd->sendcmd(lcd, ILI9341_GAMMA_SET);    //Gamma curve selected 
	lcd->sendparam(lcd, 0x01); 

	lcd->sendcmd(lcd, ILI9341_POSITIVE_GAMMA_CORRECTION);    //Set Gamma 
	lcd->sendparam(lcd, 0x0F); 
	lcd->sendparam(lcd, 0x2A); 
	lcd->sendparam(lcd, 0x28); 
	lcd->sendparam(lcd, 0x08); 
	lcd->sendparam(lcd, 0x0E); 
	lcd->sendparam(lcd, 0x08); 
	lcd->sendparam(lcd, 0x54); 
	lcd->sendparam(lcd, 0XA9); 
	lcd->sendparam(lcd, 0x43); 
	lcd->sendparam(lcd, 0x0A); 
	lcd->sendparam(lcd, 0x0F); 
	lcd->sendparam(lcd, 0x00); 
	lcd->sendparam(lcd, 0x00); 
	lcd->sendparam(lcd, 0x00); 
	lcd->sendparam(lcd, 0x00); 

	lcd->sendcmd(lcd, ILI9341_NEGATIVE_GAMMA_CORRECTION);    //Set Gamma 
	lcd->sendparam(lcd, 0x00); 
	lcd->sendparam(lcd, 0x15); 
	lcd->sendparam(lcd, 0x17); 
	lcd->sendparam(lcd, 0x07); 
	lcd->sendparam(lcd, 0x11); 
	lcd->sendparam(lcd, 0x06); 
	lcd->sendparam(lcd, 0x2B); 
	lcd->sendparam(lcd, 0x56); 
	lcd->sendparam(lcd, 0x3C); 
	lcd->sendparam(lcd, 0x05); 
	lcd->sendparam(lcd, 0x10); 
	lcd->sendparam(lcd, 0x0F); 
	lcd->sendparam(lcd, 0x3F); 
	lcd->sendparam(lcd, 0x3F); 
	lcd->sendparam(lcd, 0x0F); 

	lcd->sendcmd(lcd, ILI9341_PAGE_ADDRESS_SET); 
	lcd->sendparam(lcd, 0x00);
	lcd->sendparam(lcd, 0x00);
	lcd->sendparam(lcd, 0x01);
	lcd->sendparam(lcd, 0x3f);

	lcd->sendcmd(lcd, ILI9341_COLUMN_ADDRESS_SET); 
	lcd->sendparam(lcd, 0x00);
	lcd->sendparam(lcd, 0x00);
	lcd->sendparam(lcd, 0x00);
	lcd->sendparam(lcd, 0xef);	 

	lcd->sendcmd(lcd, ILI9341_SLEEP_OUT); //Exit Sleep
	up_mdelay(120);

	lcd->sendcmd(lcd, ILI9341_DISPLAY_ON); //display on

  /* Deselect LCD device */

  lcd->deselect(lcd);
#endif

  stm32_gpiowrite(GPIO_LCD_BACKLIGHT, true);

  return g_lcddev;
}

/****************************************************************************
 * Name:  board_lcd_initialize
 *
 * Description:
 *   Initialize the LCD video hardware.  The initial state of the LCD is
 *   fully initialized, display memory cleared, and the LCD ready to use,
 *   but with the power setting at 0 (full off).
 *
 ****************************************************************************/

int board_lcd_initialize(void)
{
  g_lcddev = stm32_ili9341_initialize();
  if (g_lcddev == NULL)
    {
      lcdinfo("Initialize ili9341 lcd driver NULL\n");
      return ENODEV;
    }

  ili9341_clear(g_lcddev, 0);

  
  return OK;
}

/****************************************************************************
 * Name:  board_lcd_getdev
 *
 * Description:
 *   Return a a reference to the LCD object for the specified LCD.
 *   This allows support
 *   for multiple LCD devices.
 *
 ****************************************************************************/

struct lcd_dev_s *board_lcd_getdev(int lcddev)
{
  if (lcddev == 0)
    {
      return g_lcddev;
    }

  return NULL;
}

#endif /* CONFIG_LCD_ILI9341 */
