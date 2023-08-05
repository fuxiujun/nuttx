/****************************************************************************
 * boards/arm/stm32/stm32f4discovery/src/stm32f4discovery.h
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

#ifndef __BOARDS_ARM_STM32_STM32F4DISCOVERY_SRC_STM32F4DISCOVERY_H
#define __BOARDS_ARM_STM32_STM32F4DISCOVERY_SRC_STM32F4DISCOVERY_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>
#include <arch/stm32/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Assume that we have everything */

// #define HAVE_SDIO       0
// #define HAVE_RTC_DRIVER 0


/* Can't support MMC/SD features if mountpoints are disabled or if SDIO
 * support is not enabled.
 */

#if defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_STM32_SDIO)
#  undef HAVE_SDIO
#endif

#undef  SDIO_MINOR     /* Any minor number, default 0 */
#define SDIO_SLOTNO 0  /* Only one slot */

#ifdef HAVE_SDIO
#  if !defined(CONFIG_NSH_MMCSDSLOTNO)
#    define CONFIG_NSH_MMCSDSLOTNO SDIO_SLOTNO
#  elif CONFIG_NSH_MMCSDSLOTNO != 0
#    warning "Only one MMC/SD slot, slot 0"
#    undef CONFIG_NSH_MMCSDSLOTNO
#    define CONFIG_NSH_MMCSDSLOTNO SDIO_SLOTNO
#  endif

#  if defined(CONFIG_NSH_MMCSDMINOR)
#    define SDIO_MINOR CONFIG_NSH_MMCSDMINOR
#  else
#    define SDIO_MINOR 0
#  endif
#endif

/* procfs File System */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define STM32_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define STM32_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

/* STM32F4 Discovery GPIOs **************************************************/

/* LEDs */

#define GPIO_LED1       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTF|GPIO_PIN9)
#define GPIO_LED2       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTF|GPIO_PIN10)

/* BUTTONS -- NOTE that all have EXTI interrupts configured */

#define MIN_IRQBUTTON   BUTTON_USER
#define MAX_IRQBUTTON   BUTTON_USER
#define NUM_IRQBUTTONS  1

#define GPIO_BTN_USER1  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTE|GPIO_PIN4)
#define GPIO_BTN_USER2  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTE|GPIO_PIN3)

#define GPIO_LCD_BACKLIGHT (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                            GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN15)

#define      LCD_CMD_ADDR                       0x6c00007E
#define      LCD_DAT_ADDR                       0x6c000080

#define GPIO_FSMC_A6          (GPIO_ALT|GPIO_AF12|GPIO_SPEED_100MHz|GPIO_PORTF|GPIO_PIN12)
#define GPIO_FSMC_NBL1        (GPIO_ALT|GPIO_AF12|GPIO_SPEED_100MHz|GPIO_PORTE|GPIO_PIN1)
#define GPIO_FSMC_CLK         (GPIO_ALT|GPIO_AF12|GPIO_SPEED_100MHz|GPIO_PORTD|GPIO_PIN3)
#define GPIO_FSMC_D0          (GPIO_ALT|GPIO_AF12|GPIO_SPEED_100MHz|GPIO_PORTD|GPIO_PIN14)
#define GPIO_FSMC_D1          (GPIO_ALT|GPIO_AF12|GPIO_SPEED_100MHz|GPIO_PORTD|GPIO_PIN15)
#define GPIO_FSMC_D2          (GPIO_ALT|GPIO_AF12|GPIO_SPEED_100MHz|GPIO_PORTD|GPIO_PIN0)
#define GPIO_FSMC_D3          (GPIO_ALT|GPIO_AF12|GPIO_SPEED_100MHz|GPIO_PORTD|GPIO_PIN1)
#define GPIO_FSMC_D4          (GPIO_ALT|GPIO_AF12|GPIO_SPEED_100MHz|GPIO_PORTE|GPIO_PIN7)
#define GPIO_FSMC_D5          (GPIO_ALT|GPIO_AF12|GPIO_SPEED_100MHz|GPIO_PORTE|GPIO_PIN8)
#define GPIO_FSMC_D6          (GPIO_ALT|GPIO_AF12|GPIO_SPEED_100MHz|GPIO_PORTE|GPIO_PIN9)
#define GPIO_FSMC_D7          (GPIO_ALT|GPIO_AF12|GPIO_SPEED_100MHz|GPIO_PORTE|GPIO_PIN10)
#define GPIO_FSMC_D8          (GPIO_ALT|GPIO_AF12|GPIO_SPEED_100MHz|GPIO_PORTE|GPIO_PIN11)
#define GPIO_FSMC_D9          (GPIO_ALT|GPIO_AF12|GPIO_SPEED_100MHz|GPIO_PORTE|GPIO_PIN12)
#define GPIO_FSMC_D10         (GPIO_ALT|GPIO_AF12|GPIO_SPEED_100MHz|GPIO_PORTE|GPIO_PIN13)
#define GPIO_FSMC_D11         (GPIO_ALT|GPIO_AF12|GPIO_SPEED_100MHz|GPIO_PORTE|GPIO_PIN14)
#define GPIO_FSMC_D12         (GPIO_ALT|GPIO_AF12|GPIO_SPEED_100MHz|GPIO_PORTE|GPIO_PIN15)
#define GPIO_FSMC_D13         (GPIO_ALT|GPIO_AF12|GPIO_SPEED_100MHz|GPIO_PORTD|GPIO_PIN8)
#define GPIO_FSMC_D14         (GPIO_ALT|GPIO_AF12|GPIO_SPEED_100MHz|GPIO_PORTD|GPIO_PIN9)
#define GPIO_FSMC_D15         (GPIO_ALT|GPIO_AF12|GPIO_SPEED_100MHz|GPIO_PORTD|GPIO_PIN10)
#define GPIO_FSMC_NBL0        (GPIO_ALT|GPIO_AF12|GPIO_SPEED_100MHz|GPIO_PORTE|GPIO_PIN0)
#define GPIO_FSMC_NE1         (GPIO_ALT|GPIO_AF12|GPIO_SPEED_100MHz|GPIO_PORTD|GPIO_PIN7)
#define GPIO_FSMC_NE2         (GPIO_ALT|GPIO_AF12|GPIO_SPEED_100MHz|GPIO_PORTG|GPIO_PIN9)
#define GPIO_FSMC_NE3         (GPIO_ALT|GPIO_AF12|GPIO_SPEED_100MHz|GPIO_PORTG|GPIO_PIN10)
#define GPIO_FSMC_NE4         (GPIO_ALT|GPIO_AF12|GPIO_SPEED_100MHz|GPIO_PORTG|GPIO_PIN12)
#define GPIO_FSMC_NL          (GPIO_ALT|GPIO_AF12|GPIO_SPEED_100MHz|GPIO_PORTB|GPIO_PIN7)
#define GPIO_FSMC_NOE         (GPIO_ALT|GPIO_AF12|GPIO_SPEED_100MHz|GPIO_PORTD|GPIO_PIN4)
#define GPIO_FSMC_NWAIT       (GPIO_ALT|GPIO_AF12|GPIO_SPEED_100MHz|GPIO_PORTD|GPIO_PIN6)
#define GPIO_FSMC_NWE         (GPIO_ALT|GPIO_AF12|GPIO_SPEED_100MHz|GPIO_PORTD|GPIO_PIN5)

#define GPIO_LCDTP_CS         (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_OUTPUT_CLEAR|GPIO_SPEED_100MHz|GPIO_PORTC|GPIO_PIN13)
#define GPIO_LCDTP_MOSI       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_OUTPUT_CLEAR|GPIO_SPEED_100MHz|GPIO_PORTF|GPIO_PIN11)
#define GPIO_LCDTP_SCK        (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_OUTPUT_CLEAR|GPIO_SPEED_100MHz|GPIO_PORTB|GPIO_PIN0)
#define GPIO_LCDTP_MISO       (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTB|GPIO_PIN2)
#define GPIO_LCDTP_IRQ        (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTB|GPIO_PIN1)
/* PWM
 *
 * The STM32F4 Discovery has no real on-board PWM devices, but the board can
 * be configured to output a pulse train using TIM4 CH2 on PD13.
 */

#define STM32F4DISCOVERY_PWMTIMER   4
#define STM32F4DISCOVERY_PWMCHANNEL 2

/* STM32F4DIS-BB MicroSD
 *
 * ---------- ------------- ------------------------------
 * PIO        SIGNAL        Comments
 * ---------- ------------- ------------------------------
 * PB15       NCD           Pulled up externally
 * PC9        DAT1          Configured by driver
 * PC8        DAT0          "        " "" "    "
 * PC12       CLK           "        " "" "    "
 * PD2        CMD           "        " "" "    "
 * PC11       CD/DAT3       "        " "" "    "
 * PC10       DAT2          "        " "" "    "
 * ---------- ------------- ------------------------------
 */

#if defined(CONFIG_STM32F4DISBB) && defined(CONFIG_STM32_SDIO)
#  define GPIO_SDIO_NCD   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|\
                           GPIO_PORTB|GPIO_PIN15)
#endif


/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void);

/****************************************************************************
 * Name: stm32_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ****************************************************************************/

#ifdef CONFIG_PWM
int stm32_pwm_setup(void);
#endif

/****************************************************************************
 * Name: stm32_sdio_initialize
 *
 * Description:
 *   Initialize SDIO-based MMC/SD card support
 *
 ****************************************************************************/

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && defined(CONFIG_STM32_SDIO)
int stm32_sdio_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_timer_driver_setup
 *
 * Description:
 *   Configure the timer driver.
 *
 * Input Parameters:
 *   devpath - The full path to the timer device.  This should be of the
 *             form /dev/timer0
 *   timer   - The timer's number.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_TIMER
int stm32_timer_driver_setup(const char *devpath, int timer);
#endif

#ifdef CONFIG_LCD
int board_lcd_initialize(void);
struct lcd_dev_s *board_lcd_getdev(int lcddev);
#endif

#ifdef CONFIG_INPUT
int stm32_xpt2046_register(int minor);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32_STM32F4DISCOVERY_SRC_STM32F4DISCOVERY_H */
