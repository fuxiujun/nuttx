/****************************************************************************
 * boards/arm/stm32/stm32f4discovery/src/stm32_bringup.c
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

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/fs/fs.h>

#include "stm32.h"

#ifdef CONFIG_INPUT_BUTTONS
#  include <nuttx/input/buttons.h>
#endif

#ifdef CONFIG_USERLED
#  include <nuttx/leds/userled.h>
#endif

#ifdef CONFIG_LCD_DEV
#include <nuttx/lcd/lcd_dev.h>
#endif

#include "stm32f4devebox.h"

/* Conditional logic in stm32f4discovery.h will determine if certain features
 * are supported.  Tests for these features need to be made after including
 * stm32f4discovery.h.
 */

/****************************************************************************
 * Public Functions
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
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void)
{
  int ret = OK;

#ifdef HAVE_SDIO
  /* Initialize the SDIO block driver */

  ret = stm32_sdio_initialize();
  if (ret != OK)
    {
      ferr("ERROR: Failed to initialize MMC/SD driver: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_PWM
  /* Initialize PWM and register the PWM device. */

  ret = stm32_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_pwm_setup() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_TIMER
  /* Initialize TIMER and register the TIMER device. */

  ret = stm32_timer_driver_setup("/dev/timer0", CONFIG_STM32F4DISCO_TIMER);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_timer_driver_setup() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_INPUT_BUTTONS
  /* Register the BUTTON driver */

  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: btn_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_USERLED
  /* Register the LED driver */

  ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef HAVE_RTC_DRIVER
  /* Instantiate the STM32 lower-half RTC driver */

  lower = stm32_rtc_lowerhalf();
  if (!lower)
    {
      serr("ERROR: Failed to instantiate the RTC lower-half driver\n");
      return -ENOMEM;
    }
  else
    {
      /* Bind the lower half driver and register the combined RTC driver
       * as /dev/rtc0
       */

      ret = rtc_initialize(0, lower);
      if (ret < 0)
        {
          serr("ERROR: Failed to bind/register the RTC driver: %d\n", ret);
          return ret;
        }
    }
#endif

#ifdef CONFIG_LCD_DEV
  ret = lcddev_register(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: lcddev_register() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_INPUT
  /* Initialize the touchscreen */

  ret = stm32_xpt2046_register(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_tsc_setup failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, STM32_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      serr("ERROR: Failed to mount procfs at %s: %d\n",
           STM32_PROCFS_MOUNTPOINT, ret);
    }
#endif

  return ret;
}