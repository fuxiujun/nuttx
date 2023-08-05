/****************************************************************************
 * boards/arm/stm32/viewtool-stm32f107/src/stm32_xpt2046.c
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

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <sys/types.h>

#include "stm32_gpio.h"
#include "stm32f4devebox.h"
#include <nuttx/board.h>
#include <nuttx/input/touchscreen.h>
#include <nuttx/kthread.h>
#include <nuttx/signal.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define DEV_FORMAT "/dev/input%d"
#define DEV_NAMELEN 16
#define XPT2046_CMD_YPOSITION 0x90
#define XPT2046_CMD_XPOSITION 0xd0
#define XPT2046_WORKER_THREAD_PRIORITY 100
#define XPT2046_WORKER_STACKSIZE 64
#define XPT2046_WORKER_PERIOD 10 * 1000
#define XPT2046_WORKER_DEBOUNCE 5

#define TOUCH_PARA_X -15.379601f
#define TOUCH_PARA_XX 0.131853f
#define TOUCH_PARA_XY 0.002315f
#define TOUCH_PARA_Y 336.286469f
#define TOUCH_PARA_YX 0.001157f
#define TOUCH_PARA_YY -0.175611f

/****************************************************************************
 * Private Types
 ****************************************************************************/
typedef enum {
    TOUCH_STATE_RELEASE = 0,
    TOUCH_STATE_WAITING,
    TOUCH_STATE_PRESSED,
} xpt2046_touch_state_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static int xpt2046_fops_open(FAR struct file* filep);
static int xpt2046_fops_close(FAR struct file* filep);
static ssize_t xpt2046_fops_read(FAR struct file* filep, FAR char* buffer,
                                 size_t len);
static void xpt2046_write_data(uint8_t data);
static uint8_t xpt2046_read_data(void);
static uint16_t xpt2046_sendcmd(uint8_t cmd);
static void xpt2046_gpio_init(void);
static int xpt2046_worker(int argc, char* argv[]);
/****************************************************************************
 * Private Data
 ****************************************************************************/
static const struct file_operations g_xpt2046_fops = {
    xpt2046_fops_open, /* open */
    xpt2046_fops_close, /* close */
    xpt2046_fops_read, /* read */
    NULL, /* write */
    NULL, /* seek */
    NULL, /* ioctl */
    NULL, /* mmap */
    NULL, /* truncate */
    NULL /* poll */
};

static xpt2046_touch_state_t g_touch_state;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int xpt2046_fops_open(FAR struct file* filep)
{
    iinfo("Opening\n");
    return OK;
}

static int xpt2046_fops_close(FAR struct file* filep)
{
    iinfo("Closing\n");
    return OK;
}

static ssize_t xpt2046_fops_read(FAR struct file* filep, FAR char* buffer,
                                 size_t len)
{
    FAR struct touch_sample_s* report;
    int ret = len;

    if (len < SIZEOF_TOUCH_SAMPLE_S(1)) {
        ierr("ERROR: Unsupported read size: %d\n", len);
        return -ENOSYS;
    }

    report = (FAR struct touch_sample_s*)buffer;
    memset(report, 0, SIZEOF_TOUCH_SAMPLE_S(1));
    report->npoints = 1;

    if (g_touch_state != TOUCH_STATE_PRESSED) {
        report->point[0].flags = TOUCH_UP;
        goto exit;
    }

    uint16_t x = xpt2046_sendcmd(XPT2046_CMD_XPOSITION);
    uint16_t y = xpt2046_sendcmd(XPT2046_CMD_YPOSITION);

    int16_t para_x = (TOUCH_PARA_XX * x) + (TOUCH_PARA_XY * y) + TOUCH_PARA_X;
    int16_t para_y = (TOUCH_PARA_YX * x) + (TOUCH_PARA_YY * y) + TOUCH_PARA_Y;

    report->point->flags = TOUCH_DOWN;
    report->point[0].x = para_x;
    report->point[0].y = para_y;

    iinfo("  id:      %d\n", report->point[0].id);
    iinfo("  flags:   %02x\n", report->point[0].flags);
    iinfo("  x:       %d\n", report->point[0].x);
    iinfo("  y:       %d\n", report->point[0].y);

exit:
    return ret;
}

static void xpt2046_write_data(uint8_t data)
{
    uint8_t i = 0;
    for (i = 0; i < 8; i++) {
        if (data & 0x80) {
            stm32_gpiowrite(GPIO_LCDTP_MOSI, true);
        } else {
            stm32_gpiowrite(GPIO_LCDTP_MOSI, false);
        }
        data <<= 1;
        stm32_gpiowrite(GPIO_LCDTP_SCK, false);
        up_mdelay(1);
        stm32_gpiowrite(GPIO_LCDTP_SCK, true);
    }
}

static uint8_t xpt2046_read_data(void)
{
    uint8_t i = 0, rd = 0;
    for (i = 0; i < 8; i++) {
        rd <<= 1;
        stm32_gpiowrite(GPIO_LCDTP_SCK, false);
        up_mdelay(1);
        stm32_gpiowrite(GPIO_LCDTP_SCK, true);
        if (stm32_gpioread(GPIO_LCDTP_MISO)) {
            rd++;
        }
    }

    return rd;
}

static uint16_t xpt2046_sendcmd(uint8_t cmd)
{
    uint8_t buffer[2];
    uint16_t result;

    xpt2046_write_data(cmd);

    /* Wait a tiny amount to make sure that the acquisition time is complete */

    up_udelay(3); /* 3 microseconds */

    buffer[0] = xpt2046_read_data();
    buffer[1] = xpt2046_read_data();

    /* Read the 12-bit data (LS 4 bits will be padded with zero) */
    result = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
    result = result >> 4;

    iinfo("cmd:0x%02x response:0x%04x\n", cmd, result);
    return result;
}

static void xpt2046_gpio_init(void)
{
    stm32_configgpio(GPIO_LCDTP_CS);
    stm32_configgpio(GPIO_LCDTP_MOSI);
    stm32_configgpio(GPIO_LCDTP_MISO);
    stm32_configgpio(GPIO_LCDTP_SCK);
    stm32_configgpio(GPIO_LCDTP_IRQ);

    stm32_gpiowrite(GPIO_LCDTP_CS, false);
}

static int xpt2046_worker(int argc, char* argv[])
{
    static uint32_t i;
    while (1) {
        bool pendown = stm32_gpioread(GPIO_LCDTP_IRQ);
        switch (g_touch_state) {
        case TOUCH_STATE_RELEASE:
            if (pendown == 0) {
                g_touch_state = TOUCH_STATE_WAITING;
            } else {
                g_touch_state = TOUCH_STATE_RELEASE;
            }
            break;
        case TOUCH_STATE_WAITING:
            if (pendown == 0) {
                i++;
                if (i > XPT2046_WORKER_DEBOUNCE) {
                    i = 0;
                    g_touch_state = TOUCH_STATE_PRESSED;
                } else {
                    i++;
                    g_touch_state = TOUCH_STATE_WAITING;
                }
            } else {
                i = 0;
                g_touch_state = TOUCH_STATE_RELEASE;
            }
            break;
        case TOUCH_STATE_PRESSED:

            if (pendown == 0) {
                g_touch_state = TOUCH_STATE_PRESSED;
            } else {
                g_touch_state = TOUCH_STATE_RELEASE;
            }
            break;
        default:
            g_touch_state = TOUCH_STATE_RELEASE;
        }

        nxsig_usleep(XPT2046_WORKER_PERIOD);
    }

    return 0;
}

#if 0
#include <errno.h>
#include <fcntl.h>
#include <nuttx/lcd/lcd_dev.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>

typedef struct {
    int16_t x;
    int16_t y;
} Touch_Coordinate;

typedef struct {
    float dX_X, dX_Y, dX, dY_X, dY_Y, dY;
} Touch_Calibration;

static uint8_t
Touch_Calculate_CalibrationFactor(Touch_Coordinate* pDisplayCoordinate,
                                  Touch_Coordinate* pScreenSample,
                                  Touch_Calibration* pCalibrationFactor)
{
    uint8_t ucRet = 1;
    float Divider, An, Bn, Cn, Dn, En, Fn;
    uint16_t usTest_x = 0, usTest_y = 0, usGap_x = 0, usGap_y = 0;
    /* K＝ ( X0－X2 )  ( Y1－Y2 )－ ( X1－X2 )  ( Y0－Y2 ) */
    Divider = ((pScreenSample[0].x - pScreenSample[2].x)
               * (pScreenSample[1].y - pScreenSample[2].y))
        - ((pScreenSample[1].x - pScreenSample[2].x)
           * (pScreenSample[0].y - pScreenSample[2].y));

    if (Divider == 0) {
        ucRet = 0;
    } else {
        /* A＝ (  ( XD0－XD2 )  ( Y1－Y2 )－ ( XD1－XD2 )  ( Y0－Y2 ) )／K
         */
        An = ((pDisplayCoordinate[0].x - pDisplayCoordinate[2].x)
              * (pScreenSample[1].y - pScreenSample[2].y))
            - ((pDisplayCoordinate[1].x - pDisplayCoordinate[2].x)
               * (pScreenSample[0].y - pScreenSample[2].y));

        /* B＝ (  ( X0－X2 )  ( XD1－XD2 )－ ( XD0－XD2 )  ( X1－X2 ) )／K
         */
        Bn = ((pScreenSample[0].x - pScreenSample[2].x)
              * (pDisplayCoordinate[1].x - pDisplayCoordinate[2].x))
            - ((pDisplayCoordinate[0].x - pDisplayCoordinate[2].x)
               * (pScreenSample[1].x - pScreenSample[2].x));

        /* C＝ ( Y0 ( X2XD1－X1XD2 )+Y1 ( X0XD2－X2XD0 )+Y2 ( X1XD0－X0XD1 )
         * )／K */
        Cn = (pScreenSample[2].x * pDisplayCoordinate[1].x
              - pScreenSample[1].x * pDisplayCoordinate[2].x)
                * pScreenSample[0].y
            + (pScreenSample[0].x * pDisplayCoordinate[2].x
               - pScreenSample[2].x * pDisplayCoordinate[0].x)
                * pScreenSample[1].y
            + (pScreenSample[1].x * pDisplayCoordinate[0].x
               - pScreenSample[0].x * pDisplayCoordinate[1].x)
                * pScreenSample[2].y;

        /* D＝ (  ( YD0－YD2 )  ( Y1－Y2 )－ ( YD1－YD2 )  ( Y0－Y2 ) )／K
         */
        Dn = ((pDisplayCoordinate[0].y - pDisplayCoordinate[2].y)
              * (pScreenSample[1].y - pScreenSample[2].y))
            - ((pDisplayCoordinate[1].y - pDisplayCoordinate[2].y)
               * (pScreenSample[0].y - pScreenSample[2].y));

        /* E＝ (  ( X0－X2 )  ( YD1－YD2 )－ ( YD0－YD2 )  ( X1－X2 ) )／K
         */
        En = ((pScreenSample[0].x - pScreenSample[2].x)
              * (pDisplayCoordinate[1].y - pDisplayCoordinate[2].y))
            - ((pDisplayCoordinate[0].y - pDisplayCoordinate[2].y)
               * (pScreenSample[1].x - pScreenSample[2].x));

        /* F＝ ( Y0 ( X2YD1－X1YD2 )+Y1 ( X0YD2－X2YD0 )+Y2 ( X1YD0－X0YD1 )
         * )／K */
        Fn = (pScreenSample[2].x * pDisplayCoordinate[1].y
              - pScreenSample[1].x * pDisplayCoordinate[2].y)
                * pScreenSample[0].y
            + (pScreenSample[0].x * pDisplayCoordinate[2].y
               - pScreenSample[2].x * pDisplayCoordinate[0].y)
                * pScreenSample[1].y
            + (pScreenSample[1].x * pDisplayCoordinate[0].y
               - pScreenSample[0].x * pDisplayCoordinate[1].y)
                * pScreenSample[2].y;

        usTest_x = ((An * pScreenSample[3].x) + (Bn * pScreenSample[3].y) + Cn)
            / Divider; //取一个点计算X值
        usTest_y = ((Dn * pScreenSample[3].x) + (En * pScreenSample[3].y) + Fn)
            / Divider; //取一个点计算Y值

        usGap_x = (usTest_x > pDisplayCoordinate[3].x)
            ? (usTest_x - pDisplayCoordinate[3].x)
            : (pDisplayCoordinate[3].x
               - usTest_x); //实际X坐标与计算坐标的绝对差
        usGap_y = (usTest_y > pDisplayCoordinate[3].y)
            ? (usTest_y - pDisplayCoordinate[3].y)
            : (pDisplayCoordinate[3].y
               - usTest_y); //实际Y坐标与计算坐标的绝对差

        if ((usGap_x > 15) || (usGap_y > 15))
            ucRet = 0;
        pCalibrationFactor->dX_X = (An * 1.0f) / Divider;
        pCalibrationFactor->dX_Y = (Bn * 1.0f) / Divider;
        pCalibrationFactor->dX = (Cn * 1.0f) / Divider;

        pCalibrationFactor->dY_X = (Dn * 1.0f) / Divider;
        pCalibrationFactor->dY_Y = (En * 1.0f) / Divider;
        pCalibrationFactor->dY = (Fn * 1.0f) / Divider;
    }
    return ucRet;
}

static void stm32_xpt2046_calibrate(void)
{
    Touch_Calibration calibration = { 0 };
    Touch_Coordinate pos[4];
    Touch_Coordinate point[4] = {
        { 60, 80 },
        { 60, 240 },
        { 180, 240 },
        { 180, 80 },
    };

    struct lcddev_area_s area;
    uint16_t data[512];
    for (int i = 0; i < 512; i++) {
        data[i] = 0XF800;
    }

    int fd = open("/dev/lcd0", 0);
    for (int i = 0; i < 4; i++) {
        area.row_start = point[i].y - 3;
        area.row_end = point[i].y + 3;
        area.col_start = point[i].x - 3;
        area.col_end = point[i].x + 3;
        area.data = (uint8_t*)&data;

        ioctl(fd, LCDDEVIO_PUTAREA, &area);
        up_mdelay(300);

        while (stm32_gpioread(GPIO_LCDTP_IRQ) == 1)
            ;
        up_mdelay(100);

        pos[i].x = xpt2046_sendcmd(XPT2046_CMD_XPOSITION);
        pos[i].y = xpt2046_sendcmd(XPT2046_CMD_YPOSITION);

        syslog(LOG_INFO, "x: %d, y:%d \n", pos[i].x, pos[i].y);
    }

    Touch_Calculate_CalibrationFactor(point, pos, &calibration);
    
    syslog(LOG_INFO, "dx   : %f \n", calibration.dX);
    syslog(LOG_INFO, "dx_x : %f \n", calibration.dX_X);
    syslog(LOG_INFO, "dx_y : %f \n", calibration.dX_Y);
    syslog(LOG_INFO, "dy   : %f \n", calibration.dY);
    syslog(LOG_INFO, "dy_x : %f \n", calibration.dY_X);
    syslog(LOG_INFO, "dy_y : %f \n", calibration.dY_Y);
}
#endif
/****************************************************************************
 * Public Functions
 ****************************************************************************/
int stm32_xpt2046_register(int minor)
{
    char devname[DEV_NAMELEN];
    iinfo("minor %d\n", minor);

    xpt2046_gpio_init();

    snprintf(devname, sizeof(devname), DEV_FORMAT, minor);
    iinfo("Registering %s\n", devname);

    int ret = register_driver(devname, &g_xpt2046_fops, 0666, NULL);
    if (ret < 0) {
        ierr("ERROR: register_driver() failed: %d\n", ret);
        return ERROR;
    }

    int pid = kthread_create("lcd_tp", XPT2046_WORKER_THREAD_PRIORITY,
                             XPT2046_WORKER_STACKSIZE, xpt2046_worker, NULL);
    if (pid < 0) {
        ierr("ERROR: kthread_create() failed \n");
        return ERROR;
    }

    return OK;
}
