/*
 * Copyright (c) 2006-2021, XRobot Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-04-25     zhb          first version
 */

#include <rtthread.h>
#include "spi_flash.h"
#include "spi_flash_sfud.h"
#include "drv_spi.h"
#include "pins_port.h"
#if defined(RT_USING_FAL)

#if defined(BSP_USING_SPI_FLASH)
static int spi_flash_init(void)
{
    rt_hw_spi_device_attach("spi2", FLASH_MOUNT_SPI_DEVICE_NAME, FLASH_CS_PIN);

    if (RT_NULL == rt_sfud_flash_probe(FAL_USING_NOR_FLASH_DEV_NAME, FLASH_MOUNT_SPI_DEVICE_NAME))
    {
        return -RT_ERROR;
    };

    return RT_EOK;
}
INIT_COMPONENT_EXPORT(spi_flash_init);
#endif

int user_fal_init(void)
{
    extern int fal_init(void);
    return fal_init();
}
INIT_COMPONENT_EXPORT(user_fal_init);
#endif


