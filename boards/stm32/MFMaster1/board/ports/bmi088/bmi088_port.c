/*
 * Copyright (c) 2006-2021, XRobot Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-09-09     zhb          first version
 */
#include <rtthread.h>
#include "drv_spi.h"
#include "pins_port.h"
#include "sensor_bosch_bmi08x.h"

/******************************************************************************/
/*!                       Macro definitions                                   */
#define BMI088_BUS_NAME         "spi2"
#define BMI088A_SPI_NAME        "spi_bmia"
#define BMI088G_SPI_NAME        "spi_bmig"
#define BMI08X_READ_WRITE_LEN   UINT8_C(46)
#define BMI088_SPI_MAX_SPEED    (10 * 1000 * 1000) // M

/******************************************************************************/
/*!                Static variable definition                                 */
struct bmi08x_dev bmi088;

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * @brief Function for initialization of SPI bus.
 */
static int8_t bmi088_spi_init(struct bmi08x_dev *bmi08x)
{
    rt_hw_spi_device_attach(BMI088_BUS_NAME, BMI088A_SPI_NAME, BMI088A_CS_PIN);
    rt_hw_spi_device_attach(BMI088_BUS_NAME, BMI088G_SPI_NAME, BMI088G_CS_PIN);

    bmi08x->intf_ptr_accel = rt_device_find(BMI088A_SPI_NAME);
    bmi08x->intf_ptr_gyro = rt_device_find(BMI088G_SPI_NAME);

    if ((bmi08x->intf_ptr_accel == RT_NULL) || (bmi08x->intf_ptr_gyro == RT_NULL))
    {
        rt_kprintf("Can't find device:'%s' of '%s'", BMI088A_SPI_NAME, BMI088G_SPI_NAME);
    }

    if (((rt_device_t)bmi08x->intf_ptr_accel)->type == RT_Device_Class_SPIDevice)
    {
        struct rt_spi_configuration cfg;

        cfg.data_width = 8;
        cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_0 | RT_SPI_MSB;
        cfg.max_hz = BMI088_SPI_MAX_SPEED; /* Set spi max speed */
        rt_spi_configure((struct rt_spi_device *)bmi08x->intf_ptr_accel, &cfg);
    }

    if (((rt_device_t)bmi08x->intf_ptr_gyro)->type == RT_Device_Class_SPIDevice)
    {
        struct rt_spi_configuration cfg;

        cfg.data_width = 8;
        cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_0 | RT_SPI_MSB;
        cfg.max_hz = BMI088_SPI_MAX_SPEED; /* Set spi max speed */
        rt_spi_configure((struct rt_spi_device *)bmi08x->intf_ptr_gyro, &cfg);
    }

    return 0;
}

/*!
 * @brief Function for reading the sensor's registers through SPI bus.
 */
static int8_t bmi088_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    reg_addr |= 0x80;
    
    rt_spi_send_then_recv((struct rt_spi_device *)intf_ptr, &reg_addr, 1, reg_data, length);
    
    return 0;
}

/*!
 * @brief Function for writing the sensor's registers through SPI bus.
 */
static int8_t bmi088_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    reg_addr &= 0x7f;
    
    rt_spi_send_then_send((struct rt_spi_device *)intf_ptr, &reg_addr, 1, reg_data, length);
    
    return RT_EOK;
}


/*!
 * @brief This function provides the delay for required time (Microseconds) as per the input provided in some of the
 * APIs.
 */
static void bmi088_delay(uint32_t period_us, void *intf_ptr)
{
    if (period_us < 1000)
    {
        rt_hw_us_delay(period_us);
    }
    else
    {
        rt_thread_mdelay(period_us/1000);
    }
}

static void bmi088_cfg(struct bmi08x_dev *bmi08x)
{
    bmi08x->accel_cfg.odr = BMI08X_ACCEL_ODR_800_HZ;
    bmi08x->accel_cfg.range = BMI088_ACCEL_RANGE_3G;
    bmi08x->accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;   /*user_accel_power_modes[user_bmi088_accel_low_power]; */
    bmi08x->accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL;      /* Bandwidth and OSR are same */
    
    bmi08x->gyro_cfg.odr = BMI08X_GYRO_BW_116_ODR_1000_HZ;
    bmi08x->gyro_cfg.range = BMI08X_GYRO_RANGE_2000_DPS;
    bmi08x->gyro_cfg.bw = BMI08X_GYRO_BW_116_ODR_1000_HZ;
    bmi08x->gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;
}

/*!
 *  @brief Function to select the interface between SPI and I2C.
 */
int8_t bmi08x_interface_init(struct bmi08x_dev *bmi08x, uint8_t intf, uint8_t variant)
{
    int8_t rslt = BMI08X_OK;

    if (bmi08x != NULL)
    {
        /* Bus configuration : I2C */
        if (intf == BMI08X_I2C_INTF)
        {
            rt_kprintf("BMI088 not support I2C temporarily");
        }

        /* Bus configuration : SPI */
        else if (intf == BMI08X_SPI_INTF)
        {
            /* To initialize the user SPI function */
            bmi088_spi_init(bmi08x);

            bmi08x->intf = BMI08X_SPI_INTF;
            bmi08x->read = bmi088_spi_read;
            bmi08x->write = bmi088_spi_write;

        }

        /* Selection of bmi085 or bmi088 sensor variant */
        bmi08x->variant = variant;

        /* Configure delay in microseconds */
        bmi08x->delay_us = bmi088_delay;

        /* Configure max read/write length (in bytes) ( Supported length depends on target machine) */
        bmi08x->read_write_len = BMI08X_READ_WRITE_LEN;
        bmi088_cfg(bmi08x);
    }
    else
    {
        rslt = BMI08X_E_NULL_PTR;
    }

    return rslt;

}

static int bmi08x_port_init(void)
{
    rt_hw_bmi08x_init("bmi", &bmi088);

    return RT_EOK;
}
INIT_COMPONENT_EXPORT(bmi08x_port_init);
        
    