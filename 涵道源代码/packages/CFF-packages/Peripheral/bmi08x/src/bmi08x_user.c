#include "bmi08x_user.h"
#include <rtthread.h>
#include <rtdef.h>

#define DBG_TAG  "sensor.bosch.bmi088"
#define DBG_LVL  DBG_LOG
#include <rtdbg.h>

/*! Earth's gravity in m/s^2 */
#define SENSORS_GRAVITY_EARTH  (9.80665f)

rt_weak int8_t bmi08x_interface_init(bmi08x_dev_t bmi08x, uint8_t intf, uint8_t variant)
{
    LOG_E("bmi08x_interface_init not implemented");
    return BMI08X_E_INVALID_CONFIG;
}

/*!
 *  @brief Prints the execution status of the APIs.
 */
void bmi08x_error_codes_print_result(const char api_name[], int8_t rslt)
{
    if (rslt != BMI08X_OK)
    {
        if (rslt == BMI08X_E_NULL_PTR)
        {
            LOG_E("%s Error [%d] : Null pointer", api_name, rslt);
        }
        else if (rslt == BMI08X_E_COM_FAIL)
        {
            LOG_E("%s Error [%d] : Communication failure", api_name, rslt);
        }
        else if (rslt == BMI08X_E_DEV_NOT_FOUND)
        {
            LOG_E("%s Error [%d] : Device not found", api_name, rslt);
        }
        else if (rslt == BMI08X_E_OUT_OF_RANGE)
        {
            LOG_E("%s Error [%d] : Out of Range", api_name, rslt);
        }
        else if (rslt == BMI08X_E_INVALID_INPUT)
        {
            LOG_E("%s Error [%d] : Invalid input", api_name, rslt);
        }
        else if (rslt == BMI08X_E_CONFIG_STREAM_ERROR)
        {
            LOG_E("%s Error [%d] : Config stream error", api_name, rslt);
        }
        else if (rslt == BMI08X_E_RD_WR_LENGTH_INVALID)
        {
            LOG_E("%s Error [%d] : Invalid Read write length", api_name, rslt);
        }
        else if (rslt == BMI08X_E_INVALID_CONFIG)
        {
            LOG_E("%s Error [%d] : Invalid config", api_name, rslt);
        }
        else if (rslt == BMI08X_E_FEATURE_NOT_SUPPORTED)
        {
            LOG_E("%s Error [%d] : Feature not supported", api_name, rslt);
        }
        else
        {
            LOG_E("%s Error [%d] : Unknown error code", api_name, rslt);
        }
    }
}

/*!
 *  @brief This API is used to enable bmi08x interrupt
 *
 *  @return void
 *
 */
int8_t bmi08x_enable_interrupt(bmi08x_dev_t bmi08x)
{
    int8_t result;
    struct bmi08x_accel_int_channel_cfg accel_int_config;
    struct bmi08x_gyro_int_channel_cfg gyro_int_config;

    /*set accel interrupt pin configuration*/
    accel_int_config.int_channel = BMI08X_INT_CHANNEL_1;
    accel_int_config.int_type = BMI08X_ACCEL_DATA_RDY_INT;
    accel_int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
    accel_int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
    accel_int_config.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

    /*Enable accel data ready interrupt channel*/
    result = bmi08a_set_int_config((const struct bmi08x_accel_int_channel_cfg*)&accel_int_config, bmi08x);
    bmi08x_error_codes_print_result("bmi08a_set_int_config", result);

    if (result == BMI08X_OK)
    {
        /*set gyro interrupt pin configuration*/
        gyro_int_config.int_channel = BMI08X_INT_CHANNEL_3;
        gyro_int_config.int_type = BMI08X_GYRO_DATA_RDY_INT;
        gyro_int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
        gyro_int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
        gyro_int_config.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

        /*Enable gyro data ready interrupt channel*/
        result = bmi08g_set_int_config((const struct bmi08x_gyro_int_channel_cfg *)&gyro_int_config, bmi08x);
        bmi08x_error_codes_print_result("bmi08g_set_int_config", result);
    }

    return result;
}

/*!
 *  @brief This API is used to disable bmi08x interrupt
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
int8_t bmi08x_disable_interrupt(bmi08x_dev_t bmi08x)
{
    int8_t result;
    struct bmi08x_accel_int_channel_cfg accel_int_config;
    struct bmi08x_gyro_int_channel_cfg gyro_int_config;

    /*set accel interrupt pin configuration*/
    accel_int_config.int_channel = BMI08X_INT_CHANNEL_1;
    accel_int_config.int_type = BMI08X_ACCEL_DATA_RDY_INT;
    accel_int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
    accel_int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
    accel_int_config.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;

    /*Disable accel data ready interrupt channel*/
    result = bmi08a_set_int_config((const struct bmi08x_accel_int_channel_cfg*)&accel_int_config, bmi08x);
    bmi08x_error_codes_print_result("bmi08a_set_int_config", result);

    if (result == BMI08X_OK)
    {
        /*set gyro interrupt pin configuration*/
        gyro_int_config.int_channel = BMI08X_INT_CHANNEL_3;
        gyro_int_config.int_type = BMI08X_GYRO_DATA_RDY_INT;
        gyro_int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
        gyro_int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
        gyro_int_config.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;

        /*Disable gyro data ready interrupt channel*/
        result = bmi08g_set_int_config((const struct bmi08x_gyro_int_channel_cfg *)&gyro_int_config, bmi08x);
        bmi08x_error_codes_print_result("bmi08g_set_int_config", result);
    }

    return result;
}

/*!
 *  @brief This internal API is used to enable data synchronization interrupt.
 *
 *  @return int8_t
 *
 */
int8_t bmi08x_enable_data_synchronization_interrupt(bmi08x_dev_t bmi08x)
{
    int8_t result = BMI08X_OK;
    struct bmi08x_int_cfg int_config;
    struct bmi08x_data_sync_cfg sync_cfg;

    /*set accel interrupt pin configuration*/
    /*configure host data ready interrupt */
    int_config.accel_int_config_1.int_channel = BMI08X_INT_CHANNEL_1;
    int_config.accel_int_config_1.int_type = BMI08X_ACCEL_SYNC_INPUT;
    int_config.accel_int_config_1.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
    int_config.accel_int_config_1.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
    int_config.accel_int_config_1.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

    /*configure Accel syncronization input interrupt pin */
    int_config.accel_int_config_2.int_channel = BMI08X_INT_CHANNEL_2;
    int_config.accel_int_config_2.int_type = BMI08X_ACCEL_SYNC_DATA_RDY_INT;
    int_config.accel_int_config_2.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
    int_config.accel_int_config_2.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
    int_config.accel_int_config_2.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

    /*set gyro interrupt pin configuration*/
    int_config.gyro_int_config_1.int_channel = BMI08X_INT_CHANNEL_3;
    int_config.gyro_int_config_1.int_type = BMI08X_GYRO_DATA_RDY_INT;
    int_config.gyro_int_config_1.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;
    int_config.gyro_int_config_1.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
    int_config.gyro_int_config_1.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;

    int_config.gyro_int_config_2.int_channel = BMI08X_INT_CHANNEL_4;
    int_config.gyro_int_config_2.int_type = BMI08X_GYRO_DATA_RDY_INT;
    int_config.gyro_int_config_2.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;
    int_config.gyro_int_config_2.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
    int_config.gyro_int_config_2.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;

    /* Enable synchronization interrupt pin */
    result = bmi08a_set_data_sync_int_config(&int_config, bmi08x);
    bmi08x_error_codes_print_result("bmi08a_set_data_sync_int_config", result);
    
    /*! Mode (0 = off, 1 = 400Hz, 2 = 1kHz, 3 = 2kHz) */
    sync_cfg.mode = BMI08X_ACCEL_DATA_SYNC_MODE_400HZ;

    result = bmi08a_configure_data_synchronization(sync_cfg, bmi08x);
    bmi08x_error_codes_print_result("bmi08a_configure_data_synchronization", result);

    return result;
}

/*!
 *  @brief This internal API is used to disable data synchronization interrupt.
 *
 *  @return int8_t
 *
 */
int8_t bmi08x_disable_data_synchronization_interrupt(bmi08x_dev_t bmi08x)
{
    int8_t result;
    struct bmi08x_int_cfg int_config;
    struct bmi08x_data_sync_cfg sync_cfg;

    /*turn off the sync feature*/
    sync_cfg.mode = BMI08X_ACCEL_DATA_SYNC_MODE_OFF;

    result = bmi08a_configure_data_synchronization(sync_cfg, bmi08x);
    bmi08x_error_codes_print_result("bmi08a_configure_data_synchronization", result);

    /* Wait for 150ms to enable the data synchronization --delay taken care inside the function */
    /* configure synchronization interrupt pins */
    if (result == BMI08X_OK)
    {
        /*set accel interrupt pin configuration*/
        /*configure host data ready interrupt */
        int_config.accel_int_config_1.int_channel = BMI08X_INT_CHANNEL_1;
        int_config.accel_int_config_1.int_type = BMI08X_ACCEL_SYNC_INPUT;
        int_config.accel_int_config_1.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
        int_config.accel_int_config_1.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
        int_config.accel_int_config_1.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;

        /*configure Accel synchronization input interrupt pin */
        int_config.accel_int_config_2.int_channel = BMI08X_INT_CHANNEL_2;
        int_config.accel_int_config_2.int_type = BMI08X_ACCEL_SYNC_DATA_RDY_INT;
        int_config.accel_int_config_2.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
        int_config.accel_int_config_2.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
        int_config.accel_int_config_2.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;

        /*set gyro interrupt pin configuration*/
        int_config.gyro_int_config_1.int_channel = BMI08X_INT_CHANNEL_3;
        int_config.gyro_int_config_1.int_type = BMI08X_GYRO_DATA_RDY_INT;
        int_config.gyro_int_config_1.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
        int_config.gyro_int_config_1.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
        int_config.gyro_int_config_1.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;

        int_config.gyro_int_config_2.int_channel = BMI08X_INT_CHANNEL_4;
        int_config.gyro_int_config_2.int_type = BMI08X_GYRO_DATA_RDY_INT;
        int_config.gyro_int_config_2.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;
        int_config.gyro_int_config_2.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
        int_config.gyro_int_config_2.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;

        /* Disable synchronization interrupt pin */
        result = bmi08a_set_data_sync_int_config(&int_config, bmi08x);
        bmi08x_error_codes_print_result("bmi08a_set_data_sync_int_config", result);
    }

    return result;
}

/**
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
float lsb_to_mps2(int16_t val, int8_t g_range, uint8_t bit_width)
{
    float gravity;

    uint32_t half_scale = ((1 << bit_width) / 2);

    gravity = (float)((SENSORS_GRAVITY_EARTH * val * g_range) / half_scale);

    return gravity;
}

float lsb_to_G(int16_t val, int8_t g_range, uint8_t bit_width)
{
    float gravity;

    uint32_t half_scale = ((1 << bit_width) / 2);

    gravity = ((float)val * g_range / half_scale);

    return gravity;
}

/**
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    float half_scale = ((float)(1 << bit_width) / 2.0f);

    return (dps / ((half_scale) + BMI08X_GYRO_RANGE_2000_DPS)) * (val);
}


int8_t bmi08x_init(bmi08x_dev_t bmi08x)
{
    int8_t result;

    result = bmi08x_interface_init(bmi08x, BMI08X_SPI_INTF, BMI088_VARIANT);
    if (result != BMI08X_OK)
    {
        return result;
    }

    result = bmi08a_init(bmi08x);
    bmi08x_error_codes_print_result("bmi08a_init", result);
    if (result != BMI08X_OK)
    {
        return result;
    }

    result = bmi08g_init(bmi08x);
    bmi08x_error_codes_print_result("bmi08g_init", result);
    if (result != BMI08X_OK)
    {
        return result;
    }

    result = bmi08a_set_power_mode(bmi08x);
    bmi08x_error_codes_print_result("bmi08a_set_power_mode", result);
    bmi08x->delay_us(10, bmi08x);
    result = bmi08a_set_meas_conf(bmi08x);
    bmi08x_error_codes_print_result("bmi08a_set_meas_conf", result);
    bmi08x->delay_us(10, bmi08x);

    result = bmi08g_set_power_mode(bmi08x);
    bmi08x_error_codes_print_result("bmi08g_set_power_mode", result);
    bmi08x->delay_us(10, bmi08x);
    result = bmi08g_set_meas_conf(bmi08x);
    bmi08x_error_codes_print_result("bmi08g_set_meas_conf", result);
    bmi08x->delay_us(10, bmi08x);

    return result;
}
