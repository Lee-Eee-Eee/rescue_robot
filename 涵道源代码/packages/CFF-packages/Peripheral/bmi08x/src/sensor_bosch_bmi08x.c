#include <rtthread.h>
#include "sensor_bosch_bmi08x.h"
#include "bmi08x.h"
#include <math.h>
#ifdef RT_USING_SENSOR_V2
#include <drivers/sensor_v2.h>
#else
#include <drivers/sensor.h>
#endif

#define DBG_TAG  "sensor.bosch.bmi088"
#define DBG_LVL  DBG_LOG
#include <rtdbg.h>

#ifdef RT_USING_SENSOR_V2
#define ACCEL_TYPE                  RT_SENSOR_TYPE_ACCE
#define GYRO_TYPE                   RT_SENSOR_TYPE_GYRO
#define TEMP_TYPE                   RT_SENSOR_TYPE_TEMP
#define IS_ACCEL_SENSOR(sensor)     (sensor->info.type == RT_SENSOR_TYPE_ACCE)
#define IS_GYRO_SENSOR(sensor)      (sensor->info.type == RT_SENSOR_TYPE_GYRO)
#define IS_TEMP_SENSOR(sensor)      (sensor->info.type == RT_SENSOR_TYPE_TEMP)
#define SENSOR_OFF(power)           (power == RT_SENSOR_MODE_POWER_DOWN)
#define SENSOR_ON(power)            (power == RT_SENSOR_MODE_POWER_MEDIUM)
#define SENSOR_SLEEP(power)         (power == RT_SENSOR_MODE_POWER_LOW)
#else
#define ACCEL_TYPE                  RT_SENSOR_CLASS_ACCE
#define GYRO_TYPE                   RT_SENSOR_CLASS_GYRO
#define TEMP_TYPE                   RT_SENSOR_CLASS_TEMP
#define IS_ACCEL_SENSOR(sensor)     (sensor->info.type == RT_SENSOR_CLASS_ACCE)
#define IS_GYRO_SENSOR(sensor)      (sensor->info.type == RT_SENSOR_CLASS_GYRO)
#define IS_TEMP_SENSOR(sensor)      (sensor->info.type == RT_SENSOR_CLASS_TEMP)
#define SENSOR_OFF(power)           (power == RT_SENSOR_POWER_DOWN)
#define SENSOR_ON(power)            (power == RT_SENSOR_POWER_NORMAL)
#define SENSOR_SLEEP(power)         (power == RT_SENSOR_POWER_LOW)
#endif

static rt_err_t _bmi08x_init(bmi08x_dev_t bmi08x)
{
    rt_err_t result;

    result = bmi08x_init(bmi08x);
    if (result != BMI08X_OK)
    {
        goto __exit;
    }

    result = bmi08x_enable_interrupt(bmi08x);
    if (result != BMI08X_OK)
    {
        goto __exit;
    }

    return RT_EOK;

__exit:
    return -RT_ERROR;
}


static rt_err_t _bmi08x_get_id(rt_sensor_t sensor, rt_uint8_t *chip_id)
{
    struct bmi08x_dev *bmi08x = (struct bmi08x_dev *)sensor->parent.user_data;
    RT_ASSERT(bmi08x != RT_NULL);
    

    if (IS_ACCEL_SENSOR(sensor) || IS_TEMP_SENSOR(sensor))
    {
        *chip_id = bmi08x->accel_chip_id;
    }
    else if (IS_GYRO_SENSOR(sensor))
    {
        *chip_id = bmi08x->gyro_chip_id;
    }
    else 
    {
        LOG_E("Unsupported type %d", sensor->info.type);
        return -RT_ERROR;
    }
    return RT_EOK;
}

static rt_err_t _bmi08x_set_power_mode(rt_sensor_t sensor, rt_uint8_t power)
{   
    rt_err_t result;
    struct bmi08x_dev *bmi08x = (struct bmi08x_dev *)sensor->parent.user_data;
    RT_ASSERT(bmi08x != RT_NULL);


    if (IS_ACCEL_SENSOR(sensor))
    {
        if (SENSOR_OFF(power))
        {
            bmi08x->accel_cfg.power = BMI08X_ACCEL_PM_SUSPEND;
        }
        else if (SENSOR_ON(power))
        {
            bmi08x->accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;
        }
        else 
        {
            LOG_E("Unsupported power mode %d", power);
            return -RT_ERROR;        
        }
        
        result = bmi08a_set_power_mode(bmi08x);
        bmi08x_error_codes_print_result("bmi08a_set_power_mode", result);
    }
    else if (IS_GYRO_SENSOR(sensor))
    {
        if (SENSOR_SLEEP(power)) 
        {
            bmi08x->gyro_cfg.power = BMI08X_GYRO_PM_SUSPEND;
        }
        else if (SENSOR_ON(power))
        {
            bmi08x->gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;
        }
        else if (SENSOR_OFF(power))
        {
            bmi08x->gyro_cfg.power = BMI08X_GYRO_PM_DEEP_SUSPEND;
        }
        else 
        {
            LOG_E("Unsupported power mode %d", power);
            return -RT_ERROR;        
        }
        
        result = bmi08g_set_power_mode(bmi08x);
        bmi08x_error_codes_print_result("bmi08g_set_power_mode", result);
    }
    else 
    {
        LOG_E("Unsupported type %d", sensor->info.type);
        return -RT_ERROR;
    }
    return RT_EOK;
}

static rt_err_t _bmi08x_set_odr(rt_sensor_t sensor)
{
    rt_err_t result;
    struct bmi08x_dev *bmi08x = (struct bmi08x_dev *)sensor->parent.user_data;
    RT_ASSERT(bmi08x != RT_NULL);

    if (IS_ACCEL_SENSOR(sensor))
    {
        result = bmi08a_set_meas_conf(bmi08x);
        bmi08x_error_codes_print_result("bmi08a_set_meas_conf", result);
        
    }
    else if (IS_GYRO_SENSOR(sensor))
    {
        result = bmi08g_set_meas_conf(bmi08x);
        bmi08x_error_codes_print_result("bmi08g_set_meas_conf", result);
        
    }
    else 
    {
        LOG_E("Unsupported type %d", sensor->info.type);
        return -RT_ERROR;
    }
    return RT_EOK;
}


static rt_err_t _bmi08x_set_range(rt_sensor_t sensor, rt_uint32_t range)
{
    rt_err_t result = -RT_ERROR;
    static uint8_t bmi085_a_list[4] = {2, 4, 8, 16};
    static uint8_t bmi088_a_list[4] = {3, 6, 12, 24};
    static uint16_t bmi088_g_list[5] = {2000, 1000, 500, 250, 150};
    struct bmi08x_dev *bmi08x = (struct bmi08x_dev *)sensor->parent.user_data;
    RT_ASSERT(bmi08x != RT_NULL);

    if (IS_ACCEL_SENSOR(sensor))
    {
        uint8_t* selected_range_list;
        if (bmi08x->variant == BMI085_VARIANT)
        {
            selected_range_list = bmi085_a_list;
        }
        else if (bmi08x->variant == BMI088_VARIANT)
        {
            selected_range_list = bmi088_a_list;
        }
        for (uint16_t i=0; i<3; i++)
        {
            if (selected_range_list[i] == range)
            {
                bmi08x->accel_cfg.range = i;
                break;
            }
        }
        result = bmi08a_set_meas_conf(bmi08x);
        bmi08x_error_codes_print_result("bmi08a_set_meas_conf", result);
        
    }
    else if (IS_GYRO_SENSOR(sensor))
    {
        for (uint16_t i=0; i<5; i++)
        {
            if (bmi088_g_list[i] == range)
            {
                bmi08x->gyro_cfg.range = i;
                break;
            }
        }
        
        result = bmi08g_set_meas_conf(bmi08x);
        bmi08x_error_codes_print_result("bmi08g_set_meas_conf", result);
        
    }
    else 
    {
        LOG_E("Unsupported type %d", sensor->info.type);
        return -RT_ERROR;
    }
    return result;
}

static rt_err_t _bmi08x_self_test(rt_sensor_t sensor)
{
    rt_err_t result;
    struct bmi08x_dev *bmi08x = (struct bmi08x_dev *)sensor->parent.user_data;
    RT_ASSERT(bmi08x != RT_NULL);
    
    if (IS_ACCEL_SENSOR(sensor))
    {
        result = bmi08a_perform_selftest(bmi08x);
        bmi08x_error_codes_print_result("bmi08a_perform_selftest", result);
    }
    else if (IS_GYRO_SENSOR(sensor))
    {
        result = bmi08g_perform_selftest(bmi08x);
        bmi08x_error_codes_print_result("bmi08g_perform_selftest", result);
    }
    else 
    {
        LOG_E("Unsupported type %d", sensor->info.type);
        return -RT_ERROR;
    }
    return RT_EOK;
}


/**
* This function get the data of bmi088 sensor, unit: mg, mdps
 *
 * @param sensor the pointer of rt_sensor_device.
 * @param data the pointer of rt_sensor_data
 * 
 * @return the reading number.
 */
static rt_size_t _bmi08x_get_data(rt_sensor_t sensor, struct rt_sensor_data *data)
{
    rt_err_t result;
    struct bmi08x_dev *bmi08x = (struct bmi08x_dev *)sensor->parent.user_data;
    RT_ASSERT(bmi08x != RT_NULL);

    if (IS_ACCEL_SENSOR(sensor))
    {
        struct bmi08x_sensor_data bmi08x_accel;
        result = bmi08a_get_data(&bmi08x_accel, bmi08x);

        data->type = ACCEL_TYPE;
        rt_uint8_t range = 0;
        if (bmi08x->variant == BMI088_VARIANT)
        {
            range = 3*pow(2, bmi08x->accel_cfg.range);
        }
        else
        {
            range = 2*pow(2, bmi08x->accel_cfg.range);
        }
        data->data.acce.x = lsb_to_G(bmi08x_accel.x, range, 16) * 1000;
        data->data.acce.y = lsb_to_G(bmi08x_accel.y, range, 16) * 1000;
        data->data.acce.z = lsb_to_G(bmi08x_accel.z, range, 16) * 1000;
        data->timestamp = rt_sensor_get_ts();
        return 1;
    }
    else if (IS_GYRO_SENSOR(sensor))
    {
        struct bmi08x_sensor_data bmi08x_gyro;
        rt_uint32_t range = 2000;
        result = bmi08g_get_data(&bmi08x_gyro, bmi08x);

        for (rt_uint8_t i=0; i<bmi08x->gyro_cfg.range; i++)
        {
            range = range / 2;
        }

        data->type = GYRO_TYPE;
        data->data.gyro.x = lsb_to_dps(bmi08x_gyro.x, range, 16) * 1000;
        data->data.gyro.y = lsb_to_dps(bmi08x_gyro.y, range, 16) * 1000;
        data->data.gyro.z = lsb_to_dps(bmi08x_gyro.z, range, 16) * 1000;

        data->timestamp = rt_sensor_get_ts();
        return 1;
    }
    else if (IS_TEMP_SENSOR(sensor))
    {
        rt_int32_t temp;
        result = bmi08a_get_sensor_temperature(bmi08x, &temp);
        data->type = TEMP_TYPE;
        data->data.temp = temp/100;
        data->timestamp = rt_sensor_get_ts();
        return 1;
    }
    return 0;
}


/**
 * This function get the data of bmi08x sensor
 *
 * @param sensor the pointer of rt_sensor_device.
 * @param buf the pointer of data buffer.
 * @param len the length of data.
 * 
 * @return the reading number.
 */
#ifdef RT_USING_SENSOR_V2
static rt_ssize_t _bmi08x_fetch_data(struct rt_sensor_device *sensor, rt_sensor_data_t buf, rt_size_t len)
#else
static rt_ssize_t _bmi08x_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
#endif
{
    return _bmi08x_get_data(sensor, (struct rt_sensor_data *)buf);
}

/**
 * This function control the bmi08x sensor
 *
 * @param sensor the pointer of rt_sensor_device.
 * @param cmd the type of command.
 * @param args the null pointer of commmand parameter, notice the pointer is four bytes.
 * 
 * @return the reading number.
 */
static rt_err_t _bmi08x_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t result = RT_EOK;

    switch (cmd)
    {
    case RT_SENSOR_CTRL_GET_ID:
    {
        result = _bmi08x_get_id(sensor, (rt_uint8_t *)args);
    }break;
#ifdef RT_USING_SENSOR_V2
    case RT_SENSOR_CTRL_SET_ACCURACY_MODE:
#else
    case RT_SENSOR_CTRL_SET_RANGE:
#endif
    {
        rt_uint32_t range = (rt_uint32_t)args;
        result = _bmi08x_set_range(sensor, range);
    }break;
#ifdef RT_USING_SENSOR_V2
    case RT_SENSOR_CTRL_SET_POWER_MODE:
#else
    case RT_SENSOR_CTRL_SET_POWER:
#endif
    {
        result = _bmi08x_set_power_mode(sensor, (rt_uint32_t)args & 0xff);
    }break;
    case RT_SENSOR_CTRL_SELF_TEST:
    {
        result = _bmi08x_self_test(sensor);
    }break;
    default:
        return -RT_EINVAL;
    }
    return result;
}



static struct rt_sensor_ops sensor_ops =
{
    _bmi08x_fetch_data, 
    _bmi08x_control
};

static int rt_hw_bmx08x_gyro_init(const char *name, struct bmi08x_dev *bmi08x)
{
    rt_int8_t result;
    rt_sensor_t sensor = RT_NULL;

    sensor = rt_calloc(1, sizeof(struct rt_sensor_device));
    if (sensor == RT_NULL)
        return -(RT_ENOMEM);
    rt_memset(sensor, 0, sizeof(struct rt_sensor_device));

    sensor->info.type            = GYRO_TYPE;
    sensor->info.vendor          = RT_SENSOR_VENDOR_BOSCH;
    sensor->info.name            = "bmi08x_gyro";
    sensor->info.unit            = RT_SENSOR_UNIT_MDPS;
    sensor->info.intf_type       = RT_SENSOR_INTF_SPI;
    sensor->info.scale.range_max = 2000;
    sensor->info.scale.range_min = 125;
    sensor->info.acquire_min     = 10;

    sensor->ops = &sensor_ops;
    
    result = rt_hw_sensor_register(sensor, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
    if (result != RT_EOK)
    {
        LOG_E("device register: %d", result);
        rt_free(sensor);
        return -RT_ERROR;
    }
    sensor->parent.user_data = bmi08x;
    return RT_EOK;
}

static int rt_hw_bmx08x_accel_init(const char *name, struct bmi08x_dev *bmi08x)
{
    rt_int8_t result;
    rt_sensor_t sensor = RT_NULL;

    sensor = rt_calloc(1, sizeof(struct rt_sensor_device));
    if (sensor == RT_NULL)
        return -(RT_ENOMEM);
    rt_memset(sensor, 0, sizeof(struct rt_sensor_device));

    sensor->info.type            = ACCEL_TYPE;
    sensor->info.vendor          = RT_SENSOR_VENDOR_BOSCH;
    sensor->info.name            = "bmi08x_acc";
    sensor->info.unit            = RT_SENSOR_UNIT_MG;
    sensor->info.intf_type       = RT_SENSOR_INTF_SPI;
    sensor->info.scale.range_max = 24000;
    sensor->info.scale.range_min = 2000;
    sensor->info.acquire_min     = 10;

    sensor->ops = &sensor_ops;
    
    result = rt_hw_sensor_register(sensor, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
    if (result != RT_EOK)
    {
        LOG_E("device register: %d", result);
        rt_free(sensor);
        return -RT_ERROR;
    }
    sensor->parent.user_data = bmi08x;
    return RT_EOK;
}

static int rt_hw_bmx055_temp_init(const char *name, struct bmi08x_dev *bmi08x)
{
    rt_int8_t result;
    rt_sensor_t sensor = RT_NULL;

    sensor = rt_calloc(1, sizeof(struct rt_sensor_device));
    if (sensor == RT_NULL)
        return -(RT_ENOMEM);
    rt_memset(sensor, 0, sizeof(struct rt_sensor_device));

    sensor->info.type            = TEMP_TYPE;
    sensor->info.vendor          = RT_SENSOR_VENDOR_BOSCH;
    sensor->info.name            = "bmi08x_temp";
#ifdef RT_USING_SENSOR_V2
    sensor->info.unit            = RT_SENSOR_UNIT_CELSIUS;
#else
    sensor->info.unit            = RT_SENSOR_UNIT_DCELSIUS;
#endif
    sensor->info.intf_type       = RT_SENSOR_INTF_SPI;
    sensor->info.scale.range_max = 100;
    sensor->info.scale.range_min = 0;
    sensor->info.acquire_min     = 100;

    sensor->ops = &sensor_ops;

    result = rt_hw_sensor_register(sensor, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
    if (result != RT_EOK)
    {
        LOG_E("device register: %d", result);
        rt_free(sensor);
        return -RT_ERROR;
    }
    sensor->parent.user_data = bmi08x;
    return RT_EOK;
}


/**
 * This function initialize the bmi088
 *
 * @param name the name of bmi088, just first three characters will be used.
 * 
 * @return the reading number.
 */
int rt_hw_bmi08x_init(const char *name, struct bmi08x_dev *bmi08x)
{
    struct rt_sensor_intf *intf;
    rt_int8_t result;
    
    RT_ASSERT(name != NULL);
    RT_ASSERT(bmi08x != NULL);

    if (rt_hw_bmx08x_accel_init(name, bmi08x))
    {
        goto __exit;
    }
    if (rt_hw_bmx08x_gyro_init(name, bmi08x))
    {
        goto __exit;
    }
    if (rt_hw_bmx055_temp_init(name, bmi08x))
    {
        goto __exit;
    }

    result = _bmi08x_init(bmi08x);
    if (result != RT_EOK)
    {
        LOG_E("_bmi088_init err code: %d", result);
        goto __exit;
    }

    LOG_I("sensor init success");
    return RT_EOK;
__exit:
    return -RT_ERROR;
}
