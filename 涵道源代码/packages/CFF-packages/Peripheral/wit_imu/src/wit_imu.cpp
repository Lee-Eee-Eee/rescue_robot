#include "wit_imu.h"
#include <string.h>
#include <math.h>

namespace CFF_Sensor
{

#define WIT_Z_TO_ZERO_CMD 0x05  // Z轴归零
#define WIT_ACC_CALI_CMD 0x67   // 加速度校准
#define WIT_HINSTALL_CMD 0x65   // 水平安装
#define WIT_VERTICAL_CMD 0x66   // 垂直安装

#ifdef WIT_IMU_USING_THREAD
static rt_thread_t wit_imu_thread = RT_NULL;
#endif

static rt_err_t serial_recv_ind_hook(rt_device_t dev, rt_size_t size)
{
    WIT_Imu *imu = (PortHandler *)(dev->user_data);
    imu->received(size);
    
    return RT_EOK;
}

#ifdef WIT_IMU_USING_THREAD
static void imu_thread_entry(void *parameter)
{
    WIT_Imu *imu = (WIT_Imu *)parameter;
    while (1)
    {
        imu.run();
    }
}
#endif

WIT_Imu::WIT_Imu(const char *device_name, long baud_rate)
{
    baud_ = baud_rate;
    device_name_ = device_name;
}

bool WIT_Imu::init(void)
{
    serial_ = rt_device_find(device_name_);
    if (serial_  == RT_NULL)
    {
        // Device Not Fond
        LOG_E("Serial device(%s) no found.", device_name_);
        RT_ASSERT(0);
    }

    if (serial_ != RT_Device_Class_Char)
    {
        LOG_E("Device(%s) type is not char.", device_name_);
        RT_ASSERT(0);
    }

    /* Initialize Semaphore */
    rx_sem_ = rt_sem_create(device_name_, 0, RT_IPC_FLAG_FIFO);
    if (rx_sem_ == RT_NULL)
    {
        LOG_E("Create Semaphore fail.");
        RT_ASSERT(0);
    }

    /* config Serial */
    struct serial_configure* config = &((struct rt_serial_device *)serial_)->config;
    config->baud_rate = this->baud_;
    rt_device_control(serial_, RT_DEVICE_CTRL_CONFIG, config);

    serial_->user_data = this;
    /* Set RX Callback */
    rt_device_set_rx_indicate(serial_, serial_recv_ind_hook);

    /* Open Serial */
#ifdef RT_USING_SERIAL_V2
    rt_device_open(serial_, RT_DEVICE_FLAG_RX_NON_BLOCKING | RT_DEVICE_FLAG_TX_BLOCKING);
#else
    rt_device_open(serial_, RT_DEVICE_FLAG_INT_RX);
#endif

#ifdef WIT_IMU_USING_THREAD
    if (wit_imu_thread == RT_NULL)
    {
        wit_imu_thread = rt_thread_create("imu", imu_thread_entry, this, 256, 1, 20)
        if (wit_imu_thread != RT_NULL)
            rt_thread_startup(wit_imu_thread);
    }
    else
    {
        LOG_E("Only one thread can be started.");
    }

#endif

    return true;
}


int WIT_Imu::read()
{
    uint8_t ch;
    if(rt_device_read(serial_, -1, &ch, 1) != 1) {
        rt_sem_take(rx_sem_, 0);
        return -1;
    };
    return ch;
}

int WIT_Imu::received(rt_size_t size)
{
    if (rx_sem_ == RT_NULL)
    {
        return 0;
    }

    rt_event_send(rx_sem_, SERIAL_EVT_RX_IND);

    return 0;
}

void WIT_Imu::write_cmd(uint8_t cmd)
{
    uint8_t WIT_write_cmd[3] = {0xFF,0xAA,0x00};
    WIT_write_cmd[2] = cmd;
    rt_device_write(serial_, 0, WIT_write_cmd, 3);
}

void WIT_Imu::run(void)
{
    uint8_t ch;
    while (rt_device_read(serial_, -1, &ch, 1) != 1)
    {
        /* 阻塞等待接收信号量，等到信号量后再次读取数据 */
        rt_sem_take(rx_sem_, RT_WAITING_FOREVER);
    }

    rx_buffer_[rx_cnt_++] = ch;
    if (rx_buffer_[0] != 0x55)
    {
        rx_cnt_ = 0;
        return;
    }

    if (rx_cnt_<11)
    {
        return;
    }

    rx_cnt_ = 0;
    if( checksum((uint8_t*)&rx_buffer_, 10) != rx_buffer_[10] )
    {
        return;
    }

    IMU_data_.update_time = ((unsigned long)rt_tick_get() * 1000 / RT_TICK_PER_SECOND);
    switch (rx_buffer_[1])
    {
        case 0x50:  memcpy(&IMU_data_.time,    &rx_buffer_[2], 8); break;    //时间
        case 0x51:  memcpy(&IMU_data_.acc,     &rx_buffer_[2], 8); break;    //加速度
        case 0x52:  memcpy(&IMU_data_.gyro,    &rx_buffer_[2], 8); break;    //角速度
        case 0x53:  memcpy(&IMU_data_.angle,   &rx_buffer_[2], 8); break;    //角度
        case 0x54:  memcpy(&IMU_data_.mag,     &rx_buffer_[2], 8); break;    //磁场
    }
    
}


uint8_t WIT_Imu::checksum(uint8_t *qdata,uint8_t len)
{
    uint8_t sum = 0;
    uint8_t i;

    for(i=0; i<len; i++)
    {
        sum += qdata[i];
    }

    return sum;
}


uint16_t WIT_Imu::getTime(const char* str)
{
    if (strcmp(str, "year") == 0)       //年
        return IMU_data_.time.year;

    if (strcmp(str, "month") == 0)      //月
        return IMU_data_.time.month;

    if (strcmp(str, "day") == 0)        //日
        return IMU_data_.time.day;

    if (strcmp(str, "hour") == 0)       //时
        return IMU_data_.time.hour;

    if (strcmp(str, "minute") == 0)     //分
        return IMU_data_.time.minute;

    if (strcmp(str, "second") == 0)     //秒
        return IMU_data_.time.second;

    if (strcmp(str, "milisecond") == 0) //毫秒
        return IMU_data_.time.milisecond;

    return 0;
}

double WIT_Imu::getAccX()
{
    return IMU_data_.acc.x / (32768.0/16.0);
}

double WIT_Imu::getAccY()
{
    return IMU_data_.acc.y / (32768.0/16.0);
}

double WIT_Imu::getAccZ()
{
    return IMU_data_.acc.z / (32768.0/16.0);
}

double WIT_Imu::getGyroX()
{
    return IMU_data_.gyro.x / (32768.0/2000.0);
}

double WIT_Imu::getGyroY()
{
    return IMU_data_.gyro.y / (32768.0/2000.0);
}

double WIT_Imu::getGyroZ()
{
    return IMU_data_.gyro.z / (32768.0/2000.0);
}

double WIT_Imu::getMagX()
{
    return IMU_data_.mag.x / (32768.0/180.0);
}

double WIT_Imu::getMagY()
{
    return IMU_data_.mag.y / (32768.0/180.0);
}

double WIT_Imu::getMagZ()
{
    return IMU_data_.mag.z / (32768.0/180.0);
}

int16_t WIT_Imu::getAccRawX()
{
    return IMU_data_.acc.x;
}

int16_t WIT_Imu::getAccRawY()
{
    return IMU_data_.acc.y;
}

int16_t WIT_Imu::getAccRawZ()
{
    return IMU_data_.acc.z;
}

int16_t WIT_Imu::getGyroRawX()
{
    return IMU_data_.gyro.x;
}

int16_t WIT_Imu::getGyroRawY()
{
    return IMU_data_.gyro.y;
}

int16_t WIT_Imu::getGyroRawZ()
{
    return IMU_data_.gyro.z;
}

int16_t WIT_Imu::getMagRawX()
{
    return IMU_data_.mag.x;
}

int16_t WIT_Imu::getMagRawY()
{
    return IMU_data_.mag.y;
}

int16_t WIT_Imu::getMagRawZ()
{
    return IMU_data_.mag.z;
}

double WIT_Imu::getRoll()
{
    return IMU_data_.angle.roll / (32768.0/180.0);
}

double WIT_Imu::getPitch()
{
    return IMU_data_.angle.pitch / (32768.0/180.0);
}
double WIT_Imu::getYaw()
{
    return IMU_data_.angle.yaw / (32768.0/180.0);
}

double WIT_Imu::getTemp()    //温度
{
    return IMU_data_.acc.temperature / 340.0 + 36.53;
}

Quaternion WIT_Imu::getQuaternion() 
{
    double yaw = getYaw();
    double pitch = getPitch();
    double roll = getRoll();

    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
 
    Quaternion q;
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;
 
    return q;
}

uint32_t WIT_Imu::getUpdateTime()
{
    return IMU_data_.update_time;
}

void WIT_Imu::resetYaw()
{
    write_cmd(WIT_Z_TO_ZERO_CMD);
}

} // CFF_Sensor
