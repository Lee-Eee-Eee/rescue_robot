#ifndef _WIT_IMU_H_
#define _WIT_IMU_H_

#include <rtthread.h>
#include <rtdevice.h>

#define DBG_TAG "Sensor.imu"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

namespace CFF_Sensor
{

struct Quaternion
{
    double w, x, y, z;
};

class WIT_Imu
{
public:
    WIT_Imu(const char *device_name, long baud_rate);

    bool init(void);
    void run(void);

    uint16_t getTime(const char*);                  //获取时间，'Y'年，'M'月，'D'天，'h'时，'m'分，'s'秒，'l'毫秒
    double   getAccX();                             //获取加速度
    double   getAccY();                             //获取加速度
    double   getAccZ();                             //获取加速度
    double   getGyroX();                            //获取角速度
    double   getGyroY();                            //获取角速度
    double   getGyroZ();                            //获取角速度
    double   getMagX();                             //获取磁场
    double   getMagY();                             //获取磁场
    double   getMagZ();                             //获取磁场
    int16_t  getAccRawX();                          //获取加速度计原始数据
    int16_t  getAccRawY();                          //获取加速度计原始数据
    int16_t  getAccRawZ();                          //获取加速度计原始数据
    int16_t  getGyroRawX();                         //获取陀螺仪原始数据
    int16_t  getGyroRawY();                         //获取陀螺仪原始数据
    int16_t  getGyroRawZ();                         //获取陀螺仪原始数据
    int16_t  getMagRawX();                          //获取磁力计原始数据
    int16_t  getMagRawY();                          //获取磁力计原始数据
    int16_t  getMagRawZ();                          //获取磁力计原始数据
    double   getRoll();
    double   getPitch();
    double   getYaw();
    double   getTemp();                             //温度
    Quaternion getQuaternion();                     //四元素
    uint32_t getUpdateTime();

    void resetYaw();

    int received(rt_size_t size);

private:
    int read();
    void write_cmd(uint8_t cmd);
    uint8_t checksum(uint8_t *qdata,uint8_t len);

    long baud_;                 //serial baudrate
    const char* device_name_;   //serial port name
    rt_device_t serial_;        //serial device handle
    rt_sem_t rx_sem_;           //event handle
    rt_bool_t status_;          //connect status
    uint8_t rx_buffer_[64]={0};
    uint8_t rx_cnt_ = 0;

    struct
    {
        struct
        {
            uint8_t  year;
            uint8_t  month;
            uint8_t  day;
            uint8_t  hour;
            uint8_t  minute;
            uint8_t  second;
            uint16_t milisecond;
        }time;
        struct
        {
            int16_t x;
            int16_t y;
            int16_t z;
            int16_t temperature;
        }acc;
        struct
        {
            int16_t x;
            int16_t y;
            int16_t z;
            int16_t temperature;
        }gyro;
        struct
        {
            int16_t roll;
            int16_t pitch;
            int16_t yaw;
            int16_t temperature;
        }angle;
        struct
        {
            int16_t x;
            int16_t y;
            int16_t z;
            int16_t temperature;
        }mag;

        uint32_t update_time;

    } IMU_data_;
};

} // CFF_Sensor

#endif // _WIT_IMU_H_