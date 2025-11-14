#ifndef __ROBOT_DRIVER_H__
#define __ROBOT_DRIVER_H__
#include <rtthread.h>
#include "easy_math.h"
#include "cff_utils.h"
#include "pins_port.h"

#include "bus_servo.hpp"
#include "hub_motor.hpp"
#include "TinyFrame.h"

#include "AHRS.h"

#include "beep_port.h"
#include "tone_player.hpp"
#include "music_code.h"

#include "tusbh.h"
#include "tusbh_hid.h"
#include "hid.h"

using namespace CFF;

#define GAMEPAD_DEAD_ZONE 5

typedef hid_gamepad_xbox_report_t Gamepad_t;

/**
 * @brief setLedColor(color) 可用的LED颜色选项
 * 
 */
enum LED_Color
{
    LED_COLOR_OFF     = 0b000, // 关闭状态，无颜色
    LED_COLOR_RED     = 0b001, // 只有红色
    LED_COLOR_GREEN   = 0b010, // 只有绿色
    LED_COLOR_YELLOW  = 0b011, // 红色+绿色=黄色
    LED_COLOR_BLUE    = 0b100, // 只有蓝色
    LED_COLOR_MAGENTA = 0b101, // 红色+蓝色=品红色
    LED_COLOR_CYAN    = 0b110, // 绿色+蓝色=青色
    LED_COLOR_WHITE   = 0b111  // 红色+绿色+蓝色=白色
};

typedef struct switchgear
{
    uint16_t SG1_PWR : 1;
    uint16_t SG1_IO : 1;
    uint16_t SG2_PWR : 1;
    uint16_t SG2_IO : 1;
    uint16_t SG3_PWR : 1;
    uint16_t SG3_IO : 1;
    uint16_t SG4_PWR : 1;
    uint16_t SG4_IO : 1;
    uint16_t SG5_PWR : 1;
    uint16_t SG5_IO : 1;
    uint16_t SG6_PWR : 1;
    uint16_t SG6_IO : 1;
    uint16_t reserve : 4;
} switchgear_t;

typedef struct
{
    int16_t type;
    int16_t x;
    int16_t y;
    int16_t w;
    int16_t h;
    uint32_t timestamp;
} ball_t;

namespace io
{
    void setLedColor(uint8_t color);
    void setSG1(uint8_t pwr_state, uint8_t io_state);
    void setSG2(uint8_t pwr_state, uint8_t io_state);
    void setSG3(uint8_t pwr_state, uint8_t io_state);
    void setSG4(uint8_t pwr_state, uint8_t io_state);
    void setSG5(uint8_t pwr_state, uint8_t io_state);
    void setSG6(uint8_t pwr_state, uint8_t io_state);
}

namespace servo
{
    ServoPort *PortPointer();
}

namespace motor
{
    HubMotorPort *PortPointer();
}

namespace audio
{
    bool playMusic(const char *name, bool loop=false);
}

namespace usb
{
    Gamepad_t *GamepadPointer(void);
    bool isConnected(void);
}


#endif /* __ROBOT_DRIVER_H__ */