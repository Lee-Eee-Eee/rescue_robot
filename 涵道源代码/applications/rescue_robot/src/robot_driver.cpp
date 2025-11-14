#include "robot_driver.h"
#include "robot_config.h"

#define DBG_TAG "Robot.driver"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/*>>>>>> Driver IO device >>>>>>*/
namespace io
{
#define SG4_PWM_DEV_CHANNEL     2
#define SG4_PWM_START_VALUE     900000
#define SG4_PWM_MAX_VALUE       600000
static switchgear_t io_SGs = {0};
struct rt_device_pwm *SG4_pwm_dev;
uint32_t catapult_timestamp = 0;

/**
 * @brief 初始化引脚IO
 * 
 */
void init(void)
{
    rt_pin_mode(SG1_PWR_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(SG1_IO_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(SG2_PWR_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(SG2_IO_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(SG3_PWR_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(SG3_IO_PIN, PIN_MODE_OUTPUT);
#ifdef SWITCHGEAR_4_USE_PWM
    /* 使能外设4接口的PWM功能 */
    rt_pin_mode(SG4_PWR_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(SG4_PWR_PIN, PIN_HIGH);
    SG4_pwm_dev = (struct rt_device_pwm *)rt_device_find("pwm2");
#else
    rt_pin_mode(SG4_PWR_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(SG4_IO_PIN, PIN_MODE_OUTPUT);
#endif
    rt_pin_mode(SG5_PWR_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(SG5_IO_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(SG6_PWR_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(SG6_IO_PIN, PIN_MODE_OUTPUT);

    rt_pin_mode(LEDR_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(LEDG_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(LEDB_PIN, PIN_MODE_OUTPUT);

    rt_pin_mode(IN1_PIN, PIN_MODE_INPUT);
    rt_pin_mode(IN2_PIN, PIN_MODE_INPUT);
    rt_pin_mode(IN3_PIN, PIN_MODE_INPUT);
    rt_pin_mode(IN4_PIN, PIN_MODE_INPUT);

    LOG_I("IO init success");
}

void updata(void)
{
#ifdef SWITCHGEAR_4_USE_PWM
    uint32_t time = millis();
    if (time > 3500 && time < 6000)
    {
        rt_pwm_set(SG4_pwm_dev, SG4_PWM_DEV_CHANNEL, 2000000, SG4_PWM_START_VALUE);
        rt_pwm_enable(SG4_pwm_dev, SG4_PWM_DEV_CHANNEL);
    }
#endif
#ifdef SWITCHGEAR_6_CATAPULT
    if (millis() - catapult_timestamp > 250)
    {
        rt_pin_write(SG6_PWR_PIN, 0);
        rt_pin_write(SG6_IO_PIN, 1);
    }
#endif
}

/**
 * @brief 根据颜色编码设置LED灯的颜色
 * 
 * @param color ([LED_Color]枚举) 代表LED颜色的位字段值, 可用的颜色值定义在LED_Color枚举中.
 */
void setLedColor(uint8_t color)
{
    if (color & LED_COLOR_RED)
        rt_pin_write(LEDR_PIN, PIN_HIGH);
    else
        rt_pin_write(LEDR_PIN, PIN_LOW);

    if (color & LED_COLOR_GREEN)
        rt_pin_write(LEDG_PIN, PIN_HIGH);
    else
        rt_pin_write(LEDG_PIN, PIN_LOW);

    if (color & LED_COLOR_BLUE)
        rt_pin_write(LEDB_PIN, PIN_HIGH);
    else
        rt_pin_write(LEDB_PIN, PIN_LOW);
}

/**
 * @brief 设置外设1接口状态
 * 
 * @param pwr_state 外设接口电源
 * @param io_state 外设接口IO
 */
void setSG1(uint8_t pwr_state, uint8_t io_state)
{
    rt_pin_write(SG1_PWR_PIN, pwr_state);
    rt_pin_write(SG1_IO_PIN, io_state);
}

void setSG2(uint8_t pwr_state, uint8_t io_state)
{
    rt_pin_write(SG2_PWR_PIN, pwr_state);
    rt_pin_write(SG2_IO_PIN, io_state);
}

void setSG3(uint8_t pwr_state, uint8_t io_state)
{
    rt_pin_write(SG3_PWR_PIN, pwr_state);
    rt_pin_write(SG3_IO_PIN, io_state);
}

/**
 * @brief 设置外设1接口状态
 * 
 * @param pwr_state 外设接口电源
 * @param io_state 外设接口IO, 如果在PWM模式可以255级细分控制输出
 */
void setSG4(uint8_t pwr_state, uint8_t io_state)
{
#ifdef SWITCHGEAR_4_USE_PWM
    uint32_t power = SG4_PWM_START_VALUE-io_state*940;
    rt_pin_write(SG4_PWR_PIN, PIN_HIGH);
    rt_pwm_set(SG4_pwm_dev, SG4_PWM_DEV_CHANNEL, 2000000, power);
#else
    rt_pin_write(SG4_PWR_PIN, pwr_state);
    rt_pin_write(SG4_IO_PIN, io_state);
#endif
}

void setSG5(uint8_t pwr_state, uint8_t io_state)
{
    rt_pin_write(SG5_PWR_PIN, pwr_state);
    rt_pin_write(SG5_IO_PIN, io_state);
}

void setSG6(uint8_t pwr_state, uint8_t io_state)
{
#ifdef SWITCHGEAR_6_CATAPULT
    if (millis() - catapult_timestamp < 1000) // 1s只触发一次击球，给电容充电留时间
    {
        catapult_timestamp =millis();
        rt_pin_write(SG6_PWR_PIN, 1);
        rt_pin_write(SG6_IO_PIN, 1);
    }
#else
    rt_pin_write(SG6_PWR_PIN, pwr_state);
    rt_pin_write(SG6_IO_PIN, io_state);
#endif
}

}
/*<<<<<< Driver IO device <<<<<<*/

/*>>>>>> Driver Servo device >>>>>>*/
namespace servo
{
ServoPort *port = RT_NULL;

/**
 * @brief 初始化总线舵机接口
 * 
 */
void init(void)
{
    port = new ServoPort("uart4");
    if (port == RT_NULL)
    {
        LOG_E("Servo port creation fail");
        return;
    }
    if (!port->init())
    {
        LOG_E("Servo init fail");
        return;
    }
    if (!port->open())
    {
        LOG_E("Servo open fail");
        return;
    }
    
    LOG_I("Servo init success");
}

/**
 * @brief 获取舵机接口
 * 
 * @return ServoPort* 总线舵机接口指针
 */
ServoPort *PortPointer()
{
    return port;
}

}
/*<<<<<< Driver Servo device <<<<<<*/

/*>>>>>> Driver Motor device >>>>>>*/
namespace motor
{
HubMotorPort *port = RT_NULL;

/**
 * @brief 初始化电机接口
 * 
 */
void init(void)
{
    port = new HubMotorPort("uart6", 115200, RS485_EN_PIN, PIN_HIGH);
    if (port == RT_NULL)
    {
        LOG_E("Motor port creation fail");
        return;
    }
    if (!port->init())
    {
        LOG_E("Motor init fail");
        return;
    }
    if (!port->open())
    {
        LOG_E("Motor open fail");
        return;
    }

    LOG_I("Motor init success");
}

/**
 * @brief 获取电机接口
 * 
 * @return ServoPort* 电机接口指针
 */
HubMotorPort *PortPointer()
{
    return port;
}

}
/*<<<<<< Driver Motor device <<<<<<*/

/*>>>>>> Driver IMU device >>>>>>*/
namespace imu
{
rt_device_t gyro_sensor;
rt_device_t accle_sensor;

/**
 * @brief 陀螺仪接口
 * 
 */
void init(void)
{
    gyro_sensor = rt_device_find("gy-bmi");
    accle_sensor = rt_device_find("ac-bmi");
}

void update(void)
{
    
}

}
/*<<<<<< Driver IMU device <<<<<<*/

/*>>>>>> Driver Audio device >>>>>>*/
namespace audio
{

static TonePlayer player;

/**
 * @brief 蜂鸣器播放器初始化
 * 
 */
void init(void)
{
    beep_on();
    player.setCallback([](uint32_t freq, uint16_t volume)
    {
        beep_set(freq, volume);
    });
    LOG_I("Audio init success");
}

/**
 * @brief 更新播放器时间
 * 
 * 线程中循环执行
 */
void update(void)
{
    player.update(millis());
}

/**
 * @brief 从 MusicList 选择播放的音乐
 * 
 * @param name 音乐名字
 * @param loop 是否循环播放
 * @return true 
 * @return false 
 */
bool playMusic(const char *name, bool loop)
{
    bool retval = false;
    for (int i = 0; i < sizeof(MusicList) / sizeof(MusicList[0]); i++)
    {
        if (rt_strcmp(name, MusicList[i].name) == 0)
        {
            player.play(MusicList[i].mc, MusicList[i].length, loop);
            retval = true;
            break;
        }
    }
    return retval;
}

/**
 * @brief 停止播放音乐
 * 
 */
void stop(void)
{
    player.stop();
}

}
/*<<<<<< Driver Audio device <<<<<<*/

/*>>>>>> Driver visual device >>>>>>*/
namespace visual
{
rt_device_t visual_port;
TinyFrame *visual_tf;
ball_t balls[10]; 

TF_Result visual_listener(TinyFrame *tf, TF_Msg *msg)
{
//    msg.
    return TF_STAY;
}

void init(void)
{
    visual_port = rt_device_find("uart3");
    if (visual_port == RT_NULL)
    {
        LOG_E("Can't find visual serial device");
        return;
    }
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    rt_device_control(visual_port, RT_DEVICE_CTRL_CONFIG, &config);
    if ( rt_device_open(visual_port, RT_DEVICE_FLAG_INT_RX) != RT_EOK)
    {
        LOG_E("Visual serial open fail.");
        return;
    }
    visual_tf = TF_Init(TF_MASTER);
    if (visual_tf == RT_NULL)
    {
        LOG_E("TinyFrame create fail.");
        return;
    }

    TF_AddGenericListener(visual_tf, visual_listener);
}

void update()
{
    static uint8_t ch;
    if (visual_port == RT_NULL)
    {
        LOG_E("Can't find visual serial device");
        return;
    }
    if (rt_device_read(visual_port, -1, &ch, 1) == 1)
    {
        TF_Accept(visual_tf, &ch, 1);
    }
    TF_Tick(visual_tf);
}

}
void TF_WriteImpl(TinyFrame *tf, const uint8_t *buff, uint32_t len)
{
    rt_device_write(visual::visual_port, 0, buff, len);
}
/*<<<<<< Driver visual device <<<<<<*/

/*>>>>>> Driver USB device >>>>>>*/
namespace usb
{
tusb_mq_t *host_mq;
static tusb_host_t *host_fs;
static tusbh_root_hub_t root_fs;
static Gamepad_t gamepad_;
static uint32_t timestamp;

int gamepad_xfered(tusbh_ep_info_t *ep);

const tusbh_interface_backend_t gamepad_backend = {
    .vid = 0,                                   // 供应商ID, 这里设为0, 通常需要根据实际设备设置
    .pid = 0,                                   // 产品ID, 同上
    .bInterfaceClass = USB_CLASS_VEND_SPECIFIC, // 自定义接口类别，表明是厂商特定设备
    .bInterfaceSubClass = 0x5d,                 // 子类别，特定于游戏手柄
    .bInterfaceProtocol = 1,                    // 协议, 具体意义依据设备而定
    .init = tusbh_hid_init,                     // 初始化函数指针
    .deinit = tusbh_hid_deinit,                 // 反初始化函数指针
    .data_xfered = gamepad_xfered,              // 数据传输完成回调函数
    .desc = "Gamepad",                          // 设备描述字符串, 便于识别
};

static const tusbh_hid_class_t cls_gamepad = {
    .backend = &gamepad_backend,
};

static const tusbh_hid_class_t cls_hid = {
    .backend = &tusbh_hid_backend,
//    .on_recv_data = hid_recv_process,
//    .on_send_done = hid_send_process,
};

static const tusbh_class_reg_t class_table[] = {
    (tusbh_class_reg_t)&cls_gamepad,
    (tusbh_class_reg_t)&cls_hid,
    0,
};

/**
 * @brief 手柄数据解析函数
 * 
 * @param ep 
 * @return int 
 */
int gamepad_xfered(tusbh_ep_info_t *ep)
{
    uint16_t vid = ep->interface->device->device_desc.idVendor;
    uint16_t pid = ep->interface->device->device_desc.idProduct;
    
    rt_memcpy(&gamepad_, ((char *)(ep->data)) + 2, 12);
    if (gamepad_.x == -1 && gamepad_.y == -1 && gamepad_.z == -1 && gamepad_.rz == -1)
    {
        gamepad_.buttons = 0;
        gamepad_.hat = 0;
        gamepad_.lt = 0;
        gamepad_.rt = 0;
    }
    // if (abs(gamepad_.x)<GAMEPAD_DEAD_ZONE) gamepad_.x = 0;
    // if (abs(gamepad_.y)<GAMEPAD_DEAD_ZONE) gamepad_.y = 0;
    // if (abs(gamepad_.z)<GAMEPAD_DEAD_ZONE) gamepad_.z = 0;
    // if (abs(gamepad_.rz)<GAMEPAD_DEAD_ZONE) gamepad_.rz = 0;
        
    timestamp = millis();
    return 0;
}

/**
 * @brief usb设备初始化
 * 
 */
void init(void)
{
    host_mq = tusb_mq_create();
    tusb_mq_init(host_mq);

    rt_pin_mode(USB_PWR_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(USB_PWR_PIN, PIN_HIGH);

    host_fs = tusb_get_host(TUSB_APP_USB_CORE);
    root_fs.mq = host_mq;
    root_fs.id = "FS";
    root_fs.support_classes = class_table;
    tusb_host_init(host_fs, &root_fs);
    tusb_open_host(host_fs);
    rt_memset(&gamepad_, 0, sizeof(Gamepad_t));

    LOG_I("USB init success");
}

/**
 * @brief 更新usb消息
 * 
 * 线程中循环执行
 */
void update(void)
{
    tusbh_msg_loop(host_mq);
}

/**
 * @brief 获取手柄指针
 * 
 * @return Gamepad_t* 手柄指针
 */
Gamepad_t *GamepadPointer(void)
{
    return &gamepad_;
}

bool isConnected(void)
{
    return (millis() - timestamp) < 100;
}

}
/*<<<<<< Driver USB device <<<<<<*/


/*>>>>>>>>>>> Thread >>>>>>>>>>>>*/
#define MISC_THREAD_PRIORITY    25
#define MISC_THREAD_STACK_SIZE  1024
#define MISC_THREAD_TIMESLICE   5

#define USB_THREAD_PRIORITY     25
#define USB_THREAD_STACK_SIZE   1024
#define USB_THREAD_TIMESLICE    10

static rt_thread_t misc_thread = RT_NULL;
static rt_thread_t usb_thread = RT_NULL;

static void misc_thread_entry(void *parameter)
{
    while (1)
    {
        io::updata();
        visual::update();
        audio::update();
        delay(1);
    }
}

static void usb_thread_entry(void *parameter)
{
    while (1)
    {
        usb::update();
    }
}

/**
 * @brief 机器人使用硬件驱动统一初始化位置
 * 
 * 这个函数跟随RT-Thread系统自动运行
 * @return int 
 */
int robot_driver_init(void)
{
    io::init();
    motor::init();
    servo::init();
    usb::init();
    audio::init();
    imu::init();
    visual::init();

    misc_thread = rt_thread_create("misc",
                            misc_thread_entry, RT_NULL,
                            MISC_THREAD_STACK_SIZE,
                            MISC_THREAD_PRIORITY, 
                            MISC_THREAD_TIMESLICE);

    if (misc_thread != RT_NULL)
        rt_thread_startup(misc_thread);

    usb_thread = rt_thread_create("usbh",
                            usb_thread_entry, RT_NULL,
                            USB_THREAD_STACK_SIZE,
                            USB_THREAD_PRIORITY,
                            USB_THREAD_TIMESLICE);

    if (usb_thread != RT_NULL)
        rt_thread_startup(usb_thread);

    return 0;
}
INIT_APP_EXPORT(robot_driver_init);