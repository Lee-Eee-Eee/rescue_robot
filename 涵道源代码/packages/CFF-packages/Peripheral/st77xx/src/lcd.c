/**
 ******************************************************************************
* @file    lcd.c
* @brief   STxx LCD 驱动代码 
* @details  
******************************************************************************
* @attention
*
* Copyright (c) 2024, 创非凡未来技术研究院.
* All rights reserved.
*
* This software is licensed under terms that can be found in the LICENSE file
* in the root directory of this software component.
* If no LICENSE file comes with this software, it is provided AS-IS.
*
******************************************************************************
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2024/07/17  <td>0.1.0    <td>zhb       <td>创建初始版本
* </table>
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include <rtdevice.h>
#include <drv_spi.h>
#include "lcd.h"
#include "pins_port.h"

#define DBG_SECTION_NAME    "LCD"
#define DBG_LEVEL           DBG_LOG
#include <rtdbg.h>

#ifdef PKG_USING_CFF_LCD

#ifdef CONFIG_LCD_DRIVE_ST7735
#include "st7735.h"
#endif

#ifdef CONFIG_LCD_DRIVE_ST7789
#include "st7789.h"
#endif

/* Private defines -----------------------------------------------------------*/
// #define LCD_CS_PIN          CONFIG_LCD_SPI_CS_PIN
// #define LCD_DC_PIN          CONFIG_LCD_DC_PIN
// #ifdef CONFIG_LCD_USING_RESET_PIN
// #define LCD_RESET_PIN       CONFIG_LCD_RESET_PIN
// #endif
// #ifdef CONFIG_LCD_USING_BL_PIN
// #define LCD_BL_PIN          CONFIG_LCD_BL_PIN
// #endif

#ifndef CONFIG_LCD_DISPLAY_ORIENTATION
#define CONFIG_LCD_DISPLAY_ORIENTATION   LCD_PORTRAIT
#endif

#if   ( CONFIG_LCD_DISPLAY_ORIENTATION == LCD_PORTRAIT )
#define LCD_WIDTH    LCD_W
#define LCD_HEIGHT   LCD_H
#elif (CONFIG_LCD_DISPLAY_ORIENTATION == LCD_PORTRAIT_INVERTED)
#define LCD_WIDTH    LCD_W
#define LCD_HEIGHT   LCD_H
#elif (CONFIG_LCD_DISPLAY_ORIENTATION == LCD_LANDSCAPE)
#define LCD_WIDTH    LCD_H
#define LCD_HEIGHT   LCD_W
#elif (CONFIG_LCD_DISPLAY_ORIENTATION == LCD_LANDSCAPE_INVERTED)
#define LCD_WIDTH    LCD_H
#define LCD_HEIGHT   LCD_W
#else
#define LCD_WIDTH    LCD_W
#define LCD_HEIGHT   LCD_H
#endif

#define LCD_XSTART   0
#define LCD_YSTART   0

#define LCD_CLEAR_SEND_NUMBER (LCD_WIDTH*LCD_HEIGHT)

// Delay between some initialisation commands
#define TFT_INIT_DELAY      0x80

#define TFT_NOP             0x00
#define TFT_SWRST           0x01

#define TFT_SLPIN           0x10
#define TFT_SLPOUT          0x11
#define TFT_DISPOFF         0x28
#define TFT_DISPON          0x29

#define TFT_CASET           0x2A
#define TFT_PASET           0x2B
#define TFT_RAMWR           0x2C

#define TFT_RAMRD           0x2E
#define TFT_IDXRD           0x00

#define TFT_MADCTL          0x36
/* Page Address Order ('0': Top to Bottom, '1': the opposite) */
#define TFT_MAD_MY          0x80
/* Column Address Order ('0': Left to Right, '1': the opposite) */
#define TFT_MAD_MX          0x40
/* Page/Column Order ('0' = Normal Mode, '1' = Reverse Mode) */
#define TFT_MAD_MV          0x20
/* Line Address Order ('0' = LCD Refresh Top to Bottom, '1' = the opposite) */
#define TFT_MAD_ML          0x10
/* RGB/BGR Order ('0' = RGB, '1' = BGR) */
#define TFT_MAD_BGR         0x08
#define TFT_MAD_MH          0x04
#define TFT_MAD_RGB         0x00

/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static struct rt_spi_device *lcd_dev;
static lcd_info_t lcd_info = {
    .width = LCD_WIDTH,
    .height = LCD_HEIGHT,
    .xstart = LCD_XSTART,
    .ystart = LCD_YSTART,
    .back_color = LCD_WHITE,
    .fore_color = LCD_BLACK,
};

/* Private function prototypes -----------------------------------------------*/
static void lcd_hw_init(void);
static void lcd_hw_spi_flash_init(void);
static void lcd_write_cmd(const rt_uint8_t cmd);
static void lcd_write_data(const rt_uint8_t data);
static void lcd_write_half_word(const rt_uint16_t data);
static void lcd_write_ndata(const rt_uint8_t *data, rt_uint32_t len);

static void lcd_hw_init(void)
{
    rt_uint16_t cmd = 0;

#ifdef LCD_RESET_PIN
    rt_pin_write(LCD_RESET_PIN, PIN_HIGH);
    rt_thread_mdelay(1);
    rt_pin_write(LCD_RESET_PIN, PIN_LOW);
    rt_thread_mdelay(1);
    rt_pin_write(LCD_RESET_PIN, PIN_HIGH);
    rt_thread_mdelay(150);
#endif

    while ( init_cmds[cmd].databytes != 0xff )
    {
        lcd_write_cmd(init_cmds[cmd].cmd);
        if((init_cmds[cmd].databytes&0x7F) != 0)
        {
            lcd_write_ndata(init_cmds[cmd].data, init_cmds[cmd].databytes&0x7F);
        }
        if (init_cmds[cmd].databytes & 0x80)
        {
            rt_thread_mdelay(120);
        }
        cmd++;
    }
    lcd_set_orientation(CONFIG_LCD_DISPLAY_ORIENTATION);
    lcd_clear(lcd_info.back_color);
    lcd_display_on();
}

static void lcd_hw_spi_flash_init(void)
{
    rt_err_t res;
    rt_pin_mode(LCD_DC_PIN, PIN_MODE_OUTPUT);
#ifdef LCD_RESET_PIN
    rt_pin_mode(LCD_RESET_PIN, PIN_MODE_OUTPUT);
#endif
#ifdef LCD_BL_PIN
    rt_pin_mode(LCD_BL_PIN, PIN_MODE_OUTPUT);
#endif
    res = rt_hw_spi_device_attach(CONFIG_LCD_SPI_BUS_NAME, CONFIG_LCD_MOUNT_SPI_DEVICE_NAME, LCD_CS_PIN);
    if (res != RT_EOK)
    {
        LOG_E("Failed to attach device %s\n", CONFIG_LCD_MOUNT_SPI_DEVICE_NAME);
    }
    
    lcd_dev = (struct rt_spi_device *)rt_device_find(CONFIG_LCD_MOUNT_SPI_DEVICE_NAME);
    /* config spi */
    {
        struct rt_spi_configuration cfg;
        cfg.data_width = 8;
        cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_0 | RT_SPI_MSB;
        cfg.max_hz = 42 * 1000 * 1000; /* 42M,SPI max 42MHz,lcd 4-wire spi */

        rt_spi_configure(lcd_dev, &cfg);
    }
}

static void lcd_write_cmd(const rt_uint8_t cmd)
{
    rt_pin_write(LCD_DC_PIN, PIN_LOW);
    rt_spi_send(lcd_dev, &cmd, 1);
}

static void lcd_write_data(const rt_uint8_t data)
{
    rt_pin_write(LCD_DC_PIN, PIN_HIGH);
    rt_spi_send(lcd_dev, &data, 1);
}

static void lcd_write_half_word(const rt_uint16_t data)
{
    char buf[2] = {0};

    buf[0] = data >> 8;
    buf[1] = data;

    rt_pin_write(LCD_DC_PIN, PIN_HIGH);
    rt_spi_send(lcd_dev, buf, 2);
}

static void lcd_write_ndata(const rt_uint8_t *data, rt_uint32_t len)
{
    rt_pin_write(LCD_DC_PIN, PIN_HIGH);
    rt_spi_send(lcd_dev, data, len);
}

/* Exported functions --------------------------------------------------------*/
// 开启显示
void lcd_display_on(void)
{
    lcd_write_cmd(TFT_SLPOUT);  //sleep out and booster on 
    rt_thread_mdelay(20);    //延时120ms
    lcd_write_cmd(TFT_DISPON); 
    rt_thread_mdelay(20);    //延时120ms
#ifdef LCD_BL_PIN
    rt_pin_write(LCD_BL_PIN, PIN_HIGH);
#endif
}
// 关闭显示
void lcd_display_off(void)
{
#ifdef LCD_BL_PIN
    rt_pin_write(LCD_BL_PIN, PIN_LOW);
#endif
    lcd_write_cmd(TFT_SLPIN);  //sleep in and booster off	
    rt_thread_mdelay(20);    //延时120ms
    lcd_write_cmd(TFT_DISPOFF);
    rt_thread_mdelay(20);    //延时120ms
}


void lcd_set_orientation(uint8_t orientation)
{
    lcd_write_cmd(TFT_MADCTL);
    switch (orientation) {
    case LCD_PORTRAIT:
        lcd_write_data(TFT_MAD_MX | TFT_MAD_MV | TFT_MAD_ML | TFT_MAD_RGB);
        break;
    case LCD_PORTRAIT_INVERTED:
        lcd_write_data(TFT_MAD_MY | TFT_MAD_MV | TFT_MAD_RGB);
        break;
    case LCD_LANDSCAPE:
        lcd_write_data(TFT_MAD_RGB);
        break;
    case LCD_LANDSCAPE_INVERTED:
        lcd_write_data(TFT_MAD_MX | TFT_MAD_MY | TFT_MAD_RGB);
        break;
    default:
        break;
    }
}



/**
 * Set drawing area
 *
 * @param   x1      start of x position
 * @param   y1      start of y position
 * @param   x2      end of x position
 * @param   y2      end of y position
 *
 * @return  void
 */
static void lcd_address_set(rt_uint16_t x1, rt_uint16_t y1, rt_uint16_t x2, rt_uint16_t y2)
{
    lcd_write_cmd(0x2a);
    lcd_write_data(x1 >> 8);
    lcd_write_data(x1);
    lcd_write_data(x2 >> 8);
    lcd_write_data(x2);

    lcd_write_cmd(0x2b);
    lcd_write_data(y1 >> 8);
    lcd_write_data(y1);
    lcd_write_data(y2 >> 8);
    lcd_write_data(y2);

    lcd_write_cmd(0x2C);
}


/**
 * Set background color and foreground color
 *
 * @param   back    background color
 * @param   fore    fore color
 *
 * @return  void
 */
void lcd_set_color(rt_uint16_t back, rt_uint16_t fore)
{
    lcd_info.back_color = back;
    lcd_info.fore_color = fore;
}



/**
 * clear the lcd.
 *
 * @param   color       Fill color
 *
 * @return  void
 */
void lcd_clear(rt_uint16_t color)
{
    rt_uint16_t i, j;
    rt_uint8_t data[2] = {0};
    rt_uint8_t *buf = NULL;

    data[0] = color >> 8;
    data[1] = color;
    lcd_address_set(0, 0, lcd_info.width - 1, lcd_info.height - 1);

    /* 5760 = 240*240/20 */
    buf = rt_malloc(LCD_CLEAR_SEND_NUMBER);
    if (buf)
    {
        /* 2880 = 5760/2 color is 16 bit */
        for (j = 0; j < LCD_CLEAR_SEND_NUMBER / 2; j++)
        {
            buf[j * 2] =  data[0];
            buf[j * 2 + 1] =  data[1];
        }

        rt_pin_write(LCD_DC_PIN, PIN_HIGH);
        for (i = 0; i < 20; i++)
        {
            rt_spi_send(lcd_dev, buf, LCD_CLEAR_SEND_NUMBER);
        }
        rt_free(buf);
    }
    else
    {
        rt_pin_write(LCD_DC_PIN, PIN_HIGH);
        for (i = 0; i < lcd_info.width; i++)
        {
            for (j = 0; j < lcd_info.height; j++)
            {
                rt_spi_send(lcd_dev, data, 2);
            }
        }
    }
}

/**
 * display a point on the lcd.
 *
 * @param   x   x position
 * @param   y   y position
 *
 * @return  void
 */
void lcd_draw_point(rt_uint16_t x, rt_uint16_t y)
{
    lcd_address_set(x, y, x, y);
    lcd_write_half_word(lcd_info.fore_color);
}

/**
 * display a point on the lcd using the given colour.
 *
 * @param   x       x position
 * @param   y       y position
 * @param   color   color of point
 *
 * @return  void
 */
void lcd_draw_point_color(rt_uint16_t x, rt_uint16_t y, rt_uint16_t color)
{
    lcd_address_set(x, y, x, y);
    lcd_write_half_word(color);
}


/**
 * full color on the lcd.
 *
 * @param   x_start     start of x position
 * @param   y_start     start of y position
 * @param   x_end       end of x position
 * @param   y_end       end of y position
 * @param   color       Fill color
 *
 * @return  void
 */
void lcd_fill(rt_uint16_t x_start, rt_uint16_t y_start, rt_uint16_t x_end, rt_uint16_t y_end, rt_uint16_t color)
{
    rt_uint16_t i = 0, j = 0;
    rt_uint32_t size = 0, size_remain = 0;
    rt_uint8_t *fill_buf = RT_NULL;

    size = (x_end - x_start + 1) * (y_end - y_start + 1) * 2;

    if (size > LCD_CLEAR_SEND_NUMBER)
    {
        /* the number of remaining to be filled */
        size_remain = size - LCD_CLEAR_SEND_NUMBER;
        size = LCD_CLEAR_SEND_NUMBER;
    }

    lcd_address_set(x_start, y_start, x_end, y_end);

    fill_buf = (uint8_t *)rt_malloc(size);
    if (fill_buf)
    {
        /* fast fill */
        while (1)
        {
            for (i = 0; i < size / 2; i++)
            {
                fill_buf[2 * i] = color >> 8;
                fill_buf[2 * i + 1] = color;
            }
            rt_pin_write(LCD_DC_PIN, PIN_HIGH);
            rt_spi_send(lcd_dev, fill_buf, size);

            /* Fill completed */
            if (size_remain == 0)
                break;

            /* calculate the number of fill next time */
            if (size_remain > LCD_CLEAR_SEND_NUMBER)
            {
                size_remain = size_remain - LCD_CLEAR_SEND_NUMBER;
            }
            else
            {
                size = size_remain;
                size_remain = 0;
            }
        }
        rt_free(fill_buf);
    }
    else
    {
        for (i = y_start; i <= y_end; i++)
        {
            for (j = x_start; j <= x_end; j++)lcd_write_half_word(color);
        }
    }
}


//在指定区域内填充指定颜色块			 
//(sx,sy),(ex,ey):填充矩形对角坐标,区域大小为:(ex-sx+1)*(ey-sy+1)   
//color:要填充的颜色
void lcd_fill_color(rt_uint16_t x_start, rt_uint16_t y_start, rt_uint16_t x_end, rt_uint16_t y_end, rt_uint16_t *color)
{  
    rt_uint16_t i = 0, j = 0;
    rt_uint32_t size = 0, size_remain = 0;
    rt_uint8_t *fill_buf = RT_NULL;

    size = (x_end - x_start + 1) * (y_end - y_start + 1) * 2;

    if (size > LCD_CLEAR_SEND_NUMBER)
    {
        /* the number of remaining to be filled */
        size_remain = size - LCD_CLEAR_SEND_NUMBER;
        size = LCD_CLEAR_SEND_NUMBER;
    }

    lcd_address_set(x_start, y_start, x_end, y_end);

    fill_buf = (rt_uint8_t *)rt_malloc(size);
    if (fill_buf)
    {
        /* fast fill */
        while (1)
        {
            for (i = 0; i < size / 2; i++)
            {
                fill_buf[2 * i] = (*color) >> 8;
                fill_buf[2 * i + 1] = (*color);
                color++;
            }
            rt_pin_write(LCD_DC_PIN, PIN_HIGH);
            rt_spi_send(lcd_dev, fill_buf, size);

            /* Fill completed */
            if (size_remain == 0)
                break;

            /* calculate the number of fill next time */
            if (size_remain > LCD_CLEAR_SEND_NUMBER)
            {
                size_remain = size_remain - LCD_CLEAR_SEND_NUMBER;
            }
            else
            {
                size = size_remain;
                size_remain = 0;
            }
        }
        rt_free(fill_buf);
    }
    else
    {
        for (i = y_start; i <= y_end; i++)
        {
            for (j = x_start; j <= x_end; j++)
            {
                lcd_write_half_word(*color);
                color++;
            }
        }
    }
} 
/**
 * display a line on the lcd.
 *
 * @param   x1      x1 position
 * @param   y1      y1 position
 * @param   x2      x2 position
 * @param   y2      y2 position
 *
 * @return  void
 */
void lcd_draw_line(rt_uint16_t x1, rt_uint16_t y1, rt_uint16_t x2, rt_uint16_t y2)
{
    rt_uint16_t t;
    rt_uint32_t i = 0;
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, row, col;

    if (y1 == y2)
    {
        /* fast draw transverse line */
        lcd_address_set(x1, y1, x2, y2);

        rt_uint8_t line_buf[480] = {0};

        for (i = 0; i < x2 - x1; i++)
        {
            line_buf[2 * i] = lcd_info.fore_color >> 8;
            line_buf[2 * i + 1] = lcd_info.fore_color;
        }

        rt_pin_write(LCD_DC_PIN, PIN_HIGH);
        rt_spi_send(lcd_dev, line_buf, (x2 - x1) * 2);

        return ;
    }

    delta_x = x2 - x1;
    delta_y = y2 - y1;
    row = x1;
    col = y1;
    if (delta_x > 0)incx = 1;
    else if (delta_x == 0)incx = 0;
    else
    {
        incx = -1;
        delta_x = -delta_x;
    }
    if (delta_y > 0)incy = 1;
    else if (delta_y == 0)incy = 0;
    else
    {
        incy = -1;
        delta_y = -delta_y;
    }
    if (delta_x > delta_y)distance = delta_x;
    else distance = delta_y;
    for (t = 0; t <= distance + 1; t++)
    {
        lcd_draw_point(row, col);
        xerr += delta_x ;
        yerr += delta_y ;
        if (xerr > distance)
        {
            xerr -= distance;
            row += incx;
        }
        if (yerr > distance)
        {
            yerr -= distance;
            col += incy;
        }
    }
}

/**
 * display a rectangle on the lcd.
 *
 * @param   x1      x1 position
 * @param   y1      y1 position
 * @param   x2      x2 position
 * @param   y2      y2 position
 *
 * @return  void
 */
void lcd_draw_rectangle(rt_uint16_t x1, rt_uint16_t y1, rt_uint16_t x2, rt_uint16_t y2)
{
    lcd_draw_line(x1, y1, x2, y1);
    lcd_draw_line(x1, y1, x1, y2);
    lcd_draw_line(x1, y2, x2, y2);
    lcd_draw_line(x2, y1, x2, y2);
}

/**
 * display a circle on the lcd.
 *
 * @param   x       x position of Center
 * @param   y       y position of Center
 * @param   r       radius
 *
 * @return  void
 */
void lcd_draw_circle(rt_uint16_t x0, rt_uint16_t y0, rt_uint8_t r)
{
    int a, b;
    int di;
    a = 0;
    b = r;
    di = 3 - (r << 1);
    while (a <= b)
    {
        lcd_draw_point(x0 - b, y0 - a);
        lcd_draw_point(x0 + b, y0 - a);
        lcd_draw_point(x0 - a, y0 + b);
        lcd_draw_point(x0 - b, y0 - a);
        lcd_draw_point(x0 - a, y0 - b);
        lcd_draw_point(x0 + b, y0 + a);
        lcd_draw_point(x0 + a, y0 - b);
        lcd_draw_point(x0 + a, y0 + b);
        lcd_draw_point(x0 - b, y0 + a);
        a++;
        //Bresenham
        if (di < 0)di += 4 * a + 6;
        else
        {
            di += 10 + 4 * (a - b);
            b--;
        }
        lcd_draw_point(x0 + a, y0 + b);
    }
}

static void lcd_show_char(rt_uint16_t x, rt_uint16_t y, rt_uint8_t data, rt_uint32_t size)
{
    rt_uint8_t temp;
    rt_uint8_t num = 0;;
    rt_uint8_t pos, t;
    rt_uint16_t colortemp = lcd_info.fore_color;
    rt_uint8_t *font_buf = RT_NULL;

    if (x > lcd_info.width - size / 2 || y > lcd_info.height - size)return;

    data = data - ' ';

#ifdef ASC2_1608
    if (size == 16)
    {
        lcd_address_set(x, y, x + size / 2 - 1, y + size - 1);//(x,y,x+8-1,y+16-1)

        font_buf = (rt_uint8_t *)rt_malloc(size * size);
        if (!font_buf)
        {
            /* fast show char */
            for (pos = 0; pos < size * (size / 2) / 8; pos++)
            {
                temp = asc2_1608[(rt_uint16_t)data * size * (size / 2) / 8 + pos];
                for (t = 0; t < 8; t++)
                {
                    if (temp & 0x80)colortemp = lcd_info.fore_color;
                    else colortemp = lcd_info.back_color;
                    lcd_write_half_word(colortemp);
                    temp <<= 1;
                }
            }
        }
        else
        {
            for (pos = 0; pos < size * (size / 2) / 8; pos++)
            {
                temp = asc2_1608[(rt_uint16_t)data * size * (size / 2) / 8 + pos];
                for (t = 0; t < 8; t++)
                {
                    if (temp & 0x80)colortemp = lcd_info.fore_color;
                    else colortemp = lcd_info.back_color;
                    font_buf[2 * (8 * pos + t)] = colortemp >> 8;
                    font_buf[2 * (8 * pos + t) + 1] = colortemp;
                    temp <<= 1;
                }
            }
            rt_pin_write(LCD_DC_PIN, PIN_HIGH);
            rt_spi_send(lcd_dev, font_buf, size * size);
            rt_free(font_buf);
        }
    }
    else
#endif

#ifdef ASC2_2412
        if (size == 24)
        {
            lcd_address_set(x, y, x + size / 2 - 1, y + size - 1);

            font_buf = (rt_uint8_t *)rt_malloc(size * size);
            if (!font_buf)
            {
                /* fast show char */
                for (pos = 0; pos < (size * 16) / 8; pos++)
                {
                    temp = asc2_2412[(rt_uint16_t)data * (size * 16) / 8 + pos];
                    if (pos % 2 == 0)
                    {
                        num = 8;
                    }
                    else
                    {
                        num = 4;
                    }

                    for (t = 0; t < num; t++)
                    {
                        if (temp & 0x80)colortemp = lcd_info.fore_color;
                        else colortemp = lcd_info.back_color;
                        lcd_write_half_word(colortemp);
                        temp <<= 1;
                    }
                }
            }
            else
            {
                for (pos = 0; pos < (size * 16) / 8; pos++)
                {
                    temp = asc2_2412[(rt_uint16_t)data * (size * 16) / 8 + pos];
                    if (pos % 2 == 0)
                    {
                        num = 8;
                    }
                    else
                    {
                        num = 4;
                    }

                    for (t = 0; t < num; t++)
                    {
                        if (temp & 0x80)colortemp = lcd_info.fore_color;
                        else colortemp = lcd_info.back_color;
                        if (num == 8)
                        {
                            font_buf[2 * (12 * (pos / 2) + t)] = colortemp >> 8;
                            font_buf[2 * (12 * (pos / 2) + t) + 1] = colortemp;
                        }
                        else
                        {
                            font_buf[2 * (8 + 12 * (pos / 2) + t)] = colortemp >> 8;
                            font_buf[2 * (8 + 12 * (pos / 2) + t) + 1] = colortemp;
                        }
                        temp <<= 1;
                    }
                }
                rt_pin_write(LCD_DC_PIN, PIN_HIGH);
                rt_spi_send(lcd_dev, font_buf, size * size);
                rt_free(font_buf);
            }
        }
        else
#endif

#ifdef ASC2_3216
        if (size == 32)
        {
            lcd_address_set(x, y, x + size / 2 - 1, y + size - 1);

            font_buf = (rt_uint8_t *)rt_malloc(size * size);
            if (!font_buf)
            {
                /* fast show char */
                for (pos = 0; pos < size * (size / 2) / 8; pos++)
                {
                    temp = asc2_3216[(rt_uint16_t)data * size * (size / 2) / 8 + pos];
                    for (t = 0; t < 8; t++)
                    {
                        if (temp & 0x80)colortemp = lcd_info.fore_color;
                        else colortemp = lcd_info.back_color;
                        lcd_write_half_word(colortemp);
                        temp <<= 1;
                    }
                }
            }
            else
            {
                for (pos = 0; pos < size * (size / 2) / 8; pos++)
                {
                    temp = asc2_3216[(rt_uint16_t)data * size * (size / 2) / 8 + pos];
                    for (t = 0; t < 8; t++)
                    {
                        if (temp & 0x80)colortemp = lcd_info.fore_color;
                        else colortemp = lcd_info.back_color;
                        font_buf[2 * (8 * pos + t)] = colortemp >> 8;
                        font_buf[2 * (8 * pos + t) + 1] = colortemp;
                        temp <<= 1;
                    }
                }
                rt_pin_write(LCD_DC_PIN, PIN_HIGH);
                rt_spi_send(lcd_dev, font_buf, size * size);
                rt_free(font_buf);
            }
        }
        else
#endif

#ifdef ASC2_4824
        if (size == 48)
        {
            lcd_address_set(x, y, x + size / 2 - 1, y + size - 1);

            font_buf = (uint8_t *)malloc(size * size);
            if (!font_buf)
            {
                /* fast show char */
                for (pos = 0; pos < size * (size / 2) / 8; pos++)
                {
                    temp = asc2_4824[(uint16_t)data * size * (size / 2) / 8 + pos];
                    for (t = 0; t < 8; t++)
                    {
                        if (temp & 0x80)colortemp = lcd_info.fore_color;
                        else colortemp = lcd_info.back_color;
                        lcd_write_half_word(colortemp);
                        temp <<= 1;
                    }
                }
            }
            else
            {
                for (pos = 0; pos < size * (size / 2) / 8; pos++)
                {
                    temp = asc2_4824[(uint16_t)data * size * (size / 2) / 8 + pos];
                    for (t = 0; t < 8; t++)
                    {
                        if (temp & 0x80)colortemp = lcd_info.fore_color;
                        else colortemp = lcd_info.back_color;
                        font_buf[2 * (8 * pos + t)] = colortemp >> 8;
                        font_buf[2 * (8 * pos + t) + 1] = colortemp;
                        temp <<= 1;
                    }
                }
                lcd_write_ndata(font_buf, size * size);
                free(font_buf);
            }
        }
#endif
#ifdef ASC2_6432
        if (size == 64)
        {
            lcd_address_set(x, y, x + size / 2 - 1, y + size - 1);

            font_buf = (uint8_t *)malloc(size * size);
            if (!font_buf)
            {
                /* fast show char */
                for (pos = 0; pos < size * (size / 2) / 8; pos++)
                {
                    temp = asc2_6432[(uint16_t)data * size * (size / 2) / 8 + pos];
                    for (t = 0; t < 8; t++)
                    {
                        if (temp & 0x80)colortemp = lcd_info.fore_color;
                        else colortemp = lcd_info.back_color;
                        lcd_write_half_word(colortemp);
                        temp <<= 1;
                    }
                }
            }
            else
            {
                for (pos = 0; pos < size * (size / 2) / 8; pos++)
                {
                    temp = asc2_6432[(uint16_t)data * size * (size / 2) / 8 + pos];
                    for (t = 0; t < 8; t++)
                    {
                        if (temp & 0x80)colortemp = lcd_info.fore_color;
                        else colortemp = lcd_info.back_color;
                        font_buf[2 * (8 * pos + t)] = colortemp >> 8;
                        font_buf[2 * (8 * pos + t) + 1] = colortemp;
                        temp <<= 1;
                    }
                }
                lcd_write_ndata(font_buf, size * size);
                free(font_buf);
            }
        }
        else
#endif
        {
            LOG_E("There is no any define ASC2_1208 && ASC2_2412 && ASC2_2416 && ASC2_3216 !");
        }
}

/**
 * display the string on the lcd.
 *
 * @param   x       x position
 * @param   y       y position
 * @param   size    size of font
 * @param   p       the string to be display
 *
 * @return   0: display success
 *          -1: size of font is not support
 */
rt_err_t lcd_show_string(rt_uint16_t x, rt_uint16_t y, rt_uint32_t size, const char *fmt, ...)
{
#define LCD_STRING_BUF_LEN 128

    va_list args;
    rt_uint8_t buf[LCD_STRING_BUF_LEN] = {0};
    rt_uint8_t *p = RT_NULL;

    if (size != 12 & size != 16 && size != 24 && size != 32 && size != 48 && size != 64)
    {
        LOG_E("font size(%d) is not support!", size);
        return -RT_ERROR;
    }

    va_start(args, fmt);
    rt_vsnprintf((char *)buf, 100, (const char *)fmt, args);
    va_end(args);

    p = buf;
    while (*p != '\0')
    {
        if (x > lcd_info.width - size / 2)
        {
            x = 0;
            y += size;
        }
        if (y > lcd_info.height - size)
        {
            y = x = 0;
            lcd_clear(LCD_RED);
        }
        lcd_show_char(x, y, *p, size);
        x += size / 2;
        p++;
    }

    return RT_EOK;
}

/**
 * display the number on the lcd.
 *
 * @param   x       x position
 * @param   y       y position
 * @param   num     number
 * @param   len     length of number
 * @param   size    size of font
 *
 * @return  void
 */
void lcd_show_num(rt_uint16_t x, rt_uint16_t y, rt_uint32_t num, rt_uint8_t len, rt_uint32_t size)
{
    lcd_show_string(x, y, size, "%d", num);
}

/**
 * display the image on the lcd.
 *
 * @param   x       x position
 * @param   y       y position
 * @param   length  length of image
 * @param   wide    wide of image
 * @param   p       image
 *
 * @return   0: display success
 *          -1: the image is too large
 */
rt_err_t lcd_show_image(rt_uint16_t x, rt_uint16_t y, rt_uint16_t length, rt_uint16_t wide, const rt_uint16_t *p)
{
    RT_ASSERT(p);

    if (x + length > lcd_info.width || y + wide > lcd_info.height)
    {
        return -RT_ERROR;
    }

    lcd_address_set(x, y, x + length - 1, y + wide - 1);

    rt_pin_write(LCD_DC_PIN, PIN_HIGH);
    rt_spi_send(lcd_dev, p, length * wide * 2);

    return RT_EOK;
}


/* RTT start function --------------------------------------------------------*/
static int lcd_init(void)
{
    lcd_hw_spi_flash_init();
    lcd_hw_init();
    return 0;
}
INIT_COMPONENT_EXPORT(lcd_init);



#endif /* PKG_USING_CFF_LCD */