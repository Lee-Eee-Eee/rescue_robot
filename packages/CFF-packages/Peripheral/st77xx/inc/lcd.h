/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EXT_LCD_H__
#define __EXT_LCD_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include <rtthread.h>

#ifdef PKG_USING_CFF_LCD

#include "font.h"

/* Exported define -----------------------------------------------------------*/

#define LCD_PORTRAIT             0       /* 横屏 */
#define LCD_PORTRAIT_INVERTED    1       /* 横屏倒置 */
#define LCD_LANDSCAPE            2       /* 竖屏 */
#define LCD_LANDSCAPE_INVERTED   3       /* 竖屏倒置 */

//POINT_COLOR
#define LCD_WHITE            0xFFFF
#define LCD_BLACK            0x0000
#define LCD_BLUE             0x001F
#define LCD_BRED             0XF81F
#define LCD_GRED             0XFFE0
#define LCD_GBLUE            0X07FF
#define LCD_RED              0xF800
#define LCD_MAGENTA          0xF81F
#define LCD_GREEN            0x07E0
#define LCD_CYAN             0x7FFF
#define LCD_YELLOW           0xFFE0
#define LCD_BROWN            0XBC40
#define LCD_BRRED            0XFC07
#define LCD_GRAY             0X8430
#define LCD_GRAY175          0XAD75
#define LCD_GRAY151          0X94B2
#define LCD_GRAY187          0XBDD7
#define LCD_GRAY240          0XF79E
#define LCD_GRAY0            0xEF7D //灰色0 
#define LCD_GRAY1            0x8410 //灰色1      
#define LCD_GRAY2            0x4208 //灰色2 

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef struct {
    rt_uint8_t cmd;
    rt_uint8_t data[20];
    rt_uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

typedef struct {
    rt_uint16_t width;
    rt_uint16_t height;
    rt_uint16_t xstart;
    rt_uint16_t ystart;
    
    rt_uint16_t back_color;
    rt_uint16_t fore_color;
} lcd_info_t;

/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void lcd_display_on(void);
void lcd_display_off(void);
void lcd_set_orientation(uint8_t orientation);

void lcd_clear(rt_uint16_t color);
void lcd_set_color(rt_uint16_t back, rt_uint16_t fore);

void lcd_draw_point(rt_uint16_t x, rt_uint16_t y);
void lcd_draw_point_color(rt_uint16_t x, rt_uint16_t y, rt_uint16_t color);
void lcd_fill(rt_uint16_t x_start, rt_uint16_t y_start, rt_uint16_t x_end, rt_uint16_t y_end, rt_uint16_t color);
void lcd_fill_color(rt_uint16_t x_start, rt_uint16_t y_start, rt_uint16_t x_end, rt_uint16_t y_end, rt_uint16_t *color);
void lcd_draw_line(rt_uint16_t x1, rt_uint16_t y1, rt_uint16_t x2, rt_uint16_t y2);
void lcd_draw_rectangle(rt_uint16_t x1, rt_uint16_t y1, rt_uint16_t x2, rt_uint16_t y2);
void lcd_draw_circle(rt_uint16_t x0, rt_uint16_t y0, rt_uint8_t r);

void lcd_show_num(rt_uint16_t x, rt_uint16_t y, rt_uint32_t num, rt_uint8_t len, rt_uint32_t size);
rt_err_t lcd_show_string(rt_uint16_t x, rt_uint16_t y, rt_uint32_t size, const char *fmt, ...);
rt_err_t lcd_show_image(rt_uint16_t x, rt_uint16_t y, rt_uint16_t length, rt_uint16_t wide, const rt_uint16_t *p);


#endif /* PKG_USING_CFF_LCD */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif  // __EXT_LCD_H__