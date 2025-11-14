/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LCD_ST7789_H__
#define __LCD_ST7789_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef CONFIG_LCD_DRIVE_ST7789

/* Includes ------------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
// ST7789 specific commands used in init
#define ST7789_NOP          0x00
#define ST7789_SWRESET      0x01
#define ST7789_RDDID        0x04
#define ST7789_RDDST        0x09

#define ST7789_SLPIN        0x10
#define ST7789_SLPOUT       0x11
#define ST7789_PTLON        0x12
#define ST7789_NORON        0x13

#define ST7789_INVOFF       0x20
#define ST7789_INVON        0x21
#define ST7789_DISPOFF      0x28
#define ST7789_DISPON       0x29
#define ST7789_CASET        0x2A
#define ST7789_RASET        0x2B
#define ST7789_RAMWR        0x2C
#define ST7789_RAMRD        0x2E

#define ST7789_PTLAR        0x30
#define ST7789_VSCRDEF      0x33
#define ST7789_COLMOD       0x3A
#define ST7789_MADCTL       0x36
#define ST7789_VSCRSADD     0x37

#define ST7789_FRMCTR1      0xB1
#define ST7789_FRMCTR2      0xB2
#define ST7789_FRMCTR3      0xB3
#define ST7789_INVCTR       0xB4
#define ST7789_DISSET5      0xB6

#define ST7789_PWCTR1       0xC0
#define ST7789_PWCTR2       0xC1
#define ST7789_PWCTR3       0xC2
#define ST7789_PWCTR4       0xC3
#define ST7789_PWCTR5       0xC4
#define ST7789_VMCTR1       0xC5

#define ST7789_RDID1        0xDA
#define ST7789_RDID2        0xDB
#define ST7789_RDID3        0xDC
#define ST7789_RDID4        0xDD

#define ST7789_PWCTR6       0xFC

#define ST7789_GMCTRP1      0xE0
#define ST7789_GMCTRN1      0xE1

#define ST77XX_MADCTL_MY    0x80
#define ST77XX_MADCTL_MX    0x40
#define ST77XX_MADCTL_MV    0x20
#define ST77XX_MADCTL_ML    0x10
#define ST77XX_MADCTL_RGB   0x00
#define ST77XX_MADCTL_BGR   0x08

#define LCD_W 240
#define LCD_H 240

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
const lcd_init_cmd_t init_cmds[]={
    {ST7789_SWRESET, {0}, 0x80},         		 // Software reset, 0 args, w/delay 150
    {ST7789_SLPOUT,  {0}, 0x80},                 // Out of sleep mode, 0 args, w/delay 500
    {ST7789_COLMOD, {0x05}, 1},
    {ST7789_FRMCTR2, {0x0C,0x0C,0x00,0x33,0x33}, 5},
    {0xB7, {0x35}, 1},
    {0xBB, {0x32}, 1},
    {ST7789_PWCTR3, {0x01}, 1},
    {ST7789_PWCTR4, {0x15}, 1},	
    {ST7789_PWCTR5, {0x20}, 1},	
    {0xC6, {0x0F}, 1},	
    {0xD0, {0xA4,0xA1}, 1},		
    {ST7789_GMCTRP1, {0xD0,0x08,0x0E,0x09,0x09,0x05,0x31,0x33,0x48,0x17,0x14,0x15,0x31,0x34}, 14},	
    {ST7789_GMCTRN1, {0xD0,0x08,0x0E,0x09,0x09,0x15,0x31,0x33,0x48,0x17,0x14,0x15,0x31,0x34}, 14},	
    {ST7789_INVON, {0}, 0x80},         		 // Software reset, 0 args, w/delay 150
    {ST7789_DISPON,  {0}, 0x80},                 // Out of sleep mode, 0 args, w/delay 500
    {0, {0}, 0xff}
};


/* Exported functions --------------------------------------------------------*/

#endif  // CONFIG_LCD_DRIVE_ST7789

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif  // __LCD_ST7789_H__
