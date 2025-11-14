/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LCD_ST7735_H__
#define __LCD_ST7735_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef CONFIG_LCD_DRIVE_ST7735

/* Includes ------------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
// ST7735 specific commands used in init
#define ST7735_NOP          0x00
#define ST7735_SWRESET      0x01
#define ST7735_RDDID        0x04
#define ST7735_RDDST        0x09

#define ST7735_SLPIN        0x10
#define ST7735_SLPOUT       0x11
#define ST7735_PTLON        0x12
#define ST7735_NORON        0x13

#define ST7735_INVOFF       0x20
#define ST7735_INVON        0x21
#define ST7735_DISPOFF      0x28
#define ST7735_DISPON       0x29
#define ST7735_CASET        0x2A
#define ST7735_RASET        0x2B
#define ST7735_RAMWR        0x2C
#define ST7735_RAMRD        0x2E

#define ST7735_PTLAR        0x30
#define ST7735_VSCRDEF      0x33
#define ST7735_COLMOD       0x3A
#define ST7735_MADCTL       0x36
#define ST7735_VSCRSADD     0x37

#define ST7735_FRMCTR1      0xB1
#define ST7735_FRMCTR2      0xB2
#define ST7735_FRMCTR3      0xB3
#define ST7735_INVCTR       0xB4
#define ST7735_DISSET5      0xB6

#define ST7735_PWCTR1       0xC0
#define ST7735_PWCTR2       0xC1
#define ST7735_PWCTR3       0xC2
#define ST7735_PWCTR4       0xC3
#define ST7735_PWCTR5       0xC4
#define ST7735_VMCTR1       0xC5

#define ST7735_RDID1        0xDA
#define ST7735_RDID2        0xDB
#define ST7735_RDID3        0xDC
#define ST7735_RDID4        0xDD

#define ST7735_PWCTR6       0xFC

#define ST7735_GMCTRP1      0xE0
#define ST7735_GMCTRN1      0xE1

#define ST77XX_MADCTL_MY    0x80
#define ST77XX_MADCTL_MX    0x40
#define ST77XX_MADCTL_MV    0x20
#define ST77XX_MADCTL_ML    0x10
#define ST77XX_MADCTL_RGB   0x00
#define ST77XX_MADCTL_BGR   0x08

#define ST7735S_INVERT_COLORS 0

#ifdef LCD_ST7735S
    #define LCD_W 132
    #define LCD_H 132
#else
    #define LCD_W 162
    #define LCD_H 132
#endif


/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
const lcd_init_cmd_t init_cmds[]={
		{ST7735_SWRESET, {0}, 0x80},         		 // Software reset, 0 args, w/delay 150
		{ST7735_SLPOUT,  {0}, 0x80},                 // Out of sleep mode, 0 args, w/delay 500
		{ST7735_FRMCTR1, {0x05, 0x3C, 0x3C}, 3},     // Frame rate ctrl - normal mode, 3 args: Rate = fosc/(1x2+40) * (LINE+2C+2D)
		{ST7735_FRMCTR2, {0x05, 0x3C, 0x3C}, 3},     // Frame rate control - idle mode, 3 args:Rate = fosc/(1x2+40) * (LINE+2C+2D)
		{ST7735_FRMCTR3, {0x05, 0x3C, 0x3C, 0x05, 0x3C, 0x3C}, 6}, //Frame rate ctrl - partial mode, 6 args:Dot inversion mode. Line inversion mode
		{ST7735_INVCTR,  {0x03}, 1},                 // Display inversion ctrl, 1 arg, no delay:No inversion
		{ST7735_PWCTR1,  {0x28, 0x08, 0x04}, 3},     // Power control, 3 args, no delay:-4.6V AUTO mode
		{ST7735_PWCTR2,  {0xC0}, 1},                 // Power control, 1 arg, no delay:VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
		{ST7735_PWCTR3,  {0x0D, 0x00}, 2},           // Power control, 2 args, no delay: Opamp current small, Boost frequency
		{ST7735_PWCTR4,  {0x8D, 0x2A}, 2},           // Power control, 2 args, no delay: BCLK/2, Opamp current small & Medium low
		{ST7735_PWCTR5,  {0x8D, 0xEE}, 2},           // Power control, 2 args, no delay:
		{ST7735_VMCTR1,  {0x1A}, 1},                 // Power control, 1 arg, no delay:
#if ST7735S_INVERT_COLORS == 1
		{ST7735_INVON, {0}, 0},                     // set inverted mode
#else
 		{ST7735_INVOFF, {0}, 0},                    // set non-inverted mode
#endif
		{ST7735_GMCTRP1, {0x04, 0x22, 0x07, 0x0A,
			              0x2E, 0x30, 0x25, 0x2A,
			              0x28, 0x26, 0x2E, 0x3A,
			              0x00, 0x01, 0x03, 0x13}, 16},           // 16 args, no delay:
		{ST7735_GMCTRN1, {0x04, 0x16, 0x06, 0x0D,
			              0x2D, 0x26, 0x23, 0x27,
			              0x27, 0x25, 0x2D, 0x3B,
			              0x00, 0x01, 0x04, 0x13}, 16},           // 16 args, no delay:
        {ST7735_COLMOD,  {0x05}, 1},               	// set color mode, 1 arg, no delay: 16-bit color
		{ST7735_DISPON,  {0}, TFT_INIT_DELAY},       // Main screen turn on, no args w/delay 100 ms delay
		{0, {0}, 0xff}
    };

/* Exported functions --------------------------------------------------------*/

#endif  // CONFIG_LCD_DRIVE_ST7735

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif  // __LCD_ST7735_H__
