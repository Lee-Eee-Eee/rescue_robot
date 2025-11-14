/*
TM1652.h - Library for TM1652 led display driver. Only DIN, 8x5/7x6 LED, no buttons.

Part of the TM16xx library by Maxint. See https://github.com/maxint-rd/TM16xx
The Arduino TM16xx library supports LED & KEY and LED Matrix modules based on TM1638, TM1637, TM1640 as well as individual chips.
Simply use print() on 7-segment displays and use Adafruit GFX on matrix displays.

Made by Maxint R&D. See https://github.com/maxint-rd
*/

#ifndef TM1652_h
#define TM1652_h

#include "TM16xx.h"

#define TM1652_MAX_POS 6

// TM1652 has two display modes: 8 seg x 4 grd and 7 seg x 4 grd
#define TM1652_DISPMODE_5x8 0x00
#define TM1652_DISPMODE_6x7 0x01

#define TM1652_CMD_MODE 0x18
#define TM1652_CMD_ADDRESS 0x08

class TM1652 : public TM16xx
{
public:
  /** Instantiate a TM1652 module specifying the data pin, number of digits */
  /** DEPRECATED: activation, intensity (0-7) and display mode are no longer used by constructor. */
  TM1652(TM16xx_PIN_TYPE dataPin, uint8_t numDigits = 2, bool activateDisplay = true, uint8_t intensity = 1, uint8_t displaymode = TM1652_DISPMODE_5x8);
  // TODO: remove deprecated parameters - TM1652(byte dataPin, byte numDigits=4);
  virtual void clearDisplay();
  virtual void setupDisplay(bool active, uint8_t intensity);
  virtual void begin(bool activateDisplay = true, uint8_t intensity = 7);

protected:
  virtual void start();
  virtual void stop();
  virtual void send(uint8_t data);
  virtual void sendData(uint8_t address, uint8_t data);
  virtual void sendCommand(uint8_t cmd);

private:
  uint8_t reverseByte(uint8_t b);
  void waitCmd(void);
  void endCmd(void);
  uint32_t tLastCmd;
};

#endif
