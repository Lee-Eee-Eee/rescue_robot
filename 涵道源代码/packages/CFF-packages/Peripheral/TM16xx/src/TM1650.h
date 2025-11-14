/*
TM1650.cpp - Library implementation for TM1650.

Part of the TM16xx library by Maxint. See https://github.com/maxint-rd/TM16xx
The Arduino TM16xx library supports LED & KEY and LED Matrix modules based on TM1638, TM1637, TM1640 as well as individual chips.
Simply use print() on 7-segment displays and use Adafruit GFX on matrix displays.

Adjusted for TM1650 by Maxint R&D, based on TM1637 code.
Partially based on TM1640 library by MRicardo Batista. See https://github.com/rjbatista/tm1638-library
*/

#ifndef TM1650_h
#define TM1650_h

#include "TM16xx.h"

#define TM1650_MAX_POS 4

// TM1650 has two display modes: 8 seg x 4 grd and 7 seg x 4 grd
#define TM1650_DISPMODE_4x8 0x01
#define TM1650_DISPMODE_4x7 0x09

#define TM1650_CMD_MODE 0x48
#define TM1650_CMD_DATA_READ 0x49
#define TM1650_CMD_ADDRESS 0x68

class TM1650 : public TM16xx
{
public:
  /** Instantiate a TM1650 module specifying the  data and clock pins, number of digits, display state, the starting intensity (0-7). */
  TM1650(TM16xx_PIN_TYPE dataPin, TM16xx_PIN_TYPE clockPin, uint8_t numDigits = 4, bool activateDisplay = true, uint8_t intensity = 7, uint8_t displaymode = TM1650_DISPMODE_4x8);
  virtual void clearDisplay();
  virtual void setupDisplay(bool active, uint8_t intensity);
  virtual uint32_t getButtons();

protected:
#if defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny13__) || defined(__AVR_ATtiny44__)
  // On slow processors we may not need this bitDelay, so save some flash
#else
  virtual void bitDelay();
#endif
  virtual void start();
  virtual void stop();
  virtual void send(uint8_t data);
  virtual void sendData(uint8_t address, uint8_t data);
  virtual uint8_t receive();
};

#endif
