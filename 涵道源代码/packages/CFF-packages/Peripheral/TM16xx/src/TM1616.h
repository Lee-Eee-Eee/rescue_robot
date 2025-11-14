/*
TM1616.h - Library for TM1616. DIN/CLK/STB, 7x4 LED, no buttons.
Made by Maxint R&D. See https://github.com/maxint-rd/TM16xx
*/

#ifndef TM1616_h
#define TM1616_h

#include "TM16xx.h"

#define TM1616_MAX_POS 4

class TM1616 : public TM16xx
{
public:
  /** Instantiate a TM1616 module specifying data, clock and strobe pins, the number of digits, the display state, the starting intensity (0-7). */
  TM1616(TM16xx_PIN_TYPE dataPin, TM16xx_PIN_TYPE clockPin, TM16xx_PIN_TYPE strobePin, uint8_t numDigits = TM1616_MAX_POS, bool activateDisplay = true, uint8_t intensity = 7);

  /** Override standard behaviour where this chip acts differently */
  virtual void setSegments(uint8_t segments, uint8_t position);
  virtual void clearDisplay();
};

#endif
