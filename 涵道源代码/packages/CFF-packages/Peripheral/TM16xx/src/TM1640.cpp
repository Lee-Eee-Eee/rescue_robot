/*
TM1640.cpp - Library implementation for TM1640.

Copyright (C) 2011 Ricardo Batista (rjbatista <at> gmail <dot> com)
Adjusted for TM1640 by Maxint R&D, based on orignal code. See https://github.com/maxint-rd/

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "TM1640.h"

TM1640::TM1640(TM16xx_PIN_TYPE dataPin, TM16xx_PIN_TYPE clockPin, uint8_t numDigits, bool activateDisplay, uint8_t intensity)
	: TM16xx(dataPin, clockPin, dataPin, TM1640_MAX_POS, numDigits, activateDisplay, intensity)
{ // NOTE: Like the TM1637, the TM1640 only has DIO and CLK. Therefor the DIO-pin is initialized as strobe in the constructor
	clearDisplay();
	setupDisplay(activateDisplay, intensity);
}

/*
void TM1640::bitDelay()
{
	//delayMicroseconds(5);
}
*/

void TM1640::start()
{	// if needed derived classes can use different patterns to start a command (eg. for TM1637)
	// Datasheet: The starting condition of data input is: when CLK is high, the DIN becomes low from high;
	TM16xx_PIN_WRITE(dataPin, PIN_LOW);
	TM16xx_PIN_WRITE(clockPin, PIN_LOW);
	bitDelay();
}

void TM1640::stop()
{ // if needed derived classes can use different patterns to stop a command (eg. for TM1637)
  // datasheet: the ending condition is: when CLK is high, the DIN becomes high from low.
	TM16xx_PIN_WRITE(clockPin, PIN_HIGH);
	TM16xx_PIN_WRITE(dataPin, PIN_HIGH);
	bitDelay();
}

void TM1640::send(uint8_t data)
{
	// MOLE 180514: TM1640 wants data and clock to be low after sending the data
	TM16xx::send(data);
	TM16xx_PIN_WRITE(clockPin, PIN_LOW); // first clock low
	TM16xx_PIN_WRITE(dataPin, PIN_LOW);	 // then data low
	bitDelay();
}
