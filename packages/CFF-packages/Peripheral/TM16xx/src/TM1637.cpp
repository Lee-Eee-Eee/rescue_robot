/*
TM1637.cpp - Library implementation for TM1637.

Copyright (C) 2011 Ricardo Batista (rjbatista <at> gmail <dot> com)
see https://github.com/rjbatista/tm1638-library
Adjusted for TM1637 by Maxint R&D, based on TM1640 code. See https://github.com/maxint-rd/

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

/*
#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif
*/

#include "TM1637.h"

TM1637::TM1637(TM16xx_PIN_TYPE dataPin, TM16xx_PIN_TYPE clockPin, uint8_t numDigits, bool activateDisplay, uint8_t intensity)
	: TM16xx(dataPin, clockPin, dataPin, TM1637_MAX_POS, numDigits, activateDisplay, intensity)
{ // NOTE: Like the TM1640, the TM1637 only has DIO and CLK. Therefor the DIO-pin is initialized as strobe in the constructor
	clearDisplay();
	setupDisplay(activateDisplay, intensity);
	// delay(1);		// NOTE: using delay in constructor (called prior to Setup) may hang the MCU!
}

void TM1637::bitDelay()
{
	TM16xx_DELAY(5);
	// NOTE: on TM1637 reading keys should be slower than 250Khz (see datasheet p3)
	// for that reason the delay between reading bits should be more than 4us
	// When using a fast clock (such as ESP8266) a delay is needed to read bits correctly
}

void TM1637::stop()
{ // to stop TM1637 expects the clock to go high, when strobing DIO high
	TM16xx_PIN_WRITE(strobePin, PIN_LOW);
	TM16xx_PIN_WRITE(clockPin, PIN_LOW);
	bitDelay();
	TM16xx_PIN_WRITE(clockPin, PIN_HIGH);
	// bitDelay();
	TM16xx_PIN_WRITE(strobePin, PIN_HIGH);
	bitDelay();
}

void TM1637::send(uint8_t data)
{	// send a byte to the chip the way the TM1637 likes it
	/*
	  for (int i = 0; i < 8; i++) {
		digitalWrite(clockPin, LOW);
		bitDelay();
		digitalWrite(dataPin, data & 1 ? HIGH : LOW);
	//    bitDelay();
		data >>= 1;
		digitalWrite(clockPin, HIGH);
		bitDelay();
	  }
	*/

	// MOLE 180514: TM1637 uses acknowledgement after sending the data
	// (method derived from https://github.com/avishorp/TM1637 but using pins in standard output mode when writing)
	TM16xx::send(data);

	// unlike TM1638/TM1668 and TM1640, the TM1637 uses an ACK to confirm reception of command/data
	// read the acknowledgement
	// TODO? return the ack?
	TM16xx_PIN_WRITE(clockPin, PIN_LOW);
	TM16xx_PIN_MODE(dataPin, PIN_MODE_INPUT);
	bitDelay();
	TM16xx_PIN_WRITE(clockPin, PIN_HIGH);
	bitDelay();
	uint8_t ack = TM16xx_PIN_READ(dataPin);
	if (ack == 0)
		TM16xx_PIN_WRITE(dataPin, PIN_LOW);
	TM16xx_PIN_MODE(dataPin, PIN_MODE_OUTPUT);

	//  digitalWrite(clockPin, LOW);  // MMOLE: moved to stop()
	//  bitDelay();
}

uint32_t TM1637::getButtons()
{ // Keyscan data on the TM1637 is one byte, with index of the button that is pressed.
  // Simultaneous presses are not supported.
  // Received value is 0xFF when no buttons are pressed or in second call when button is not released,
  // According datasheet: 0xF7-0xF0 for K1, 0xEF-0xE8 for K2 (in testing 0xC7 to 0xC0 for K1?)
  // Button state is reset only when another command is issued, otherwise the second call returns no button pressed
  // For compatibility with the rest of the library the buttonstate is returned as a 32-bit value
  // with K1 in the highest byte (bits 8-15) and K2 in the lowest byte (bits 0-7)
	start();
	send(TM16XX_CMD_DATA_READ); // send read buttons command
	uint8_t received = receive();
	stop();
	if (received == 0xFF)
		return (0);
	return (_BV((0xFF - received) & 0x0F)); // return bit set for the button that is pressed (bits 0-15)
}
