/**
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 by ThingPulse, Daniel Eichhorn
 * Copyright (c) 2018 by Fabrice Weinberg
 * Copyright (c) 2019 by Helmut Tschemernjak - www.radioshuttle.de
 * Copyright (c) 2020 by Brad Goodman - www.bradgoodman.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * ThingPulse invests considerable time and money to develop these open source libraries.
 * Please support us by buying our products (and not the clones) from
 * https://thingpulse.com
 *
 */

#ifndef OLEDDISPLAY_h
#define OLEDDISPLAY_h


#include <sys/types.h>
#include "driver/i2c.h"
#include "OLEDDisplayFonts.h"

//#define DEBUG_OLEDDISPLAY(...) Serial.printf( __VA_ARGS__ )
//#define DEBUG_OLEDDISPLAY(...) dprintf("%s",  __VA_ARGS__ )

#ifndef DEBUG_OLEDDISPLAY
#define DEBUG_OLEDDISPLAY(...)
#endif

// Use DOUBLE BUFFERING by default
#ifndef OLEDDISPLAY_REDUCE_MEMORY
#define OLEDDISPLAY_DOUBLE_BUFFER
#endif

// Header Values
#define JUMPTABLE_BYTES 4

#define JUMPTABLE_LSB   1
#define JUMPTABLE_SIZE  2
#define JUMPTABLE_WIDTH 3
#define JUMPTABLE_START 4

#define WIDTH_POS 0
#define HEIGHT_POS 1
#define FIRST_CHAR_POS 2
#define CHAR_NUM_POS 3


// Display commands
#define CHARGEPUMP 0x8D
#define COLUMNADDR 0x21
#define COMSCANDEC 0xC8
#define COMSCANINC 0xC0
#define DISPLAYALLON 0xA5
#define DISPLAYALLON_RESUME 0xA4
#define DISPLAYOFF 0xAE
#define DISPLAYON 0xAF
#define EXTERNALVCC 0x1
#define INVERTDISPLAY 0xA7
#define MEMORYMODE 0x20
#define NORMALDISPLAY 0xA6
#define PAGEADDR 0x22
#define SEGREMAP 0xA0
#define SETCOMPINS 0xDA
#define SETCONTRAST 0x81
#define SETDISPLAYCLOCKDIV 0xD5
#define SETDISPLAYOFFSET 0xD3
#define SETHIGHCOLUMN 0x10
#define SETLOWCOLUMN 0x00
#define SETMULTIPLEX 0xA8
#define SETPRECHARGE 0xD9
#define SETSEGMENTREMAP 0xA1
#define SETSTARTLINE 0x40
#define SETVCOMDETECT 0xDB
#define SWITCHCAPVCC 0x2

#ifndef _swap_int16_t
#define _swap_int16_t(a, b) { int16_t t = a; a = b; b = t; }
#endif

typedef enum OLEDDISPLAY_COLOR {
  BLACK = 0,
  WHITE = 1,
  INVERSE = 2
} OLEDDISPLAY_COLOR;

typedef enum OLEDDISPLAY_TEXT_ALIGNMENT {
  TEXT_ALIGN_LEFT = 0,
  TEXT_ALIGN_RIGHT = 1,
  TEXT_ALIGN_CENTER = 2,
  TEXT_ALIGN_CENTER_BOTH = 3
} OLEDDISPLAY_TEXT_ALIGNMENT;


typedef enum OLEDDISPLAY_GEOMETRY {
  GEOMETRY_128_64   = 0,
  GEOMETRY_128_32   = 1,
  GEOMETRY_64_48    = 2,
  GEOMETRY_64_32    = 3,
  GEOMETRY_RAWMODE  = 4
} OLEDDISPLAY_GEOMETRY;

typedef enum HW_I2C {
  I2C_ONE,
  I2C_TWO
} HW_I2C;

typedef char (*FontTableLookupFunction)(const uint8_t ch);
char DefaultFontTableLookup(const uint8_t ch);
#define pgm_read_byte(addr)   (*(const unsigned char *)(addr))


typedef struct OLEDDisplay_s {
	uint8_t            *buffer;

	#ifdef OLEDDISPLAY_DOUBLE_BUFFER
	uint8_t            *buffer_back;
	#endif


	OLEDDISPLAY_GEOMETRY geometry;

	uint16_t  width;
	uint16_t  height;
	uint16_t  displayBufferSize;


	OLEDDISPLAY_TEXT_ALIGNMENT   textAlignment;
	OLEDDISPLAY_COLOR            color;

	const uint8_t	 *fontData;

	// State values for logBuffer
	uint16_t   logBufferSize;
	uint16_t   logBufferFilled;
	uint16_t   logBufferLine;
	uint16_t   logBufferMaxLines;
	char      *logBuffer;
	FontTableLookupFunction fontTableLookupFunction;

	// ESP I2C
	uint16_t	i2c_sda_gpio;
	uint16_t	i2c_scl_gpio;
	uint16_t	i2c_port;
	uint16_t	i2c_addr;
	i2c_cmd_handle_t i2c_cmd;

} OLEDDisplay_t;



  // Use this to resume after a deep sleep without resetting the display (what init() would do).
  // Returns true if connection to the display was established and the buffer allocated, false otherwise.
  int OLEDDisplay_allocateBuffer(OLEDDisplay_t *oled);

  // Allocates the buffer and initializes the driver & display. Resets the display!
  // Returns false if buffer allocation failed, true otherwise.
	OLEDDisplay_t * OLEDDisplay_init(int port, int addr, int sda, int scl);

  // Free the memory used by the display
  void OLEDDisplay_end(OLEDDisplay_t *oled);

  // Cycle through the initialization
  void OLEDDisplay_resetDisplay(OLEDDisplay_t *oled);

  /* Drawing functions */
  // Sets the color of all pixel operations
  void OLEDDisplay_setColor(OLEDDisplay_t *oled, OLEDDISPLAY_COLOR color);

  // Returns the current color.
  OLEDDISPLAY_COLOR OLEDDisplay_getColor();

  // Draw a pixel at given position
  void OLEDDisplay_setPixel(OLEDDisplay_t *oled, int16_t x, int16_t y);

  // Draw a pixel at given position and color
  void OLEDDisplay_setPixelColor(OLEDDisplay_t *oled, int16_t x, int16_t y, OLEDDISPLAY_COLOR color);

  // Clear a pixel at given position FIXME: INVERSE is untested with this function
  void OLEDDisplay_clearPixel(OLEDDisplay_t *oled, int16_t x, int16_t y);

  // Draw a line from position 0 to position 1
  void OLEDDisplay_drawLine(OLEDDisplay_t *oled, int16_t x0, int16_t y0, int16_t x1, int16_t y1);

  // Draw the border of a rectangle at the given location
  void OLEDDisplay_drawRect(OLEDDisplay_t *oled, int16_t x, int16_t y, int16_t width, int16_t height);

  // Fill the rectangle
  void OLEDDisplay_fillRect(OLEDDisplay_t *oled, int16_t x, int16_t y, int16_t width, int16_t height);

  // Draw the border of a circle
  void OLEDDisplay_drawCircle(OLEDDisplay_t *oled, int16_t x, int16_t y, int16_t radius);

  // Draw all Quadrants specified in the quads bit mask
  void OLEDDisplay_drawCircleQuads(OLEDDisplay_t *oled, int16_t x0, int16_t y0, int16_t radius, uint8_t quads);

  // Fill circle
  void OLEDDisplay_fillCircle(OLEDDisplay_t *oled, int16_t x, int16_t y, int16_t radius);

  // Draw a line horizontally
  void OLEDDisplay_drawHorizontalLine(OLEDDisplay_t *oled, int16_t x, int16_t y, int16_t length);

  // Draw a line vertically
  void OLEDDisplay_drawVerticalLine(OLEDDisplay_t *oled, int16_t x, int16_t y, int16_t length);

  // Draws a rounded progress bar with the outer dimensions given by width and height. Progress is
  // a unsigned byte value between 0 and 100
  void OLEDDisplay_drawProgressBar(OLEDDisplay_t *oled, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t progress);

  // Draw a bitmap in the internal image format
  void OLEDDisplay_drawFastImage(OLEDDisplay_t *oled, int16_t x, int16_t y, int16_t width, int16_t height, const uint8_t *image);

	// Draw a XBM
	void OLEDDisplay_drawXbm(OLEDDisplay_t *oled, int16_t x, int16_t y, int16_t width, int16_t height, const uint8_t *xbm);

	// Draw icon 16x16 xbm format
	void OLEDDisplay_drawIco16x16(OLEDDisplay_t *oled, int16_t x, int16_t y, const char *ico, int inverse);

	/* Text functions */

	// Draws a string at the given location
	void OLEDDisplay_drawString(OLEDDisplay_t *oled, int16_t x, int16_t y, char *text);

	// Draws a formatted string (like printf) at the given location
	void OLEDDisplay_drawStringf(OLEDDisplay_t *oled, int16_t x, int16_t y, char* buffer, char * format, ... );

	// Draws a String with a maximum width at the given location.
	// If the given String is wider than the specified width
	// The text will be wrapped to the next line at a space or dash
	void OLEDDisplay_drawStringMaxWidth(OLEDDisplay_t *oled, int16_t x, int16_t y, uint16_t maxLineWidth, char * text);

	// Returns the width of the const char* with the current
	// font settings
	uint16_t OLEDDisplay_getStringWidthLen(OLEDDisplay_t *oled, const char* text, uint16_t length);
	uint16_t OLEDDisplay_getStringWidth(OLEDDisplay_t *oled, const char* text);


	// Specifies relative to which anchor point
	// the text is rendered. Available constants:
	// TEXT_ALIGN_LEFT, TEXT_ALIGN_CENTER, TEXT_ALIGN_RIGHT, TEXT_ALIGN_CENTER_BOTH
	void OLEDDisplay_setTextAlignment(OLEDDisplay_t *oled, OLEDDISPLAY_TEXT_ALIGNMENT textAlignment);

	// Sets the current font. Available default fonts
	// ArialMT_Plain_10, ArialMT_Plain_16, ArialMT_Plain_24
	void OLEDDisplay_setFont(OLEDDisplay_t *oled, const uint8_t *fontData);

	// Set the function that will convert utf-8 to font table index
	void OLEDDisplay_setFontTableLookupFunction(OLEDDisplay_t *oled, FontTableLookupFunction function);

	/* Display functions */

	// Turn the display on
	void OLEDDisplay_displayOn(OLEDDisplay_t *oled);

	// Turn the display offs
	void OLEDDisplay_displayOff(OLEDDisplay_t *oled);

	// Inverted display mode
	void OLEDDisplay_invertDisplay(OLEDDisplay_t *oled);

	// Normal display mode
	void OLEDDisplay_normalDisplay(OLEDDisplay_t *oled);

	// Set display contrast
	// really low brightness & contrast: contrast = 10, precharge = 5, comdetect = 0
	// normal brightness & contrast:  contrast = 100
	void OLEDDisplay_setContrast_internal(OLEDDisplay_t *oled, uint8_t contrast, uint8_t precharge, uint8_t comdetect);
static	inline void OLEDDisplay_setContrast(OLEDDisplay_t *oled, uint8_t contrast) {
	OLEDDisplay_setContrast_internal(oled,contrast,241,64);
}


	// Convenience method to access
	void OLEDDisplay_setBrightness(OLEDDisplay_t *oled, uint8_t);

	// Reset display rotation or mirroring
	void OLEDDisplay_resetOrientation(OLEDDisplay_t *oled);

	// Turn the display upside down
	void OLEDDisplay_flipScreenVertically(OLEDDisplay_t *oled);

	// Mirror the display (to be used in a mirror or as a projector)
	void OLEDDisplay_mirrorScreen(OLEDDisplay_t *oled);

	// Clear the local pixel buffer
	void OLEDDisplay_clear(OLEDDisplay_t *oled);

	// Log buffer implementation

	// This will define the lines and characters you can
	// print to the screen. When you exeed the buffer size (lines * chars)
	// the output may be truncated due to the size constraint.
	int OLEDDisplay_setLogBuffer(OLEDDisplay_t *oled, uint16_t lines, uint16_t chars);

	// Draw the log buffer at position (x, y)
	void OLEDDisplay_drawLogBuffer(OLEDDisplay_t *oled, uint16_t x, uint16_t y);

	// Get screen geometry
	uint16_t OLEDDisplay_getWidth(OLEDDisplay_t *oled);
	uint16_t OLEDDisplay_getHeight(OLEDDisplay_t *oled);

	// Implement needed function to be compatible with Print class
	size_t OLEDDisplay_write(OLEDDisplay_t *oled, uint8_t ch);
	size_t OLEDDisplay_writeString(OLEDDisplay_t *oled, const char* s);

	// Set the correct height, width and buffer for the geometry
	void OLEDDisplay_setGeometry(OLEDDisplay_t *oled, OLEDDISPLAY_GEOMETRY g, uint16_t width, uint16_t height);


	// Send all the init commands
	void OLEDDisplay_sendInitCommands(OLEDDisplay_t *oled);

	// converts utf8 characters to extended ascii
	char* OLEDDisplay_utf8ascii(OLEDDisplay_t *oled, char *s);

	void OLEDDisplay_drawInternal(OLEDDisplay_t *oled, int16_t xMove, int16_t yMove, int16_t width, int16_t height, const uint8_t *data, uint16_t offset, uint16_t bytesInData);

	void OLEDDisplay_drawStringInternal(OLEDDisplay_t *oled, int16_t xMove, int16_t yMove, char* text, uint16_t textLength, uint16_t textWidth);
	void OLEDDisplay_display(OLEDDisplay_t *oled);



#endif
