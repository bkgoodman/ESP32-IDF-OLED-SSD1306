/**
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 by ThingPulse, Daniel Eichhorn
 * Copyright (c) 2018 by Fabrice Weinberg
 * Copyright (c) 2019 by Helmut Tschemernjak - www.radioshuttle.de
 * Copyright (c) 2020 by Brad Goodman - www.bradgoodman.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of oled software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and oled permission notice shall be included in all
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

 /*
  * TODO Helmut
  * - test/finish dislplay.printf() on mbed-os
  * - Finish _putc with drawLogBuffer when running display
  */

#include "esp_log.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "sdkconfig.h"
#include "OLEDDisplay.h"
#include "driver/i2c.h"

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

static inline uint16_t _max(uint16_t x,uint16_t y) { if (x>y) return (x); return(y);}
static inline uint16_t _min(uint16_t x,uint16_t y) { if (x<y) return (x); return(y);}

#define TAG "OLED"

OLEDDisplay_t * OLEDDisplay_alloc() {

	OLEDDisplay_t *oled = malloc(sizeof(OLEDDisplay_t));
	if (!oled)
		return 0L;
	oled->width = 128;
	oled->height = 64;
	oled->displayBufferSize = oled->width * oled->height / 8;
	oled->color = WHITE;
	oled->geometry = GEOMETRY_128_64;
	oled->textAlignment = TEXT_ALIGN_LEFT;
	oled->fontData = ArialMT_Plain_10;
	oled->fontTableLookupFunction = DefaultFontTableLookup;
	oled->buffer = NULL;
#ifdef OLEDDISPLAY_DOUBLE_BUFFER
	oled->buffer_back = NULL;
#endif
	return (oled);
}

void OLEDDisplay_free(OLEDDisplay_t *oled) {
  if (oled)
    free(oled);
}

static int OLEDDisplay_getBufferOffset(OLEDDisplay_t *oled) {return(0);}
int OLEDDisplay_allocateBuffer(OLEDDisplay_t *oled) {

  oled->logBufferSize = 0;
  oled->logBufferFilled = 0;
  oled->logBufferLine = 0;
  oled->logBufferMaxLines = 0;
  oled->logBuffer = NULL;

  if(oled->buffer==NULL) {
    oled->buffer = (uint8_t*) malloc((sizeof(uint8_t) * oled->displayBufferSize) + OLEDDisplay_getBufferOffset(oled));
    oled->buffer += OLEDDisplay_getBufferOffset(oled);

    if(!oled->buffer) {
      DEBUG_OLEDDISPLAY("[OLEDDISPLAY][init] Not enough memory to create display\n");
      return -1;
    }
  }

  #ifdef OLEDDISPLAY_DOUBLE_BUFFER
  if(oled->buffer_back==NULL) {
    oled->buffer_back = (uint8_t*) malloc((sizeof(uint8_t) * oled->displayBufferSize) + OLEDDisplay_getBufferOffset(oled));
    oled->buffer_back += OLEDDisplay_getBufferOffset(oled);

    if(!oled->buffer_back) {
      DEBUG_OLEDDISPLAY("[OLEDDISPLAY][init] Not enough memory to create back buffer\n");
      free(oled->buffer - OLEDDisplay_getBufferOffset(oled));
      return -1;
    }
  }
  #endif

  return 0;
}

OLEDDisplay_t * OLEDDisplay_init(int port, int addr, int sda, int scl)
{
		OLEDDisplay_t *oled = OLEDDisplay_alloc();
		if (!oled) {
			ESP_LOGW(TAG, "OLED Allocation failed\n");
			return 0L;
		}


    int i2c_master_port = port; //I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda; //I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = scl; // I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 700000; //I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    //ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0));
		oled->i2c_sda_gpio = sda;
		oled->i2c_scl_gpio = scl;
		oled->i2c_port = port;
		oled->i2c_addr = addr;

		if(OLEDDisplay_allocateBuffer(oled)) {
			ESP_LOGW(TAG, "OLED Display Buffer Allocation failed\n");
			return 0L;
		}

		OLEDDisplay_sendInitCommands(oled);
		OLEDDisplay_resetDisplay(oled);
		return (oled);
		
}

#define BKG_ACK_CHECK 0 /* ACK_CHECK_EN */
static void OLEDDisplay_sendCommand(OLEDDisplay_t *oled, uint8_t opcode)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, oled->i2c_addr | WRITE_BIT,  BKG_ACK_CHECK  );
    i2c_master_write_byte(cmd, 0x80, BKG_ACK_CHECK  );
    i2c_master_write_byte(cmd, opcode,  BKG_ACK_CHECK  );
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(oled->i2c_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
}

static void OLEDDisplay_beginPayload(OLEDDisplay_t *oled)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, oled->i2c_addr | WRITE_BIT,  BKG_ACK_CHECK  );
		oled->i2c_cmd = cmd;
}

static void OLEDDisplay_i2cwrite(OLEDDisplay_t *oled,uint8_t d)
{
    i2c_cmd_handle_t cmd = oled->i2c_cmd;
    i2c_master_write_byte(cmd, d, BKG_ACK_CHECK  );
}

static esp_err_t OLEDDisplay_endPayload(OLEDDisplay_t *oled) {
    i2c_cmd_handle_t cmd = oled->i2c_cmd;
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(oled->i2c_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
static esp_err_t OLEDDisplay_sendPayload(OLEDDisplay_t *oled, uint8_t *buf, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, oled->i2c_addr | WRITE_BIT,  BKG_ACK_CHECK  );
    i2c_master_write_byte(cmd, 0x40, BKG_ACK_CHECK  );
    i2c_master_write(cmd, buf, size, BKG_ACK_CHECK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(oled->i2c_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
void OLEDDisplay_end(OLEDDisplay_t *oled) {
  if (oled->buffer) { free(oled->buffer - OLEDDisplay_getBufferOffset(oled)); oled->buffer = NULL; }
  #ifdef OLEDDISPLAY_DOUBLE_BUFFER
  if (oled->buffer_back) { free(oled->buffer_back - OLEDDisplay_getBufferOffset(oled)); oled->buffer_back = NULL; }
  #endif
  if (oled->logBuffer != NULL) { free(oled->logBuffer); oled->logBuffer = NULL; }
}

void OLEDDisplay_resetDisplay(OLEDDisplay_t *oled) {
  OLEDDisplay_clear(oled);
  #ifdef OLEDDISPLAY_DOUBLE_BUFFER
  memset(oled->buffer_back, 1, oled->displayBufferSize);
  #endif
  OLEDDisplay_display(oled);
}

void OLEDDisplay_setColor(OLEDDisplay_t *oled, OLEDDISPLAY_COLOR color) {
  oled->color = color;
}

OLEDDISPLAY_COLOR OLEDDisplay_getColor(OLEDDisplay_t *oled) {
  return oled->color;
}

void OLEDDisplay_setPixel(OLEDDisplay_t *oled, int16_t x, int16_t y) {
  if (x >= 0 && x < oled->width && y >= 0 && y < oled->height) {
    switch (oled->color) {
      case WHITE:   oled->buffer[x + (y / 8) * oled->width] |=  (1 << (y & 7)); break;
      case BLACK:   oled->buffer[x + (y / 8) * oled->width] &= ~(1 << (y & 7)); break;
      case INVERSE: oled->buffer[x + (y / 8) * oled->width] ^=  (1 << (y & 7)); break;
    }
  }
}

void OLEDDisplay_setPixelColor(OLEDDisplay_t *oled, int16_t x, int16_t y, OLEDDISPLAY_COLOR color) {
  if (x >= 0 && x < oled->width && y >= 0 && y < oled->height) {
    switch (color) {
      case WHITE:   oled->buffer[x + (y / 8) * oled->width] |=  (1 << (y & 7)); break;
      case BLACK:   oled->buffer[x + (y / 8) * oled->width] &= ~(1 << (y & 7)); break;
      case INVERSE: oled->buffer[x + (y / 8) * oled->width] ^=  (1 << (y & 7)); break;
    }
  }
}

void OLEDDisplay_clearPixel(OLEDDisplay_t *oled, int16_t x, int16_t y) {
  if (x >= 0 && x < oled->width && y >= 0 && y < oled->height) {
    switch (oled->color) {
      case BLACK:   oled->buffer[x + (y >> 3) * oled->width] |=  (1 << (y & 7)); break;
      case WHITE:   oled->buffer[x + (y >> 3) * oled->width] &= ~(1 << (y & 7)); break;
      case INVERSE: oled->buffer[x + (y >> 3) * oled->width] ^=  (1 << (y & 7)); break;
    }
  }
}


// Bresenham's algorithm - thx wikipedia and Adafruit_GFX
void OLEDDisplay_drawLine(OLEDDisplay_t *oled, int16_t x0, int16_t y0, int16_t x1, int16_t y1) {
  int16_t steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    _swap_int16_t(x0, y0);
    _swap_int16_t(x1, y1);
  }

  if (x0 > x1) {
    _swap_int16_t(x0, x1);
    _swap_int16_t(y0, y1);
  }

  int16_t dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);

  int16_t err = dx / 2;
  int16_t ystep;

  if (y0 < y1) {
    ystep = 1;
  } else {
    ystep = -1;
  }

  for (; x0<=x1; x0++) {
    if (steep) {
      OLEDDisplay_setPixel(oled, y0, x0);
    } else {
      OLEDDisplay_setPixel(oled, x0, y0);
    }
    err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  }
}

void OLEDDisplay_drawRect(OLEDDisplay_t *oled, int16_t x, int16_t y, int16_t width, int16_t height) {
  OLEDDisplay_drawHorizontalLine(oled, x, y, width);
  OLEDDisplay_drawVerticalLine(oled, x, y, height);
  OLEDDisplay_drawVerticalLine(oled, x + width - 1, y, height);
  OLEDDisplay_drawHorizontalLine(oled, x, y + height - 1, width);
}

void OLEDDisplay_fillRect(OLEDDisplay_t *oled, int16_t xMove, int16_t yMove, int16_t width, int16_t height) {
  for (int16_t x = xMove; x < xMove + width; x++) {
    OLEDDisplay_drawVerticalLine(oled, x, yMove, height);
  }
}

void OLEDDisplay_drawCircle(OLEDDisplay_t *oled, int16_t x0, int16_t y0, int16_t radius) {
  int16_t x = 0, y = radius;
	int16_t dp = 1 - radius;
	do {
		if (dp < 0)
			dp = dp + (x++) * 2 + 3;
		else
			dp = dp + (x++) * 2 - (y--) * 2 + 5;

		OLEDDisplay_setPixel(oled, x0 + x, y0 + y);     //For the 8 octants
		OLEDDisplay_setPixel(oled, x0 - x, y0 + y);
		OLEDDisplay_setPixel(oled, x0 + x, y0 - y);
		OLEDDisplay_setPixel(oled, x0 - x, y0 - y);
		OLEDDisplay_setPixel(oled, x0 + y, y0 + x);
		OLEDDisplay_setPixel(oled, x0 - y, y0 + x);
		OLEDDisplay_setPixel(oled, x0 + y, y0 - x);
		OLEDDisplay_setPixel(oled, x0 - y, y0 - x);

	} while (x < y);

  OLEDDisplay_setPixel(oled, x0 + radius, y0);
  OLEDDisplay_setPixel(oled, x0, y0 + radius);
  OLEDDisplay_setPixel(oled, x0 - radius, y0);
  OLEDDisplay_setPixel(oled, x0, y0 - radius);
}

void OLEDDisplay_drawCircleQuads(OLEDDisplay_t *oled, int16_t x0, int16_t y0, int16_t radius, uint8_t quads) {
  int16_t x = 0, y = radius;
  int16_t dp = 1 - radius;
  while (x < y) {
    if (dp < 0)
      dp = dp + (x++) * 2 + 3;
    else
      dp = dp + (x++) * 2 - (y--) * 2 + 5;
    if (quads & 0x1) {
      OLEDDisplay_setPixel(oled, x0 + x, y0 - y);
      OLEDDisplay_setPixel(oled, x0 + y, y0 - x);
    }
    if (quads & 0x2) {
      OLEDDisplay_setPixel(oled, x0 - y, y0 - x);
      OLEDDisplay_setPixel(oled, x0 - x, y0 - y);
    }
    if (quads & 0x4) {
      OLEDDisplay_setPixel(oled, x0 - y, y0 + x);
      OLEDDisplay_setPixel(oled, x0 - x, y0 + y);
    }
    if (quads & 0x8) {
      OLEDDisplay_setPixel(oled, x0 + x, y0 + y);
      OLEDDisplay_setPixel(oled, x0 + y, y0 + x);
    }
  }
  if (quads & 0x1 && quads & 0x8) {
    OLEDDisplay_setPixel(oled, x0 + radius, y0);
  }
  if (quads & 0x4 && quads & 0x8) {
    OLEDDisplay_setPixel(oled, x0, y0 + radius);
  }
  if (quads & 0x2 && quads & 0x4) {
    OLEDDisplay_setPixel(oled, x0 - radius, y0);
  }
  if (quads & 0x1 && quads & 0x2) {
    OLEDDisplay_setPixel(oled, x0, y0 - radius);
  }
}


void OLEDDisplay_fillCircle(OLEDDisplay_t *oled, int16_t x0, int16_t y0, int16_t radius) {
  int16_t x = 0, y = radius;
	int16_t dp = 1 - radius;
	do {
		if (dp < 0)
      dp = dp + (x++) * 2 + 3;
    else
      dp = dp + (x++) * 2 - (y--) * 2 + 5;

    OLEDDisplay_drawHorizontalLine(oled, x0 - x, y0 - y, 2*x);
    OLEDDisplay_drawHorizontalLine(oled, x0 - x, y0 + y, 2*x);
    OLEDDisplay_drawHorizontalLine(oled, x0 - y, y0 - x, 2*y);
    OLEDDisplay_drawHorizontalLine(oled, x0 - y, y0 + x, 2*y);


	} while (x < y);
  OLEDDisplay_drawHorizontalLine(oled, x0 - radius, y0, 2 * radius);

}

void OLEDDisplay_drawHorizontalLine(OLEDDisplay_t *oled, int16_t x, int16_t y, int16_t length) {
  if (y < 0 || y >= oled->height) { return; }

  if (x < 0) {
    length += x;
    x = 0;
  }

  if ( (x + length) > oled->width) {
    length = (oled->width - x);
  }

  if (length <= 0) { return; }

  uint8_t * bufferPtr = oled->buffer;
  bufferPtr += (y >> 3) * oled->width;
  bufferPtr += x;

  uint8_t drawBit = 1 << (y & 7);

  switch (oled->color) {
    case WHITE:   while (length--) {
        *bufferPtr++ |= drawBit;
      }; break;
    case BLACK:   drawBit = ~drawBit;   while (length--) {
        *bufferPtr++ &= drawBit;
      }; break;
    case INVERSE: while (length--) {
        *bufferPtr++ ^= drawBit;
      }; break;
  }
}

void OLEDDisplay_drawVerticalLine(OLEDDisplay_t *oled, int16_t x, int16_t y, int16_t length) {
  if (x < 0 || x >= oled->width) return;

  if (y < 0) {
    length += y;
    y = 0;
  }

  if ( (y + length) > oled->height) {
    length = (oled->height - y);
  }

  if (length <= 0) return;


  uint8_t yOffset = y & 7;
  uint8_t drawBit;
  uint8_t *bufferPtr = oled->buffer;

  bufferPtr += (y >> 3) * oled->width;
  bufferPtr += x;

  if (yOffset) {
    yOffset = 8 - yOffset;
    drawBit = ~(0xFF >> (yOffset));

    if (length < yOffset) {
      drawBit &= (0xFF >> (yOffset - length));
    }

    switch (oled->color) {
      case WHITE:   *bufferPtr |=  drawBit; break;
      case BLACK:   *bufferPtr &= ~drawBit; break;
      case INVERSE: *bufferPtr ^=  drawBit; break;
    }

    if (length < yOffset) return;

    length -= yOffset;
    bufferPtr += oled->width;
  }

  if (length >= 8) {
    switch (oled->color) {
      case WHITE:
      case BLACK:
        drawBit = (oled->color == WHITE) ? 0xFF : 0x00;
        do {
          *bufferPtr = drawBit;
          bufferPtr += oled->width;
          length -= 8;
        } while (length >= 8);
        break;
      case INVERSE:
        do {
          *bufferPtr = ~(*bufferPtr);
          bufferPtr += oled->width;
          length -= 8;
        } while (length >= 8);
        break;
    }
  }

  if (length > 0) {
    drawBit = (1 << (length & 7)) - 1;
    switch (oled->color) {
      case WHITE:   *bufferPtr |=  drawBit; break;
      case BLACK:   *bufferPtr &= ~drawBit; break;
      case INVERSE: *bufferPtr ^=  drawBit; break;
    }
  }
}

void OLEDDisplay_drawProgressBar(OLEDDisplay_t *oled, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t progress) {
  uint16_t radius = height / 2;
  uint16_t xRadius = x + radius;
  uint16_t yRadius = y + radius;
  uint16_t doubleRadius = 2 * radius;
  uint16_t innerRadius = radius - 2;

  OLEDDisplay_setColor(oled, WHITE);
  OLEDDisplay_drawCircleQuads(oled, xRadius, yRadius, radius, 0b00000110);
  OLEDDisplay_drawHorizontalLine(oled, xRadius, y, width - doubleRadius + 1);
  OLEDDisplay_drawHorizontalLine(oled, xRadius, y + height, width - doubleRadius + 1);
  OLEDDisplay_drawCircleQuads(oled, x + width - radius, yRadius, radius, 0b00001001);

  uint16_t maxProgressWidth = (width - doubleRadius + 1) * progress / 100;

  OLEDDisplay_fillCircle(oled, xRadius, yRadius, innerRadius);
  OLEDDisplay_fillRect(oled, xRadius + 1, y + 2, maxProgressWidth, height - 3);
  OLEDDisplay_fillCircle(oled, xRadius + maxProgressWidth, yRadius, innerRadius);
}

void OLEDDisplay_drawFastImage(OLEDDisplay_t *oled, int16_t xMove, int16_t yMove, int16_t width, int16_t height, const uint8_t *image) {
  OLEDDisplay_drawInternal(oled, xMove, yMove, width, height, image, 0, 0);
}

void OLEDDisplay_drawXbm(OLEDDisplay_t *oled, int16_t xMove, int16_t yMove, int16_t width, int16_t height, const uint8_t *xbm) {
  int16_t widthInXbm = (width + 7) / 8;
  uint8_t data = 0;

  for(int16_t y = 0; y < height; y++) {
    for(int16_t x = 0; x < width; x++ ) {
      if (x & 7) {
        data >>= 1; // Move a bit
      } else {  // Read new data every 8 bit
        data = pgm_read_byte(xbm + (x / 8) + y * widthInXbm);
      }
      // if there is a bit draw it
      if (data & 0x01) {
        OLEDDisplay_setPixel(oled, xMove + x, yMove + y);
      }
    }
  }
}

void OLEDDisplay_drawIco16x16(OLEDDisplay_t *oled, int16_t xMove, int16_t yMove, const char *ico, int inverse) {
  uint16_t data;

  for(int16_t y = 0; y < 16; y++) {
    data = pgm_read_byte(ico + (y << 1)) + (pgm_read_byte(ico + (y << 1) + 1) << 8);
    for(int16_t x = 0; x < 16; x++ ) {
      if ((data & 0x01) ^ inverse) {
        OLEDDisplay_setPixelColor(oled, xMove + x, yMove + y, WHITE);
      } else {
        OLEDDisplay_setPixelColor(oled, xMove + x, yMove + y, BLACK);
      }
      data >>= 1; // Move a bit
    }
  }
}

void OLEDDisplay_drawStringInternal(OLEDDisplay_t *oled, int16_t xMove, int16_t yMove, char* text, uint16_t textLength, uint16_t textWidth) {
  uint8_t textHeight       = pgm_read_byte(oled->fontData + HEIGHT_POS);
  uint8_t firstChar        = pgm_read_byte(oled->fontData + FIRST_CHAR_POS);
  uint16_t sizeOfJumpTable = pgm_read_byte(oled->fontData + CHAR_NUM_POS)  * JUMPTABLE_BYTES;

  uint16_t cursorX         = 0;
  uint16_t cursorY         = 0;

  switch (oled->textAlignment) {
    case TEXT_ALIGN_CENTER_BOTH:
      yMove -= textHeight >> 1;
    // Fallthrough
    case TEXT_ALIGN_CENTER:
      xMove -= textWidth >> 1; // divide by 2
      break;
    case TEXT_ALIGN_RIGHT:
      xMove -= textWidth;
      break;
    case TEXT_ALIGN_LEFT:
      break;
  }

  // Don't draw anything if it is not on the screen.
  if (xMove + textWidth  < 0 || xMove > oled->width ) {return;}
  if (yMove + textHeight < 0 || yMove > oled->height ) {return;}

  for (uint16_t j = 0; j < textLength; j++) {
    int16_t xPos = xMove + cursorX;
    int16_t yPos = yMove + cursorY;

    uint8_t code = text[j];
    if (code >= firstChar) {
      uint8_t charCode = code - firstChar;

      // 4 Bytes per char code
      uint8_t msbJumpToChar    = pgm_read_byte( oled->fontData + JUMPTABLE_START + charCode * JUMPTABLE_BYTES );                  // MSB  \ JumpAddress
      uint8_t lsbJumpToChar    = pgm_read_byte( oled->fontData + JUMPTABLE_START + charCode * JUMPTABLE_BYTES + JUMPTABLE_LSB);   // LSB /
      uint8_t charByteSize     = pgm_read_byte( oled->fontData + JUMPTABLE_START + charCode * JUMPTABLE_BYTES + JUMPTABLE_SIZE);  // Size
      uint8_t currentCharWidth = pgm_read_byte( oled->fontData + JUMPTABLE_START + charCode * JUMPTABLE_BYTES + JUMPTABLE_WIDTH); // Width

      // Test if the char is drawable
      if (!(msbJumpToChar == 255 && lsbJumpToChar == 255)) {
        // Get the position of the char data
        uint16_t charDataPosition = JUMPTABLE_START + sizeOfJumpTable + ((msbJumpToChar << 8) + lsbJumpToChar);
        OLEDDisplay_drawInternal(oled, xPos, yPos, currentCharWidth, textHeight, oled->fontData, charDataPosition, charByteSize);
      }

      cursorX += currentCharWidth;
    }
  }
}


void OLEDDisplay_drawString(OLEDDisplay_t *oled, int16_t xMove, int16_t yMove, char * strUser) {
	printf("DEBUG MARK %d\n",__LINE__);
  uint16_t lineHeight = pgm_read_byte(oled->fontData + HEIGHT_POS);

  // char* text must be freed!
  char* text = strUser; //utf8 to ASCII MUST FREE BELOW IF USED

	printf("DEBUG MARK %d\n",__LINE__);
  uint16_t yOffset = 0;
  // If the string should be centered vertically too
  // we need to now how heigh the string is.
  if (oled->textAlignment == TEXT_ALIGN_CENTER_BOTH) {
    uint16_t lb = 0;
    // Find number of linebreaks in text
    for (uint16_t i=0;text[i] != 0; i++) {
      lb += (text[i] == 10);
    }
    // Calculate center
    yOffset = (lb * lineHeight) / 2;
  }
	printf("DEBUG MARK %d\n",__LINE__);

  uint16_t line = 0;
  char* textPart = strtok(text,"\n");
	printf("DEBUG MARK %d\n",__LINE__);
  while (textPart != NULL) {
    uint16_t length = strlen(textPart);
	printf("DEBUG MARK %d\n",__LINE__);
    OLEDDisplay_drawStringInternal(oled, xMove, yMove - yOffset + (line++) * lineHeight, textPart, length, OLEDDisplay_getStringWidthLen(oled, textPart, length));
    textPart = strtok(NULL, "\n");
  }
	printf("DEBUG MARK %d\n",__LINE__);
  // NEED FOR UTF8 if used free(text);
}

#if 0
void OLEDDisplay_drawStringf(OLEDDisplay_t *oled,  int16_t x, int16_t y, char* buffer, char * format, ... )
{
  va_list myargs;
  va_start(myargs, format);
  vsprintf(buffer, format.c_str(), myargs);
  va_end(myargs);
  OLEDDisplay_drawString(oled, x, y, buffer );
}
#endif

void OLEDDisplay_drawStringMaxWidth(OLEDDisplay_t *oled, int16_t xMove, int16_t yMove, uint16_t maxLineWidth, char * strUser) {
  uint16_t firstChar  = pgm_read_byte(oled->fontData + FIRST_CHAR_POS);
  uint16_t lineHeight = pgm_read_byte(oled->fontData + HEIGHT_POS);

  char* text = strUser; //utf8ascii(strUser);

  uint16_t length = strlen(text);
  uint16_t lastDrawnPos = 0;
  uint16_t lineNumber = 0;
  uint16_t strWidth = 0;

  uint16_t preferredBreakpoint = 0;
  uint16_t widthAtBreakpoint = 0;

  for (uint16_t i = 0; i < length; i++) {
    strWidth += pgm_read_byte(oled->fontData + JUMPTABLE_START + (text[i] - firstChar) * JUMPTABLE_BYTES + JUMPTABLE_WIDTH);

    // Always try to break on a space or dash
    if (text[i] == ' ' || text[i]== '-') {
      preferredBreakpoint = i;
      widthAtBreakpoint = strWidth;
    }

    if (strWidth >= maxLineWidth) {
      if (preferredBreakpoint == 0) {
        preferredBreakpoint = i;
        widthAtBreakpoint = strWidth;
      }
      OLEDDisplay_drawStringInternal(oled, xMove, yMove + (lineNumber++) * lineHeight , &text[lastDrawnPos], preferredBreakpoint - lastDrawnPos, widthAtBreakpoint);
      lastDrawnPos = preferredBreakpoint + 1;
      // It is possible that we did not draw all letters to i so we need
      // to account for the width of the chars from `i - preferredBreakpoint`
      // by calculating the width we did not draw yet.
      strWidth = strWidth - widthAtBreakpoint;
      preferredBreakpoint = 0;
    }
  }

  // Draw last part if needed
  if (lastDrawnPos < length) {
    OLEDDisplay_drawStringInternal(oled, xMove, yMove + lineNumber * lineHeight , &text[lastDrawnPos], length - lastDrawnPos, OLEDDisplay_getStringWidthLen(oled,&text[lastDrawnPos], length - lastDrawnPos));
  }

  free(text);
}

uint16_t OLEDDisplay_getStringWidthLen(OLEDDisplay_t *oled, const char* text, uint16_t length) {
  uint16_t firstChar        = pgm_read_byte(oled->fontData + FIRST_CHAR_POS);

  uint16_t stringWidth = 0;
  uint16_t maxWidth = 0;

  while (length--) {
    stringWidth += pgm_read_byte(oled->fontData + JUMPTABLE_START + (text[length] - firstChar) * JUMPTABLE_BYTES + JUMPTABLE_WIDTH);
    if (text[length] == 10) {
      maxWidth = _max(maxWidth, stringWidth);
      stringWidth = 0;
    }
  }

  return _max(maxWidth, stringWidth);
}

uint16_t OLEDDisplay_getStringWidth(OLEDDisplay_t *oled, const char * strUser) {
  char* text = (char *) strUser; //utf8ascii(strUser);
  uint16_t length = strlen(text);
  uint16_t width = OLEDDisplay_getStringWidthLen(oled,text, length);
  free(text);
  return width;
}

void OLEDDisplay_setTextAlignment(OLEDDisplay_t *oled, OLEDDISPLAY_TEXT_ALIGNMENT textAlignment) {
  oled->textAlignment = textAlignment;
}

void OLEDDisplay_setFont(OLEDDisplay_t *oled, const uint8_t *fontData) {
  oled->fontData = fontData;
}

void OLEDDisplay_displayOn(OLEDDisplay_t *oled) {
  OLEDDisplay_sendCommand(oled, DISPLAYON);
}

void OLEDDisplay_displayOff(OLEDDisplay_t *oled) {
  OLEDDisplay_sendCommand(oled, DISPLAYOFF);
}

void OLEDDisplay_invertDisplay(OLEDDisplay_t *oled) {
  OLEDDisplay_sendCommand(oled, INVERTDISPLAY);
}

void OLEDDisplay_normalDisplay(OLEDDisplay_t *oled) {
  OLEDDisplay_sendCommand(oled, NORMALDISPLAY);
}

void OLEDDisplay_setContrast_internal(OLEDDisplay_t *oled, uint8_t contrast, uint8_t precharge, uint8_t comdetect) {
  OLEDDisplay_sendCommand(oled, SETPRECHARGE); //0xD9
  OLEDDisplay_sendCommand(oled, precharge); //0xF1 default, to lower the contrast, put 1-1F
  OLEDDisplay_sendCommand(oled, SETCONTRAST);
  OLEDDisplay_sendCommand(oled, contrast); // 0-255
  OLEDDisplay_sendCommand(oled, SETVCOMDETECT); //0xDB, (additionally needed to lower the contrast)
  OLEDDisplay_sendCommand(oled, comdetect);	//0x40 default, to lower the contrast, put 0
  OLEDDisplay_sendCommand(oled, DISPLAYALLON_RESUME);
  OLEDDisplay_sendCommand(oled, NORMALDISPLAY);
  OLEDDisplay_sendCommand(oled, DISPLAYON);
}

void OLEDDisplay_setBrightness(OLEDDisplay_t *oled, uint8_t brightness) {
  uint8_t contrast = brightness;
  if (brightness < 128) {
    // Magic values to get a smooth/ step-free transition
    contrast = brightness * 1.171;
  } else {
    contrast = brightness * 1.171 - 43;
  }

  uint8_t precharge = 241;
  if (brightness == 0) {
    precharge = 0;
  }
  uint8_t comdetect = brightness / 8;

  OLEDDisplay_setContrast_internal(oled, contrast, precharge, comdetect);
}

void OLEDDisplay_resetOrientation(OLEDDisplay_t *oled) {
  OLEDDisplay_sendCommand(oled, SEGREMAP);
  OLEDDisplay_sendCommand(oled, COMSCANINC);           //Reset screen rotation or mirroring
}

void OLEDDisplay_flipScreenVertically(OLEDDisplay_t *oled) {
  OLEDDisplay_sendCommand(oled, SEGREMAP | 0x01);
  OLEDDisplay_sendCommand(oled, COMSCANDEC);           //Rotate screen 180 Deg
}

void OLEDDisplay_mirrorScreen(OLEDDisplay_t *oled) {
  OLEDDisplay_sendCommand(oled, SEGREMAP);
  OLEDDisplay_sendCommand(oled, COMSCANDEC);           //Mirror screen
}

void OLEDDisplay_clear(OLEDDisplay_t *oled) {
  memset(oled->buffer, 0, oled->displayBufferSize);
}

void OLEDDisplay_drawLogBuffer(OLEDDisplay_t *oled, uint16_t xMove, uint16_t yMove) {
  uint16_t lineHeight = pgm_read_byte(oled->fontData + HEIGHT_POS);
  // Always align left
  OLEDDisplay_setTextAlignment(oled, TEXT_ALIGN_LEFT);

  // State values
  uint16_t length   = 0;
  uint16_t line     = 0;
  uint16_t lastPos  = 0;

  for (uint16_t i=0;i<oled->logBufferFilled;i++){
    // Everytime we have a \n print
    if (oled->logBuffer[i] == 10) {
      length++;
      // Draw string on line `line` from lastPos to length
      // Passing 0 as the lenght because we are in TEXT_ALIGN_LEFT
      OLEDDisplay_drawStringInternal(oled, xMove, yMove + (line++) * lineHeight, &oled->logBuffer[lastPos], length, 0);
      // Remember last pos
      lastPos = i;
      // Reset length
      length = 0;
    } else {
      // Count chars until next linebreak
      length++;
    }
  }
  // Draw the remaining string
  if (length > 0) {
    OLEDDisplay_drawStringInternal(oled,xMove, yMove + line * lineHeight, &oled->logBuffer[lastPos], length, 0);
  }
}

uint16_t OLEDDisplay_getWidth(OLEDDisplay_t *oled) {
  return oled->width;
}

uint16_t OLEDDisplay_getHeight(OLEDDisplay_t *oled) {
  return oled->height;
}

int OLEDDisplay_setLogBuffer(OLEDDisplay_t *oled, uint16_t lines, uint16_t chars){
  if (oled->logBuffer != NULL) free(oled->logBuffer);
  uint16_t size = lines * chars;
  if (size > 0) {
    oled->logBufferLine     = 0;      // Lines printed
    oled->logBufferFilled   = 0;      // Nothing stored yet
    oled->logBufferMaxLines = lines;  // Lines max printable
    oled->logBufferSize     = size;   // Total number of characters the buffer can hold
    oled->logBuffer         = (char *) malloc(size * sizeof(uint8_t));
    if(!oled->logBuffer) {
      DEBUG_OLEDDISPLAY("[OLEDDISPLAY][setLogBuffer] Not enough memory to create log buffer\n");
      return -1;
    }
  }
  return 0;
}

size_t OLEDDisplay_write(OLEDDisplay_t *oled, uint8_t c) {
  if (oled->logBufferSize > 0) {
    // Don't waste space on \r\n line endings, dropping \r
    if (c == 13) return 1;

    // convert UTF-8 character to font table index
    c = (oled->fontTableLookupFunction)(c);
    // drop unknown character
    if (c == 0) return 1;

    int maxLineNotReached = oled->logBufferLine < oled->logBufferMaxLines;
    int bufferNotFull = oled->logBufferFilled < oled->logBufferSize;

    // Can we write to the buffer?
    if (bufferNotFull && maxLineNotReached) {
      oled->logBuffer[oled->logBufferFilled] = c;
      oled->logBufferFilled++;
      // Keep track of lines written
      if (c == 10) oled->logBufferLine++;
    } else {
      // Max line number is reached
      if (!maxLineNotReached) oled->logBufferLine--;

      // Find the end of the first line
      uint16_t firstLineEnd = 0;
      for (uint16_t i=0;i<oled->logBufferFilled;i++) {
        if (oled->logBuffer[i] == 10){
          // Include last char too
          firstLineEnd = i + 1;
          break;
        }
      }
      // If there was a line ending
      if (firstLineEnd > 0) {
        // Calculate the new logBufferFilled value
        oled->logBufferFilled = oled->logBufferFilled - firstLineEnd;
        // Now we move the lines infront of the buffer
        memcpy(oled->logBuffer, &oled->logBuffer[firstLineEnd], oled->logBufferFilled);
      } else {
        // Let's reuse the buffer if it was full
        if (!bufferNotFull) {
          oled->logBufferFilled = 0;
        }// else {
        //  Nothing to do here
        //}
      }
      OLEDDisplay_write(oled, c);
    }
  }
  // We are always writing all uint8_t to the buffer
  return 1;
}

size_t OLEDDisplay_writeString(OLEDDisplay_t *oled, const char* str) {
  if (str == NULL) return 0;
  size_t length = strlen(str);
  for (size_t i = 0; i < length; i++) {
    OLEDDisplay_write(oled,str[i]);
  }
  return length;
}


void OLEDDisplay_setGeometry(OLEDDisplay_t *oled, OLEDDISPLAY_GEOMETRY g, uint16_t width, uint16_t height) {
  oled->geometry = g;

  switch (g) {
    case GEOMETRY_128_64:
      oled->width = 128;
      oled->height = 64;
      break;
    case GEOMETRY_128_32:
      oled->width = 128;
      oled->height = 32;
      break;
    case GEOMETRY_64_48:
      oled->width = 64;
      oled->height = 48;
      break;
    case GEOMETRY_64_32:
      oled->width = 64;
      oled->height = 32;
      break;
    case GEOMETRY_RAWMODE:
      oled->width = width > 0 ? width : 128;
      oled->height = height > 0 ? height : 64;
      break;
  }
  oled->displayBufferSize = width * oled->height / 8;
}

void OLEDDisplay_sendInitCommands(OLEDDisplay_t *oled) {
  if (oled->geometry == GEOMETRY_RAWMODE)
  	return;
  OLEDDisplay_sendCommand(oled,DISPLAYOFF);
  OLEDDisplay_sendCommand(oled,SETDISPLAYCLOCKDIV);
  OLEDDisplay_sendCommand(oled,0xF0); // Increase speed of the display max ~96Hz
  OLEDDisplay_sendCommand(oled,SETMULTIPLEX);
  OLEDDisplay_sendCommand(oled,oled->height - 1);
  OLEDDisplay_sendCommand(oled,SETDISPLAYOFFSET);
  OLEDDisplay_sendCommand(oled,0x00);
  if(oled->geometry == GEOMETRY_64_32)
    OLEDDisplay_sendCommand(oled,0x00);  
  else
    OLEDDisplay_sendCommand(oled,SETSTARTLINE);
  OLEDDisplay_sendCommand(oled,CHARGEPUMP);
  OLEDDisplay_sendCommand(oled,0x14);
  OLEDDisplay_sendCommand(oled,MEMORYMODE);
  OLEDDisplay_sendCommand(oled,0x00);
  OLEDDisplay_sendCommand(oled,SEGREMAP);
  OLEDDisplay_sendCommand(oled,COMSCANINC);
  OLEDDisplay_sendCommand(oled,SETCOMPINS);

  if (oled->geometry == GEOMETRY_128_64 || oled->geometry == GEOMETRY_64_48 || oled->geometry == GEOMETRY_64_32) {
    OLEDDisplay_sendCommand(oled,0x12);
  } else if (oled->geometry == GEOMETRY_128_32) {
    OLEDDisplay_sendCommand(oled,0x02);
  }

  OLEDDisplay_sendCommand(oled,SETCONTRAST);

  if (oled->geometry == GEOMETRY_128_64 || oled->geometry == GEOMETRY_64_48 || oled->geometry == GEOMETRY_64_32) {
    OLEDDisplay_sendCommand(oled,0xCF);
  } else if (oled->geometry == GEOMETRY_128_32) {
    OLEDDisplay_sendCommand(oled,0x8F);
  }

  OLEDDisplay_sendCommand(oled,SETPRECHARGE);
  OLEDDisplay_sendCommand(oled,0xF1);
  OLEDDisplay_sendCommand(oled,SETVCOMDETECT); //0xDB, (additionally needed to lower the contrast)
  OLEDDisplay_sendCommand(oled,0x40);	        //0x40 default, to lower the contrast, put 0
  OLEDDisplay_sendCommand(oled,DISPLAYALLON_RESUME);
  OLEDDisplay_sendCommand(oled,NORMALDISPLAY);
  OLEDDisplay_sendCommand(oled,0x2e);            // stop scroll
  OLEDDisplay_sendCommand(oled,DISPLAYON);

  OLEDDisplay_sendCommand(oled,COLUMNADDR);
  OLEDDisplay_sendCommand(oled,0x00);
  OLEDDisplay_sendCommand(oled,0x7f);

  OLEDDisplay_sendCommand(oled,PAGEADDR);
  OLEDDisplay_sendCommand(oled,0x00);
  OLEDDisplay_sendCommand(oled,0x07);
}

void inline OLEDDisplay_drawInternal(OLEDDisplay_t *oled, int16_t xMove, int16_t yMove, int16_t width, int16_t height, const uint8_t *data, uint16_t offset, uint16_t bytesInData) {
  if (width < 0 || height < 0) return;
  if (yMove + height < 0 || yMove > oled->height)  return;
  if (xMove + width  < 0 || xMove > oled->width)   return;

  uint8_t  rasterHeight = 1 + ((height - 1) >> 3); // fast ceil(height / 8.0)
  int8_t   yOffset      = yMove & 7;

  bytesInData = bytesInData == 0 ? width * rasterHeight : bytesInData;

  int16_t initYMove   = yMove;
  int8_t  initYOffset = yOffset;


  for (uint16_t i = 0; i < bytesInData; i++) {

    // Reset if next horizontal drawing phase is started.
    if ( i % rasterHeight == 0) {
      yMove   = initYMove;
      yOffset = initYOffset;
    }

    uint8_t currentByte = pgm_read_byte(data + offset + i);

    int16_t xPos = xMove + (i / rasterHeight);
    int16_t yPos = ((yMove >> 3) + (i % rasterHeight)) * oled->width;

//    int16_t yScreenPos = yMove + yOffset;
    int16_t dataPos    = xPos  + yPos;

    if (dataPos >=  0  && dataPos < oled->displayBufferSize &&
        xPos    >=  0  && xPos    < oled->width ) {

      if (yOffset >= 0) {
        switch (oled->color) {
          case WHITE:   oled->buffer[dataPos] |= currentByte << yOffset; break;
          case BLACK:   oled->buffer[dataPos] &= ~(currentByte << yOffset); break;
          case INVERSE: oled->buffer[dataPos] ^= currentByte << yOffset; break;
        }

        if (dataPos < (oled->displayBufferSize - oled->width)) {
          switch (oled->color) {
            case WHITE:   oled->buffer[dataPos + oled->width] |= currentByte >> (8 - yOffset); break;
            case BLACK:   oled->buffer[dataPos + oled->width] &= ~(currentByte >> (8 - yOffset)); break;
            case INVERSE: oled->buffer[dataPos + oled->width] ^= currentByte >> (8 - yOffset); break;
          }
        }
      } else {
        // Make new offset position
        yOffset = -yOffset;

        switch (oled->color) {
          case WHITE:   oled->buffer[dataPos] |= currentByte >> yOffset; break;
          case BLACK:   oled->buffer[dataPos] &= ~(currentByte >> yOffset); break;
          case INVERSE: oled->buffer[dataPos] ^= currentByte >> yOffset; break;
        }

        // Prepare for next iteration by moving one block up
        yMove -= 8;

        // and setting the new yOffset
        yOffset = 8 - yOffset;
      }
    }
  }
}

// You need to free the char!
char* OLEDDisplay_utf8ascii(OLEDDisplay_t *oled, char * str) {
  uint16_t k = 0;
  uint16_t length = strlen(str) + 1;

  // Copy the string into a char array
  char* s = (char*) malloc(length * sizeof(char));
  if(!s) {
    DEBUG_OLEDDISPLAY("[OLEDDISPLAY][utf8ascii] Can't allocate another char array. Drop support for UTF-8.\n");
    return (char*) str;
  }
  //str.toCharArray(s, length);

  length--;

  for (uint16_t i=0; i < length; i++) {
    char c = (oled->fontTableLookupFunction)(s[i]);
    if (c!=0) {
      s[k++]=c;
    }
  }

  s[k]=0;

  // This will leak 's' be sure to free it in the calling function.
  return s;
}

void OLEDDisplay_setFontTableLookupFunction(OLEDDisplay_t *oled, FontTableLookupFunction function) {
  oled->fontTableLookupFunction = function;
}


char DefaultFontTableLookup(const uint8_t ch) {
    // UTF-8 to font table index converter
    // Code form http://playground.arduino.cc/Main/Utf8ascii
	static uint8_t LASTCHAR;

	if (ch < 128) { // Standard ASCII-set 0..0x7F handling
		LASTCHAR = 0;
		return ch;
	}

	uint8_t last = LASTCHAR;   // get last char
	LASTCHAR = ch;

	switch (last) {    // conversion depnding on first UTF8-character
		case 0xC2: return (uint8_t) ch;
		case 0xC3: return (uint8_t) (ch | 0xC0);
		case 0x82: if (ch == 0xAC) return (uint8_t) 0x80;    // special case Euro-symbol
	}

	return (uint8_t) 0; // otherwise: return zero, if character has to be ignored
}


void OLEDDisplay_displayOLD(OLEDDisplay_t *oled) {
	#ifdef OLEDDISPLAY_DOUBLE_BUFFER
	uint8_t minBoundY = UINT8_MAX;
	uint8_t maxBoundY = 0;

	uint8_t minBoundX = UINT8_MAX;
	uint8_t maxBoundX = 0;

	uint8_t x, y;

	// Calculate the Y bounding box of changes
	// and copy buffer[pos] to buffer_back[pos];
	for (y = 0; y < (oled->height / 8); y++) {
	  for (x = 0; x < oled->width; x++) {
	   uint16_t pos = x + y * oled->width;
	   if (oled->buffer[pos] != oled->buffer_back[pos]) {
	     minBoundY = _min(minBoundY, y);
	     maxBoundY = _max(maxBoundY, y);
	     minBoundX = _min(minBoundX, x);
	     maxBoundX = _max(maxBoundX, x);
	   }
	   oled->buffer_back[pos] = oled->buffer[pos];
	 }
	 //yield();
	}

	printf("DISPLAY OFFSET %d,%d -> %d,%d\n",minBoundX,minBoundY,maxBoundX,maxBoundY);
	// If the minBoundY wasn't updated
	// we can savely assume that buffer_back[pos] == buffer[pos]
	// holdes true for all values of pos
	if (minBoundY == UINT8_MAX) return;

	// Calculate the colum offset
	uint8_t minBoundXp2H = (minBoundX + 2) & 0x0F;
	uint8_t minBoundXp2L = 0x10 | ((minBoundX + 2) >> 4 );

	uint8_t k = 0;
	for (y = minBoundY; y <= maxBoundY; y++) {
	  OLEDDisplay_sendCommand(oled, 0xB0 + y);
	  OLEDDisplay_sendCommand(oled, minBoundXp2H);
	  OLEDDisplay_sendCommand(oled, minBoundXp2L);
	  for (x = minBoundX; x <= maxBoundX; x++) {
	    if (k == 0) {
	      OLEDDisplay_beginPayload(oled);
	      OLEDDisplay_i2cwrite(oled,0x40);
	    }
	    OLEDDisplay_i2cwrite(oled,oled->buffer[x + y * oled->width]);
	    k++;
	    if (k == 16)  {
	      OLEDDisplay_endPayload(oled);
	      k = 0;
	    }
	  }
	  if (k != 0)  {
	    OLEDDisplay_endPayload(oled);
	    k = 0;
	  }
	  //yield();
	}

	if (k != 0) {
	  OLEDDisplay_endPayload(oled);
	}
	#else
	uint8_t * p = &oled->buffer[0];
	for (uint8_t y=0; y<8; y++) {
	  //OLEDDisplay_sendCommand(oled, 0xB0+y);
	  //OLEDDisplay_sendCommand(oled, 0x02);
	  //OLEDDisplay_sendCommand(oled, 0x10);
	  for( uint8_t x=0; x<8; x++) {
	    OLEDDisplay_beginPayload(oled);
	    OLEDDisplay_i2cwrite(oled,0x40);
	    for (uint8_t k = 0; k < 16; k++) {
	      OLEDDisplay_i2cwrite(oled,*p++);
	    }
			OLEDDisplay_endPayload(oled);
	  }
	}
	#endif
}

void OLEDDisplay_display(OLEDDisplay_t *oled) {
	const int x_offset = (128 - oled->width) / 2;
#ifdef OLEDDISPLAY_DOUBLE_BUFFER
		uint8_t minBoundY = UINT8_MAX;
		uint8_t maxBoundY = 0;

		uint8_t minBoundX = UINT8_MAX;
		uint8_t maxBoundX = 0;
		uint8_t x, y;

		// Calculate the Y bounding box of changes
		// and copy buffer[pos] to buffer_back[pos];
		for (y = 0; y < (oled->height / 8); y++) {
			for (x = 0; x < oled->width; x++) {
			 uint16_t pos = x + y * oled->width;
			 if (oled->buffer[pos] != oled->buffer_back[pos]) {
				 minBoundY = _min(minBoundY, y);
				 maxBoundY = _max(maxBoundY, y);
				 minBoundX = _min(minBoundX, x);
				 maxBoundX = _max(maxBoundX, x);
			 }
			 oled->buffer_back[pos] = oled->buffer[pos];
		 }
		}

		// If the minBoundY wasn't updated
		// we can savely assume that buffer_back[pos] == buffer[pos]
		// holdes true for all values of pos

		if (minBoundY == UINT8_MAX) return;

		OLEDDisplay_sendCommand(oled,COLUMNADDR);
		OLEDDisplay_sendCommand(oled,x_offset + minBoundX);	// column start address (0 = reset)
		OLEDDisplay_sendCommand(oled,x_offset + maxBoundX);	// column end address (127 = reset)

		OLEDDisplay_sendCommand(oled,PAGEADDR);
		OLEDDisplay_sendCommand(oled,minBoundY);				// page start address
		OLEDDisplay_sendCommand(oled,maxBoundY);				// page end address

		for (y = minBoundY; y <= maxBoundY; y++) {
			uint8_t *start = &oled->buffer[(minBoundX + y * oled->width)];
			OLEDDisplay_sendPayload(oled,start, (maxBoundX-minBoundX) + 1);
		}
#else

		OLEDDisplay_sendCommand(oled,COLUMNADDR);
		OLEDDisplay_sendCommand(oled,x_offset);						// column start address (0 = reset)
		OLEDDisplay_sendCommand(oled,x_offset + (oled->width - 1));// column end address (127 = reset)

		OLEDDisplay_sendCommand(oled,PAGEADDR);
		OLEDDisplay_sendCommand(oled,0x0);							// page start address (0 = reset)

		if (oled->geometry == GEOMETRY_128_64) {
			OLEDDisplay_sendCommand(oled,0x7);
		} else if (oled->geometry == GEOMETRY_128_32) {
			OLEDDisplay_sendCommand(oled,0x3);
		}

		OLEDDisplay_sendPayload(oled,oled->buffer,oled->displayBufferSize);
#endif
}
