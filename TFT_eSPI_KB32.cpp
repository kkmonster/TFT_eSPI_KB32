/***************************************************
  Arduino TFT graphics library targeted at 32 bit
  processors such as ESP32, ESP8266 and STM32.

  This is a standalone library that contains the
  hardware driver, the graphics functions and the
  proportional fonts.

  The larger fonts are Run Length Encoded to reduce their
  size.

  Created by Bodmer 2/12/16
  Last update by Bodmer 20/03/20
 ****************************************************/


#include "TFT_eSPI_KB32.h"
#include "Processors/TFT_eSPI_ESP32.c"

 // Clipping macro for pushImage
#define PI_CLIP                                        \
  if (_vpOoB) return;                                  \
  x+= _xDatum;                                         \
  y+= _yDatum;                                         \
                                                       \
  if ((x >= _vpW) || (y >= _vpH)) return;              \
                                                       \
  int32_t dx = 0;                                      \
  int32_t dy = 0;                                      \
  int32_t dw = w;                                      \
  int32_t dh = h;                                      \
                                                       \
  if (x < _vpX) { dx = _vpX - x; dw -= dx; x = _vpX; } \
  if (y < _vpY) { dy = _vpY - y; dh -= dy; y = _vpY; } \
                                                       \
  if ((x + dw) > _vpW ) dw = _vpW - x;                 \
  if ((y + dh) > _vpH ) dh = _vpH - y;                 \
                                                       \
  if (dw < 1 || dh < 1) return;


void TFT_eSPI_KB32::LED_select(void) {
  delay(10);
  Wire.beginTransmission(0x70); // transmit to device #4
  delayMicroseconds(1);
  Wire.write(0x01);              // sends one byte
  Wire.write(0x00);              // sends one byte
  Wire.endTransmission();    // stop transmitting
  delay(10);

}

void TFT_eSPI_KB32::LCD_select(void) {
  delay(10);
  Wire.beginTransmission(0x70); // transmit to device #4
  delayMicroseconds(1);
  Wire.write(0x01);              // sends one byte
  Wire.write(0x01);              // sends one byte
  Wire.endTransmission();    // stop transmitting
  delay(10);

}

void TFT_eSPI_KB32::Set_brightness(uint8_t value) {

  if (value > 0x0F)
    value = 0x0F;

  delay(10);
  Wire.beginTransmission(0x70); // transmit to device #4
  delayMicroseconds(1);
  Wire.write(0xE0 | value);     // sends one byte
  Wire.endTransmission();    // stop transmitting
  delay(10);

}

/***************************************************************************************
** Function name:           begin_tft_write (was called spi_begin)
** Description:             Start SPI transaction for writes and select TFT
***************************************************************************************/
inline void TFT_eSPI_KB32::begin_tft_write(void) {
#if defined (SPI_HAS_TRANSACTION) && defined (SUPPORT_TRANSACTIONS) 
  if (locked) {
    locked = false;
    spi.beginTransaction(SPISettings(SPI_FREQUENCY, MSBFIRST, TFT_SPI_MODE));
    CS_L;
    SET_BUS_WRITE_MODE;
  }
#else
  CS_L;
  SET_BUS_WRITE_MODE;
#endif

}

/***************************************************************************************
** Function name:           end_tft_write (was called spi_end)
** Description:             End transaction for write and deselect TFT
***************************************************************************************/
inline void TFT_eSPI_KB32::end_tft_write(void) {
#if defined (SPI_HAS_TRANSACTION) && defined (SUPPORT_TRANSACTIONS) 
  if (!inTransaction) {
    if (!locked) {
      locked = true;
      CS_H;
      spi.endTransaction();
    }
    SET_BUS_READ_MODE;
  }
#else
  if (!inTransaction) { CS_H; SET_BUS_READ_MODE; }
#endif
}

/***************************************************************************************
** Function name:           begin_tft_read  (was called spi_begin_read)
** Description:             Start transaction for reads and select TFT
***************************************************************************************/
// Reads require a lower SPI clock rate than writes
inline void TFT_eSPI_KB32::begin_tft_read(void) {
  DMA_BUSY_CHECK; // Wait for any DMA transfer to complete before changing SPI settings
#if defined (SPI_HAS_TRANSACTION) && defined (SUPPORT_TRANSACTIONS) 
  if (locked) {
    locked = false;
    spi.beginTransaction(SPISettings(SPI_READ_FREQUENCY, MSBFIRST, TFT_SPI_MODE));
    CS_L;
  }
#else

  spi.setFrequency(SPI_READ_FREQUENCY);
  CS_L;

#endif
  SET_BUS_READ_MODE;
}

/***************************************************************************************
** Function name:           setViewport
** Description:             Set the clipping region for the TFT screen
***************************************************************************************/
void TFT_eSPI_KB32::setViewport(int32_t x, int32_t y, int32_t w, int32_t h, bool vpDatum)
{
  // Viewport metrics (not clipped)
  _xDatum = x; // Datum x position in screen coordinates
  _yDatum = y; // Datum y position in screen coordinates
  _xWidth = w; // Viewport width
  _yHeight = h; // Viewport height

  // Full size default viewport
  _vpDatum = false; // Datum is at top left corner of screen (true = top left of viewport)
  _vpOoB = false; // Out of Bounds flag (true is all of viewport is off screen)
  _vpX = 0;         // Viewport top left corner x coordinate
  _vpY = 0;         // Viewport top left corner y coordinate
  _vpW = width();   // Equivalent of TFT width  (Nb: viewport right edge coord + 1)
  _vpH = height();  // Equivalent of TFT height (Nb: viewport bottom edge coord + 1)

  // Clip viewport to screen area
  if (x < 0) { w += x; x = 0; }
  if (y < 0) { h += y; y = 0; }
  if ((x + w) > width()) { w = width() - x; }
  if ((y + h) > height()) { h = height() - y; }

  //Serial.print(" x=");Serial.print( x);Serial.print(", y=");Serial.print( y);
  //Serial.print(", w=");Serial.print(w);Serial.print(", h=");Serial.println(h);

  // Check if viewport is entirely out of bounds
  if (w < 1 || h < 1)
  {
    // Set default values and Out of Bounds flag in case of error
    _xDatum = 0;
    _yDatum = 0;
    _xWidth = width();
    _yHeight = height();
    _vpOoB = true;      // Set Out of Bounds flag to inhibit all drawing
    return;
  }

  if (!vpDatum)
  {
    _xDatum = 0; // Reset to top left of screen if not using a viewport datum
    _yDatum = 0;
    _xWidth = width();
    _yHeight = height();
  }

  // Store the clipped screen viewport metrics and datum position
  _vpX = x;
  _vpY = y;
  _vpW = x + w;
  _vpH = y + h;
  _vpDatum = vpDatum;

  //Serial.print(" _xDatum=");Serial.print( _xDatum);Serial.print(", _yDatum=");Serial.print( _yDatum);
  //Serial.print(", _xWidth=");Serial.print(_xWidth);Serial.print(", _yHeight=");Serial.println(_yHeight);

  //Serial.print(" _vpX=");Serial.print( _vpX);Serial.print(", _vpY=");Serial.print( _vpY);
  //Serial.print(", _vpW=");Serial.print(_vpW);Serial.print(", _vpH=");Serial.println(_vpH);

}

/***************************************************************************************
** Function name:           checkViewport
** Description:             Check if any part of specified area is visible in viewport
***************************************************************************************/
// Note: Setting w and h to 1 will check if coordinate x,y is in area
bool TFT_eSPI_KB32::checkViewport(int32_t x, int32_t y, int32_t w, int32_t h)
{
  if (_vpOoB) return false;
  x += _xDatum;
  y += _yDatum;

  if ((x >= _vpW) || (y >= _vpH)) return false;

  int32_t dx = 0;
  int32_t dy = 0;
  int32_t dw = w;
  int32_t dh = h;

  if (x < _vpX) { dx = _vpX - x; dw -= dx; x = _vpX; }
  if (y < _vpY) { dy = _vpY - y; dh -= dy; y = _vpY; }

  if ((x + dw) > _vpW) dw = _vpW - x;
  if ((y + dh) > _vpH) dh = _vpH - y;

  if (dw < 1 || dh < 1) return false;

  return true;
}

/***************************************************************************************
** Function name:           resetViewport
** Description:             Reset viewport to whle TFT screen, datum at 0,0
***************************************************************************************/
void TFT_eSPI_KB32::resetViewport(void)
{
  // Reset viewport to the whole screen (or sprite) area
  _vpDatum = false;
  _vpOoB = false;
  _xDatum = 0;
  _yDatum = 0;
  _vpX = 0;
  _vpY = 0;
  _vpW = width();
  _vpH = height();
  _xWidth = width();
  _yHeight = height();
}

/***************************************************************************************
** Function name:           getViewportX
** Description:             Get x position of the viewport datum
***************************************************************************************/
int32_t  TFT_eSPI_KB32::getViewportX(void)
{
  return _xDatum;
}

/***************************************************************************************
** Function name:           getViewportY
** Description:             Get y position of the viewport datum
***************************************************************************************/
int32_t  TFT_eSPI_KB32::getViewportY(void)
{
  return _yDatum;
}

/***************************************************************************************
** Function name:           getViewportWidth
** Description:             Get width of the viewport
***************************************************************************************/
int32_t TFT_eSPI_KB32::getViewportWidth(void)
{
  return _xWidth;
}

/***************************************************************************************
** Function name:           getViewportHeight
** Description:             Get height of the viewport
***************************************************************************************/
int32_t TFT_eSPI_KB32::getViewportHeight(void)
{
  return _yHeight;
}

/***************************************************************************************
** Function name:           getViewportDatum
** Description:             Get datum flag of the viewport (true = viewport corner)
***************************************************************************************/
bool  TFT_eSPI_KB32::getViewportDatum(void)
{
  return _vpDatum;
}

/***************************************************************************************
** Function name:           frameViewport
** Description:             Draw a frame inside or outside the viewport of width w
***************************************************************************************/
void TFT_eSPI_KB32::frameViewport(uint16_t color, int32_t w)
{
  // Save datum position
  bool _dT = _vpDatum;

  // If w is positive the frame is drawn inside the viewport
  // a large positive width will clear the screen inside the viewport
  if (w > 0)
  {
    // Set vpDatum true to simplify coordinate derivation
    _vpDatum = true;
    fillRect(0, 0, _vpW - _vpX, w, color);                // Top
    fillRect(0, w, w, _vpH - _vpY - w - w, color);        // Left
    fillRect(_xWidth - w, w, w, _yHeight - w - w, color); // Right
    fillRect(0, _yHeight - w, _xWidth, w, color);         // Bottom
  }
  else
    // If w is negative the frame is drawn outside the viewport
    // a large negative width will clear the screen outside the viewport
  {
    w = -w;

    // Save old values
    int32_t _xT = _vpX; _vpX = 0;
    int32_t _yT = _vpY; _vpY = 0;
    int32_t _wT = _vpW;
    int32_t _hT = _vpH;

    // Set vpDatum false so frame can be drawn outside window
    _vpDatum = false; // When false the full width and height is accessed
    _vpH = height();
    _vpW = width();

    // Draw frame
    fillRect(_xT - w - _xDatum, _yT - w - _yDatum, _wT - _xT + w + w, w, color); // Top
    fillRect(_xT - w - _xDatum, _yT - _yDatum, w, _hT - _yT, color);             // Left
    fillRect(_wT - _xDatum, _yT - _yDatum, w, _hT - _yT, color);                 // Right
    fillRect(_xT - w - _xDatum, _hT - _yDatum, _wT - _xT + w + w, w, color);     // Bottom

    // Restore old values
    _vpX = _xT;
    _vpY = _yT;
    _vpW = _wT;
    _vpH = _hT;
  }

  // Restore vpDatum
  _vpDatum = _dT;
}

/***************************************************************************************
** Function name:           end_tft_read (was called spi_end_read)
** Description:             End transaction for reads and deselect TFT
***************************************************************************************/
inline void TFT_eSPI_KB32::end_tft_read(void) {
#if defined (SPI_HAS_TRANSACTION) && defined (SUPPORT_TRANSACTIONS)
  if (!inTransaction) {
    if (!locked) {
      locked = true;
      CS_H;
      spi.endTransaction();
    }
  }
#else
  spi.setFrequency(SPI_FREQUENCY);
  if (!inTransaction) { CS_H; }
#endif
  SET_BUS_WRITE_MODE;
}

/***************************************************************************************
** Function name:           Legacy - deprecated
** Description:             Start/end transaction
***************************************************************************************/
void TFT_eSPI_KB32::spi_begin() { begin_tft_write(); }
void TFT_eSPI_KB32::spi_end() { end_tft_write(); }
void TFT_eSPI_KB32::spi_begin_read() { begin_tft_read(); }
void TFT_eSPI_KB32::spi_end_read() { end_tft_read(); }

/***************************************************************************************
** Function name:           TFT_eSPI
** Description:             Constructor , we must use hardware SPI pins
***************************************************************************************/
TFT_eSPI_KB32::TFT_eSPI_KB32(int16_t w, int16_t h)
{

  Wire.begin(21, 22, 400000);
  // The control pins are deliberately set to the inactive state (CS high) as setup()
  // might call and initialise other SPI peripherals which would could cause conflicts
  // if CS is floating or undefined.
  pinMode(TFT_CS, OUTPUT);
  digitalWrite(TFT_CS, HIGH); // Chip select high (inactive)


// Configure chip select for touchscreen controller if present

  pinMode(TFT_DC, OUTPUT);
  digitalWrite(TFT_DC, HIGH); // Data/Command high = data mode



  if (TFT_RST >= 0) {
    pinMode(TFT_RST, OUTPUT);
    digitalWrite(TFT_RST, HIGH); // Set high, do not share pin with another SPI device
  }



  _init_width = _width = w; // Set by specific xxxxx_Defines.h file or by users sketch
  _init_height = _height = h; // Set by specific xxxxx_Defines.h file or by users sketch

  // Reset the viewport to the whole screen
  resetViewport();

  rotation = 0;
  cursor_y = cursor_x = 0;
  textfont = 1;
  textsize = 1;
  textcolor = bitmap_fg = 0xFFFF; // White
  textbgcolor = bitmap_bg = 0x0000; // Black
  padX = 0;             // No padding
  isDigits = false;   // No bounding box adjustment
  textwrapX = true;    // Wrap text at end of line when using print stream
  textwrapY = false;   // Wrap text at bottom of screen when using print stream
  textdatum = TL_DATUM; // Top Left text alignment is default
  fontsloaded = 0;

  _swapBytes = false;   // Do not swap colour bytes by default

  locked = true;        // Transaction mutex lock flags
  inTransaction = false;

  _booted = true;     // Default attributes
  _cp437 = true;
  _utf8 = true;

#ifdef FONT_FS_AVAILABLE
  fs_font = true;     // Smooth font filing system or array (fs_font = false) flag
#endif

#if defined (ESP32) && defined (CONFIG_SPIRAM_SUPPORT)
  if (psramFound()) _psram_enable = true; // Enable the use of PSRAM (if available)
  else
#endif
    _psram_enable = false;

  addr_row = 0xFFFF;
  addr_col = 0xFFFF;

  _xPivot = 0;
  _yPivot = 0;

  cspinmask = 0;
  dcpinmask = 0;
  wrpinmask = 0;
  sclkpinmask = 0;

#ifdef LOAD_GLCD
  fontsloaded = 0x0002; // Bit 1 set
#endif

#ifdef LOAD_FONT2
  fontsloaded |= 0x0004; // Bit 2 set
#endif

#ifdef LOAD_FONT4
  fontsloaded |= 0x0010; // Bit 4 set
#endif

#ifdef LOAD_FONT6
  fontsloaded |= 0x0040; // Bit 6 set
#endif

#ifdef LOAD_FONT7
  fontsloaded |= 0x0080; // Bit 7 set
#endif

#ifdef LOAD_FONT8
  fontsloaded |= 0x0100; // Bit 8 set
#endif

#ifdef LOAD_FONT8N
  fontsloaded |= 0x0200; // Bit 9 set
#endif

#ifdef SMOOTH_FONT
  fontsloaded |= 0x8000; // Bit 15 set
#endif
}


/***************************************************************************************
** Function name:           begin
** Description:             Included for backwards compatibility
***************************************************************************************/
void TFT_eSPI_KB32::begin(uint8_t tc)
{
  init(tc);
}


/***************************************************************************************
** Function name:           init (tc is tab colour for ST7735 displays only)
** Description:             Reset, then initialise the TFT display registers
***************************************************************************************/
void TFT_eSPI_KB32::init(uint8_t tc)
{
  LCD_select();

  if (_booted)
  {
#if defined (TFT_CS) && (TFT_CS >= 0)
    cspinmask = (uint32_t)digitalPinToBitMask(TFT_CS);
#endif

#if defined (TFT_DC) && (TFT_DC >= 0)
    dcpinmask = (uint32_t)digitalPinToBitMask(TFT_DC);
#endif

#if defined (TFT_SCLK) && (TFT_SCLK >= 0)
    sclkpinmask = (uint32_t)digitalPinToBitMask(TFT_SCLK);
#endif

#if defined (TFT_SPI_OVERLAP) && defined (ESP8266)
    // Overlap mode SD0=MISO, SD1=MOSI, CLK=SCLK must use D3 as CS
    //    pins(int8_t sck, int8_t miso, int8_t mosi, int8_t ss);
    //spi.pins(        6,          7,           8,          0);
    spi.pins(6, 7, 8, 0);
#endif

    spi.begin(); // This will set HMISO to input
    inTransaction = false;
    locked = true;

    INIT_TFT_DATA_BUS;

    // Set to output once again in case D6 (MISO) is used for CS
    pinMode(TFT_CS, OUTPUT);
    digitalWrite(TFT_CS, HIGH); // Chip select high (inactive)




  // Set to output once again in case D6 (MISO) is used for DC

    pinMode(TFT_DC, OUTPUT);
    digitalWrite(TFT_DC, HIGH); // Data/Command high = data mode


    _booted = false;
    end_tft_write();
  } // end of: if just _booted

  // Toggle RST low to reset
  if (TFT_RST >= 0) {
    digitalWrite(TFT_RST, HIGH);
    delay(5);
    digitalWrite(TFT_RST, LOW);
    delay(20);
    digitalWrite(TFT_RST, HIGH);
  }
  delay(150); // Wait for reset to complete

  begin_tft_write();
  tc = tc; // Supress warning
  tabcolor = INITR_GREENTAB160x80;
#include "TFT_Drivers/ST7735_Init.h"
  end_tft_write();

  setRotation(1);
  invertDisplay(1);
}


/***************************************************************************************
** Function name:           setRotation
** Description:             rotate the screen orientation m = 0-3 or 4-7 for BMP drawing
***************************************************************************************/
void TFT_eSPI_KB32::setRotation(uint8_t m)
{

  begin_tft_write();

#include "TFT_Drivers/ST7735_Rotation.h"

  delayMicroseconds(10);

  end_tft_write();

  addr_row = 0xFFFF;
  addr_col = 0xFFFF;

  // Reset the viewport to the whole screen
  resetViewport();
}


/***************************************************************************************
** Function name:           commandList, used for FLASH based lists only (e.g. ST7735)
** Description:             Get initialisation commands from FLASH and send to TFT
***************************************************************************************/
void TFT_eSPI_KB32::commandList(const uint8_t* addr)
{
  uint8_t  numCommands;
  uint8_t  numArgs;
  uint8_t  ms;

  numCommands = pgm_read_byte(addr++);   // Number of commands to follow

  while (numCommands--)                  // For each command...
  {
    writecommand(pgm_read_byte(addr++)); // Read, issue command
    numArgs = pgm_read_byte(addr++);     // Number of args to follow
    ms = numArgs & TFT_INIT_DELAY;       // If hibit set, delay follows args
    numArgs &= ~TFT_INIT_DELAY;          // Mask out delay bit

    while (numArgs--)                    // For each argument...
    {
      writedata(pgm_read_byte(addr++));  // Read, issue argument
    }

    if (ms)
    {
      ms = pgm_read_byte(addr++);        // Read post-command delay time (ms)
      delay((ms == 255 ? 500 : ms));
    }
  }

}


/***************************************************************************************
** Function name:           spiwrite
** Description:             Write 8 bits to SPI port (legacy support only)
***************************************************************************************/
void TFT_eSPI_KB32::spiwrite(uint8_t c)
{
  begin_tft_write();
  tft_Write_8(c);
  end_tft_write();
}


/***************************************************************************************
** Function name:           writecommand
** Description:             Send an 8 bit command to the TFT
***************************************************************************************/
void TFT_eSPI_KB32::writecommand(uint8_t c)
{
  begin_tft_write();

  DC_C;

  tft_Write_8(c);

  DC_D;

  end_tft_write();

}


/***************************************************************************************
** Function name:           writedata
** Description:             Send a 8 bit data value to the TFT
***************************************************************************************/
void TFT_eSPI_KB32::writedata(uint8_t d)
{
  begin_tft_write();

  DC_D;        // Play safe, but should already be in data mode

  tft_Write_8(d);

  CS_L;        // Allow more hold time for low VDI rail

  end_tft_write();
}


/***************************************************************************************
** Function name:           readcommand8
** Description:             Read a 8 bit data value from an indexed command register
***************************************************************************************/
uint8_t TFT_eSPI_KB32::readcommand8(uint8_t cmd_function, uint8_t index)
{
  uint8_t reg = 0;
  // SPI interface
    // Tested with ILI9341 set to Interface II i.e. IM [3:0] = "1101"
  begin_tft_read();
  index = 0x10 + (index & 0x0F);

  DC_C; tft_Write_8(0xD9);
  DC_D; tft_Write_8(index);

  CS_H; // Some displays seem to need CS to be pulsed here, or is just a delay needed?
  CS_L;

  DC_C; tft_Write_8(cmd_function);
  DC_D;
  reg = tft_Read_8();

  end_tft_read();

  return reg;
}


/***************************************************************************************
** Function name:           readcommand16
** Description:             Read a 16 bit data value from an indexed command register
***************************************************************************************/
uint16_t TFT_eSPI_KB32::readcommand16(uint8_t cmd_function, uint8_t index)
{
  uint32_t reg;

  reg = (readcommand8(cmd_function, index + 0) << 8);
  reg |= (readcommand8(cmd_function, index + 1) << 0);

  return reg;
}


/***************************************************************************************
** Function name:           readcommand32
** Description:             Read a 32 bit data value from an indexed command register
***************************************************************************************/
uint32_t TFT_eSPI_KB32::readcommand32(uint8_t cmd_function, uint8_t index)
{
  uint32_t reg;

  reg = ((uint32_t)readcommand8(cmd_function, index + 0) << 24);
  reg |= ((uint32_t)readcommand8(cmd_function, index + 1) << 16);
  reg |= ((uint32_t)readcommand8(cmd_function, index + 2) << 8);
  reg |= ((uint32_t)readcommand8(cmd_function, index + 3) << 0);

  return reg;
}


/***************************************************************************************
** Function name:           read pixel (for SPI Interface II i.e. IM [3:0] = "1101")
** Description:             Read 565 pixel colours from a pixel
***************************************************************************************/
uint16_t TFT_eSPI_KB32::readPixel(int32_t x0, int32_t y0)
{
  if (_vpOoB) return 0;

  x0 += _xDatum;
  y0 += _yDatum;

  // Range checking
  if ((x0 < _vpX) || (y0 < _vpY) || (x0 >= _vpW) || (y0 >= _vpH)) return 0;

  // This function can get called during antialiased font rendering
  // so a transaction may be in progress
  bool wasInTransaction = inTransaction;
  if (inTransaction) { inTransaction = false; end_tft_write(); }

  uint16_t color = 0;

  begin_tft_read();

  readAddrWindow(x0, y0, 1, 1); // Sets CS low



  // Dummy read to throw away don't care value
  tft_Read_8();


  // Read the 3 RGB bytes, colour is actually only in the top 6 bits of each byte
  // as the TFT stores colours as 18 bits
  uint8_t r = tft_Read_8();
  uint8_t g = tft_Read_8();
  uint8_t b = tft_Read_8();
  color = color565(r, g, b);

  CS_H;

  end_tft_read();

  // Reinstate the transaction if one was in progress
  if (wasInTransaction) { begin_tft_write(); inTransaction = true; }

  return color;
}

void TFT_eSPI_KB32::setCallback(getColorCallback getCol)
{
  getColor = getCol;
}


/***************************************************************************************
** Function name:           read rectangle (for SPI Interface II i.e. IM [3:0] = "1101")
** Description:             Read 565 pixel colours from a defined area
***************************************************************************************/
void TFT_eSPI_KB32::readRect(int32_t x, int32_t y, int32_t w, int32_t h, uint16_t* data)
{
  PI_CLIP;

  uint16_t color = 0;

  begin_tft_read();

  readAddrWindow(x, y, dw, dh);

  data += dx + dy * w;

  // Dummy read to throw away don't care value
  tft_Read_8();

  // Read window pixel 24 bit RGB values
  while (dh--) {
    int32_t lw = dw;
    uint16_t* line = data;
    while (lw--) {


      // Read the 3 RGB bytes, colour is actually only in the top 6 bits of each byte
      // as the TFT stores colours as 18 bits
      uint8_t r = tft_Read_8();
      uint8_t g = tft_Read_8();
      uint8_t b = tft_Read_8();
      color = color565(r, g, b);

      // Swapped colour byte order for compatibility with pushRect()
      *line++ = color << 8 | color >> 8;
    }
    data += w;
  }
  //CS_H;
  end_tft_read();
}


/***************************************************************************************
** Function name:           push rectangle
** Description:             push 565 pixel colours into a defined area
***************************************************************************************/
void TFT_eSPI_KB32::pushRect(int32_t x, int32_t y, int32_t w, int32_t h, uint16_t* data)
{
  bool swap = _swapBytes; _swapBytes = false;
  pushImage(x, y, w, h, data);
  _swapBytes = swap;
}


/***************************************************************************************
** Function name:           pushImage
** Description:             plot 16 bit colour sprite or image onto TFT
***************************************************************************************/
void TFT_eSPI_KB32::pushImage(int32_t x, int32_t y, int32_t w, int32_t h, uint16_t* data)
{
  PI_CLIP;

  begin_tft_write();
  inTransaction = true;

  setWindow(x, y, x + dw - 1, y + dh - 1);

  data += dx + dy * w;

  // Check if whole image can be pushed
  if (dw == w) pushPixels(data, dw * dh);
  else {
    // Push line segments to crop image
    while (dh--)
    {
      pushPixels(data, dw);
      data += w;
    }
  }

  inTransaction = false;
  end_tft_write();
}

/***************************************************************************************
** Function name:           pushImage
** Description:             plot 16 bit sprite or image with 1 colour being transparent
***************************************************************************************/
void TFT_eSPI_KB32::pushImage(int32_t x, int32_t y, int32_t w, int32_t h, uint16_t* data, uint16_t transp)
{
  PI_CLIP;

  begin_tft_write();
  inTransaction = true;

  data += dx + dy * w;

  int32_t xe = x + dw - 1, ye = y + dh - 1;

  uint16_t  lineBuf[dw]; // Use buffer to minimise setWindow call count

  // The little endian transp color must be byte swapped if the image is big endian
  if (!_swapBytes) transp = transp >> 8 | transp << 8;

  while (dh--)
  {
    int32_t len = dw;
    uint16_t* ptr = data;
    int32_t px = x;
    bool move = true;
    uint16_t np = 0;

    while (len--)
    {
      if (transp != *ptr)
      {
        if (move) { move = false; setWindow(px, y, xe, ye); }
        lineBuf[np] = *ptr;
        np++;
      }
      else
      {
        move = true;
        if (np)
        {
          pushPixels((uint16_t*)lineBuf, np);
          np = 0;
        }
      }
      px++;
      ptr++;
    }
    if (np) pushPixels((uint16_t*)lineBuf, np);

    y++;
    data += w;
  }

  inTransaction = false;
  end_tft_write();
}


/***************************************************************************************
** Function name:           pushImage - for FLASH (PROGMEM) stored images
** Description:             plot 16 bit image
***************************************************************************************/
void TFT_eSPI_KB32::pushImage(int32_t x, int32_t y, int32_t w, int32_t h, const uint16_t* data)
{
  // Requires 32 bit aligned access, so use PROGMEM 16 bit word functions
  PI_CLIP;

  begin_tft_write();
  inTransaction = true;

  data += dx + dy * w;

  uint16_t  buffer[dw];

  setWindow(x, y, x + dw - 1, y + dh - 1);

  // Fill and send line buffers to TFT
  for (int32_t i = 0; i < dh; i++) {
    for (int32_t j = 0; j < dw; j++) {
      buffer[j] = pgm_read_word(&data[i * w + j]);
    }
    pushPixels(buffer, dw);
  }

  inTransaction = false;
  end_tft_write();
}

/***************************************************************************************
** Function name:           pushImage - for FLASH (PROGMEM) stored images
** Description:             plot 16 bit image with 1 colour being transparent
***************************************************************************************/
void TFT_eSPI_KB32::pushImage(int32_t x, int32_t y, int32_t w, int32_t h, const uint16_t* data, uint16_t transp)
{
  // Requires 32 bit aligned access, so use PROGMEM 16 bit word functions
  PI_CLIP;

  begin_tft_write();
  inTransaction = true;

  data += dx + dy * w;

  int32_t xe = x + dw - 1, ye = y + dh - 1;

  uint16_t  lineBuf[dw];

  // The little endian transp color must be byte swapped if the image is big endian
  if (!_swapBytes) transp = transp >> 8 | transp << 8;

  while (dh--) {
    int32_t len = dw;
    uint16_t* ptr = (uint16_t*)data;
    int32_t px = x;
    bool move = true;

    uint16_t np = 0;

    while (len--) {
      uint16_t color = pgm_read_word(ptr);
      if (transp != color) {
        if (move) { move = false; setWindow(px, y, xe, ye); }
        lineBuf[np] = color;
        np++;
      }
      else {
        move = true;
        if (np) {
          pushPixels(lineBuf, np);
          np = 0;
        }
      }
      px++;
      ptr++;
    }
    if (np) pushPixels(lineBuf, np);

    y++;
    data += w;
  }

  inTransaction = false;
  end_tft_write();
}


/***************************************************************************************
** Function name:           pushImage
** Description:             plot 8 bit or 4 bit or 1 bit image or sprite using a line buffer
***************************************************************************************/
void TFT_eSPI_KB32::pushImage(int32_t x, int32_t y, int32_t w, int32_t h, uint8_t* data, bool bpp8, uint16_t* cmap)
{
  PI_CLIP;

  begin_tft_write();
  inTransaction = true;
  bool swap = _swapBytes;

  setWindow(x, y, x + dw - 1, y + dh - 1); // Sets CS low and sent RAMWR

  // Line buffer makes plotting faster
  uint16_t  lineBuf[dw];

  if (bpp8)
  {
    _swapBytes = false;

    uint8_t  blue[] = { 0, 11, 21, 31 }; // blue 2 to 5 bit colour lookup table

    _lastColor = -1; // Set to illegal value

    // Used to store last shifted colour
    uint8_t msbColor = 0;
    uint8_t lsbColor = 0;

    data += dx + dy * w;
    while (dh--) {
      uint32_t len = dw;
      uint8_t* ptr = data;
      uint8_t* linePtr = (uint8_t*)lineBuf;

      while (len--) {
        uint32_t color = *ptr++;

        // Shifts are slow so check if colour has changed first
        if (color != _lastColor) {
          //          =====Green=====     ===============Red==============
          msbColor = (color & 0x1C) >> 2 | (color & 0xC0) >> 3 | (color & 0xE0);
          //          =====Green=====    =======Blue======
          lsbColor = (color & 0x1C) << 3 | blue[color & 0x03];
          _lastColor = color;
        }

        *linePtr++ = msbColor;
        *linePtr++ = lsbColor;
      }

      pushPixels(lineBuf, dw);

      data += w;
    }
    _swapBytes = swap; // Restore old value
  }
  else if (cmap != nullptr) // Must be 4bpp
  {
    _swapBytes = true;

    w = (w + 1) & 0xFFFE;   // if this is a sprite, w will already be even; this does no harm.
    bool splitFirst = (dx & 0x01) != 0; // split first means we have to push a single px from the left of the sprite / image

    if (splitFirst) {
      data += ((dx - 1 + dy * w) >> 1);
    }
    else {
      data += ((dx + dy * w) >> 1);
    }

    while (dh--) {
      uint32_t len = dw;
      uint8_t* ptr = data;
      uint16_t* linePtr = lineBuf;
      uint8_t colors; // two colors in one byte
      uint16_t index;

      if (splitFirst) {
        colors = *ptr;
        index = (colors & 0x0F);
        *linePtr++ = cmap[index];
        len--;
        ptr++;
      }

      while (len--)
      {
        colors = *ptr;
        index = ((colors & 0xF0) >> 4) & 0x0F;
        *linePtr++ = cmap[index];

        if (len--)
        {
          index = colors & 0x0F;
          *linePtr++ = cmap[index];
        }
        else {
          break;  // nothing to do here
        }

        ptr++;
      }

      pushPixels(lineBuf, dw);
      data += (w >> 1);
    }
    _swapBytes = swap; // Restore old value
  }
  else // Must be 1bpp
  {
    _swapBytes = false;

    uint32_t ww = (w + 7) >> 3; // Width of source image line in bytes
    for (int32_t yp = dy; yp < dy + dh; yp++)
    {
      uint8_t* linePtr = (uint8_t*)lineBuf;
      for (int32_t xp = dx; xp < dx + dw; xp++)
      {
        uint16_t col = (data[(xp >> 3)] & (0x80 >> (xp & 0x7)));
        if (col) { *linePtr++ = bitmap_fg >> 8; *linePtr++ = (uint8_t)bitmap_fg; }
        else { *linePtr++ = bitmap_bg >> 8; *linePtr++ = (uint8_t)bitmap_bg; }
      }
      data += ww;
      pushPixels(lineBuf, dw);
    }
  }

  _swapBytes = swap; // Restore old value
  inTransaction = false;
  end_tft_write();
}


/***************************************************************************************
** Function name:           pushImage
** Description:             plot 8 or 4 or 1 bit image or sprite with a transparent colour
***************************************************************************************/
void TFT_eSPI_KB32::pushImage(int32_t x, int32_t y, int32_t w, int32_t h, uint8_t* data, uint8_t transp, bool bpp8, uint16_t* cmap)
{
  PI_CLIP;

  begin_tft_write();
  inTransaction = true;
  bool swap = _swapBytes;

  int32_t xe = x + dw - 1, ye = y + dh - 1;

  // Line buffer makes plotting faster
  uint16_t  lineBuf[dw];

  if (bpp8) { // 8 bits per pixel
    _swapBytes = false;

    data += dx + dy * w;

    uint8_t  blue[] = { 0, 11, 21, 31 }; // blue 2 to 5 bit colour lookup table

    _lastColor = -1; // Set to illegal value

    // Used to store last shifted colour
    uint8_t msbColor = 0;
    uint8_t lsbColor = 0;

    while (dh--) {
      int32_t len = dw;
      uint8_t* ptr = data;
      uint8_t* linePtr = (uint8_t*)lineBuf;

      int32_t px = x;
      bool move = true;
      uint16_t np = 0;

      while (len--) {
        if (transp != *ptr) {
          if (move) { move = false; setWindow(px, y, xe, ye); }
          uint8_t color = *ptr;

          // Shifts are slow so check if colour has changed first
          if (color != _lastColor) {
            //          =====Green=====     ===============Red==============
            msbColor = (color & 0x1C) >> 2 | (color & 0xC0) >> 3 | (color & 0xE0);
            //          =====Green=====    =======Blue======
            lsbColor = (color & 0x1C) << 3 | blue[color & 0x03];
            _lastColor = color;
          }
          *linePtr++ = msbColor;
          *linePtr++ = lsbColor;
          np++;
        }
        else {
          move = true;
          if (np) {
            pushPixels(lineBuf, np);
            linePtr = (uint8_t*)lineBuf;
            np = 0;
          }
        }
        px++;
        ptr++;
      }

      if (np) pushPixels(lineBuf, np);
      y++;
      data += w;
    }
  }
  else if (cmap != nullptr) // 4bpp with color map
  {
    _swapBytes = true;

    w = (w + 1) & 0xFFFE; // here we try to recreate iwidth from dwidth.
    bool splitFirst = ((dx & 0x01) != 0);
    if (splitFirst) {
      data += ((dx - 1 + dy * w) >> 1);
    }
    else {
      data += ((dx + dy * w) >> 1);
    }

    while (dh--) {
      uint32_t len = dw;
      uint8_t* ptr = data;

      int32_t px = x;
      bool move = true;
      uint16_t np = 0;

      uint8_t index;  // index into cmap.

      if (splitFirst) {
        index = (*ptr & 0x0F);  // odd = bits 3 .. 0
        if (index != transp) {
          move = false; setWindow(px, y, xe, ye);
          lineBuf[np] = cmap[index];
          np++;
        }
        px++; ptr++;
        len--;
      }

      while (len--)
      {
        uint8_t color = *ptr;

        // find the actual color you care about.  There will be two pixels here!
        // but we may only want one at the end of the row
        uint16_t index = ((color & 0xF0) >> 4) & 0x0F;  // high bits are the even numbers
        if (index != transp) {
          if (move) {
            move = false; setWindow(px, y, xe, ye);
          }
          lineBuf[np] = cmap[index];
          np++; // added a pixel
        }
        else {
          move = true;
          if (np) {
            pushPixels(lineBuf, np);
            np = 0;
          }
        }
        px++;

        if (len--)
        {
          index = color & 0x0F; // the odd number is 3 .. 0
          if (index != transp) {
            if (move) {
              move = false; setWindow(px, y, xe, ye);
            }
            lineBuf[np] = cmap[index];
            np++;
          }
          else {
            move = true;
            if (np) {
              pushPixels(lineBuf, np);
              np = 0;
            }
          }
          px++;
        }
        else {
          break;  // we are done with this row.
        }
        ptr++;  // we only increment ptr once in the loop (deliberate)
      }

      if (np) {
        pushPixels(lineBuf, np);
        np = 0;
      }
      data += (w >> 1);
      y++;
    }
  }
  else { // 1 bit per pixel
    _swapBytes = false;

    uint32_t ww = (w + 7) >> 3; // Width of source image line in bytes
    uint16_t np = 0;

    for (int32_t yp = dy; yp < dy + dh; yp++)
    {
      int32_t px = x;
      bool move = true;
      for (int32_t xp = dx; xp < dx + dw; xp++)
      {
        if (data[(xp >> 3)] & (0x80 >> (xp & 0x7))) {
          if (move) {
            move = false;
            setWindow(px, y, xe, ye);
          }
          np++;
        }
        else {
          if (np) {
            pushBlock(bitmap_fg, np);
            np = 0;
            move = true;
          }
        }
        px++;
      }
      y++;
      data += ww;
      if (np) { pushBlock(bitmap_fg, np); np = 0; }
    }
  }
  _swapBytes = swap; // Restore old value
  inTransaction = false;
  end_tft_write();
}


/***************************************************************************************
** Function name:           setSwapBytes
** Description:             Used by 16 bit pushImage() to swap byte order in colours
***************************************************************************************/
void TFT_eSPI_KB32::setSwapBytes(bool swap)
{
  _swapBytes = swap;
}


/***************************************************************************************
** Function name:           getSwapBytes
** Description:             Return the swap byte order for colours
***************************************************************************************/
bool TFT_eSPI_KB32::getSwapBytes(void)
{
  return _swapBytes;
}


/***************************************************************************************
** Function name:           read rectangle (for SPI Interface II i.e. IM [3:0] = "1101")
** Description:             Read RGB pixel colours from a defined area
***************************************************************************************/
// If w and h are 1, then 1 pixel is read, *data array size must be 3 bytes per pixel
void  TFT_eSPI_KB32::readRectRGB(int32_t x0, int32_t y0, int32_t w, int32_t h, uint8_t* data)
{
  begin_tft_read();
  readAddrWindow(x0, y0, w, h); // Sets CS low
  // Dummy read to throw away don't care value
  tft_Read_8();
  // Read window pixel 24 bit RGB values, buffer must be set in sketch to 3 * w * h
  uint32_t len = w * h;
  while (len--) {
    // Read the 3 RGB bytes, colour is actually only in the top 6 bits of each byte
    // as the TFT stores colours as 18 bits
    *data++ = tft_Read_8();
    *data++ = tft_Read_8();
    *data++ = tft_Read_8();

  }
  CS_H;
  end_tft_read();
}


/***************************************************************************************
** Function name:           drawCircle
** Description:             Draw a circle outline
***************************************************************************************/
// Optimised midpoint circle algorithm
void TFT_eSPI_KB32::drawCircle(int32_t x0, int32_t y0, int32_t r, uint32_t color)
{
  int32_t  x = 1;
  int32_t  dx = 1;
  int32_t  dy = r + r;
  int32_t  p = -(r >> 1);

  //begin_tft_write();          // Sprite class can use this function, avoiding begin_tft_write()
  inTransaction = true;

  // These are ordered to minimise coordinate changes in x or y
  // drawPixel can then send fewer bounding box commands
  drawPixel(x0 + r, y0, color);
  drawPixel(x0 - r, y0, color);
  drawPixel(x0, y0 - r, color);
  drawPixel(x0, y0 + r, color);

  while (x < r) {

    if (p >= 0) {
      dy -= 2;
      p -= dy;
      r--;
    }

    dx += 2;
    p += dx;

    // These are ordered to minimise coordinate changes in x or y
    // drawPixel can then send fewer bounding box commands
    drawPixel(x0 + x, y0 + r, color);
    drawPixel(x0 - x, y0 + r, color);
    drawPixel(x0 - x, y0 - r, color);
    drawPixel(x0 + x, y0 - r, color);
    if (r != x) {
      drawPixel(x0 + r, y0 + x, color);
      drawPixel(x0 - r, y0 + x, color);
      drawPixel(x0 - r, y0 - x, color);
      drawPixel(x0 + r, y0 - x, color);
    }
    x++;
  }

  inTransaction = false;
  end_tft_write();              // Does nothing if Sprite class uses this function
}


/***************************************************************************************
** Function name:           drawCircleHelper
** Description:             Support function for drawRoundRect()
***************************************************************************************/
void TFT_eSPI_KB32::drawCircleHelper(int32_t x0, int32_t y0, int32_t r, uint8_t cornername, uint32_t color)
{
  int32_t f = 1 - r;
  int32_t ddF_x = 1;
  int32_t ddF_y = -2 * r;
  int32_t x = 0;

  while (x < r) {
    if (f >= 0) {
      r--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;
    if (cornername & 0x4) {
      drawPixel(x0 + x, y0 + r, color);
      drawPixel(x0 + r, y0 + x, color);
    }
    if (cornername & 0x2) {
      drawPixel(x0 + x, y0 - r, color);
      drawPixel(x0 + r, y0 - x, color);
    }
    if (cornername & 0x8) {
      drawPixel(x0 - r, y0 + x, color);
      drawPixel(x0 - x, y0 + r, color);
    }
    if (cornername & 0x1) {
      drawPixel(x0 - r, y0 - x, color);
      drawPixel(x0 - x, y0 - r, color);
    }
  }
}


/***************************************************************************************
** Function name:           fillCircle
** Description:             draw a filled circle
***************************************************************************************/
// Optimised midpoint circle algorithm, changed to horizontal lines (faster in sprites)
// Improved algorithm avoids repetition of lines
void TFT_eSPI_KB32::fillCircle(int32_t x0, int32_t y0, int32_t r, uint32_t color)
{
  int32_t  x = 0;
  int32_t  dx = 1;
  int32_t  dy = r + r;
  int32_t  p = -(r >> 1);

  //begin_tft_write();          // Sprite class can use this function, avoiding begin_tft_write()
  inTransaction = true;

  drawFastHLine(x0 - r, y0, dy + 1, color);

  while (x < r) {

    if (p >= 0) {
      drawFastHLine(x0 - x, y0 + r, dx, color);
      drawFastHLine(x0 - x, y0 - r, dx, color);
      dy -= 2;
      p -= dy;
      r--;
    }

    dx += 2;
    p += dx;
    x++;

    drawFastHLine(x0 - r, y0 + x, dy + 1, color);
    drawFastHLine(x0 - r, y0 - x, dy + 1, color);

  }

  inTransaction = false;
  end_tft_write();              // Does nothing if Sprite class uses this function
}

/***************************************************************************************
** Function name:           fillCircleHelper
** Description:             Support function for fillRoundRect()
***************************************************************************************/
// Support drawing roundrects, changed to horizontal lines (faster in sprites)
void TFT_eSPI_KB32::fillCircleHelper(int32_t x0, int32_t y0, int32_t r, uint8_t cornername, int32_t delta, uint32_t color)
{
  int32_t f = 1 - r;
  int32_t ddF_x = 1;
  int32_t ddF_y = -r - r;
  int32_t y = 0;

  delta++;

  while (y < r) {
    if (f >= 0) {
      if (cornername & 0x1) drawFastHLine(x0 - y, y0 + r, y + y + delta, color);
      if (cornername & 0x2) drawFastHLine(x0 - y, y0 - r, y + y + delta, color);
      r--;
      ddF_y += 2;
      f += ddF_y;
    }

    y++;
    ddF_x += 2;
    f += ddF_x;

    if (cornername & 0x1) drawFastHLine(x0 - r, y0 + y, r + r + delta, color);
    if (cornername & 0x2) drawFastHLine(x0 - r, y0 - y, r + r + delta, color);
  }
}


/***************************************************************************************
** Function name:           drawEllipse
** Description:             Draw a ellipse outline
***************************************************************************************/
void TFT_eSPI_KB32::drawEllipse(int16_t x0, int16_t y0, int32_t rx, int32_t ry, uint16_t color)
{
  if (rx < 2) return;
  if (ry < 2) return;
  int32_t x, y;
  int32_t rx2 = rx * rx;
  int32_t ry2 = ry * ry;
  int32_t fx2 = 4 * rx2;
  int32_t fy2 = 4 * ry2;
  int32_t s;

  //begin_tft_write();          // Sprite class can use this function, avoiding begin_tft_write()
  inTransaction = true;

  for (x = 0, y = ry, s = 2 * ry2 + rx2 * (1 - 2 * ry); ry2 * x <= rx2 * y; x++) {
    // These are ordered to minimise coordinate changes in x or y
    // drawPixel can then send fewer bounding box commands
    drawPixel(x0 + x, y0 + y, color);
    drawPixel(x0 - x, y0 + y, color);
    drawPixel(x0 - x, y0 - y, color);
    drawPixel(x0 + x, y0 - y, color);
    if (s >= 0) {
      s += fx2 * (1 - y);
      y--;
    }
    s += ry2 * ((4 * x) + 6);
  }

  for (x = rx, y = 0, s = 2 * rx2 + ry2 * (1 - 2 * rx); rx2 * y <= ry2 * x; y++) {
    // These are ordered to minimise coordinate changes in x or y
    // drawPixel can then send fewer bounding box commands
    drawPixel(x0 + x, y0 + y, color);
    drawPixel(x0 - x, y0 + y, color);
    drawPixel(x0 - x, y0 - y, color);
    drawPixel(x0 + x, y0 - y, color);
    if (s >= 0)
    {
      s += fy2 * (1 - x);
      x--;
    }
    s += rx2 * ((4 * y) + 6);
  }

  inTransaction = false;
  end_tft_write();              // Does nothing if Sprite class uses this function
}


/***************************************************************************************
** Function name:           fillEllipse
** Description:             draw a filled ellipse
***************************************************************************************/
void TFT_eSPI_KB32::fillEllipse(int16_t x0, int16_t y0, int32_t rx, int32_t ry, uint16_t color)
{
  if (rx < 2) return;
  if (ry < 2) return;
  int32_t x, y;
  int32_t rx2 = rx * rx;
  int32_t ry2 = ry * ry;
  int32_t fx2 = 4 * rx2;
  int32_t fy2 = 4 * ry2;
  int32_t s;

  //begin_tft_write();          // Sprite class can use this function, avoiding begin_tft_write()
  inTransaction = true;

  for (x = 0, y = ry, s = 2 * ry2 + rx2 * (1 - 2 * ry); ry2 * x <= rx2 * y; x++) {
    drawFastHLine(x0 - x, y0 - y, x + x + 1, color);
    drawFastHLine(x0 - x, y0 + y, x + x + 1, color);

    if (s >= 0) {
      s += fx2 * (1 - y);
      y--;
    }
    s += ry2 * ((4 * x) + 6);
  }

  for (x = rx, y = 0, s = 2 * rx2 + ry2 * (1 - 2 * rx); rx2 * y <= ry2 * x; y++) {
    drawFastHLine(x0 - x, y0 - y, x + x + 1, color);
    drawFastHLine(x0 - x, y0 + y, x + x + 1, color);

    if (s >= 0) {
      s += fy2 * (1 - x);
      x--;
    }
    s += rx2 * ((4 * y) + 6);
  }

  inTransaction = false;
  end_tft_write();              // Does nothing if Sprite class uses this function
}


/***************************************************************************************
** Function name:           fillScreen
** Description:             Clear the screen to defined colour
***************************************************************************************/
void TFT_eSPI_KB32::fillScreen(uint32_t color)
{
  fillRect(0, 0, _width, _height, color);
}


/***************************************************************************************
** Function name:           drawRect
** Description:             Draw a rectangle outline
***************************************************************************************/
// Draw a rectangle
void TFT_eSPI_KB32::drawRect(int32_t x, int32_t y, int32_t w, int32_t h, uint32_t color)
{
  //begin_tft_write();          // Sprite class can use this function, avoiding begin_tft_write()
  inTransaction = true;

  drawFastHLine(x, y, w, color);
  drawFastHLine(x, y + h - 1, w, color);
  // Avoid drawing corner pixels twice
  drawFastVLine(x, y + 1, h - 2, color);
  drawFastVLine(x + w - 1, y + 1, h - 2, color);

  inTransaction = false;
  end_tft_write();              // Does nothing if Sprite class uses this function
}


/***************************************************************************************
** Function name:           drawRoundRect
** Description:             Draw a rounded corner rectangle outline
***************************************************************************************/
// Draw a rounded rectangle
void TFT_eSPI_KB32::drawRoundRect(int32_t x, int32_t y, int32_t w, int32_t h, int32_t r, uint32_t color)
{
  //begin_tft_write();          // Sprite class can use this function, avoiding begin_tft_write()
  inTransaction = true;

  // smarter version
  drawFastHLine(x + r, y, w - r - r, color); // Top
  drawFastHLine(x + r, y + h - 1, w - r - r, color); // Bottom
  drawFastVLine(x, y + r, h - r - r, color); // Left
  drawFastVLine(x + w - 1, y + r, h - r - r, color); // Right
  // draw four corners
  drawCircleHelper(x + r, y + r, r, 1, color);
  drawCircleHelper(x + w - r - 1, y + r, r, 2, color);
  drawCircleHelper(x + w - r - 1, y + h - r - 1, r, 4, color);
  drawCircleHelper(x + r, y + h - r - 1, r, 8, color);

  inTransaction = false;
  end_tft_write();              // Does nothing if Sprite class uses this function
}


/***************************************************************************************
** Function name:           fillRoundRect
** Description:             Draw a rounded corner filled rectangle
***************************************************************************************/
// Fill a rounded rectangle, changed to horizontal lines (faster in sprites)
void TFT_eSPI_KB32::fillRoundRect(int32_t x, int32_t y, int32_t w, int32_t h, int32_t r, uint32_t color)
{
  //begin_tft_write();          // Sprite class can use this function, avoiding begin_tft_write()
  inTransaction = true;

  // smarter version
  fillRect(x, y + r, w, h - r - r, color);

  // draw four corners
  fillCircleHelper(x + r, y + h - r - 1, r, 1, w - r - r - 1, color);
  fillCircleHelper(x + r, y + r, r, 2, w - r - r - 1, color);

  inTransaction = false;
  end_tft_write();              // Does nothing if Sprite class uses this function
}


/***************************************************************************************
** Function name:           drawTriangle
** Description:             Draw a triangle outline using 3 arbitrary points
***************************************************************************************/
// Draw a triangle
void TFT_eSPI_KB32::drawTriangle(int32_t x0, int32_t y0, int32_t x1, int32_t y1, int32_t x2, int32_t y2, uint32_t color)
{
  //begin_tft_write();          // Sprite class can use this function, avoiding begin_tft_write()
  inTransaction = true;

  drawLine(x0, y0, x1, y1, color);
  drawLine(x1, y1, x2, y2, color);
  drawLine(x2, y2, x0, y0, color);

  inTransaction = false;
  end_tft_write();              // Does nothing if Sprite class uses this function
}


/***************************************************************************************
** Function name:           fillTriangle
** Description:             Draw a filled triangle using 3 arbitrary points
***************************************************************************************/
// Fill a triangle - original Adafruit function works well and code footprint is small
void TFT_eSPI_KB32::fillTriangle(int32_t x0, int32_t y0, int32_t x1, int32_t y1, int32_t x2, int32_t y2, uint32_t color)
{
  int32_t a, b, y, last;

  // Sort coordinates by Y order (y2 >= y1 >= y0)
  if (y0 > y1) {
    swap_coord(y0, y1); swap_coord(x0, x1);
  }
  if (y1 > y2) {
    swap_coord(y2, y1); swap_coord(x2, x1);
  }
  if (y0 > y1) {
    swap_coord(y0, y1); swap_coord(x0, x1);
  }

  if (y0 == y2) { // Handle awkward all-on-same-line case as its own thing
    a = b = x0;
    if (x1 < a)      a = x1;
    else if (x1 > b) b = x1;
    if (x2 < a)      a = x2;
    else if (x2 > b) b = x2;
    drawFastHLine(a, y0, b - a + 1, color);
    return;
  }

  //begin_tft_write();          // Sprite class can use this function, avoiding begin_tft_write()
  inTransaction = true;

  int32_t
    dx01 = x1 - x0,
    dy01 = y1 - y0,
    dx02 = x2 - x0,
    dy02 = y2 - y0,
    dx12 = x2 - x1,
    dy12 = y2 - y1,
    sa = 0,
    sb = 0;

  // For upper part of triangle, find scanline crossings for segments
  // 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
  // is included here (and second loop will be skipped, avoiding a /0
  // error there), otherwise scanline y1 is skipped here and handled
  // in the second loop...which also avoids a /0 error here if y0=y1
  // (flat-topped triangle).
  if (y1 == y2) last = y1;  // Include y1 scanline
  else         last = y1 - 1; // Skip it

  for (y = y0; y <= last; y++) {
    a = x0 + sa / dy01;
    b = x0 + sb / dy02;
    sa += dx01;
    sb += dx02;

    if (a > b) swap_coord(a, b);
    drawFastHLine(a, y, b - a + 1, color);
  }

  // For lower part of triangle, find scanline crossings for segments
  // 0-2 and 1-2.  This loop is skipped if y1=y2.
  sa = dx12 * (y - y1);
  sb = dx02 * (y - y0);
  for (; y <= y2; y++) {
    a = x1 + sa / dy12;
    b = x0 + sb / dy02;
    sa += dx12;
    sb += dx02;

    if (a > b) swap_coord(a, b);
    drawFastHLine(a, y, b - a + 1, color);
  }

  inTransaction = false;
  end_tft_write();              // Does nothing if Sprite class uses this function
}


/***************************************************************************************
** Function name:           drawBitmap
** Description:             Draw an image stored in an array on the TFT
***************************************************************************************/
void TFT_eSPI_KB32::drawBitmap(int16_t x, int16_t y, const uint8_t* bitmap, int16_t w, int16_t h, uint16_t color)
{
  //begin_tft_write();          // Sprite class can use this function, avoiding begin_tft_write()
  inTransaction = true;

  int32_t i, j, byteWidth = (w + 7) / 8;

  for (j = 0; j < h; j++) {
    for (i = 0; i < w; i++) {
      if (pgm_read_byte(bitmap + j * byteWidth + i / 8) & (128 >> (i & 7))) {
        drawPixel(x + i, y + j, color);
      }
    }
  }

  inTransaction = false;
  end_tft_write();              // Does nothing if Sprite class uses this function
}


/***************************************************************************************
** Function name:           drawBitmap
** Description:             Draw an image stored in an array on the TFT
***************************************************************************************/
void TFT_eSPI_KB32::drawBitmap(int16_t x, int16_t y, const uint8_t* bitmap, int16_t w, int16_t h, uint16_t fgcolor, uint16_t bgcolor)
{
  //begin_tft_write();          // Sprite class can use this function, avoiding begin_tft_write()
  inTransaction = true;

  int32_t i, j, byteWidth = (w + 7) / 8;

  for (j = 0; j < h; j++) {
    for (i = 0; i < w; i++) {
      if (pgm_read_byte(bitmap + j * byteWidth + i / 8) & (128 >> (i & 7)))
        drawPixel(x + i, y + j, fgcolor);
      else drawPixel(x + i, y + j, bgcolor);
    }
  }

  inTransaction = false;
  end_tft_write();              // Does nothing if Sprite class uses this function
}

/***************************************************************************************
** Function name:           drawXBitmap
** Description:             Draw an image stored in an XBM array onto the TFT
***************************************************************************************/
void TFT_eSPI_KB32::drawXBitmap(int16_t x, int16_t y, const uint8_t* bitmap, int16_t w, int16_t h, uint16_t color)
{
  //begin_tft_write();          // Sprite class can use this function, avoiding begin_tft_write()
  inTransaction = true;

  int32_t i, j, byteWidth = (w + 7) / 8;

  for (j = 0; j < h; j++) {
    for (i = 0; i < w; i++) {
      if (pgm_read_byte(bitmap + j * byteWidth + i / 8) & (1 << (i & 7))) {
        drawPixel(x + i, y + j, color);
      }
    }
  }

  inTransaction = false;
  end_tft_write();              // Does nothing if Sprite class uses this function
}


/***************************************************************************************
** Function name:           drawXBitmap
** Description:             Draw an XBM image with foreground and background colors
***************************************************************************************/
void TFT_eSPI_KB32::drawXBitmap(int16_t x, int16_t y, const uint8_t* bitmap, int16_t w, int16_t h, uint16_t color, uint16_t bgcolor)
{
  //begin_tft_write();          // Sprite class can use this function, avoiding begin_tft_write()
  inTransaction = true;

  int32_t i, j, byteWidth = (w + 7) / 8;

  for (j = 0; j < h; j++) {
    for (i = 0; i < w; i++) {
      if (pgm_read_byte(bitmap + j * byteWidth + i / 8) & (1 << (i & 7)))
        drawPixel(x + i, y + j, color);
      else drawPixel(x + i, y + j, bgcolor);
    }
  }

  inTransaction = false;
  end_tft_write();              // Does nothing if Sprite class uses this function
}


/***************************************************************************************
** Function name:           setCursor
** Description:             Set the text cursor x,y position
***************************************************************************************/
void TFT_eSPI_KB32::setCursor(int16_t x, int16_t y)
{
  cursor_x = x;
  cursor_y = y;
}


/***************************************************************************************
** Function name:           setCursor
** Description:             Set the text cursor x,y position and font
***************************************************************************************/
void TFT_eSPI_KB32::setCursor(int16_t x, int16_t y, uint8_t font)
{
  textfont = font;
  cursor_x = x;
  cursor_y = y;
}


/***************************************************************************************
** Function name:           getCursorX
** Description:             Get the text cursor x position
***************************************************************************************/
int16_t TFT_eSPI_KB32::getCursorX(void)
{
  return cursor_x;
}

/***************************************************************************************
** Function name:           getCursorY
** Description:             Get the text cursor y position
***************************************************************************************/
int16_t TFT_eSPI_KB32::getCursorY(void)
{
  return cursor_y;
}


/***************************************************************************************
** Function name:           setTextSize
** Description:             Set the text size multiplier
***************************************************************************************/
void TFT_eSPI_KB32::setTextSize(uint8_t s)
{
  if (s > 7) s = 7; // Limit the maximum size multiplier so byte variables can be used for rendering
  textsize = (s > 0) ? s : 1; // Don't allow font size 0
}


/***************************************************************************************
** Function name:           setTextColor
** Description:             Set the font foreground colour (background is transparent)
***************************************************************************************/
void TFT_eSPI_KB32::setTextColor(uint16_t c)
{
  // For 'transparent' background, we'll set the bg
  // to the same as fg instead of using a flag
  textcolor = textbgcolor = c;
}


/***************************************************************************************
** Function name:           setTextColor
** Description:             Set the font foreground and background colour
***************************************************************************************/
void TFT_eSPI_KB32::setTextColor(uint16_t c, uint16_t b)
{
  textcolor = c;
  textbgcolor = b;
}


/***************************************************************************************
** Function name:           setPivot
** Description:             Set the pivot point on the TFT
*************************************************************************************x*/
void TFT_eSPI_KB32::setPivot(int16_t x, int16_t y)
{
  _xPivot = x;
  _yPivot = y;
}


/***************************************************************************************
** Function name:           getPivotX
** Description:             Get the x pivot position
***************************************************************************************/
int16_t TFT_eSPI_KB32::getPivotX(void)
{
  return _xPivot;
}


/***************************************************************************************
** Function name:           getPivotY
** Description:             Get the y pivot position
***************************************************************************************/
int16_t TFT_eSPI_KB32::getPivotY(void)
{
  return _yPivot;
}


/***************************************************************************************
** Function name:           setBitmapColor
** Description:             Set the foreground foreground and background colour
***************************************************************************************/
void TFT_eSPI_KB32::setBitmapColor(uint16_t c, uint16_t b)
{
  if (c == b) b = ~c;
  bitmap_fg = c;
  bitmap_bg = b;
}


/***************************************************************************************
** Function name:           setTextWrap
** Description:             Define if text should wrap at end of line
***************************************************************************************/
void TFT_eSPI_KB32::setTextWrap(bool wrapX, bool wrapY)
{
  textwrapX = wrapX;
  textwrapY = wrapY;
}


/***************************************************************************************
** Function name:           setTextDatum
** Description:             Set the text position reference datum
***************************************************************************************/
void TFT_eSPI_KB32::setTextDatum(uint8_t d)
{
  textdatum = d;
}


/***************************************************************************************
** Function name:           setTextPadding
** Description:             Define padding width (aids erasing old text and numbers)
***************************************************************************************/
void TFT_eSPI_KB32::setTextPadding(uint16_t x_width)
{
  padX = x_width;
}

/***************************************************************************************
** Function name:           setTextPadding
** Description:             Define padding width (aids erasing old text and numbers)
***************************************************************************************/
uint16_t TFT_eSPI_KB32::getTextPadding(void)
{
  return padX;
}

/***************************************************************************************
** Function name:           getRotation
** Description:             Return the rotation value (as used by setRotation())
***************************************************************************************/
uint8_t TFT_eSPI_KB32::getRotation(void)
{
  return rotation;
}

/***************************************************************************************
** Function name:           getTextDatum
** Description:             Return the text datum value (as used by setTextDatum())
***************************************************************************************/
uint8_t TFT_eSPI_KB32::getTextDatum(void)
{
  return textdatum;
}


/***************************************************************************************
** Function name:           width
** Description:             Return the pixel width of display (per current rotation)
***************************************************************************************/
// Return the size of the display (per current rotation)
int16_t TFT_eSPI_KB32::width(void)
{
  if (_vpDatum) return _xWidth;
  return _width;
}


/***************************************************************************************
** Function name:           height
** Description:             Return the pixel height of display (per current rotation)
***************************************************************************************/
int16_t TFT_eSPI_KB32::height(void)
{
  if (_vpDatum) return _yHeight;
  return _height;
}


/***************************************************************************************
** Function name:           textWidth
** Description:             Return the width in pixels of a string in a given font
***************************************************************************************/
int16_t TFT_eSPI_KB32::textWidth(const String& string)
{
  int16_t len = string.length() + 2;
  char buffer[len];
  string.toCharArray(buffer, len);
  return textWidth(buffer, textfont);
}

int16_t TFT_eSPI_KB32::textWidth(const String& string, uint8_t font)
{
  int16_t len = string.length() + 2;
  char buffer[len];
  string.toCharArray(buffer, len);
  return textWidth(buffer, font);
}

int16_t TFT_eSPI_KB32::textWidth(const char* string)
{
  return textWidth(string, textfont);
}

int16_t TFT_eSPI_KB32::textWidth(const char* string, uint8_t font)
{
  int32_t str_width = 0;
  uint16_t uniCode = 0;

#ifdef SMOOTH_FONT
  if (fontLoaded) {
    while (*string) {
      uniCode = decodeUTF8(*string++);
      if (uniCode) {
        if (uniCode == 0x20) str_width += gFont.spaceWidth;
        else {
          uint16_t gNum = 0;
          bool found = getUnicodeIndex(uniCode, &gNum);
          if (found) {
            if (str_width == 0 && gdX[gNum] < 0) str_width -= gdX[gNum];
            if (*string || isDigits) str_width += gxAdvance[gNum];
            else str_width += (gdX[gNum] + gWidth[gNum]);
          }
          else str_width += gFont.spaceWidth + 1;
        }
      }
    }
    isDigits = false;
    return str_width;
  }
#endif

  if (font > 1 && font < 9) {
    char* widthtable = (char*)pgm_read_dword(&(fontdata[font].widthtbl)) - 32; //subtract the 32 outside the loop

    while (*string) {
      uniCode = *(string++);
      if (uniCode > 31 && uniCode < 128)
        str_width += pgm_read_byte(widthtable + uniCode); // Normally we need to subtract 32 from uniCode
      else str_width += pgm_read_byte(widthtable + 32); // Set illegal character = space width
    }

  }
  else {

#ifdef LOAD_GFXFF
    if (gfxFont) { // New font
      while (*string) {
        uniCode = decodeUTF8(*string++);
        if ((uniCode >= pgm_read_word(&gfxFont->first)) && (uniCode <= pgm_read_word(&gfxFont->last))) {
          uniCode -= pgm_read_word(&gfxFont->first);
          GFXglyph* glyph = &(((GFXglyph*)pgm_read_dword(&gfxFont->glyph))[uniCode]);
          // If this is not the  last character or is a digit then use xAdvance
          if (*string || isDigits) str_width += pgm_read_byte(&glyph->xAdvance);
          // Else use the offset plus width since this can be bigger than xAdvance
          else str_width += ((int8_t)pgm_read_byte(&glyph->xOffset) + pgm_read_byte(&glyph->width));
        }
      }
    }
    else
#endif
    {
#ifdef LOAD_GLCD
      while (*string++) str_width += 6;
#endif
    }
  }
  isDigits = false;
  return str_width * textsize;
}


/***************************************************************************************
** Function name:           fontsLoaded
** Description:             return an encoded 16 bit value showing the fonts loaded
***************************************************************************************/
// Returns a value showing which fonts are loaded (bit N set =  Font N loaded)

uint16_t TFT_eSPI_KB32::fontsLoaded(void)
{
  return fontsloaded;
}


/***************************************************************************************
** Function name:           fontHeight
** Description:             return the height of a font (yAdvance for free fonts)
***************************************************************************************/
int16_t TFT_eSPI_KB32::fontHeight(int16_t font)
{
#ifdef SMOOTH_FONT
  if (fontLoaded) return gFont.yAdvance;
#endif

#ifdef LOAD_GFXFF
  if (font == 1) {
    if (gfxFont) { // New font
      return pgm_read_byte(&gfxFont->yAdvance) * textsize;
    }
  }
#endif
  return pgm_read_byte(&fontdata[font].height) * textsize;
}

int16_t TFT_eSPI_KB32::fontHeight(void)
{
  return fontHeight(textfont);
}

/***************************************************************************************
** Function name:           drawChar
** Description:             draw a single character in the GLCD or GFXFF font
***************************************************************************************/
void TFT_eSPI_KB32::drawChar(int32_t x, int32_t y, uint16_t c, uint32_t color, uint32_t bg, uint8_t size)
{
  if (_vpOoB) return;

  int32_t xd = x + _xDatum;
  int32_t yd = y + _yDatum;

  if (c < 32) return;
#ifdef LOAD_GLCD
  //>>>>>>>>>>>>>>>>>>
#ifdef LOAD_GFXFF
  if (!gfxFont) { // 'Classic' built-in font
#endif
//>>>>>>>>>>>>>>>>>>

    if ((xd >= _vpW) || // Clip right
      (yd >= _vpH) || // Clip bottom
      ((xd + 6 * size - 1) < _vpX) || // Clip left
      ((yd + 8 * size - 1) < _vpY))    // Clip top
      return;

    bool fillbg = (bg != color);
    bool clip = xd < _vpX || xd + 6 * textsize >= _vpW || yd < _vpY || yd + 8 * textsize >= _vpH;

    if ((size == 1) && fillbg && !clip) {
      uint8_t column[6];
      uint8_t mask = 0x1;
      begin_tft_write();

      setWindow(xd, yd, xd + 5, yd + 8);

      for (int8_t i = 0; i < 5; i++) column[i] = pgm_read_byte(font + (c * 5) + i);
      column[5] = 0;

      for (int8_t j = 0; j < 8; j++) {
        for (int8_t k = 0; k < 5; k++) {
          if (column[k] & mask) { tft_Write_16(color); }
          else { tft_Write_16(bg); }
        }
        mask <<= 1;
        tft_Write_16(bg);
      }

      end_tft_write();
    }
    else {
      //begin_tft_write();          // Sprite class can use this function, avoiding begin_tft_write()
      inTransaction = true;

      for (int8_t i = 0; i < 6; i++) {
        uint8_t line;
        if (i == 5)
          line = 0x0;
        else
          line = pgm_read_byte(font + (c * 5) + i);

        if (size == 1 && !fillbg) { // default size
          for (int8_t j = 0; j < 8; j++) {
            if (line & 0x1) drawPixel(x + i, y + j, color);
            line >>= 1;
          }
        }
        else {  // big size or clipped
          for (int8_t j = 0; j < 8; j++) {
            if (line & 0x1) fillRect(x + (i * size), y + (j * size), size, size, color);
            else if (fillbg) fillRect(x + i * size, y + j * size, size, size, bg);
            line >>= 1;
          }
        }
      }
      inTransaction = false;
      end_tft_write();              // Does nothing if Sprite class uses this function
    }

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>
#ifdef LOAD_GFXFF
  }
  else { // Custom font
#endif
//>>>>>>>>>>>>>>>>>>>>>>>>>>>
#endif // LOAD_GLCD

#ifdef LOAD_GFXFF
    // Filter out bad characters not present in font
    if ((c >= pgm_read_word(&gfxFont->first)) && (c <= pgm_read_word(&gfxFont->last))) {
      //begin_tft_write();          // Sprite class can use this function, avoiding begin_tft_write()
      inTransaction = true;
      //>>>>>>>>>>>>>>>>>>>>>>>>>>>

      c -= pgm_read_word(&gfxFont->first);
      GFXglyph* glyph = &(((GFXglyph*)pgm_read_dword(&gfxFont->glyph))[c]);
      uint8_t* bitmap = (uint8_t*)pgm_read_dword(&gfxFont->bitmap);

      uint32_t bo = pgm_read_word(&glyph->bitmapOffset);
      uint8_t  w = pgm_read_byte(&glyph->width),
        h = pgm_read_byte(&glyph->height);
      //xa = pgm_read_byte(&glyph->xAdvance);
      int8_t   xo = pgm_read_byte(&glyph->xOffset),
        yo = pgm_read_byte(&glyph->yOffset);
      uint8_t  xx, yy, bits = 0, bit = 0;
      int16_t  xo16 = 0, yo16 = 0;

      if (size > 1) {
        xo16 = xo;
        yo16 = yo;
      }

      // GFXFF rendering speed up
      uint16_t hpc = 0; // Horizontal foreground pixel count
      for (yy = 0; yy < h; yy++) {
        for (xx = 0; xx < w; xx++) {
          if (bit == 0) {
            bits = pgm_read_byte(&bitmap[bo++]);
            bit = 0x80;
          }
          if (bits & bit) hpc++;
          else {
            if (hpc) {
              if (size == 1) drawFastHLine(x + xo + xx - hpc, y + yo + yy, hpc, color);
              else fillRect(x + (xo16 + xx - hpc) * size, y + (yo16 + yy) * size, size * hpc, size, color);
              hpc = 0;
            }
          }
          bit >>= 1;
        }
        // Draw pixels for this line as we are about to increment yy
        if (hpc) {
          if (size == 1) drawFastHLine(x + xo + xx - hpc, y + yo + yy, hpc, color);
          else fillRect(x + (xo16 + xx - hpc) * size, y + (yo16 + yy) * size, size * hpc, size, color);
          hpc = 0;
        }
      }

      inTransaction = false;
      end_tft_write();              // Does nothing if Sprite class uses this function
    }
#endif

#ifdef LOAD_GLCD
#ifdef LOAD_GFXFF
  } // End classic vs custom font
#endif
#endif

}


/***************************************************************************************
** Function name:           setAddrWindow
** Description:             define an area to receive a stream of pixels
***************************************************************************************/
// Chip select is high at the end of this function
void TFT_eSPI_KB32::setAddrWindow(int32_t x0, int32_t y0, int32_t w, int32_t h)
{
  begin_tft_write();

  setWindow(x0, y0, x0 + w - 1, y0 + h - 1);

  end_tft_write();
}


/***************************************************************************************
** Function name:           setWindow
** Description:             define an area to receive a stream of pixels
***************************************************************************************/
// Chip select stays low, call begin_tft_write first. Use setAddrWindow() from sketches
void TFT_eSPI_KB32::setWindow(int32_t x0, int32_t y0, int32_t x1, int32_t y1)
{
  addr_row = 0xFFFF;
  addr_col = 0xFFFF;

#ifdef CGRAM_OFFSET
  x0 += colstart;
  x1 += colstart;
  y0 += rowstart;
  y1 += rowstart;
#endif

  DC_C; tft_Write_8(TFT_CASET);
  DC_D; tft_Write_32C(x0, x1);
  DC_C; tft_Write_8(TFT_PASET);
  DC_D; tft_Write_32C(y0, y1);
  DC_C; tft_Write_8(TFT_RAMWR);
  DC_D;

  //end_tft_write(); // Must be called after setWindow
}


/***************************************************************************************
** Function name:           readAddrWindow
** Description:             define an area to read a stream of pixels
***************************************************************************************/
void TFT_eSPI_KB32::readAddrWindow(int32_t xs, int32_t ys, int32_t w, int32_t h)
{
  //begin_tft_write(); // Must be called before readAddrWindow or CS set low

  int32_t xe = xs + w - 1;
  int32_t ye = ys + h - 1;

  addr_col = 0xFFFF;
  addr_row = 0xFFFF;

#ifdef CGRAM_OFFSET
  xs += colstart;
  xe += colstart;
  ys += rowstart;
  ye += rowstart;
#endif

#if defined (SSD1963_DRIVER)
  if ((rotation & 0x1) == 0) { swap_coord(xs, ys); swap_coord(xe, ye); }
#endif

  // Column addr set
  DC_C; tft_Write_8(TFT_CASET);
  DC_D; tft_Write_32C(xs, xe);

  // Row addr set
  DC_C; tft_Write_8(TFT_PASET);
  DC_D; tft_Write_32C(ys, ye);

  // Read CGRAM command
  DC_C; tft_Write_8(TFT_RAMRD);

  DC_D;

  //end_tft_write(); // Must be called after readAddrWindow or CS set high
}


/***************************************************************************************
** Function name:           drawPixel
** Description:             push a single pixel at an arbitrary position
***************************************************************************************/
void TFT_eSPI_KB32::drawPixel(int32_t x, int32_t y, uint32_t color)
{
  if (_vpOoB) return;

  x += _xDatum;
  y += _yDatum;

  // Range checking 
  if ((x < _vpX) || (y < _vpY) || (x >= _vpW) || (y >= _vpH)) return;

#ifdef CGRAM_OFFSET
  x += colstart;
  y += rowstart;
#endif

  begin_tft_write();

  // No need to send x if it has not changed (speeds things up)
  if (addr_col != (x << 16 | x)) {
    DC_C; tft_Write_8(TFT_CASET);
    DC_D; tft_Write_32D(x);
    addr_col = (x << 16 | x);
  }

  // No need to send y if it has not changed (speeds things up)
  if (addr_row != (y << 16 | y)) {
    DC_C; tft_Write_8(TFT_PASET);
    DC_D; tft_Write_32D(y);
    addr_row = (y << 16 | y);
  }

  DC_C; tft_Write_8(TFT_RAMWR);
  DC_D; tft_Write_16(color);

  end_tft_write();
}

/***************************************************************************************
** Function name:           pushColor
** Description:             push a single pixel
***************************************************************************************/
void TFT_eSPI_KB32::pushColor(uint16_t color)
{
  begin_tft_write();

  tft_Write_16(color);

  end_tft_write();
}


/***************************************************************************************
** Function name:           pushColor
** Description:             push a single colour to "len" pixels
***************************************************************************************/
void TFT_eSPI_KB32::pushColor(uint16_t color, uint32_t len)
{
  begin_tft_write();

  pushBlock(color, len);

  end_tft_write();
}

/***************************************************************************************
** Function name:           startWrite
** Description:             begin transaction with CS low, MUST later call endWrite
***************************************************************************************/
void TFT_eSPI_KB32::startWrite(void)
{
  begin_tft_write();
  inTransaction = true;
}

/***************************************************************************************
** Function name:           endWrite
** Description:             end transaction with CS high
***************************************************************************************/
void TFT_eSPI_KB32::endWrite(void)
{
  inTransaction = false;
  DMA_BUSY_CHECK;         // Safety check - user code should have checked this!
  end_tft_write();
}

/***************************************************************************************
** Function name:           writeColor (use startWrite() and endWrite() before & after)
** Description:             raw write of "len" pixels avoiding transaction check
***************************************************************************************/
void TFT_eSPI_KB32::writeColor(uint16_t color, uint32_t len)
{
  pushBlock(color, len);
}

/***************************************************************************************
** Function name:           pushColors
** Description:             push an array of pixels for 16 bit raw image drawing
***************************************************************************************/
// Assumed that setAddrWindow() has previously been called
// len is number of bytes, not pixels
void TFT_eSPI_KB32::pushColors(uint8_t* data, uint32_t len)
{
  begin_tft_write();

  pushPixels(data, len >> 1);

  end_tft_write();
}


/***************************************************************************************
** Function name:           pushColors
** Description:             push an array of pixels, for image drawing
***************************************************************************************/
void TFT_eSPI_KB32::pushColors(uint16_t* data, uint32_t len, bool swap)
{
  begin_tft_write();
  if (swap) { swap = _swapBytes; _swapBytes = true; }

  pushPixels(data, len);

  _swapBytes = swap; // Restore old value
  end_tft_write();
}


/***************************************************************************************
** Function name:           drawLine
** Description:             draw a line between 2 arbitrary points
***************************************************************************************/
// Bresenham's algorithm - thx wikipedia - speed enhanced by Bodmer to use
// an efficient FastH/V Line draw routine for line segments of 2 pixels or more
void TFT_eSPI_KB32::drawLine(int32_t x0, int32_t y0, int32_t x1, int32_t y1, uint32_t color)
{
  if (_vpOoB) return;

  //begin_tft_write();       // Sprite class can use this function, avoiding begin_tft_write()
  inTransaction = true;

  //x+= _xDatum;             // Not added here, added by drawPixel & drawFastXLine
  //y+= _yDatum;

  bool steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    swap_coord(x0, y0);
    swap_coord(x1, y1);
  }

  if (x0 > x1) {
    swap_coord(x0, x1);
    swap_coord(y0, y1);
  }

  int32_t dx = x1 - x0, dy = abs(y1 - y0);;

  int32_t err = dx >> 1, ystep = -1, xs = x0, dlen = 0;

  if (y0 < y1) ystep = 1;

  // Split into steep and not steep for FastH/V separation
  if (steep) {
    for (; x0 <= x1; x0++) {
      dlen++;
      err -= dy;
      if (err < 0) {
        err += dx;
        if (dlen == 1) drawPixel(y0, xs, color);
        else drawFastVLine(y0, xs, dlen, color);
        dlen = 0;
        y0 += ystep; xs = x0 + 1;
      }
    }
    if (dlen) drawFastVLine(y0, xs, dlen, color);
  }
  else
  {
    for (; x0 <= x1; x0++) {
      dlen++;
      err -= dy;
      if (err < 0) {
        err += dx;
        if (dlen == 1) drawPixel(xs, y0, color);
        else drawFastHLine(xs, y0, dlen, color);
        dlen = 0;
        y0 += ystep; xs = x0 + 1;
      }
    }
    if (dlen) drawFastHLine(xs, y0, dlen, color);
  }

  inTransaction = false;
  end_tft_write();
}


/***************************************************************************************
** Function name:           drawFastVLine
** Description:             draw a vertical line
***************************************************************************************/
void TFT_eSPI_KB32::drawFastVLine(int32_t x, int32_t y, int32_t h, uint32_t color)
{
  if (_vpOoB) return;

  x += _xDatum;
  y += _yDatum;

  // Clipping
  if ((x < _vpX) || (x >= _vpW) || (y >= _vpH)) return;

  if (y < _vpY) { h += y - _vpY; y = _vpY; }

  if ((y + h) > _vpH) h = _vpH - y;

  if (h < 1) return;

  begin_tft_write();

  setWindow(x, y, x, y + h - 1);

  pushBlock(color, h);

  end_tft_write();
}


/***************************************************************************************
** Function name:           drawFastHLine
** Description:             draw a horizontal line
***************************************************************************************/
void TFT_eSPI_KB32::drawFastHLine(int32_t x, int32_t y, int32_t w, uint32_t color)
{
  if (_vpOoB) return;

  x += _xDatum;
  y += _yDatum;

  // Clipping
  if ((y < _vpY) || (x >= _vpW) || (y >= _vpH)) return;

  if (x < _vpX) { w += x - _vpX; x = _vpX; }

  if ((x + w) > _vpW) w = _vpW - x;

  if (w < 1) return;

  begin_tft_write();

  setWindow(x, y, x + w - 1, y);

  pushBlock(color, w);

  end_tft_write();
}


/***************************************************************************************
** Function name:           fillRect
** Description:             draw a filled rectangle
***************************************************************************************/
void TFT_eSPI_KB32::fillRect(int32_t x, int32_t y, int32_t w, int32_t h, uint32_t color)
{
  if (_vpOoB) return;

  x += _xDatum;
  y += _yDatum;

  // Clipping
  if ((x >= _vpW) || (y >= _vpH)) return;

  if (x < _vpX) { w += x - _vpX; x = _vpX; }
  if (y < _vpY) { h += y - _vpY; y = _vpY; }

  if ((x + w) > _vpW) w = _vpW - x;
  if ((y + h) > _vpH) h = _vpH - y;

  if ((w < 1) || (h < 1)) return;

  //Serial.print(" _xDatum=");Serial.print( _xDatum);Serial.print(", _yDatum=");Serial.print( _yDatum);
  //Serial.print(", _xWidth=");Serial.print(_xWidth);Serial.print(", _yHeight=");Serial.println(_yHeight);

  //Serial.print(" _vpX=");Serial.print( _vpX);Serial.print(", _vpY=");Serial.print( _vpY);
  //Serial.print(", _vpW=");Serial.print(_vpW);Serial.print(", _vpH=");Serial.println(_vpH);

  //Serial.print(" x=");Serial.print( y);Serial.print(", y=");Serial.print( y);
  //Serial.print(", w=");Serial.print(w);Serial.print(", h=");Serial.println(h);

  begin_tft_write();

  setWindow(x, y, x + w - 1, y + h - 1);

  pushBlock(color, w * h);

  end_tft_write();
}


/***************************************************************************************
** Function name:           color565
** Description:             convert three 8 bit RGB levels to a 16 bit colour value
***************************************************************************************/
uint16_t TFT_eSPI_KB32::color565(uint8_t r, uint8_t g, uint8_t b)
{
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}


/***************************************************************************************
** Function name:           color16to8
** Description:             convert 16 bit colour to an 8 bit 332 RGB colour value
***************************************************************************************/
uint8_t TFT_eSPI_KB32::color16to8(uint16_t c)
{
  return ((c & 0xE000) >> 8) | ((c & 0x0700) >> 6) | ((c & 0x0018) >> 3);
}


/***************************************************************************************
** Function name:           color8to16
** Description:             convert 8 bit colour to a 16 bit 565 colour value
***************************************************************************************/
uint16_t TFT_eSPI_KB32::color8to16(uint8_t color)
{
  uint8_t  blue[] = { 0, 11, 21, 31 }; // blue 2 to 5 bit colour lookup table
  uint16_t color16 = 0;

  //        =====Green=====     ===============Red==============
  color16 = (color & 0x1C) << 6 | (color & 0xC0) << 5 | (color & 0xE0) << 8;
  //        =====Green=====    =======Blue======
  color16 |= (color & 0x1C) << 3 | blue[color & 0x03];

  return color16;
}

/***************************************************************************************
** Function name:           color16to24
** Description:             convert 16 bit colour to a 24 bit 888 colour value
***************************************************************************************/
uint32_t TFT_eSPI_KB32::color16to24(uint16_t color565)
{
  uint8_t r = (color565 >> 8) & 0xF8; r |= (r >> 5);
  uint8_t g = (color565 >> 3) & 0xFC; g |= (g >> 6);
  uint8_t b = (color565 << 3) & 0xF8; b |= (b >> 5);

  return ((uint32_t)r << 16) | ((uint32_t)g << 8) | ((uint32_t)b << 0);
}

/***************************************************************************************
** Function name:           color24to16
** Description:             convert 24 bit colour to a 16 bit 565 colour value
***************************************************************************************/
uint32_t TFT_eSPI_KB32::color24to16(uint32_t color888)
{
  uint16_t r = (color888 >> 8) & 0xF800;
  uint16_t g = (color888 >> 5) & 0x07E0;
  uint16_t b = (color888 >> 3) & 0x001F;

  return (r | g | b);
}

/***************************************************************************************
** Function name:           invertDisplay
** Description:             invert the display colours i = 1 invert, i = 0 normal
***************************************************************************************/
void TFT_eSPI_KB32::invertDisplay(bool i)
{
  begin_tft_write();
  // Send the command twice as otherwise it does not always work!
  writecommand(i ? TFT_INVON : TFT_INVOFF);
  writecommand(i ? TFT_INVON : TFT_INVOFF);
  end_tft_write();
}


/**************************************************************************
** Function name:           setAttribute
** Description:             Sets a control parameter of an attribute
**************************************************************************/
void TFT_eSPI_KB32::setAttribute(uint8_t attr_id, uint8_t param) {
  switch (attr_id) {
    break;
  case CP437_SWITCH:
    _cp437 = param;
    break;
  case UTF8_SWITCH:
    _utf8 = param;
    decoderState = 0;
    break;
  case PSRAM_ENABLE:
#if defined (ESP32) && defined (CONFIG_SPIRAM_SUPPORT)
    if (psramFound()) _psram_enable = param; // Enable the use of PSRAM (if available)
    else
#endif
      _psram_enable = false;
    break;
    //case 4: // TBD future feature control
    //    _tbd = param;
    //    break;
  }
}


/**************************************************************************
** Function name:           getAttribute
** Description:             Get value of an attribute (control parameter)
**************************************************************************/
uint8_t TFT_eSPI_KB32::getAttribute(uint8_t attr_id) {
  switch (attr_id) {
  case CP437_SWITCH: // ON/OFF control of full CP437 character set
    return _cp437;
  case UTF8_SWITCH: // ON/OFF control of UTF-8 decoding
    return _utf8;
  case PSRAM_ENABLE:
    return _psram_enable;
    //case 3: // TBD future feature control
    //    return _tbd;
    //    break;
  }

  return false;
}

/***************************************************************************************
** Function name:           decodeUTF8
** Description:             Serial UTF-8 decoder with fall-back to extended ASCII
*************************************************************************************x*/
#define DECODE_UTF8 // Test only, comment out to stop decoding
uint16_t TFT_eSPI_KB32::decodeUTF8(uint8_t c)
{
#ifdef DECODE_UTF8
  // 7 bit Unicode Code Point
  if ((c & 0x80) == 0x00) {
    decoderState = 0;
    return (uint16_t)c;
  }

  if (decoderState == 0) {
    // 11 bit Unicode Code Point
    if ((c & 0xE0) == 0xC0) {
      decoderBuffer = ((c & 0x1F) << 6);
      decoderState = 1;
      return 0;
    }
    // 16 bit Unicode Code Point
    if ((c & 0xF0) == 0xE0) {
      decoderBuffer = ((c & 0x0F) << 12);
      decoderState = 2;
      return 0;
    }
    // 21 bit Unicode  Code Point not supported so fall-back to extended ASCII
    // if ((c & 0xF8) == 0xF0) return (uint16_t)c;
  }
  else {
    if (decoderState == 2) {
      decoderBuffer |= ((c & 0x3F) << 6);
      decoderState--;
      return 0;
    }
    else {
      decoderBuffer |= (c & 0x3F);
      decoderState = 0;
      return decoderBuffer;
    }
  }

  decoderState = 0;
#endif

  return (uint16_t)c; // fall-back to extended ASCII
}


/***************************************************************************************
** Function name:           decodeUTF8
** Description:             Line buffer UTF-8 decoder with fall-back to extended ASCII
*************************************************************************************x*/
uint16_t TFT_eSPI_KB32::decodeUTF8(uint8_t* buf, uint16_t* index, uint16_t remaining)
{
  uint16_t c = buf[(*index)++];
  //Serial.print("Byte from string = 0x"); Serial.println(c, HEX);

#ifdef DECODE_UTF8
  // 7 bit Unicode
  if ((c & 0x80) == 0x00) return c;

  // 11 bit Unicode
  if (((c & 0xE0) == 0xC0) && (remaining > 1))
    return ((c & 0x1F) << 6) | (buf[(*index)++] & 0x3F);

  // 16 bit Unicode
  if (((c & 0xF0) == 0xE0) && (remaining > 2)) {
    c = ((c & 0x0F) << 12) | ((buf[(*index)++] & 0x3F) << 6);
    return  c | ((buf[(*index)++] & 0x3F));
  }

  // 21 bit Unicode not supported so fall-back to extended ASCII
  // if ((c & 0xF8) == 0xF0) return c;
#endif

  return c; // fall-back to extended ASCII
}


/***************************************************************************************
** Function name:           alphaBlend
** Description:             Blend 16bit foreground and background
*************************************************************************************x*/
uint16_t TFT_eSPI_KB32::alphaBlend(uint8_t alpha, uint16_t fgc, uint16_t bgc)
{
  // For speed use fixed point maths and rounding to permit a power of 2 division
  uint16_t fgR = ((fgc >> 10) & 0x3E) + 1;
  uint16_t fgG = ((fgc >> 4) & 0x7E) + 1;
  uint16_t fgB = ((fgc << 1) & 0x3E) + 1;

  uint16_t bgR = ((bgc >> 10) & 0x3E) + 1;
  uint16_t bgG = ((bgc >> 4) & 0x7E) + 1;
  uint16_t bgB = ((bgc << 1) & 0x3E) + 1;

  // Shift right 1 to drop rounding bit and shift right 8 to divide by 256
  uint16_t r = (((fgR * alpha) + (bgR * (255 - alpha))) >> 9);
  uint16_t g = (((fgG * alpha) + (bgG * (255 - alpha))) >> 9);
  uint16_t b = (((fgB * alpha) + (bgB * (255 - alpha))) >> 9);

  // Combine RGB565 colours into 16 bits
  //return ((r&0x18) << 11) | ((g&0x30) << 5) | ((b&0x18) << 0); // 2 bit greyscale
  //return ((r&0x1E) << 11) | ((g&0x3C) << 5) | ((b&0x1E) << 0); // 4 bit greyscale
  return (r << 11) | (g << 5) | (b << 0);
}

/***************************************************************************************
** Function name:           alphaBlend
** Description:             Blend 16bit foreground and background with dither
*************************************************************************************x*/
uint16_t TFT_eSPI_KB32::alphaBlend(uint8_t alpha, uint16_t fgc, uint16_t bgc, uint8_t dither)
{
  if (dither) {
    int16_t alphaDither = (int16_t)alpha - dither + random(2 * dither + 1); // +/-4 randomised
    alpha = (uint8_t)alphaDither;
    if (alphaDither < 0) alpha = 0;
    if (alphaDither > 255) alpha = 255;
  }

  return alphaBlend(alpha, fgc, bgc);
}

/***************************************************************************************
** Function name:           alphaBlend
** Description:             Blend 24bit foreground and background with optional dither
*************************************************************************************x*/
uint32_t TFT_eSPI_KB32::alphaBlend24(uint8_t alpha, uint32_t fgc, uint32_t bgc, uint8_t dither)
{

  if (dither) {
    int16_t alphaDither = (int16_t)alpha - dither + random(2 * dither + 1); // +/-dither randomised
    alpha = (uint8_t)alphaDither;
    if (alphaDither < 0) alpha = 0;
    if (alphaDither > 255) alpha = 255;
  }

  // For speed use fixed point maths and rounding to permit a power of 2 division
  uint16_t fgR = ((fgc >> 15) & 0x1FE) + 1;
  uint16_t fgG = ((fgc >> 7) & 0x1FE) + 1;
  uint16_t fgB = ((fgc << 1) & 0x1FE) + 1;

  uint16_t bgR = ((bgc >> 15) & 0x1FE) + 1;
  uint16_t bgG = ((bgc >> 7) & 0x1FE) + 1;
  uint16_t bgB = ((bgc << 1) & 0x1FE) + 1;

  // Shift right 1 to drop rounding bit and shift right 8 to divide by 256
  uint16_t r = (((fgR * alpha) + (bgR * (255 - alpha))) >> 9);
  uint16_t g = (((fgG * alpha) + (bgG * (255 - alpha))) >> 9);
  uint16_t b = (((fgB * alpha) + (bgB * (255 - alpha))) >> 9);

  // Combine RGB colours into 24 bits
  return (r << 16) | (g << 8) | (b << 0);
}

/***************************************************************************************
** Function name:           write
** Description:             draw characters piped through serial stream
***************************************************************************************/
size_t TFT_eSPI_KB32::write(uint8_t utf8)
{
  if (_vpOoB) return 1;

  uint16_t uniCode = decodeUTF8(utf8);

  if (!uniCode) return 1;

  if (utf8 == '\r') return 1;

#ifdef SMOOTH_FONT
  if (fontLoaded) {
    if (uniCode < 32 && utf8 != '\n') return 1;

    drawGlyph(uniCode);

    return 1;
  }
#endif

  if (uniCode == '\n') uniCode += 22; // Make it a valid space character to stop errors
  else if (uniCode < 32) return 1;

  uint16_t cwidth = 0;
  uint16_t cheight = 0;

  //vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv DEBUG vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
    //Serial.print((uint8_t) uniCode); // Debug line sends all printed TFT text to serial port
    //Serial.println(uniCode, HEX); // Debug line sends all printed TFT text to serial port
    //delay(5);                     // Debug optional wait for serial port to flush through
  //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ DEBUG ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#ifdef LOAD_GFXFF
  if (!gfxFont) {
#endif
    //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

#ifdef LOAD_FONT2
    if (textfont == 2) {
      if (uniCode > 127) return 1;

      cwidth = pgm_read_byte(widtbl_f16 + uniCode - 32);
      cheight = chr_hgt_f16;
      // Font 2 is rendered in whole byte widths so we must allow for this
      cwidth = (cwidth + 6) / 8;  // Width in whole bytes for font 2, should be + 7 but must allow for font width change
      cwidth = cwidth * 8;        // Width converted back to pixels
    }
#ifdef LOAD_RLE
    else
#endif
#endif

#ifdef LOAD_RLE
    {
      if ((textfont > 2) && (textfont < 9)) {
        if (uniCode > 127) return 1;
        // Uses the fontinfo struct array to avoid lots of 'if' or 'switch' statements
        cwidth = pgm_read_byte((uint8_t*)pgm_read_dword(&(fontdata[textfont].widthtbl)) + uniCode - 32);
        cheight = pgm_read_byte(&fontdata[textfont].height);
      }
    }
#endif

#ifdef LOAD_GLCD
    if (textfont == 1) {
      cwidth = 6;
      cheight = 8;
    }
#else
    if (textfont == 1) return 1;
#endif

    cheight = cheight * textsize;

    if (utf8 == '\n') {
      cursor_y += cheight;
      cursor_x = 0;
    }
    else {
      if (textwrapX && (cursor_x + cwidth * textsize > width())) {
        cursor_y += cheight;
        cursor_x = 0;
      }
      if (textwrapY && (cursor_y >= (int32_t)height())) cursor_y = 0;
      cursor_x += drawChar(uniCode, cursor_x, cursor_y, textfont);
    }

    //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#ifdef LOAD_GFXFF
  } // Custom GFX font
  else {
    if (utf8 == '\n') {
      cursor_x = 0;
      cursor_y += (int16_t)textsize * (uint8_t)pgm_read_byte(&gfxFont->yAdvance);
    }
    else {
      if (uniCode > pgm_read_word(&gfxFont->last)) return 1;
      if (uniCode < pgm_read_word(&gfxFont->first)) return 1;

      uint16_t   c2 = uniCode - pgm_read_word(&gfxFont->first);
      GFXglyph* glyph = &(((GFXglyph*)pgm_read_dword(&gfxFont->glyph))[c2]);
      uint8_t   w = pgm_read_byte(&glyph->width),
        h = pgm_read_byte(&glyph->height);
      if ((w > 0) && (h > 0)) { // Is there an associated bitmap?
        int16_t xo = (int8_t)pgm_read_byte(&glyph->xOffset);
        if (textwrapX && ((cursor_x + textsize * (xo + w)) > width())) {
          // Drawing character would go off right edge; wrap to new line
          cursor_x = 0;
          cursor_y += (int16_t)textsize * (uint8_t)pgm_read_byte(&gfxFont->yAdvance);
        }
        if (textwrapY && (cursor_y >= (int32_t)height())) cursor_y = 0;
        drawChar(cursor_x, cursor_y, uniCode, textcolor, textbgcolor, textsize);
      }
      cursor_x += pgm_read_byte(&glyph->xAdvance) * (int16_t)textsize;
    }
  }
#endif // LOAD_GFXFF
  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

  return 1;
}


/***************************************************************************************
** Function name:           drawChar
** Description:             draw a Unicode glyph onto the screen
***************************************************************************************/
// TODO: Rationalise with TFT_eSprite
// Any UTF-8 decoding must be done before calling drawChar()
int16_t TFT_eSPI_KB32::drawChar(uint16_t uniCode, int32_t x, int32_t y)
{
  return drawChar(uniCode, x, y, textfont);
}

// Any UTF-8 decoding must be done before calling drawChar()
int16_t TFT_eSPI_KB32::drawChar(uint16_t uniCode, int32_t x, int32_t y, uint8_t font)
{
  if (_vpOoB || !uniCode) return 0;

  if (font == 1) {
#ifdef LOAD_GLCD
#ifndef LOAD_GFXFF
    drawChar(x, y, uniCode, textcolor, textbgcolor, textsize);
    return 6 * textsize;
#endif
#else
#ifndef LOAD_GFXFF
    return 0;
#endif
#endif

#ifdef LOAD_GFXFF
    drawChar(x, y, uniCode, textcolor, textbgcolor, textsize);
    if (!gfxFont) { // 'Classic' built-in font
#ifdef LOAD_GLCD
      return 6 * textsize;
#else
      return 0;
#endif
    }
    else {
      if ((uniCode >= pgm_read_word(&gfxFont->first)) && (uniCode <= pgm_read_word(&gfxFont->last))) {
        uint16_t   c2 = uniCode - pgm_read_word(&gfxFont->first);
        GFXglyph* glyph = &(((GFXglyph*)pgm_read_dword(&gfxFont->glyph))[c2]);
        return pgm_read_byte(&glyph->xAdvance) * textsize;
      }
      else {
        return 0;
      }
    }
#endif
  }

  if ((font > 1) && (font < 9) && ((uniCode < 32) || (uniCode > 127))) return 0;

  int32_t width = 0;
  int32_t height = 0;
  uint32_t flash_address = 0;
  uniCode -= 32;

#ifdef LOAD_FONT2
  if (font == 2) {
    flash_address = pgm_read_dword(&chrtbl_f16[uniCode]);
    width = pgm_read_byte(widtbl_f16 + uniCode);
    height = chr_hgt_f16;
  }
#ifdef LOAD_RLE
  else
#endif
#endif

#ifdef LOAD_RLE
  {
    if ((font > 2) && (font < 9)) {
      flash_address = pgm_read_dword((const void*)(pgm_read_dword(&(fontdata[font].chartbl)) + uniCode * sizeof(void*)));
      width = pgm_read_byte((uint8_t*)pgm_read_dword(&(fontdata[font].widthtbl)) + uniCode);
      height = pgm_read_byte(&fontdata[font].height);
    }
  }
#endif

  int32_t xd = x + _xDatum;
  int32_t yd = y + _yDatum;

  if ((xd + width * textsize < _vpX || xd >= _vpW) && (yd + height * textsize < _vpY || yd >= _vpH)) return width * textsize;

  int32_t w = width;
  int32_t pX = 0;
  int32_t pY = y;
  uint8_t line = 0;
  bool clip = xd < _vpX || xd + width * textsize >= _vpW || yd < _vpY || yd + height * textsize >= _vpH;

#ifdef LOAD_FONT2 // chop out code if we do not need it
  if (font == 2) {
    w = w + 6; // Should be + 7 but we need to compensate for width increment
    w = w / 8;

    if (textcolor == textbgcolor || textsize != 1 || clip) {
      //begin_tft_write();          // Sprite class can use this function, avoiding begin_tft_write()
      inTransaction = true;

      for (int32_t i = 0; i < height; i++) {
        if (textcolor != textbgcolor) fillRect(x, pY, width * textsize, textsize, textbgcolor);

        for (int32_t k = 0; k < w; k++) {
          line = pgm_read_byte((uint8_t*)flash_address + w * i + k);
          if (line) {
            if (textsize == 1) {
              pX = x + k * 8;
              if (line & 0x80) drawPixel(pX, pY, textcolor);
              if (line & 0x40) drawPixel(pX + 1, pY, textcolor);
              if (line & 0x20) drawPixel(pX + 2, pY, textcolor);
              if (line & 0x10) drawPixel(pX + 3, pY, textcolor);
              if (line & 0x08) drawPixel(pX + 4, pY, textcolor);
              if (line & 0x04) drawPixel(pX + 5, pY, textcolor);
              if (line & 0x02) drawPixel(pX + 6, pY, textcolor);
              if (line & 0x01) drawPixel(pX + 7, pY, textcolor);
            }
            else {
              pX = x + k * 8 * textsize;
              if (line & 0x80) fillRect(pX, pY, textsize, textsize, textcolor);
              if (line & 0x40) fillRect(pX + textsize, pY, textsize, textsize, textcolor);
              if (line & 0x20) fillRect(pX + 2 * textsize, pY, textsize, textsize, textcolor);
              if (line & 0x10) fillRect(pX + 3 * textsize, pY, textsize, textsize, textcolor);
              if (line & 0x08) fillRect(pX + 4 * textsize, pY, textsize, textsize, textcolor);
              if (line & 0x04) fillRect(pX + 5 * textsize, pY, textsize, textsize, textcolor);
              if (line & 0x02) fillRect(pX + 6 * textsize, pY, textsize, textsize, textcolor);
              if (line & 0x01) fillRect(pX + 7 * textsize, pY, textsize, textsize, textcolor);
            }
          }
        }
        pY += textsize;
      }

      inTransaction = false;
      end_tft_write();
    }
    else { // Faster drawing of characters and background using block write

      begin_tft_write();

      setWindow(xd, yd, xd + width - 1, yd + height - 1);

      uint8_t mask;
      for (int32_t i = 0; i < height; i++) {
        pX = width;
        for (int32_t k = 0; k < w; k++) {
          line = pgm_read_byte((uint8_t*)(flash_address + w * i + k));
          mask = 0x80;
          while (mask && pX) {
            if (line & mask) { tft_Write_16(textcolor); }
            else { tft_Write_16(textbgcolor); }
            pX--;
            mask = mask >> 1;
          }
        }
        if (pX) { tft_Write_16(textbgcolor); }
      }

      end_tft_write();
    }
  }

#ifdef LOAD_RLE
  else
#endif
#endif  //FONT2

#ifdef LOAD_RLE  //674 bytes of code
    // Font is not 2 and hence is RLE encoded
  {
    begin_tft_write();
    inTransaction = true;

    w *= height; // Now w is total number of pixels in the character
    if (textcolor == textbgcolor && !clip) {

      int32_t px = 0, py = pY; // To hold character block start and end column and row values
      int32_t pc = 0; // Pixel count
      uint8_t np = textsize * textsize; // Number of pixels in a drawn pixel

      uint8_t tnp = 0; // Temporary copy of np for while loop
      uint8_t ts = textsize - 1; // Temporary copy of textsize
      // 16 bit pixel count so maximum font size is equivalent to 180x180 pixels in area
      // w is total number of pixels to plot to fill character block
      while (pc < w) {
        line = pgm_read_byte((uint8_t*)flash_address);
        flash_address++;
        if (line & 0x80) {
          line &= 0x7F;
          line++;
          if (ts) {
            px = xd + textsize * (pc % width); // Keep these px and py calculations outside the loop as they are slow
            py = yd + textsize * (pc / width);
          }
          else {
            px = xd + pc % width; // Keep these px and py calculations outside the loop as they are slow
            py = yd + pc / width;
          }
          while (line--) { // In this case the while(line--) is faster
            pc++; // This is faster than putting pc+=line before while()?
            setWindow(px, py, px + ts, py + ts);

            if (ts) {
              tnp = np;
              while (tnp--) { tft_Write_16(textcolor); }
            }
            else { tft_Write_16(textcolor); }
            px += textsize;

            if (px >= (xd + width * textsize)) {
              px = xd;
              py += textsize;
            }
          }
        }
        else {
          line++;
          pc += line;
        }
      }
    }
    else {
      // Text colour != background and textsize = 1 and character is within viewport area
      // so use faster drawing of characters and background using block write
      if (textcolor != textbgcolor && textsize == 1 && !clip)
      {
        setWindow(xd, yd, xd + width - 1, yd + height - 1);

        // Maximum font size is equivalent to 180x180 pixels in area
        while (w > 0) {
          line = pgm_read_byte((uint8_t*)flash_address++); // 8 bytes smaller when incrementing here
          if (line & 0x80) {
            line &= 0x7F;
            line++; w -= line;
            pushBlock(textcolor, line);
          }
          else {
            line++; w -= line;
            pushBlock(textbgcolor, line);
          }
        }
      }
      else
      {
        int32_t px = 0, py = 0;  // To hold character pixel coords
        int32_t tx = 0, ty = 0;  // To hold character TFT pixel coords
        int32_t pc = 0;          // Pixel count
        int32_t pl = 0;          // Pixel line length
        uint16_t pcol = 0;       // Pixel color
        bool     pf = true;      // Flag for plotting
        while (pc < w) {
          line = pgm_read_byte((uint8_t*)flash_address);
          flash_address++;
          if (line & 0x80) { pcol = textcolor; line &= 0x7F; pf = true; }
          else { pcol = textbgcolor; if (textcolor == textbgcolor) pf = false; }
          line++;
          px = pc % width;
          tx = x + textsize * px;
          py = pc / width;
          ty = y + textsize * py;

          pl = 0;
          pc += line;
          while (line--) {
            pl++;
            if ((px + pl) >= width) {
              if (pf) fillRect(tx, ty, pl * textsize, textsize, pcol);
              pl = 0;
              px = 0;
              tx = x;
              py++;
              ty += textsize;
            }
          }
          if (pl && pf) fillRect(tx, ty, pl * textsize, textsize, pcol);
        }
      }
    }
    inTransaction = false;
    end_tft_write();
  }
  // End of RLE font rendering
#endif
  return width * textsize;    // x +
}


/***************************************************************************************
** Function name:           drawString (with or without user defined font)
** Description :            draw string with padding if it is defined
***************************************************************************************/
// Without font number, uses font set by setTextFont()
int16_t TFT_eSPI_KB32::drawString(const String& string, int32_t poX, int32_t poY)
{
  int16_t len = string.length() + 2;
  char buffer[len];
  string.toCharArray(buffer, len);
  return drawString(buffer, poX, poY, textfont);
}
// With font number
int16_t TFT_eSPI_KB32::drawString(const String& string, int32_t poX, int32_t poY, uint8_t font)
{
  int16_t len = string.length() + 2;
  char buffer[len];
  string.toCharArray(buffer, len);
  return drawString(buffer, poX, poY, font);
}

// Without font number, uses font set by setTextFont()
int16_t TFT_eSPI_KB32::drawString(const char* string, int32_t poX, int32_t poY)
{
  return drawString(string, poX, poY, textfont);
}

// With font number. Note: font number is over-ridden if a smooth font is loaded
int16_t TFT_eSPI_KB32::drawString(const char* string, int32_t poX, int32_t poY, uint8_t font)
{
  int16_t sumX = 0;
  uint8_t padding = 1, baseline = 0;
  uint16_t cwidth = textWidth(string, font); // Find the pixel width of the string in the font
  uint16_t cheight = 8 * textsize;

#ifdef LOAD_GFXFF
#ifdef SMOOTH_FONT
  bool freeFont = (font == 1 && gfxFont && !fontLoaded);
#else
  bool freeFont = (font == 1 && gfxFont);
#endif

  if (freeFont) {
    cheight = glyph_ab * textsize;
    poY += cheight; // Adjust for baseline datum of free fonts
    baseline = cheight;
    padding = 101; // Different padding method used for Free Fonts

    // We need to make an adjustment for the bottom of the string (eg 'y' character)
    if ((textdatum == BL_DATUM) || (textdatum == BC_DATUM) || (textdatum == BR_DATUM)) {
      cheight += glyph_bb * textsize;
    }
  }
#endif


  // If it is not font 1 (GLCD or free font) get the baseline and pixel height of the font
#ifdef SMOOTH_FONT
  if (fontLoaded) {
    baseline = gFont.maxAscent;
    cheight = fontHeight();
  }
  else
#endif
    if (font != 1) {
      baseline = pgm_read_byte(&fontdata[font].baseline) * textsize;
      cheight = fontHeight(font);
    }

  if (textdatum || padX) {

    switch (textdatum) {
    case TC_DATUM:
      poX -= cwidth / 2;
      padding += 1;
      break;
    case TR_DATUM:
      poX -= cwidth;
      padding += 2;
      break;
    case ML_DATUM:
      poY -= cheight / 2;
      //padding += 0;
      break;
    case MC_DATUM:
      poX -= cwidth / 2;
      poY -= cheight / 2;
      padding += 1;
      break;
    case MR_DATUM:
      poX -= cwidth;
      poY -= cheight / 2;
      padding += 2;
      break;
    case BL_DATUM:
      poY -= cheight;
      //padding += 0;
      break;
    case BC_DATUM:
      poX -= cwidth / 2;
      poY -= cheight;
      padding += 1;
      break;
    case BR_DATUM:
      poX -= cwidth;
      poY -= cheight;
      padding += 2;
      break;
    case L_BASELINE:
      poY -= baseline;
      //padding += 0;
      break;
    case C_BASELINE:
      poX -= cwidth / 2;
      poY -= baseline;
      padding += 1;
      break;
    case R_BASELINE:
      poX -= cwidth;
      poY -= baseline;
      padding += 2;
      break;
    }
  }


  int8_t xo = 0;
#ifdef LOAD_GFXFF
  if (freeFont && (textcolor != textbgcolor)) {
    cheight = (glyph_ab + glyph_bb) * textsize;
    // Get the offset for the first character only to allow for negative offsets
    uint16_t c2 = 0;
    uint16_t len = strlen(string);
    uint16_t n = 0;

    while (n < len && c2 == 0) c2 = decodeUTF8((uint8_t*)string, &n, len - n);

    if ((c2 >= pgm_read_word(&gfxFont->first)) && (c2 <= pgm_read_word(&gfxFont->last))) {
      c2 -= pgm_read_word(&gfxFont->first);
      GFXglyph* glyph = &(((GFXglyph*)pgm_read_dword(&gfxFont->glyph))[c2]);
      xo = pgm_read_byte(&glyph->xOffset) * textsize;
      // Adjust for negative xOffset
      if (xo > 0) xo = 0;
      else cwidth -= xo;
      // Add 1 pixel of padding all round
      //cheight +=2;
      //fillRect(poX+xo-1, poY - 1 - glyph_ab * textsize, cwidth+2, cheight, textbgcolor);
      fillRect(poX + xo, poY - glyph_ab * textsize, cwidth, cheight, textbgcolor);
    }
    padding -= 100;
  }
#endif

  uint16_t len = strlen(string);
  uint16_t n = 0;

#ifdef SMOOTH_FONT
  if (fontLoaded) {
    if (textcolor != textbgcolor) fillRect(poX, poY, cwidth, cheight, textbgcolor);
    /*
        // The above only works for a single text line, not if the text is going to wrap...
        // So need to use code like this in a while loop to fix it:
        if (textwrapX && (cursor_x + width * textsize > width())) {
          cursor_y += height;
          cursor_x = 0;
        }
        if (textwrapY && (cursor_y >= (int32_t)height())) cursor_y = 0;
        cursor_x += drawChar(uniCode, cursor_x, cursor_y, textfont);
    */
    setCursor(poX, poY);

    while (n < len) {
      uint16_t uniCode = decodeUTF8((uint8_t*)string, &n, len - n);
      drawGlyph(uniCode);
    }
    sumX += cwidth;
    //fontFile.close();
  }
  else
#endif
  {
    while (n < len) {
      uint16_t uniCode = decodeUTF8((uint8_t*)string, &n, len - n);
      sumX += drawChar(uniCode, poX + sumX, poY, font);
    }
  }

  //vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv DEBUG vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
  // Switch on debugging for the padding areas
  //#define PADDING_DEBUG

#ifndef PADDING_DEBUG
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ DEBUG ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  if ((padX > cwidth) && (textcolor != textbgcolor)) {
    int16_t padXc = poX + cwidth + xo;
#ifdef LOAD_GFXFF
    if (freeFont) {
      poX += xo; // Adjust for negative offset start character
      poY -= glyph_ab * textsize;
      sumX += poX;
    }
#endif
    switch (padding) {
    case 1:
      fillRect(padXc, poY, padX - cwidth, cheight, textbgcolor);
      break;
    case 2:
      fillRect(padXc, poY, (padX - cwidth) >> 1, cheight, textbgcolor);
      padXc = poX - ((padX - cwidth) >> 1);
      fillRect(padXc, poY, (padX - cwidth) >> 1, cheight, textbgcolor);
      break;
    case 3:
      if (padXc > padX) padXc = padX;
      fillRect(poX + cwidth - padXc, poY, padXc - cwidth, cheight, textbgcolor);
      break;
    }
  }


#else

//vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv DEBUG vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// This is debug code to show text (green box) and blanked (white box) areas
// It shows that the padding areas are being correctly sized and positioned

  if ((padX > sumX) && (textcolor != textbgcolor)) {
    int16_t padXc = poX + sumX; // Maximum left side padding
#ifdef LOAD_GFXFF
    if ((font == 1) && (gfxFont)) poY -= glyph_ab;
#endif
    drawRect(poX, poY, sumX, cheight, TFT_GREEN);
    switch (padding) {
    case 1:
      drawRect(padXc, poY, padX - sumX, cheight, TFT_WHITE);
      break;
    case 2:
      drawRect(padXc, poY, (padX - sumX) >> 1, cheight, TFT_WHITE);
      padXc = (padX - sumX) >> 1;
      drawRect(poX - padXc, poY, (padX - sumX) >> 1, cheight, TFT_WHITE);
      break;
    case 3:
      if (padXc > padX) padXc = padX;
      drawRect(poX + sumX - padXc, poY, padXc - sumX, cheight, TFT_WHITE);
      break;
    }
  }
#endif
  //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ DEBUG ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  return sumX;
}


/***************************************************************************************
** Function name:           drawCentreString (deprecated, use setTextDatum())
** Descriptions:            draw string centred on dX
***************************************************************************************/
int16_t TFT_eSPI_KB32::drawCentreString(const String& string, int32_t dX, int32_t poY, uint8_t font)
{
  int16_t len = string.length() + 2;
  char buffer[len];
  string.toCharArray(buffer, len);
  return drawCentreString(buffer, dX, poY, font);
}

int16_t TFT_eSPI_KB32::drawCentreString(const char* string, int32_t dX, int32_t poY, uint8_t font)
{
  uint8_t tempdatum = textdatum;
  int32_t sumX = 0;
  textdatum = TC_DATUM;
  sumX = drawString(string, dX, poY, font);
  textdatum = tempdatum;
  return sumX;
}


/***************************************************************************************
** Function name:           drawRightString (deprecated, use setTextDatum())
** Descriptions:            draw string right justified to dX
***************************************************************************************/
int16_t TFT_eSPI_KB32::drawRightString(const String& string, int32_t dX, int32_t poY, uint8_t font)
{
  int16_t len = string.length() + 2;
  char buffer[len];
  string.toCharArray(buffer, len);
  return drawRightString(buffer, dX, poY, font);
}

int16_t TFT_eSPI_KB32::drawRightString(const char* string, int32_t dX, int32_t poY, uint8_t font)
{
  uint8_t tempdatum = textdatum;
  int16_t sumX = 0;
  textdatum = TR_DATUM;
  sumX = drawString(string, dX, poY, font);
  textdatum = tempdatum;
  return sumX;
}


/***************************************************************************************
** Function name:           drawNumber
** Description:             draw a long integer
***************************************************************************************/
int16_t TFT_eSPI_KB32::drawNumber(long long_num, int32_t poX, int32_t poY)
{
  isDigits = true; // Eliminate jiggle in monospaced fonts
  char str[12];
  ltoa(long_num, str, 10);
  return drawString(str, poX, poY, textfont);
}

int16_t TFT_eSPI_KB32::drawNumber(long long_num, int32_t poX, int32_t poY, uint8_t font)
{
  isDigits = true; // Eliminate jiggle in monospaced fonts
  char str[12];
  ltoa(long_num, str, 10);
  return drawString(str, poX, poY, font);
}


/***************************************************************************************
** Function name:           drawFloat
** Descriptions:            drawFloat, prints 7 non zero digits maximum
***************************************************************************************/
// Assemble and print a string, this permits alignment relative to a datum
// looks complicated but much more compact and actually faster than using print class
int16_t TFT_eSPI_KB32::drawFloat(float floatNumber, uint8_t dp, int32_t poX, int32_t poY)
{
  return drawFloat(floatNumber, dp, poX, poY, textfont);
}

int16_t TFT_eSPI_KB32::drawFloat(float floatNumber, uint8_t dp, int32_t poX, int32_t poY, uint8_t font)
{
  isDigits = true;
  char str[14];               // Array to contain decimal string
  uint8_t ptr = 0;            // Initialise pointer for array
  int8_t  digits = 1;         // Count the digits to avoid array overflow
  float rounding = 0.5;       // Round up down delta

  if (dp > 7) dp = 7; // Limit the size of decimal portion

  // Adjust the rounding value
  for (uint8_t i = 0; i < dp; ++i) rounding /= 10.0;

  if (floatNumber < -rounding) {   // add sign, avoid adding - sign to 0.0!
    str[ptr++] = '-'; // Negative number
    str[ptr] = 0; // Put a null in the array as a precaution
    digits = 0;   // Set digits to 0 to compensate so pointer value can be used later
    floatNumber = -floatNumber; // Make positive
  }

  floatNumber += rounding; // Round up or down

  // For error put ... in string and return (all TFT_eSPI library fonts contain . character)
  if (floatNumber >= 2147483647) {
    strcpy(str, "...");
    return drawString(str, poX, poY, font);
  }
  // No chance of overflow from here on

  // Get integer part
  uint32_t temp = (uint32_t)floatNumber;

  // Put integer part into array
  ltoa(temp, str + ptr, 10);

  // Find out where the null is to get the digit count loaded
  while ((uint8_t)str[ptr] != 0) ptr++; // Move the pointer along
  digits += ptr;                  // Count the digits

  str[ptr++] = '.'; // Add decimal point
  str[ptr] = '0';   // Add a dummy zero
  str[ptr + 1] = 0; // Add a null but don't increment pointer so it can be overwritten

  // Get the decimal portion
  floatNumber = floatNumber - temp;

  // Get decimal digits one by one and put in array
  // Limit digit count so we don't get a false sense of resolution
  uint8_t i = 0;
  while ((i < dp) && (digits < 9)) { // while (i < dp) for no limit but array size must be increased
    i++;
    floatNumber *= 10;       // for the next decimal
    temp = floatNumber;      // get the decimal
    ltoa(temp, str + ptr, 10);
    ptr++; digits++;         // Increment pointer and digits count
    floatNumber -= temp;     // Remove that digit
  }

  // Finally we can plot the string and return pixel length
  return drawString(str, poX, poY, font);
}


/***************************************************************************************
** Function name:           setFreeFont
** Descriptions:            Sets the GFX free font to use
***************************************************************************************/

#ifdef LOAD_GFXFF

void TFT_eSPI_KB32::setFreeFont(const GFXfont* f)
{
  if (f == nullptr) { // Fix issue #400 (ESP32 crash)
    setTextFont(1); // Use GLCD font
    return;
  }

  textfont = 1;
  gfxFont = (GFXfont*)f;

  glyph_ab = 0;
  glyph_bb = 0;
  uint16_t numChars = pgm_read_word(&gfxFont->last) - pgm_read_word(&gfxFont->first);

  // Find the biggest above and below baseline offsets
  for (uint8_t c = 0; c < numChars; c++) {
    GFXglyph* glyph1 = &(((GFXglyph*)pgm_read_dword(&gfxFont->glyph))[c]);
    int8_t ab = -pgm_read_byte(&glyph1->yOffset);
    if (ab > glyph_ab) glyph_ab = ab;
    int8_t bb = pgm_read_byte(&glyph1->height) - ab;
    if (bb > glyph_bb) glyph_bb = bb;
  }
}


/***************************************************************************************
** Function name:           setTextFont
** Description:             Set the font for the print stream
***************************************************************************************/
void TFT_eSPI_KB32::setTextFont(uint8_t f)
{
  textfont = (f > 0) ? f : 1; // Don't allow font 0
  gfxFont = NULL;
}

#else


/***************************************************************************************
** Function name:           setFreeFont
** Descriptions:            Sets the GFX free font to use
***************************************************************************************/

// Alternative to setTextFont() so we don't need two different named functions
void TFT_eSPI_KB32::setFreeFont(uint8_t font)
{
  setTextFont(font);
}


/***************************************************************************************
** Function name:           setTextFont
** Description:             Set the font for the print stream
***************************************************************************************/
void TFT_eSPI_KB32::setTextFont(uint8_t f)
{
  textfont = (f > 0) ? f : 1; // Don't allow font 0
}
#endif


/***************************************************************************************
** Function name:           getSPIinstance
** Description:             Get the instance of the SPI class
***************************************************************************************/
SPIClass& TFT_eSPI_KB32::getSPIinstance(void)
{
  return spi;
}


/***************************************************************************************
** Function name:           getSetup
** Description:             Get the setup details for diagnostic and sketch access
***************************************************************************************/
void TFT_eSPI_KB32::getSetup(setup_t& tft_settings)
{
  // tft_settings.version is set in header file
  tft_settings.esp = PROCESSOR_ID;
#if defined (SUPPORT_TRANSACTIONS)
  tft_settings.trans = true;
#else
  tft_settings.trans = false;
#endif
  tft_settings.serial = true;
  tft_settings.tft_spi_freq = SPI_FREQUENCY / 100000;
  tft_settings.tft_rd_freq = SPI_READ_FREQUENCY / 100000;

#if defined(TFT_SPI_OVERLAP)
  tft_settings.overlap = true;
#else
  tft_settings.overlap = false;
#endif
  tft_settings.tft_driver = TFT_DRIVER;
  tft_settings.tft_width = _init_width;
  tft_settings.tft_height = _init_height;

#ifdef CGRAM_OFFSET
  tft_settings.r0_x_offset = colstart;
  tft_settings.r0_y_offset = rowstart;
  tft_settings.r1_x_offset = 0;
  tft_settings.r1_y_offset = 0;
  tft_settings.r2_x_offset = 0;
  tft_settings.r2_y_offset = 0;
  tft_settings.r3_x_offset = 0;
  tft_settings.r3_y_offset = 0;
#else
  tft_settings.r0_x_offset = 0;
  tft_settings.r0_y_offset = 0;
  tft_settings.r1_x_offset = 0;
  tft_settings.r1_y_offset = 0;
  tft_settings.r2_x_offset = 0;
  tft_settings.r2_y_offset = 0;
  tft_settings.r3_x_offset = 0;
  tft_settings.r3_y_offset = 0;
#endif

  tft_settings.pin_tft_mosi = TFT_MOSI;

#if defined (TFT_MISO)
  tft_settings.pin_tft_miso = TFT_MISO;
#else
  tft_settings.pin_tft_miso = -1;
#endif

  tft_settings.pin_tft_clk = TFT_SCLK;
  tft_settings.pin_tft_cs = TFT_CS;
  tft_settings.pin_tft_dc = TFT_DC;
  tft_settings.pin_tft_rd = -1;
  tft_settings.pin_tft_wr = -1;
  tft_settings.pin_tft_rst = TFT_RST;
  tft_settings.pin_tft_d0 = -1;
  tft_settings.pin_tft_d1 = -1;
  tft_settings.pin_tft_d2 = -1;
  tft_settings.pin_tft_d3 = -1;
  tft_settings.pin_tft_d4 = -1;
  tft_settings.pin_tft_d5 = -1;
  tft_settings.pin_tft_d6 = -1;
  tft_settings.pin_tft_d7 = -1;
  tft_settings.pin_tch_cs = -1;
  tft_settings.tch_spi_freq = 0;

}


////////////////////////////////////////////////////////////////////////////////////////

#ifdef SMOOTH_FONT
#include "Extensions/Smooth_font.cpp"
#endif

////////////////////////////////////////////////////////////////////////////////////////
