////////////////////////////////////////////////////
// TFT_eSPI driver functions for ESP32 processors //
////////////////////////////////////////////////////

#ifndef _TFT_eSPI_ESP32H_
#define _TFT_eSPI_ESP32H_

// Processor ID reported by getSetup()
#define PROCESSOR_ID 0x32

// Include processor specific header
#include "soc/spi_reg.h"
#include "driver/spi_master.h"

// Processor specific code used by SPI bus transaction startWrite and endWrite functions
#define SET_BUS_WRITE_MODE // Not used
#define SET_BUS_READ_MODE  // Not used

// SUPPORT_TRANSACTIONS is mandatory for ESP32 so the hal mutex is toggled
#if !defined (SUPPORT_TRANSACTIONS)
#define SUPPORT_TRANSACTIONS
#endif

// ESP32 specific SPI port selection
#ifdef USE_HSPI_PORT
#define SPI_PORT HSPI
#else
#define SPI_PORT VSPI
#endif

#define CMD_BITS (8-1)


// Initialise processor specific SPI functions, used by init()
#define INIT_TFT_DATA_BUS // Not used


// Code to check if DMA is busy, used by SPI bus transaction transaction and endWrite functions
#define ESP32_DMA
// Code to check if DMA is busy, used by SPI DMA + transaction + endWrite functions
#define DMA_BUSY_CHECK  dmaWait()

// If smooth font is used then it is likely SPIFFS will be needed
#ifdef SMOOTH_FONT
  // Call up the SPIFFS (SPI FLASH Filing System) for the anti-aliased fonts
#define FS_NO_GLOBALS
#include <FS.h>
#include "SPIFFS.h" // ESP32 only
#define FONT_FS_AVAILABLE
#endif

////////////////////////////////////////////////////////////////////////////////////////
// Define the DC (TFT Data/Command or Register Select (RS))pin drive code
////////////////////////////////////////////////////////////////////////////////////////

#define DC_C delayMicroseconds(2);GPIO.out1_w1tc.val = (1 << (TFT_DC - 32));delayMicroseconds(2)//;GPIO.out1_w1tc.val = (1 << (TFT_DC - 32))
#define DC_D delayMicroseconds(2);GPIO.out1_w1ts.val = (1 << (TFT_DC - 32));delayMicroseconds(2)//;GPIO.out1_w1ts.val = (1 << (TFT_DC - 32))


////////////////////////////////////////////////////////////////////////////////////////
// Define the CS (TFT chip select) pin drive code
////////////////////////////////////////////////////////////////////////////////////////

#define CS_L delayMicroseconds(2);GPIO.out1_w1tc.val = (1 << (TFT_CS - 32));delayMicroseconds(2)//; GPIO.out1_w1tc.val = (1 << (TFT_CS - 32))
#define CS_H delayMicroseconds(2);GPIO.out1_w1ts.val = (1 << (TFT_CS - 32));delayMicroseconds(2)//;GPIO.out1_w1ts.val = (1 << (TFT_CS - 32))


////////////////////////////////////////////////////////////////////////////////////////
// Define the parallel bus interface chip pin drive code
////////////////////////////////////////////////////////////////////////////////////////


  // ESP32 low level SPI writes for 8, 16 and 32 bit values
  // to avoid the function call overhead
#define TFT_WRITE_BITS(D, B) \
  WRITE_PERI_REG(SPI_MOSI_DLEN_REG(SPI_PORT), B-1); \
  WRITE_PERI_REG(SPI_W0_REG(SPI_PORT), D); \
  SET_PERI_REG_MASK(SPI_CMD_REG(SPI_PORT), SPI_USR); \
  while (READ_PERI_REG(SPI_CMD_REG(SPI_PORT))&SPI_USR);

  // Write 8 bits
#define tft_Write_8(C) TFT_WRITE_BITS(C, 8)

// Write 16 bits with corrected endianess for 16 bit colours
#define tft_Write_16(C) TFT_WRITE_BITS((C)<<8 | (C)>>8, 16)

// Write 16 bits
#define tft_Write_16S(C) TFT_WRITE_BITS(C, 16)

// Write 32 bits
#define tft_Write_32(C) TFT_WRITE_BITS(C, 32)

// Write two address coordinates
#define tft_Write_32C(C,D)  TFT_WRITE_BITS((uint16_t)((D)<<8 | (D)>>8)<<16 | (uint16_t)((C)<<8 | (C)>>8), 32)

// Write same value twice
#define tft_Write_32D(C) TFT_WRITE_BITS((uint16_t)((C)<<8 | (C)>>8)<<16 | (uint16_t)((C)<<8 | (C)>>8), 32)



////////////////////////////////////////////////////////////////////////////////////////
// Macros to read from display using SPI or software SPI
////////////////////////////////////////////////////////////////////////////////////////

#define tft_Read_8() spi.transfer(0)


// Concatenate a byte sequence A,B,C,D to CDAB, P is a uint8_t pointer
#define DAT8TO32(P) ( (uint32_t)P[0]<<8 | P[1] | P[2]<<24 | P[3]<<16 )

#endif // Header end
