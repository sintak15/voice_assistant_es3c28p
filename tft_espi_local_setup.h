#pragma once

// Local TFT_eSPI setup for LCDWiki 2.8" ESP32-S3 display.
// Keeping this in-project avoids changing global TFT_eSPI config files.

#ifndef USER_SETUP_LOADED
#define USER_SETUP_LOADED
#endif

#define USER_SETUP_INFO "LCDWiki_2p8_ESP32S3_ILI9341_Local"

#define ILI9341_DRIVER
// On Arduino-ESP32 core 3.3.7 for S3, FSPI can map to enum value 0 while
// TFT_eSPI low-level register macros expect SPI2/SPI3 style numbering.
// Force TFT_eSPI to SPI2 register block explicitly for stability.
#define USE_FSPI_PORT

#define TFT_CS   10
#define TFT_DC   46
#define TFT_MOSI 11
#define TFT_SCLK 12
#define TFT_MISO 13
#define TFT_RST  -1

// UI calls drawString(..., font=2/4), so these bitmap fonts must be enabled.
#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4

#define SPI_FREQUENCY       27000000
#define SPI_READ_FREQUENCY  20000000
#define SPI_TOUCH_FREQUENCY 2500000
