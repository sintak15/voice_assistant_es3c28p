#pragma once

// Local TFT_eSPI setup for LCDWiki 2.8" ESP32-S3 display.
// Keeping this in-project avoids changing global TFT_eSPI config files.

#ifndef USER_SETUP_LOADED
#define USER_SETUP_LOADED
#endif

#define USER_SETUP_INFO "LCDWiki_2p8_ESP32S3_ILI9341_Local"

#define ILI9341_DRIVER
#define USE_HSPI_PORT

#define TFT_CS   10
#define TFT_DC   46
#define TFT_MOSI 11
#define TFT_SCLK 12
#define TFT_MISO 13
#define TFT_RST  -1

#define LOAD_GLCD

#define SPI_FREQUENCY       27000000
#define SPI_READ_FREQUENCY  20000000
#define SPI_TOUCH_FREQUENCY 2500000
