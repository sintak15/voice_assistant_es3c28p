#line 1 "C:\\Users\\Justin\\Documents\\Arduino\\voice_assistant_es3c28p\\board_pins.h"
#pragma once

#include <Arduino.h>

// LCD pins (ILI9341 over SPI)
constexpr int PIN_TFT_CS   = 10;
constexpr int PIN_TFT_DC   = 46;
constexpr int PIN_TFT_MOSI = 11;
constexpr int PIN_TFT_SCLK = 12;
constexpr int PIN_TFT_MISO = 13;
constexpr int PIN_TFT_RST  = -1;
constexpr int PIN_TFT_BL   = 45;

// FT6336 touch controller (I2C)
constexpr int PIN_TOUCH_SDA = 16;
constexpr int PIN_TOUCH_SCL = 15;
constexpr int PIN_TOUCH_RST = 18;
constexpr int PIN_TOUCH_INT = 17;
constexpr uint8_t FT6336_I2C_ADDR = 0x38;

// Audio interface pins from Hosyond/Freenove Example_30_ai_chat:
// DINT=6 (codec data-in from ESP) and DOUT=8 (codec data-out to ESP),
// which maps to ESP I2S DOUT=8 and DIN=6 in their working i2s_std config.
constexpr int PIN_AUDIO_ENABLE = 1;   // Low = enable output path
constexpr int PIN_AUDIO_MCLK   = 4;
constexpr int PIN_AUDIO_BCLK   = 5;
constexpr int PIN_AUDIO_DOUT   = 8;
constexpr int PIN_AUDIO_LRCLK  = 7;
constexpr int PIN_AUDIO_DIN    = 6;

// RGB status LED and battery ADC
constexpr int PIN_STATUS_RGB = 42;
constexpr int PIN_BAT_ADC = 9;
constexpr int PIN_BOOT_BUTTON = 0;

// MicroSD (SDIO 4-bit)
constexpr int PIN_SD_CLK = 38;
constexpr int PIN_SD_CMD = 40;
constexpr int PIN_SD_D0 = 39;
constexpr int PIN_SD_D1 = 41;
constexpr int PIN_SD_D2 = 48;
constexpr int PIN_SD_D3 = 47;
