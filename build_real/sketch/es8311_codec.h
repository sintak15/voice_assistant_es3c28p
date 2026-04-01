#line 1 "C:\\Users\\Justin\\Documents\\Arduino\\voice_assistant_es3c28p\\es8311_codec.h"
#pragma once

#include <Arduino.h>
#include <Wire.h>

class Es8311Codec {
 public:
  bool begin(TwoWire& wire, uint8_t address = 0x18) {
    wire_ = &wire;
    address_ = address;
    return detectChip();
  }

  bool initFor16kCapture() {
    if (!wire_) {
      return false;
    }

    // Follow the Hosyond/Freenove ES3C28P Example_30_ai_chat init flow:
    // ES8311 in slave I2S mode, 16 kHz, MCLK=6.144 MHz, analog mic enabled.
    if (!writeReg(0x00, 0x1F)) {  // full reset
      return false;
    }
    delay(20);

    return writeReg(0x00, 0x00) &&
           writeReg(0x00, 0x80) &&  // power on
           writeReg(0x01, 0x3F) &&  // enable all clocks
           writeReg(0x02, 0x48) &&  // pre_div=3, pre_multi=2x
           writeReg(0x03, 0x10) &&  // adc osr
           writeReg(0x04, 0x10) &&  // dac osr
           writeReg(0x05, 0x00) &&  // adc/dac div
           writeReg(0x06, 0x03) &&  // bclk div
           writeReg(0x07, 0x00) &&  // lrck high byte
           writeReg(0x08, 0xFF) &&  // lrck low byte
           writeReg(0x09, 0x0C) &&  // I2S in, 16-bit
           writeReg(0x0A, 0x0C) &&  // I2S out, 16-bit
           writeReg(0x0D, 0x01) &&  // analog power up
           writeReg(0x0E, 0x02) &&  // ADC/PGA power up
           writeReg(0x12, 0x00) &&  // DAC power up
           writeReg(0x13, 0x10) &&  // HP output enable
           writeReg(0x31, 0x00) &&  // DAC unmute
           writeReg(0x32, 0xD0) &&  // DAC volume (higher output)
           writeReg(0x17, 0xC8) &&  // ADC digital gain
           writeReg(0x1C, 0x6A) &&  // ADC EQ bypass + DC cancel
           writeReg(0x37, 0x08) &&  // DAC EQ bypass
           writeReg(0x44, 0x00) &&  // disable loopback/test path
           writeReg(0x45, 0x00) &&
           writeReg(0x14, 0x1A);    // analog MIC + max PGA
  }

  // Gain values map to ES8311 register 0x16.
  bool setMicGainDb(uint8_t gainDb) {
    uint8_t regValue = 0x00;
    if (gainDb >= 42) {
      regValue = 0x07;
    } else {
      regValue = gainDb / 6;  // 0..7 => 0 dB .. 42 dB
    }
    return writeReg(0x16, regValue);
  }

 private:
  bool detectChip() {
    uint8_t chipIdHi = 0;
    uint8_t chipIdLo = 0;
    if (!readReg(0xFD, chipIdHi)) {
      return false;
    }
    if (!readReg(0xFE, chipIdLo)) {
      return false;
    }

    (void)chipIdHi;
    (void)chipIdLo;
    // Some modules do not expose stable ID values; successful register reads are enough.
    return true;
  }

  bool writeReg(uint8_t reg, uint8_t value) {
    wire_->beginTransmission(address_);
    wire_->write(reg);
    wire_->write(value);
    return wire_->endTransmission() == 0;
  }

  bool readReg(uint8_t reg, uint8_t& value) {
    wire_->beginTransmission(address_);
    wire_->write(reg);
    if (wire_->endTransmission(false) != 0) {
      return false;
    }
    const uint8_t readLen = wire_->requestFrom(static_cast<int>(address_), 1);
    if (readLen != 1) {
      return false;
    }
    value = wire_->read();
    return true;
  }

  TwoWire* wire_ = nullptr;
  uint8_t address_ = 0x18;
};
