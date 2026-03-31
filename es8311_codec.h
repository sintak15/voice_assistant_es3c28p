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

    // Based on the ES8311 init pattern used in Espressif's ADF driver.
    // This config sets ES8311 as I2S slave and enables ADC capture.
    return writeReg(0x44, 0x08) &&
           writeReg(0x44, 0x08) &&
           writeReg(0x01, 0x30) &&
           writeReg(0x02, 0x00) &&
           writeReg(0x03, 0x10) &&
           writeReg(0x16, 0x24) &&
           writeReg(0x04, 0x10) &&
           writeReg(0x05, 0x00) &&
           writeReg(0x0B, 0x00) &&
           writeReg(0x0C, 0x00) &&
           writeReg(0x10, 0x1F) &&
           writeReg(0x11, 0x7F) &&
           writeReg(0x00, 0x80) &&
           writeReg(0x00, 0x80) &&
           writeReg(0x01, 0x3F) &&
           writeReg(0x02, 0x00) &&
           writeReg(0x03, 0x10) &&
           writeReg(0x04, 0x20) &&
           writeReg(0x05, 0x00) &&
           writeReg(0x06, 0x03) &&
           writeReg(0x07, 0x00) &&
           writeReg(0x08, 0xFF) &&
           writeReg(0x09, 0x4C) &&
           writeReg(0x0A, 0x0C) &&
           writeReg(0x13, 0x10) &&
           writeReg(0x1B, 0x0A) &&
           writeReg(0x1C, 0x6A) &&
           writeReg(0x17, 0xBF) &&
           writeReg(0x0E, 0x02) &&
           writeReg(0x12, 0x00) &&
           writeReg(0x14, 0x1A) &&
           writeReg(0x0D, 0x01) &&
           writeReg(0x15, 0x40) &&
           writeReg(0x37, 0x08) &&
           writeReg(0x45, 0x00) &&
           writeReg(0x44, 0x58);
  }

  // Gain values map to ES8311 register 0x16.
  bool setMicGainDb(uint8_t gainDb) {
    uint8_t regValue = 0x00;
    switch (gainDb) {
      case 0:
        regValue = 0x10;
        break;
      case 6:
        regValue = 0x12;
        break;
      case 12:
        regValue = 0x14;
        break;
      case 18:
        regValue = 0x16;
        break;
      case 24:
        regValue = 0x18;
        break;
      case 30:
        regValue = 0x1A;
        break;
      case 36:
        regValue = 0x1C;
        break;
      case 42:
      default:
        regValue = 0x1E;
        break;
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
