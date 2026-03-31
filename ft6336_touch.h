#pragma once

#include <Arduino.h>
#include <Wire.h>

struct TouchPoint {
  uint16_t x;
  uint16_t y;
};

class Ft6336Touch {
 public:
  bool begin(uint8_t sdaPin,
             uint8_t sclPin,
             int8_t rstPin = -1,
             int8_t intPin = -1,
             uint32_t i2cFreq = 400000,
             bool useIntGate = false) {
    rstPin_ = rstPin;
    intPin_ = intPin;
    useIntGate_ = useIntGate;

    if (rstPin_ >= 0) {
      pinMode(rstPin_, OUTPUT);
      digitalWrite(rstPin_, LOW);
      delay(10);
      digitalWrite(rstPin_, HIGH);
      delay(50);
    }

    if (intPin_ >= 0) {
      // Some board variants already have external pull-ups on INT.
      pinMode(intPin_, INPUT);
    }

    wire_ = &Wire;
    wire_->begin(sdaPin, sclPin, i2cFreq);

    if (!probeDevice()) {
      return false;
    }

    uint8_t chipId = 0;
    if (!readRegisters(0xA3, &chipId, 1)) {
      // Some FT6336-compatible variants may not expose this register reliably.
      return true;
    }

    (void)chipId;
    return true;
  }

  bool readPoint(TouchPoint& point) {
    if (!wire_) {
      return false;
    }

    // Optional INT gate. Disabled by default because some boards keep INT high.
    if (useIntGate_ && intPin_ >= 0 && digitalRead(intPin_) == HIGH) {
      return false;
    }

    uint8_t data[5] = {0};
    if (!readRegisters(0x02, data, sizeof(data))) {
      return false;
    }

    const uint8_t touchCount = data[0] & 0x0F;
    if (touchCount == 0) {
      return false;
    }

    point.x = static_cast<uint16_t>(((data[1] & 0x0F) << 8) | data[2]);
    point.y = static_cast<uint16_t>(((data[3] & 0x0F) << 8) | data[4]);
    return true;
  }

 private:
  bool probeDevice() {
    wire_->beginTransmission(FT6336_I2C_ADDR);
    return wire_->endTransmission() == 0;
  }

  bool readRegisters(uint8_t startReg, uint8_t* buffer, size_t len) {
    wire_->beginTransmission(FT6336_I2C_ADDR);
    wire_->write(startReg);
    if (wire_->endTransmission(false) != 0) {
      return false;
    }

    const uint8_t received = wire_->requestFrom(static_cast<int>(FT6336_I2C_ADDR), static_cast<int>(len));
    if (received != len) {
      return false;
    }

    for (size_t i = 0; i < len; ++i) {
      buffer[i] = wire_->read();
    }

    return true;
  }

  TwoWire* wire_ = nullptr;
  int8_t rstPin_ = -1;
  int8_t intPin_ = -1;
  bool useIntGate_ = false;
};
