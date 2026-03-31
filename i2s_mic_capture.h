#pragma once

#include <Arduino.h>
#include <driver/i2s.h>
#include <math.h>

#include "board_pins.h"
#include "es8311_codec.h"

#ifndef I2S_COMM_FORMAT_STAND_I2S
#define I2S_COMM_FORMAT_STAND_I2S I2S_COMM_FORMAT_I2S
#endif

struct CaptureMetrics {
  float rmsNorm = 0.0f;
  float peakNorm = 0.0f;
  size_t samples = 0;
};

class I2SMicCapture {
 public:
  bool begin(Es8311Codec& codec, uint32_t sampleRate = 16000) {
    if (ready_) {
      return true;
    }

    sampleRate_ = sampleRate;

    pinMode(PIN_AUDIO_ENABLE, OUTPUT);
    // Board docs: low-level enables audio path.
    digitalWrite(PIN_AUDIO_ENABLE, LOW);

    if (!codec.initFor16kCapture()) {
      return false;
    }
    codec.setMicGainDb(36);

    const i2s_config_t i2sConfig = {
        .mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX),
        .sample_rate = static_cast<int>(sampleRate_),
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 256,
        .use_apll = false,
        .tx_desc_auto_clear = true,
        .fixed_mclk = static_cast<int>(sampleRate_ * 256),
    };

    const esp_err_t installErr = i2s_driver_install(port_, &i2sConfig, 0, nullptr);
    if (installErr != ESP_OK) {
      return false;
    }

    const i2s_pin_config_t pinConfig = {
        .mck_io_num = PIN_AUDIO_MCLK,
        .bck_io_num = PIN_AUDIO_BCLK,
        .ws_io_num = PIN_AUDIO_LRCLK,
        .data_out_num = PIN_AUDIO_DOUT,
        .data_in_num = PIN_AUDIO_DIN,
    };

    const esp_err_t pinErr = i2s_set_pin(port_, &pinConfig);
    if (pinErr != ESP_OK) {
      i2s_driver_uninstall(port_);
      return false;
    }

    i2s_zero_dma_buffer(port_);
    ready_ = true;
    return true;
  }

  bool captureBlocking(int16_t* out, size_t sampleCount, CaptureMetrics& metrics) {
    if (!ready_ || !out || sampleCount == 0) {
      return false;
    }

    if (!setClock(sampleRate_)) {
      return false;
    }

    size_t captured = 0;
    while (captured < sampleCount) {
      size_t bytesRead = 0;
      const size_t bytesToRead = (sampleCount - captured) * sizeof(int16_t);
      const esp_err_t err = i2s_read(
          port_,
          reinterpret_cast<void*>(out + captured),
          bytesToRead,
          &bytesRead,
          pdMS_TO_TICKS(1000));

      if (err != ESP_OK || bytesRead == 0) {
        return false;
      }

      captured += bytesRead / sizeof(int16_t);
    }

    metrics = analyze(out, captured);
    return true;
  }

  bool playPcm16Blocking(const uint8_t* pcmBytes, size_t byteCount, uint32_t sampleRate) {
    if (!ready_ || !pcmBytes || byteCount == 0) {
      return false;
    }

    if ((byteCount % sizeof(int16_t)) != 0) {
      return false;
    }

    if (!setClock(sampleRate)) {
      return false;
    }

    size_t played = 0;
    while (played < byteCount) {
      size_t bytesWritten = 0;
      const esp_err_t err = i2s_write(
          port_,
          reinterpret_cast<const void*>(pcmBytes + played),
          byteCount - played,
          &bytesWritten,
          pdMS_TO_TICKS(1000));

      if (err != ESP_OK || bytesWritten == 0) {
        return false;
      }

      played += bytesWritten;
    }

    return true;
  }

  static CaptureMetrics analyze(const int16_t* samples, size_t count) {
    CaptureMetrics result{};
    if (!samples || count == 0) {
      return result;
    }

    int32_t peak = 0;
    double sumSq = 0.0;

    for (size_t i = 0; i < count; ++i) {
      const int32_t v = samples[i];
      const int32_t absV = v < 0 ? -v : v;
      if (absV > peak) {
        peak = absV;
      }
      sumSq += static_cast<double>(v) * static_cast<double>(v);
    }

    result.samples = count;
    result.rmsNorm = sqrt(sumSq / static_cast<double>(count)) / 32768.0f;
    result.peakNorm = static_cast<float>(peak) / 32768.0f;
    return result;
  }

  uint32_t sampleRate() const { return sampleRate_; }

 private:
  bool setClock(uint32_t sampleRate) {
    if (!ready_) {
      return false;
    }
    if (sampleRate == activeSampleRate_) {
      return true;
    }
    const esp_err_t err = i2s_set_clk(port_, sampleRate, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
    if (err != ESP_OK) {
      return false;
    }
    activeSampleRate_ = sampleRate;
    return true;
  }

  bool ready_ = false;
  i2s_port_t port_ = I2S_NUM_0;
  uint32_t sampleRate_ = 16000;
  uint32_t activeSampleRate_ = 0;
};
