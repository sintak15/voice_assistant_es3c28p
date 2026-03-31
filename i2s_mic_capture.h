#pragma once

#include <Arduino.h>
#include <driver/i2s.h>
#include <math.h>
#include <stdint.h>

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
    setAudioEnable(false);

    if (!codec.initFor16kCapture()) {
      return false;
    }
    // Max analog mic gain to improve capture on quieter ES8311 front-ends.
    codec.setMicGainDb(42);

    const i2s_config_t i2sConfig = {
        .mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX),
        .sample_rate = static_cast<int>(sampleRate_),
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 256,
        .use_apll = false,
        .tx_desc_auto_clear = true,
        // Hosyond ES3C28P Example_30_ai_chat uses MCLK multiple of 384 at 16k.
        .fixed_mclk = static_cast<int>(sampleRate_ * 384),
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
    activeSampleRate_ = sampleRate_;
    activeChannels_ = static_cast<int>(I2S_CHANNEL_STEREO);
    activeBitsPerSample_ = static_cast<int>(I2S_BITS_PER_SAMPLE_16BIT);
    ready_ = true;
    // ES3C28P reference demos keep AP/EN low for both mic and speaker path.
    captureEnableHigh_ = false;
    setAudioEnable(false);
    Serial.println("[MIC] EN fixed LOW for capture/playback");
    return true;
  }

  bool captureBlocking(int16_t* out, size_t sampleCount, CaptureMetrics& metrics) {
    if (!ready_ || !out || sampleCount == 0) {
      return false;
    }

    setAudioEnable(false);

    if (!setClock(sampleRate_, I2S_CHANNEL_STEREO, I2S_BITS_PER_SAMPLE_16BIT)) {
      return false;
    }

    static const size_t kMaxFramesPerRead = 256;
    int16_t stereoSamples[kMaxFramesPerRead * 2] = {0};

    size_t captured = 0;
    while (captured < sampleCount) {
      const size_t framesRemaining = sampleCount - captured;
      const size_t framesToRead = framesRemaining < kMaxFramesPerRead ? framesRemaining : kMaxFramesPerRead;
      const size_t bytesToRead = framesToRead * sizeof(int16_t) * 2;

      size_t bytesRead = 0;
      const esp_err_t err = i2s_read(
          port_,
          reinterpret_cast<void*>(stereoSamples),
          bytesToRead,
          &bytesRead,
          pdMS_TO_TICKS(1000));

      if (err != ESP_OK || bytesRead == 0) {
        return false;
      }

      const size_t framesRead = bytesRead / (sizeof(int16_t) * 2);
      if (framesRead == 0) {
        return false;
      }

      for (size_t i = 0; i < framesRead && captured < sampleCount; ++i) {
        const int16_t left = stereoSamples[i * 2];
        const int16_t right = stereoSamples[i * 2 + 1];
        const int32_t absLeft = left < 0 ? -static_cast<int32_t>(left) : static_cast<int32_t>(left);
        const int32_t absRight = right < 0 ? -static_cast<int32_t>(right) : static_cast<int32_t>(right);

        // Pick the dominant channel to avoid phase cancellation and channel-wiring mismatches.
        out[captured++] = absLeft >= absRight ? left : right;
      }
    }

    metrics = analyze(out, captured);
    return true;
  }

  bool probeLevel(CaptureMetrics& metrics, uint16_t frames = 128) {
    metrics = CaptureMetrics{};
    if (!ready_) {
      return false;
    }

    if (frames == 0) {
      frames = 1;
    }
    if (frames > 256) {
      frames = 256;
    }

    setAudioEnable(false);
    if (!setClock(sampleRate_, I2S_CHANNEL_STEREO, I2S_BITS_PER_SAMPLE_16BIT)) {
      return false;
    }

    int16_t stereoSamples[256 * 2] = {0};
    const size_t bytesToRead = static_cast<size_t>(frames) * sizeof(int16_t) * 2;
    size_t bytesRead = 0;
    const esp_err_t err = i2s_read(
        port_,
        reinterpret_cast<void*>(stereoSamples),
        bytesToRead,
        &bytesRead,
        pdMS_TO_TICKS(60));
    if (err != ESP_OK || bytesRead == 0) {
      return false;
    }

    int16_t mono[256] = {0};
    const size_t framesRead = bytesRead / (sizeof(int16_t) * 2);
    if (framesRead == 0) {
      return false;
    }

    for (size_t i = 0; i < framesRead; ++i) {
      const int16_t left = stereoSamples[i * 2];
      const int16_t right = stereoSamples[i * 2 + 1];
      const int32_t absLeft = left < 0 ? -static_cast<int32_t>(left) : static_cast<int32_t>(left);
      const int32_t absRight = right < 0 ? -static_cast<int32_t>(right) : static_cast<int32_t>(right);
      mono[i] = absLeft >= absRight ? left : right;
    }

    metrics = analyze(mono, framesRead);
    return true;
  }

  bool playPcm16Blocking(const uint8_t* pcmBytes, size_t byteCount, uint32_t sampleRate) {
    if (!ready_ || !pcmBytes || byteCount == 0) {
      return false;
    }

    if ((byteCount % sizeof(int16_t)) != 0) {
      return false;
    }

    // Hosyond ES3C28P demos keep AP/EN low during playback.
    setAudioEnable(false);

    if (!setClock(sampleRate, I2S_CHANNEL_MONO, I2S_BITS_PER_SAMPLE_16BIT)) {
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
  static int16_t unpackI2SSample16(int32_t raw) {
    const int16_t hi = static_cast<int16_t>((raw >> 16) & 0xFFFF);
    const int16_t lo = static_cast<int16_t>(raw & 0xFFFF);

    const int32_t absHi = hi < 0 ? -static_cast<int32_t>(hi) : static_cast<int32_t>(hi);
    const int32_t absLo = lo < 0 ? -static_cast<int32_t>(lo) : static_cast<int32_t>(lo);
    return absHi >= absLo ? hi : lo;
  }

  void setAudioEnable(bool high) {
    digitalWrite(PIN_AUDIO_ENABLE, high ? HIGH : LOW);
    audioEnableHigh_ = high;
  }

  float probeMicPeakNormForEnable(bool enableHigh) {
    if (!ready_) {
      return 0.0f;
    }
    setAudioEnable(enableHigh);
    delay(20);

    if (!setClock(sampleRate_, I2S_CHANNEL_STEREO, I2S_BITS_PER_SAMPLE_16BIT)) {
      return 0.0f;
    }

    static const size_t kProbeFrames = 256;
    int16_t probe[kProbeFrames * 2] = {0};
    size_t bytesRead = 0;
    const esp_err_t err = i2s_read(
        port_,
        reinterpret_cast<void*>(probe),
        sizeof(probe),
        &bytesRead,
        pdMS_TO_TICKS(250));

    if (err != ESP_OK || bytesRead == 0) {
      return 0.0f;
    }

    const size_t framesRead = bytesRead / (sizeof(int16_t) * 2);
    int32_t peak = 0;
    for (size_t i = 0; i < framesRead; ++i) {
      const int16_t left = probe[i * 2];
      const int16_t right = probe[i * 2 + 1];
      const int32_t absLeft = left < 0 ? -static_cast<int32_t>(left) : static_cast<int32_t>(left);
      const int32_t absRight = right < 0 ? -static_cast<int32_t>(right) : static_cast<int32_t>(right);
      const int32_t candidate = absLeft >= absRight ? absLeft : absRight;
      if (candidate > peak) {
        peak = candidate;
      }
    }

    return static_cast<float>(peak) / 32768.0f;
  }

  void autoSelectCaptureEnableLevel() {
    const float peakHigh = probeMicPeakNormForEnable(true);
    const float peakLow = probeMicPeakNormForEnable(false);

    captureEnableHigh_ = peakHigh > peakLow;
    setAudioEnable(captureEnableHigh_);

    Serial.println(
        String("[MIC] EN probe high=") + String(peakHigh * 100.0f, 2) +
        "% low=" + String(peakLow * 100.0f, 2) +
        "% -> capture EN " + (captureEnableHigh_ ? "HIGH" : "LOW"));
  }

  bool setClock(uint32_t sampleRate, i2s_channel_t channels, i2s_bits_per_sample_t bitsPerSample) {
    if (!ready_) {
      return false;
    }
    if (sampleRate == activeSampleRate_ &&
        static_cast<int>(channels) == activeChannels_ &&
        static_cast<int>(bitsPerSample) == activeBitsPerSample_) {
      return true;
    }
    const esp_err_t err = i2s_set_clk(port_, sampleRate, bitsPerSample, channels);
    if (err != ESP_OK) {
      return false;
    }
    activeSampleRate_ = sampleRate;
    activeChannels_ = static_cast<int>(channels);
    activeBitsPerSample_ = static_cast<int>(bitsPerSample);
    return true;
  }

  bool ready_ = false;
  i2s_port_t port_ = I2S_NUM_1;
  uint32_t sampleRate_ = 16000;
  uint32_t activeSampleRate_ = 0;
  int activeChannels_ = -1;
  int activeBitsPerSample_ = -1;
  bool audioEnableHigh_ = false;
  bool captureEnableHigh_ = false;
};
