#pragma once

#include <Arduino.h>
#include <ESP_I2S.h>
#include <esp_err.h>
#include <math.h>
#include <stdint.h>

#include "board_pins.h"
#include "es8311_codec.h"

struct CaptureMetrics {
  float rmsNorm = 0.0f;
  float peakNorm = 0.0f;
  size_t samples = 0;
};

struct CaptureVadConfig {
  float startRmsNorm = 0.012f;
  float stopRmsNorm = 0.007f;
  uint16_t frameMs = 20;
  uint16_t maxWaitMs = 2200;
  uint16_t trailingSilenceMs = 900;
  uint16_t minSpeechMs = 280;
  uint16_t preRollMs = 260;
};

struct CaptureResult {
  CaptureMetrics metrics{};
  size_t sampleCount = 0;
  bool speechDetected = false;
  bool timedOutWaitingSpeech = false;
  uint32_t speechMs = 0;
};

typedef void (*CaptureProgressCallback)(
    const CaptureMetrics& frameMetrics,
    bool speechStarted,
    size_t capturedSamples,
    void* userData);

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

    i2s_.setPins(PIN_AUDIO_BCLK, PIN_AUDIO_LRCLK, PIN_AUDIO_DOUT, PIN_AUDIO_DIN, PIN_AUDIO_MCLK);
    i2s_.setTimeout(100);
    if (!i2s_.begin(I2S_MODE_STD, sampleRate_, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH)) {
      return false;
    }

    activeSampleRate_ = sampleRate_;
    activeSlotMode_ = I2S_SLOT_MODE_STEREO;
    ready_ = true;
    // ES3C28P reference demos keep AP/EN low for both mic and speaker path.
    captureEnableHigh_ = false;
    setAudioEnable(false);
    Serial.println("[MIC] EN fixed LOW for capture/playback");
    return true;
  }

  bool end() {
    if (!ready_) {
      return true;
    }
    const bool ended = i2s_.end();
    ready_ = false;
    activeSampleRate_ = 0;
    activeSlotMode_ = I2S_SLOT_MODE_MONO;
    return ended;
  }

  bool isReady() const { return ready_; }

  bool captureBlocking(int16_t* out, size_t sampleCount, CaptureMetrics& metrics) {
    if (!ready_ || !out || sampleCount == 0) {
      return false;
    }

    setAudioEnable(false);

    if (!setClock(sampleRate_, I2S_SLOT_MODE_STEREO)) {
      return false;
    }

    static const size_t kMaxFramesPerRead = 256;
    int16_t stereoSamples[kMaxFramesPerRead * 2] = {0};

    size_t captured = 0;
    while (captured < sampleCount) {
      const size_t framesRemaining = sampleCount - captured;
      const size_t framesToRead = framesRemaining < kMaxFramesPerRead ? framesRemaining : kMaxFramesPerRead;
      const size_t bytesToRead = framesToRead * sizeof(int16_t) * 2;

      const size_t bytesRead = i2s_.readBytes(reinterpret_cast<char*>(stereoSamples), bytesToRead);
      if (bytesRead == 0) {
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

  bool captureVoiceCommand(
      int16_t* out,
      size_t maxSamples,
      const CaptureVadConfig& config,
      CaptureResult& outResult,
      CaptureProgressCallback progressCallback = nullptr,
      void* progressUserData = nullptr) {
    outResult = CaptureResult{};
    if (!ready_ || !out || maxSamples == 0) {
      return false;
    }

    setAudioEnable(false);
    if (!setClock(sampleRate_, I2S_SLOT_MODE_STEREO)) {
      return false;
    }

    static const size_t kMaxFrameSamples = 640;
    static const size_t kPreRollMaxSamples = 8192;
    int16_t stereoSamples[kMaxFrameSamples * 2] = {0};
    int16_t monoFrame[kMaxFrameSamples] = {0};
    static int16_t preRollBuffer[kPreRollMaxSamples];

    uint16_t frameMs = config.frameMs;
    if (frameMs < 10) {
      frameMs = 10;
    }
    if (frameMs > 40) {
      frameMs = 40;
    }

    size_t frameSamples = (static_cast<size_t>(sampleRate_) * frameMs) / 1000;
    if (frameSamples < 80) {
      frameSamples = 80;
    }
    if (frameSamples > kMaxFrameSamples) {
      frameSamples = kMaxFrameSamples;
    }

    const uint16_t waitFrames =
        static_cast<uint16_t>((static_cast<uint32_t>(config.maxWaitMs) + frameMs - 1) / frameMs);
    uint16_t trailingSilenceFrames =
        static_cast<uint16_t>((static_cast<uint32_t>(config.trailingSilenceMs) + frameMs - 1) / frameMs);
    uint16_t minSpeechFrames =
        static_cast<uint16_t>((static_cast<uint32_t>(config.minSpeechMs) + frameMs - 1) / frameMs);
    if (trailingSilenceFrames < 1) {
      trailingSilenceFrames = 1;
    }
    if (minSpeechFrames < 1) {
      minSpeechFrames = 1;
    }

    size_t preRollSamples = (static_cast<size_t>(sampleRate_) * config.preRollMs) / 1000;
    if (preRollSamples > maxSamples / 2) {
      preRollSamples = maxSamples / 2;
    }
    if (preRollSamples > kPreRollMaxSamples) {
      preRollSamples = kPreRollMaxSamples;
    }

    size_t preRollWrite = 0;
    size_t preRollCount = 0;
    size_t captured = 0;
    bool speechStarted = false;
    uint16_t waitedFrames = 0;
    uint16_t voicedFrames = 0;
    uint16_t silenceFrames = 0;

    const float voiceRmsStart = config.startRmsNorm > 0.001f ? config.startRmsNorm : 0.001f;
    const float voiceRmsStop = config.stopRmsNorm > 0.0005f ? config.stopRmsNorm : 0.0005f;
    const float voicePeakStartBase = voiceRmsStart * 2.7f;
    const float voicePeakStopBase = voiceRmsStop * 2.7f;
    float noiseRmsEma = 0.0f;
    uint16_t noiseFrames = 0;
    float dynamicStopRms = voiceRmsStop;
    float dynamicStopPeak = voicePeakStopBase;

    while (captured < maxSamples) {
      const size_t bytesToRead = frameSamples * sizeof(int16_t) * 2;
      const size_t bytesRead = i2s_.readBytes(reinterpret_cast<char*>(stereoSamples), bytesToRead);
      if (bytesRead == 0) {
        return false;
      }

      const size_t framesRead = bytesRead / (sizeof(int16_t) * 2);
      if (framesRead == 0) {
        return false;
      }

      for (size_t i = 0; i < framesRead; ++i) {
        const int16_t left = stereoSamples[i * 2];
        const int16_t right = stereoSamples[i * 2 + 1];
        const int32_t absLeft = left < 0 ? -static_cast<int32_t>(left) : static_cast<int32_t>(left);
        const int32_t absRight = right < 0 ? -static_cast<int32_t>(right) : static_cast<int32_t>(right);
        monoFrame[i] = absLeft >= absRight ? left : right;
      }

      const CaptureMetrics frameMetrics = analyze(monoFrame, framesRead);
      if (!speechStarted) {
        if (noiseFrames == 0) {
          noiseRmsEma = frameMetrics.rmsNorm;
        } else {
          noiseRmsEma = (noiseRmsEma * 0.90f) + (frameMetrics.rmsNorm * 0.10f);
        }
        noiseFrames++;
      }

      const float adaptiveStartRms = noiseRmsEma * 2.2f;
      const float dynamicStartRms = adaptiveStartRms > voiceRmsStart ? adaptiveStartRms : voiceRmsStart;
      const float dynamicStartPeak = dynamicStartRms * 2.7f;
      const bool frameHasVoice =
          frameMetrics.rmsNorm >= dynamicStartRms ||
          frameMetrics.peakNorm >= dynamicStartPeak ||
          frameMetrics.peakNorm >= voicePeakStartBase;
      const bool frameAboveStop =
          frameMetrics.rmsNorm >= dynamicStopRms ||
          frameMetrics.peakNorm >= dynamicStopPeak;

      if (progressCallback) {
        progressCallback(frameMetrics, speechStarted, captured, progressUserData);
      }

      if (!speechStarted) {
        if (frameHasVoice) {
          speechStarted = true;
          dynamicStopRms = voiceRmsStop;
          const float adaptiveStop = noiseRmsEma * 1.35f;
          if (adaptiveStop > dynamicStopRms) {
            dynamicStopRms = adaptiveStop;
          }
          dynamicStopPeak = dynamicStopRms * 2.7f;
          if (dynamicStopPeak < voicePeakStopBase) {
            dynamicStopPeak = voicePeakStopBase;
          }
          if (preRollCount > 0) {
            const size_t start = (preRollWrite + preRollSamples - preRollCount) % preRollSamples;
            for (size_t i = 0; i < preRollCount && captured < maxSamples; ++i) {
              out[captured++] = preRollBuffer[(start + i) % preRollSamples];
            }
          }
        } else {
          if (preRollSamples > 0) {
            for (size_t i = 0; i < framesRead; ++i) {
              preRollBuffer[preRollWrite] = monoFrame[i];
              preRollWrite = (preRollWrite + 1) % preRollSamples;
              if (preRollCount < preRollSamples) {
                preRollCount++;
              }
            }
          }

          waitedFrames++;
          if (waitedFrames >= waitFrames) {
            outResult.metrics = CaptureMetrics{};
            outResult.sampleCount = 0;
            outResult.speechDetected = false;
            outResult.timedOutWaitingSpeech = true;
            outResult.speechMs = 0;
            return true;
          }
          continue;
        }
      }

      for (size_t i = 0; i < framesRead && captured < maxSamples; ++i) {
        out[captured++] = monoFrame[i];
      }

      if (progressCallback) {
        progressCallback(frameMetrics, speechStarted, captured, progressUserData);
      }

      if (frameAboveStop) {
        voicedFrames++;
        silenceFrames = 0;
      } else if (voicedFrames > 0) {
        silenceFrames++;
      }

      if (voicedFrames >= minSpeechFrames && silenceFrames >= trailingSilenceFrames) {
        break;
      }
    }

    outResult.metrics = analyze(out, captured);
    outResult.sampleCount = captured;
    outResult.speechDetected = speechStarted && captured > 0;
    outResult.timedOutWaitingSpeech = !speechStarted;
    outResult.speechMs = static_cast<uint32_t>((captured * 1000ULL) / sampleRate_);
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
    if (!setClock(sampleRate_, I2S_SLOT_MODE_STEREO)) {
      return false;
    }

    int16_t stereoSamples[256 * 2] = {0};
    const size_t bytesToRead = static_cast<size_t>(frames) * sizeof(int16_t) * 2;
    const size_t bytesRead = i2s_.readBytes(reinterpret_cast<char*>(stereoSamples), bytesToRead);
    if (bytesRead == 0) {
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

    if (!setClock(sampleRate, I2S_SLOT_MODE_MONO)) {
      return false;
    }

    size_t played = 0;
    while (played < byteCount) {
      const size_t bytesWritten = i2s_.write(reinterpret_cast<const uint8_t*>(pcmBytes + played), byteCount - played);
      if (bytesWritten == 0) {
        return false;
      }

      played += bytesWritten;
    }

    return true;
  }

  bool prepareCaptureClock() {
    if (!ready_) {
      return false;
    }
    setAudioEnable(false);
    return setClock(sampleRate_, I2S_SLOT_MODE_STEREO);
  }

  bool flushIoBuffers() {
    if (!ready_) {
      return false;
    }

    int16_t scratch[128 * 2] = {0};
    i2s_.setTimeout(6);
    for (uint8_t i = 0; i < 4; ++i) {
      const size_t bytesRead = i2s_.readBytes(reinterpret_cast<char*>(scratch), sizeof(scratch));
      if (bytesRead == 0) {
        break;
      }
    }
    i2s_.setTimeout(100);
    return true;
  }

  bool restartDriver() {
    if (!ready_) {
      return false;
    }

    i2s_.end();
    delay(8);
    i2s_.setPins(PIN_AUDIO_BCLK, PIN_AUDIO_LRCLK, PIN_AUDIO_DOUT, PIN_AUDIO_DIN, PIN_AUDIO_MCLK);
    i2s_.setTimeout(100);
    if (!i2s_.begin(I2S_MODE_STD, sampleRate_, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH)) {
      ready_ = false;
      activeSampleRate_ = 0;
      activeSlotMode_ = I2S_SLOT_MODE_MONO;
      return false;
    }

    activeSampleRate_ = sampleRate_;
    activeSlotMode_ = I2S_SLOT_MODE_STEREO;
    setAudioEnable(false);
    flushIoBuffers();
    return true;
  }

  esp_err_t readForSr(void* out, size_t len, size_t* bytesRead, uint32_t timeoutMs) {
    if (bytesRead) {
      *bytesRead = 0;
    }
    if (!ready_ || !out || len == 0 || !bytesRead) {
      return ESP_ERR_INVALID_ARG;
    }
    const uint32_t boundedTimeout = (timeoutMs == 0 || timeoutMs == UINT32_MAX) ? 100 : timeoutMs;
    i2s_.setTimeout(boundedTimeout);
    const size_t got = i2s_.readBytes(reinterpret_cast<char*>(out), len);
    *bytesRead = got;
    return got > 0 ? ESP_OK : ESP_FAIL;
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

    if (!setClock(sampleRate_, I2S_SLOT_MODE_STEREO)) {
      return 0.0f;
    }

    static const size_t kProbeFrames = 256;
    int16_t probe[kProbeFrames * 2] = {0};
    const size_t bytesRead = i2s_.readBytes(reinterpret_cast<char*>(probe), sizeof(probe));
    if (bytesRead == 0) {
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

  bool setClock(uint32_t sampleRate, i2s_slot_mode_t slotMode) {
    if (!ready_) {
      return false;
    }
    if (sampleRate == activeSampleRate_ && slotMode == activeSlotMode_) {
      return true;
    }
    if (!i2s_.configureRX(sampleRate, I2S_DATA_BIT_WIDTH_16BIT, slotMode)) {
      return false;
    }
    const int8_t slotMask = slotMode == I2S_SLOT_MODE_STEREO ? I2S_STD_SLOT_BOTH : I2S_STD_SLOT_LEFT;
    if (!i2s_.configureTX(sampleRate, I2S_DATA_BIT_WIDTH_16BIT, slotMode, slotMask)) {
      return false;
    }
    activeSampleRate_ = sampleRate;
    activeSlotMode_ = slotMode;
    return true;
  }

  I2SClass i2s_;
  bool ready_ = false;
  uint32_t sampleRate_ = 16000;
  uint32_t activeSampleRate_ = 0;
  i2s_slot_mode_t activeSlotMode_ = I2S_SLOT_MODE_MONO;
  bool audioEnableHigh_ = false;
  bool captureEnableHigh_ = false;
};
