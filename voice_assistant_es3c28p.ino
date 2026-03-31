#include "tft_espi_local_setup.h"

#include <TFT_eSPI.h>
#include <lvgl.h>

#include <ArduinoJson.h>
#include <FS.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <SD_MMC.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ctype.h>
#include <cstring>
#include <math.h>

#include "board_pins.h"
#include "es8311_codec.h"
#include "ft6336_touch.h"
#include "i2s_mic_capture.h"
#include "secrets.example.h"
#if __has_include("secrets.h")
#include "secrets.h"
#endif

enum class AssistantState {
  Booting,
  ConnectingWiFi,
  Idle,
  Listening,
  Thinking,
  Speaking,
  Error
};

static const uint16_t SCREEN_WIDTH = 320;
static const uint16_t SCREEN_HEIGHT = 240;
static const uint32_t CAPTURE_SAMPLE_RATE = 16000;
static const uint16_t CAPTURE_SECONDS = 3;
static const size_t CAPTURE_SAMPLES = CAPTURE_SAMPLE_RATE * CAPTURE_SECONDS;
static const uint16_t BOOT_LONG_PRESS_MS = 1200;
// ES3C28P mic is routed through ES8311 over I2S (not a direct ESP32 ADC pin).
// Keep ADC fallback disabled unless you wire an external analog mic.
static const int PIN_ANALOG_MIC = -1;
static const uint16_t STT_CONNECT_TIMEOUT_MS = 6000;
static const uint16_t STT_IO_TIMEOUT_MS = 12000;
static const uint8_t STT_MAX_RETRIES = 2;
static const float TTS_OUTPUT_GAIN = 1.00f;
static const size_t TTS_BUFFER_MAX_BYTES = 700 * 1024;
static const uint32_t TTS_BUFFER_IDLE_TIMEOUT_MS = 300;
static const size_t TTS_STREAM_PREFETCH_BYTES = 12 * 1024;
static const uint32_t TTS_STREAM_PREFETCH_MAX_MS = 240;
static const uint32_t TTS_STREAM_PREFETCH_IDLE_MS = 90;

static TFT_eSPI tft = TFT_eSPI(SCREEN_WIDTH, SCREEN_HEIGHT);
static lv_disp_draw_buf_t drawBuf;
static lv_color_t drawBufPixels[SCREEN_WIDTH * 20];

Ft6336Touch touch;
Es8311Codec codec;
I2SMicCapture micCapture;

struct VadSnapshot {
  float rmsNorm = 0.0f;
  float peakNorm = 0.0f;
};

AssistantState state = AssistantState::Booting;
String stateDetail = "Starting";

bool touchReady = false;
bool sdReady = false;
bool uiReady = false;
bool audioReady = false;
bool i2sMicReady = false;
bool analogMicReady = false;
bool handsFreeEnabled = false;

bool captureRequested = false;
bool wifiScanRequested = false;
bool wifiSaveRequested = false;
bool wifiConnectRequested = false;
bool assistantStopRequested = false;
bool bypassWakeWordForNextCapture = false;
uint8_t vadActiveFrames = 0;
uint32_t lastVadPollMs = 0;
uint32_t lastVadDebugMs = 0;
uint32_t handsFreeCooldownUntilMs = 0;
float lastVadRmsNorm = 0.0f;
float lastVadPeakNorm = 0.0f;
uint32_t wakeBypassUntilMs = 0;
float vadNoiseFloorNorm = 0.0f;
float vadNoiseFloorPeakNorm = 0.0f;
bool vadNoiseFloorReady = false;

bool bootPressed = false;
unsigned long bootPressStartMs = 0;

String configuredSsid;
String configuredPassword;

int16_t captureBufferStorage[CAPTURE_SAMPLES] = {0};
int16_t* captureBuffer = captureBufferStorage;
size_t captureBufferCapacity = CAPTURE_SAMPLES;
uint32_t captureIndex = 0;

lv_obj_t* uiTabview = nullptr;
lv_obj_t* uiLabelState = nullptr;
lv_obj_t* uiLabelDetail = nullptr;
lv_obj_t* uiLabelWifi = nullptr;
lv_obj_t* uiLabelMic = nullptr;
lv_obj_t* uiBtnTalk = nullptr;
lv_obj_t* uiLabelTalk = nullptr;

lv_obj_t* uiDdSsid = nullptr;
lv_obj_t* uiTaSsid = nullptr;
lv_obj_t* uiTaPassword = nullptr;
lv_obj_t* uiLabelWifiMenuStatus = nullptr;
lv_obj_t* uiKeyboard = nullptr;

String clipText(const String& in, size_t maxLen) {
  if (in.length() <= maxLen) {
    return in;
  }
  return in.substring(0, maxLen);
}

const char* stateName(AssistantState s) {
  switch (s) {
    case AssistantState::Booting:
      return "Booting";
    case AssistantState::ConnectingWiFi:
      return "Connecting Wi-Fi";
    case AssistantState::Idle:
      return "Ready";
    case AssistantState::Listening:
      return "Listening";
    case AssistantState::Thinking:
      return "Thinking";
    case AssistantState::Speaking:
      return "Speaking";
    case AssistantState::Error:
      return "Error";
  }
  return "Unknown";
}

TouchPoint mapTouchToRotation1(const TouchPoint& raw) {
  const int mappedX = raw.y;
  const int mappedY = 239 - static_cast<int>(raw.x);

  TouchPoint mapped = {
      static_cast<uint16_t>(mappedX < 0 ? 0 : (mappedX > 319 ? 319 : mappedX)),
      static_cast<uint16_t>(mappedY < 0 ? 0 : (mappedY > 239 ? 239 : mappedY)),
  };

  return mapped;
}

void uiUpdateStatus();
void buildUi();
void pollBootButton();
void pollHandsFreeTrigger();
void handleWifiScan();
void handleWifiSave();
void handleWifiConnect();
void uiShowKeyboardFor(lv_obj_t* ta);
void uiHideKeyboard();
void uiUpdateTalkButtonLabel();
bool homeAssistantTtsGetUrl(const String& text, String& outUrl, String& outError);
bool homeAssistantPlayTtsWav(const String& ttsUrl, String& outError);

void lvglFlushCb(lv_disp_drv_t* disp, const lv_area_t* area, lv_color_t* color_p);
void lvglTouchReadCb(lv_indev_drv_t* drv, lv_indev_data_t* data);

void uiTalkButtonEventCb(lv_event_t* e);
void uiWifiScanButtonEventCb(lv_event_t* e);
void uiWifiSaveButtonEventCb(lv_event_t* e);
void uiWifiConnectButtonEventCb(lv_event_t* e);
void uiWifiDropdownEventCb(lv_event_t* e);
void uiTextareaEventCb(lv_event_t* e);
void uiKeyboardEventCb(lv_event_t* e);

void setState(AssistantState nextState, const String& detail) {
  state = nextState;
  stateDetail = detail;

  Serial.print("State -> ");
  Serial.print(stateName(nextState));
  Serial.print(" | ");
  Serial.println(detail);

  uiUpdateStatus();
  uiUpdateTalkButtonLabel();
}

void uiRefreshNow() {
  if (!uiReady) {
    return;
  }
  lv_timer_handler();
  delay(5);
}

void uiSetWifiMenuStatus(const String& text) {
  if (!uiLabelWifiMenuStatus) {
    return;
  }
  lv_label_set_text(uiLabelWifiMenuStatus, clipText(text, 80).c_str());
}

void uiSetTalkEnabled(bool enabled) {
  if (!uiBtnTalk) {
    return;
  }
  if (enabled) {
    lv_obj_clear_state(uiBtnTalk, LV_STATE_DISABLED);
  } else {
    lv_obj_add_state(uiBtnTalk, LV_STATE_DISABLED);
  }
}

void uiUpdateTalkButtonLabel() {
  if (!uiLabelTalk) {
    return;
  }
  const bool busy = state != AssistantState::Idle;
  lv_label_set_text(uiLabelTalk, busy ? "Stop" : "Listen");
  lv_obj_center(uiLabelTalk);
}

void loadWifiConfig() {
  Preferences prefs;
  prefs.begin("voice_cfg", true);
  configuredSsid = prefs.getString("wifi_ssid", WIFI_SSID);
  configuredPassword = prefs.getString("wifi_pwd", WIFI_PASSWORD);
  prefs.end();
}

void saveWifiConfig(const String& ssid, const String& password) {
  Preferences prefs;
  prefs.begin("voice_cfg", false);
  prefs.putString("wifi_ssid", ssid);
  prefs.putString("wifi_pwd", password);
  prefs.end();

  configuredSsid = ssid;
  configuredPassword = password;
}

void uiSyncWifiFields() {
  if (uiTaSsid) {
    lv_textarea_set_text(uiTaSsid, configuredSsid.c_str());
  }
  if (uiTaPassword) {
    lv_textarea_set_text(uiTaPassword, configuredPassword.c_str());
  }
}

bool isHomeAssistantConfigured() {
  return strlen(HA_BASE_URL) > 0 && strlen(HA_ACCESS_TOKEN) > 0 &&
         strlen(HA_STT_PROVIDER) > 0;
}

String joinedUrl(const char* base, const String& path) {
  String url(base);
  if (url.endsWith("/")) {
    url.remove(url.length() - 1);
  }
  return url + path;
}

String baseLanguageTag(const String& language) {
  int cut = language.indexOf('-');
  if (cut < 0) {
    cut = language.indexOf('_');
  }
  if (cut <= 0) {
    return language;
  }
  return language.substring(0, cut);
}

String normalizePhraseForMatch(const String& in) {
  String out;
  out.reserve(in.length());
  bool pendingSpace = false;

  for (size_t i = 0; i < in.length(); ++i) {
    const unsigned char uc = static_cast<unsigned char>(in[i]);
    if (isalnum(uc)) {
      if (pendingSpace && out.length() > 0) {
        out += ' ';
      }
      out += static_cast<char>(tolower(uc));
      pendingSpace = false;
    } else {
      pendingSpace = true;
    }
  }

  out.trim();
  return out;
}

bool transcriptContainsToken(const String& normalizedTranscript, const String& normalizedToken) {
  if (normalizedTranscript.length() == 0 || normalizedToken.length() == 0) {
    return false;
  }
  return normalizedTranscript == normalizedToken ||
         normalizedTranscript.startsWith(normalizedToken + " ") ||
         normalizedTranscript.endsWith(" " + normalizedToken) ||
         normalizedTranscript.indexOf(" " + normalizedToken + " ") >= 0;
}

bool transcriptStartsWithToken(const String& normalizedTranscript, const String& normalizedToken) {
  if (normalizedTranscript.length() == 0 || normalizedToken.length() == 0) {
    return false;
  }
  return normalizedTranscript == normalizedToken ||
         normalizedTranscript.startsWith(normalizedToken + " ");
}

bool transcriptHasWakePhrase(const String& transcript, String& outMatchedPhrase) {
  const String normalizedTranscript = normalizePhraseForMatch(transcript);
  if (normalizedTranscript.length() == 0) {
    return false;
  }

  String configured = String(HA_WAKE_PHRASE);
  configured.replace('|', ',');

  int start = 0;
  while (start <= configured.length()) {
    const int sep = configured.indexOf(',', start);
    String token = sep >= 0 ? configured.substring(start, sep) : configured.substring(start);
    token.trim();

    const String normalizedToken = normalizePhraseForMatch(token);
    if (normalizedToken.length() > 0) {
      if (transcriptContainsToken(normalizedTranscript, normalizedToken)) {
        outMatchedPhrase = token;
        return true;
      }

      // When VAD wakes us mid-phrase, the first word can be clipped.
      const int lastSpace = normalizedToken.lastIndexOf(' ');
      if (lastSpace > 0 && lastSpace < (normalizedToken.length() - 1)) {
        const String tailWord = normalizedToken.substring(lastSpace + 1);
        if (tailWord.length() >= 3 && transcriptStartsWithToken(normalizedTranscript, tailWord)) {
          outMatchedPhrase = token;
          return true;
        }
      }
    }

    if (sep < 0) {
      break;
    }
    start = sep + 1;
  }

  return false;
}

String urlAuthority(const String& url) {
  int start = 0;
  const int scheme = url.indexOf("://");
  if (scheme >= 0) {
    start = scheme + 3;
  }
  const int slash = url.indexOf('/', start);
  if (slash < 0) {
    return url.substring(start);
  }
  return url.substring(start, slash);
}

String urlHost(const String& url) {
  String auth = urlAuthority(url);
  auth.trim();
  const int at = auth.indexOf('@');
  if (at >= 0) {
    auth = auth.substring(at + 1);
  }
  const int colon = auth.indexOf(':');
  if (colon >= 0) {
    return auth.substring(0, colon);
  }
  return auth;
}

uint16_t urlPort(const String& url) {
  String auth = urlAuthority(url);
  auth.trim();
  const int at = auth.indexOf('@');
  if (at >= 0) {
    auth = auth.substring(at + 1);
  }
  const int colon = auth.lastIndexOf(':');
  if (colon >= 0 && colon < (auth.length() - 1)) {
    const int parsed = auth.substring(colon + 1).toInt();
    if (parsed > 0 && parsed <= 65535) {
      return static_cast<uint16_t>(parsed);
    }
  }
  return url.startsWith("https://") ? 443 : 80;
}

VadSnapshot sampleAnalogVadSnapshot() {
  VadSnapshot out{};
  if (!analogMicReady) {
    return out;
  }

  static const int kSamples = 96;
  int32_t values[kSamples] = {0};
  int64_t sum = 0;

  analogReadResolution(12);
#ifdef ADC_0db
  analogSetPinAttenuation(PIN_ANALOG_MIC, ADC_0db);
#endif

  for (int i = 0; i < kSamples; ++i) {
    int v = analogRead(PIN_ANALOG_MIC);
    if (v < 0) {
      v = 0;
    }
    if (v > 4095) {
      v = 4095;
    }
    values[i] = v;
    sum += v;
  }

  const float mean = static_cast<float>(sum) / static_cast<float>(kSamples);
  double sumSq = 0.0;
  float peakAbs = 0.0f;
  for (int i = 0; i < kSamples; ++i) {
    const float centered = static_cast<float>(values[i]) - mean;
    const float absCentered = centered < 0.0f ? -centered : centered;
    if (absCentered > peakAbs) {
      peakAbs = absCentered;
    }
    sumSq += static_cast<double>(centered) * static_cast<double>(centered);
  }

  const float rms = sqrtf(static_cast<float>(sumSq / static_cast<double>(kSamples)));
  out.rmsNorm = rms / 2048.0f;
  out.peakNorm = peakAbs / 2048.0f;
  return out;
}

VadSnapshot sampleVadSnapshot() {
  VadSnapshot out{};

  if (analogMicReady) {
    const VadSnapshot analog = sampleAnalogVadSnapshot();
    if (analog.rmsNorm > out.rmsNorm) {
      out.rmsNorm = analog.rmsNorm;
    }
    if (analog.peakNorm > out.peakNorm) {
      out.peakNorm = analog.peakNorm;
    }
  }

  if (i2sMicReady) {
    CaptureMetrics probe{};
    if (micCapture.probeLevel(probe, 96)) {
      if (probe.rmsNorm > out.rmsNorm) {
        out.rmsNorm = probe.rmsNorm;
      }
      if (probe.peakNorm > out.peakNorm) {
        out.peakNorm = probe.peakNorm;
      }
    }
  }

  return out;
}

int16_t clampToInt16(int32_t v) {
  if (v > 32767) {
    return 32767;
  }
  if (v < -32768) {
    return -32768;
  }
  return static_cast<int16_t>(v);
}

float chooseCaptureGain(float peakNorm) {
  if (peakNorm < 0.0005f || peakNorm >= 0.12f) {
    return 1.0f;
  }

  const float targetPeak = 0.30f;
  float gain = targetPeak / peakNorm;
  if (gain < 1.0f) {
    gain = 1.0f;
  }
  if (gain > 12.0f) {
    gain = 12.0f;
  }
  return gain;
}

void applyPcmGain(int16_t* samples, size_t count, float gain) {
  if (!samples || count == 0 || gain <= 1.001f) {
    return;
  }

  for (size_t i = 0; i < count; ++i) {
    const float scaled = static_cast<float>(samples[i]) * gain;
    const int32_t rounded = static_cast<int32_t>(scaled > 0 ? (scaled + 0.5f) : (scaled - 0.5f));
    samples[i] = clampToInt16(rounded);
  }
}

String buildLevelText(const CaptureMetrics& metrics, float captureGain) {
  const float rmsDb = 20.0f * log10f(metrics.rmsNorm > 0.000001f ? metrics.rmsNorm : 0.000001f);
  String text = String("RMS ") + String(rmsDb, 1) + " dBFS";
  text += String("  P") + String(metrics.peakNorm * 100.0f, 1) + "%";
  if (captureGain > 1.001f) {
    text += String("  x") + String(captureGain, 1);
  }
  return text;
}

float normToDbFs(float norm) {
  const float safe = norm > 0.000001f ? norm : 0.000001f;
  return 20.0f * log10f(safe);
}

bool trimPcmSilenceInPlace(int16_t* samples, size_t& sampleCount, size_t& removedSamples) {
  removedSamples = 0;
  if (!samples || sampleCount < (CAPTURE_SAMPLE_RATE / 4)) {
    return false;
  }

  int32_t peakAbs = 0;
  for (size_t i = 0; i < sampleCount; ++i) {
    const int32_t v = samples[i];
    const int32_t a = v < 0 ? -v : v;
    if (a > peakAbs) {
      peakAbs = a;
    }
  }
  if (peakAbs < 180) {
    return false;
  }

  int32_t gate = peakAbs / 14;
  if (gate < 120) {
    gate = 120;
  }
  if (gate > 2200) {
    gate = 2200;
  }

  size_t start = 0;
  while (start < sampleCount) {
    const int32_t v = samples[start];
    const int32_t a = v < 0 ? -v : v;
    if (a >= gate) {
      break;
    }
    start++;
  }

  size_t end = sampleCount;
  while (end > start) {
    const int32_t v = samples[end - 1];
    const int32_t a = v < 0 ? -v : v;
    if (a >= gate) {
      break;
    }
    end--;
  }

  if (start >= end) {
    return false;
  }

  const size_t preRoll = CAPTURE_SAMPLE_RATE / 10;   // 100ms
  const size_t postRoll = CAPTURE_SAMPLE_RATE / 8;   // 125ms
  start = start > preRoll ? (start - preRoll) : 0;
  end = (end + postRoll) < sampleCount ? (end + postRoll) : sampleCount;

  const size_t newCount = end > start ? (end - start) : 0;
  if (newCount < (CAPTURE_SAMPLE_RATE / 5)) {
    return false;
  }

  if (start > 0) {
    memmove(samples, samples + start, newCount * sizeof(int16_t));
  }
  removedSamples = sampleCount - newCount;
  sampleCount = newCount;
  return true;
}

void uiUpdateMicLevelLine() {
  if (!uiLabelMic || state != AssistantState::Idle) {
    return;
  }

  static uint32_t lastMicUiMs = 0;
  const uint32_t now = millis();
  if ((now - lastMicUiMs) < 450) {
    return;
  }
  lastMicUiMs = now;

  String line = "Mic ";
  bool hasLine = false;

  if (i2sMicReady) {
    CaptureMetrics probe{};
    if (micCapture.probeLevel(probe, 64)) {
      line += String("I2S ") + String(normToDbFs(probe.rmsNorm), 1) +
              " dB  P" + String(probe.peakNorm * 100.0f, 1) + "%";
      hasLine = true;
    }
  }

  if (analogMicReady) {
    VadSnapshot a = sampleAnalogVadSnapshot();
    if (hasLine) {
      line += " | ";
    }
    line += String("ADC ") + String(normToDbFs(a.rmsNorm), 1) +
            " dB  P" + String(a.peakNorm * 100.0f, 1) + "%";
    hasLine = true;
  }

  if (!hasLine) {
    line += "n/a";
  }

  lv_label_set_text(uiLabelMic, clipText(line, 90).c_str());

  static uint32_t lastMicSerialMs = 0;
  if ((now - lastMicSerialMs) > 2000) {
    lastMicSerialMs = now;
    Serial.println(String("[MICDBG] ") + line);
  }
}

bool probeAnalogMicPin(int pin) {
  if (pin < 0) {
    return false;
  }

  pinMode(pin, INPUT);
  analogReadResolution(12);
#ifdef ADC_0db
  analogSetPinAttenuation(pin, ADC_0db);
#endif

  int minV = 4095;
  int maxV = 0;
  for (int i = 0; i < 600; ++i) {
    const int v = analogRead(pin);
    if (v < minV) {
      minV = v;
    }
    if (v > maxV) {
      maxV = v;
    }
    delayMicroseconds(150);
  }

  const int span = maxV - minV;
  Serial.println(String("[ADC] probe pin ") + pin + " min=" + minV + " max=" + maxV + " span=" + span);
  return span >= 8;
}

bool postProcessAnalogCapture(int16_t* out, size_t sampleCount, CaptureMetrics& metrics) {
  if (!out || sampleCount == 0) {
    return false;
  }

  int64_t rawSum = 0;
  for (size_t i = 0; i < sampleCount; ++i) {
    rawSum += out[i];
  }

  const int32_t rawMean = static_cast<int32_t>(rawSum / static_cast<int64_t>(sampleCount));
  float prevIn = 0.0f;
  float prevOut = 0.0f;

  for (size_t i = 0; i < sampleCount; ++i) {
    const float centered = static_cast<float>(static_cast<int32_t>(out[i]) - rawMean);
    // Gentle DC blocker to keep speech shape while removing offset drift.
    const float hp = centered - prevIn + 0.995f * prevOut;
    prevIn = centered;
    prevOut = hp;
    const int32_t pcm = static_cast<int32_t>(hp * 14.0f);
    out[i] = clampToInt16(pcm);
  }

  CaptureMetrics preMetrics = I2SMicCapture::analyze(out, sampleCount);
  float agcGain = 1.0f;
  if (preMetrics.rmsNorm < 0.022f) {
    agcGain = 0.022f / (preMetrics.rmsNorm > 0.0004f ? preMetrics.rmsNorm : 0.0004f);
  }
  if (preMetrics.peakNorm > 0.0f) {
    const float peakGuard = 0.45f / preMetrics.peakNorm;
    if (agcGain > peakGuard) {
      agcGain = peakGuard;
    }
  }
  if (agcGain > 10.0f) {
    agcGain = 10.0f;
  }
  if (agcGain > 1.02f) {
    applyPcmGain(out, sampleCount, agcGain);
  }

  metrics = I2SMicCapture::analyze(out, sampleCount);
  Serial.println(
      String("[ADC] n=") + sampleCount +
      String(" mean=") + rawMean +
      String(" preRMS=") + String(preMetrics.rmsNorm * 100.0f, 2) + "%" +
      String(" preP=") + String(preMetrics.peakNorm * 100.0f, 2) + "%" +
      String(" agc=") + String(agcGain, 2));
  return true;
}

bool captureAnalogMicBlocking(int16_t* out, size_t sampleCount, CaptureMetrics& metrics) {
  if (!out || sampleCount == 0) {
    return false;
  }

  analogReadResolution(12);
#ifdef ADC_0db
  analogSetPinAttenuation(PIN_ANALOG_MIC, ADC_0db);
#endif
  pinMode(PIN_ANALOG_MIC, INPUT);

  const uint32_t intervalUs = 1000000UL / CAPTURE_SAMPLE_RATE;
  uint32_t nextUs = micros();

  for (size_t i = 0; i < sampleCount; ++i) {
    if (assistantStopRequested) {
      return false;
    }
    while (static_cast<int32_t>(micros() - nextUs) < 0) {
      delayMicroseconds(5);
    }
    nextUs += intervalUs;

    int raw = analogRead(PIN_ANALOG_MIC);
    if (raw < 0) {
      raw = 0;
    }
    if (raw > 4095) {
      raw = 4095;
    }
    out[i] = static_cast<int16_t>(raw);
  }

  return postProcessAnalogCapture(out, sampleCount, metrics);
}

bool captureI2SMicUntilStop(
    int16_t* out,
    size_t capacitySamples,
    size_t& outSamples,
    CaptureMetrics& metrics) {
  outSamples = 0;
  if (!out || capacitySamples == 0 || !i2sMicReady) {
    return false;
  }

  const size_t chunkSamples = CAPTURE_SAMPLE_RATE / 5;  // ~200ms chunks for responsive stop.
  const uint32_t startMs = millis();
  const uint32_t maxMs = 8000;

  while (outSamples < capacitySamples) {
    if (assistantStopRequested) {
      break;
    }
    if ((millis() - startMs) > maxMs) {
      break;
    }

    const size_t remaining = capacitySamples - outSamples;
    const size_t toCapture = remaining < chunkSamples ? remaining : chunkSamples;
    CaptureMetrics chunkMetrics{};
    if (!micCapture.captureBlocking(out + outSamples, toCapture, chunkMetrics)) {
      if (outSamples == 0) {
        return false;
      }
      break;
    }
    outSamples += toCapture;

    lv_timer_handler();
    delay(1);
  }

  const size_t minSamples = CAPTURE_SAMPLE_RATE / 3;  // ~330ms
  const bool stopWasPressed = assistantStopRequested;
  assistantStopRequested = false;
  if (outSamples < minSamples) {
    return false;
  }

  metrics = I2SMicCapture::analyze(out, outSamples);
  if (stopWasPressed) {
    Serial.println(String("[MIC] manual stop after ") + outSamples + " samples");
  }
  return true;
}

bool captureAnalogMicUntilStop(
    int16_t* out,
    size_t capacitySamples,
    size_t& outSamples,
    CaptureMetrics& metrics) {
  outSamples = 0;
  if (!out || capacitySamples == 0) {
    return false;
  }

  analogReadResolution(12);
#ifdef ADC_0db
  analogSetPinAttenuation(PIN_ANALOG_MIC, ADC_0db);
#endif
  pinMode(PIN_ANALOG_MIC, INPUT);

  const uint32_t intervalUs = 1000000UL / CAPTURE_SAMPLE_RATE;
  uint32_t nextUs = micros();
  const uint32_t startMs = millis();
  const uint32_t maxMs = 8000;

  while (outSamples < capacitySamples) {
    if (assistantStopRequested) {
      break;
    }
    if ((millis() - startMs) > maxMs) {
      break;
    }

    while (static_cast<int32_t>(micros() - nextUs) < 0) {
      lv_timer_handler();
      if (assistantStopRequested) {
        break;
      }
      delayMicroseconds(5);
    }
    if (assistantStopRequested) {
      break;
    }
    nextUs += intervalUs;

    int raw = analogRead(PIN_ANALOG_MIC);
    if (raw < 0) {
      raw = 0;
    }
    if (raw > 4095) {
      raw = 4095;
    }
    out[outSamples++] = static_cast<int16_t>(raw);

    if ((outSamples & 0x7F) == 0) {
      lv_timer_handler();
    }
  }

  const size_t minSamples = CAPTURE_SAMPLE_RATE / 3;  // ~330ms
  const bool stopWasPressed = assistantStopRequested;
  assistantStopRequested = false;
  if (outSamples < minSamples) {
    return false;
  }

  const bool ok = postProcessAnalogCapture(out, outSamples, metrics);
  if (stopWasPressed) {
    Serial.println(String("[MIC] manual stop after ") + outSamples + " samples");
  }
  return ok;
}

uint16_t readLe16(const uint8_t* in) {
  return static_cast<uint16_t>(in[0]) | (static_cast<uint16_t>(in[1]) << 8);
}

uint32_t readLe32(const uint8_t* in) {
  return static_cast<uint32_t>(in[0]) |
         (static_cast<uint32_t>(in[1]) << 8) |
         (static_cast<uint32_t>(in[2]) << 16) |
         (static_cast<uint32_t>(in[3]) << 24);
}

int16_t applyPcmGain(int16_t sample, float gain) {
  const int32_t scaled = static_cast<int32_t>(sample * gain);
  if (scaled > 32767) {
    return 32767;
  }
  if (scaled < -32768) {
    return -32768;
  }
  return static_cast<int16_t>(scaled);
}

void applyPcmGainBuffer(int16_t* samples, size_t count, float gain) {
  if (!samples || count == 0 || gain >= 0.999f) {
    return;
  }
  for (size_t i = 0; i < count; ++i) {
    samples[i] = applyPcmGain(samples[i], gain);
  }
}

uint8_t* allocAudioBuffer(size_t bytes) {
  if (bytes == 0) {
    return nullptr;
  }
  uint8_t* ptr = nullptr;
  if (psramFound()) {
    ptr = static_cast<uint8_t*>(ps_malloc(bytes));
  }
  if (!ptr) {
    ptr = static_cast<uint8_t*>(malloc(bytes));
  }
  return ptr;
}

bool readExactFromStream(Stream& stream, uint8_t* out, size_t len, uint32_t timeoutMs) {
  if (!out || len == 0) {
    return len == 0;
  }

  size_t total = 0;
  unsigned long lastProgress = millis();
  while (total < len) {
    const size_t n = stream.readBytes(reinterpret_cast<char*>(out + total), len - total);
    if (n > 0) {
      total += n;
      lastProgress = millis();
      continue;
    }
    if ((millis() - lastProgress) > timeoutMs) {
      return false;
    }
    delay(1);
  }
  return true;
}

bool discardStreamBytes(Stream& stream, size_t len, uint32_t timeoutMs) {
  if (len == 0) {
    return true;
  }

  uint8_t scratch[128];
  size_t remaining = len;
  while (remaining > 0) {
    const size_t chunk = remaining < sizeof(scratch) ? remaining : sizeof(scratch);
    if (!readExactFromStream(stream, scratch, chunk, timeoutMs)) {
      return false;
    }
    remaining -= chunk;
  }
  return true;
}

bool streamWavDataToSpeaker(
    Stream& stream,
    uint32_t dataBytes,
    uint16_t channels,
    uint32_t sampleRate,
    String& outError) {
  if (channels != 1 && channels != 2) {
    outError = "WAV channels unsupported";
    return false;
  }

  // Some HA TTS providers emit WAV with data chunk size set to 0
  // even when PCM bytes follow. Treat size=0 as "read until idle".
  const bool unknownDataLength = (dataBytes == 0);
  const uint32_t playbackRate = CAPTURE_SAMPLE_RATE;
  if (sampleRate != playbackRate) {
    Serial.println(
        String("[TTS] resample-sw ") + sampleRate +
        " Hz -> " + playbackRate + " Hz");
  }

  const size_t frameBytes = channels * sizeof(int16_t);
  static const size_t kChunkBytes = 2048;
  static int16_t inSamples[kChunkBytes / sizeof(int16_t)] = {0};
  static int16_t monoSamples[1024] = {0};

  uint32_t remaining = dataBytes;
  uint32_t lastProgressMs = millis();
  uint32_t lastUiYieldMs = millis();
  size_t totalStreamed = 0;
  uint32_t resamplePhase = 0;
  size_t resampleOutFrames = 0;

  while (unknownDataLength || remaining > 0) {
    size_t toRead = unknownDataLength ? kChunkBytes : (remaining < kChunkBytes ? remaining : kChunkBytes);
    toRead -= (toRead % frameBytes);
    if (toRead == 0) {
      if (unknownDataLength) {
        break;
      }
      outError = "WAV frame alignment error";
      return false;
    }

    size_t readBytes = 0;
    if (unknownDataLength) {
      readBytes = stream.readBytes(reinterpret_cast<char*>(inSamples), toRead);
      if (readBytes == 0) {
        if ((millis() - lastProgressMs) > 1200) {
          break;
        }
        delay(2);
        continue;
      }
    } else {
      if (!readExactFromStream(stream, reinterpret_cast<uint8_t*>(inSamples), toRead, 12000)) {
        outError = "WAV stream timeout";
        return false;
      }
      readBytes = toRead;
    }

    lastProgressMs = millis();
    totalStreamed += readBytes;

    const size_t playableBytes = readBytes - (readBytes % frameBytes);
    if (playableBytes == 0) {
      continue;
    }

    if (sampleRate == playbackRate && channels == 1) {
      const size_t sampleCount = playableBytes / sizeof(int16_t);
      applyPcmGainBuffer(inSamples, sampleCount, TTS_OUTPUT_GAIN);
      if (!micCapture.playPcm16Blocking(
              reinterpret_cast<const uint8_t*>(inSamples), playableBytes, playbackRate)) {
        outError = "I2S mono playback failed";
        return false;
      }
    } else if (sampleRate == playbackRate && channels == 2) {
      const size_t frameCount = playableBytes / (sizeof(int16_t) * 2);
      for (size_t i = 0; i < frameCount && i < (sizeof(monoSamples) / sizeof(monoSamples[0])); ++i) {
        const int32_t mix = static_cast<int32_t>(inSamples[i * 2]) +
                            static_cast<int32_t>(inSamples[i * 2 + 1]);
        monoSamples[i] = static_cast<int16_t>(mix / 2);
      }
      applyPcmGainBuffer(monoSamples, frameCount, TTS_OUTPUT_GAIN);

      if (!micCapture.playPcm16Blocking(
              reinterpret_cast<const uint8_t*>(monoSamples),
              frameCount * sizeof(int16_t),
              playbackRate)) {
        outError = "I2S stereo downmix failed";
        return false;
      }
    } else {
      // Nearest-neighbor resampler to fixed 16k codec path.
      const size_t inFrames = playableBytes / frameBytes;

      for (size_t i = 0; i < inFrames; ++i) {
        int16_t mono = 0;
        if (channels == 1) {
          mono = inSamples[i];
        } else {
          const int32_t mix = static_cast<int32_t>(inSamples[i * 2]) +
                              static_cast<int32_t>(inSamples[i * 2 + 1]);
          mono = static_cast<int16_t>(mix / 2);
        }

        resamplePhase += playbackRate;
        while (resamplePhase >= sampleRate) {
          if (resampleOutFrames >= (sizeof(monoSamples) / sizeof(monoSamples[0]))) {
            if (!micCapture.playPcm16Blocking(
                    reinterpret_cast<const uint8_t*>(monoSamples),
                    resampleOutFrames * sizeof(int16_t),
                    playbackRate)) {
              outError = "I2S resample playback failed";
              return false;
            }
            resampleOutFrames = 0;
          }
          monoSamples[resampleOutFrames++] = applyPcmGain(mono, TTS_OUTPUT_GAIN);
          resamplePhase -= sampleRate;
        }
      }
    }

    if (!unknownDataLength) {
      remaining -= static_cast<uint32_t>(readBytes);
    }
    if ((millis() - lastUiYieldMs) > 180) {
      lv_timer_handler();
      lastUiYieldMs = millis();
    }
  }

  if (resampleOutFrames > 0) {
    if (!micCapture.playPcm16Blocking(
            reinterpret_cast<const uint8_t*>(monoSamples),
            resampleOutFrames * sizeof(int16_t),
            playbackRate)) {
      outError = "I2S resample playback failed";
      return false;
    }
  }

  if (totalStreamed == 0) {
    outError = "WAV has no data";
    return false;
  }

  return true;
}

bool playWavFromStream(Stream& stream, String& outError) {
  uint8_t riff[12] = {0};
  if (!readExactFromStream(stream, riff, sizeof(riff), 8000)) {
    outError = "WAV header timeout";
    return false;
  }

  if (memcmp(riff + 0, "RIFF", 4) != 0 || memcmp(riff + 8, "WAVE", 4) != 0) {
    String headerHex;
    for (size_t i = 0; i < sizeof(riff); ++i) {
      if (i > 0) {
        headerHex += ' ';
      }
      if (riff[i] < 0x10) {
        headerHex += '0';
      }
      headerHex += String(riff[i], HEX);
    }
    outError = String("Not a RIFF/WAVE file: ") + headerHex;
    return false;
  }

  bool fmtSeen = false;
  uint16_t wavFormat = 0;
  uint16_t wavChannels = 0;
  uint16_t wavBitsPerSample = 0;
  uint32_t wavSampleRate = 0;

  while (true) {
    uint8_t chunkHeader[8] = {0};
    if (!readExactFromStream(stream, chunkHeader, sizeof(chunkHeader), 8000)) {
      outError = "WAV chunk timeout";
      return false;
    }

    const uint32_t chunkSize = readLe32(chunkHeader + 4);

    if (memcmp(chunkHeader, "fmt ", 4) == 0) {
      if (chunkSize < 16) {
        outError = "WAV fmt too short";
        return false;
      }

      uint8_t fmt[16] = {0};
      if (!readExactFromStream(stream, fmt, sizeof(fmt), 8000)) {
        outError = "WAV fmt read failed";
        return false;
      }

      wavFormat = readLe16(fmt + 0);
      wavChannels = readLe16(fmt + 2);
      wavSampleRate = readLe32(fmt + 4);
      wavBitsPerSample = readLe16(fmt + 14);
      fmtSeen = true;

      const uint32_t extraFmtBytes = chunkSize - 16;
      if (extraFmtBytes > 0 &&
          !discardStreamBytes(stream, extraFmtBytes, 8000)) {
        outError = "WAV fmt extra timeout";
        return false;
      }
      if ((chunkSize & 1) != 0 && !discardStreamBytes(stream, 1, 2000)) {
        outError = "WAV fmt pad timeout";
        return false;
      }
      continue;
    }

    if (memcmp(chunkHeader, "data", 4) == 0) {
      if (!fmtSeen) {
        outError = "WAV fmt missing";
        return false;
      }
      if (wavFormat != 1) {
        outError = "WAV format unsupported";
        return false;
      }
      if (wavBitsPerSample != 16 || (wavChannels != 1 && wavChannels != 2) ||
          wavSampleRate == 0) {
        outError = "WAV format not PCM16 mono/stereo";
        return false;
      }

      if (!streamWavDataToSpeaker(stream, chunkSize, wavChannels, wavSampleRate, outError)) {
        return false;
      }
      if ((chunkSize & 1) != 0) {
        discardStreamBytes(stream, 1, 2000);
      }
      return true;
    }

    const uint32_t bytesToSkip = chunkSize + ((chunkSize & 1) ? 1 : 0);
    if (!discardStreamBytes(stream, bytesToSkip, 8000)) {
      outError = "WAV chunk skip timeout";
      return false;
    }
  }
}

void writeWavHeader(uint8_t* header, size_t dataBytes, uint32_t sampleRate) {
  const uint16_t audioFormatPcm = 1;
  const uint16_t channels = 1;
  const uint16_t bitsPerSample = 16;
  const uint32_t byteRate = sampleRate * channels * (bitsPerSample / 8);
  const uint16_t blockAlign = channels * (bitsPerSample / 8);
  const uint32_t riffChunkSize = 36 + dataBytes;

  memcpy(header + 0, "RIFF", 4);
  memcpy(header + 4, &riffChunkSize, 4);
  memcpy(header + 8, "WAVE", 4);
  memcpy(header + 12, "fmt ", 4);

  const uint32_t fmtChunkSize = 16;
  memcpy(header + 16, &fmtChunkSize, 4);
  memcpy(header + 20, &audioFormatPcm, 2);
  memcpy(header + 22, &channels, 2);
  memcpy(header + 24, &sampleRate, 4);
  memcpy(header + 28, &byteRate, 4);
  memcpy(header + 32, &blockAlign, 2);
  memcpy(header + 34, &bitsPerSample, 2);

  memcpy(header + 36, "data", 4);
  memcpy(header + 40, &dataBytes, 4);
}

class WavUploadStream : public Stream {
 public:
  WavUploadStream(const int16_t* samples, size_t sampleCount, uint32_t sampleRate)
      : pcmBytes_(reinterpret_cast<const uint8_t*>(samples)),
        pcmSizeBytes_(sampleCount * sizeof(int16_t)),
        cursor_(0) {
    writeWavHeader(header_, pcmSizeBytes_, sampleRate);
  }

  size_t totalSize() const {
    return sizeof(header_) + pcmSizeBytes_;
  }

  int available() override {
    const size_t remaining = totalSize() - cursor_;
    return remaining > 0x7FFFFFFF ? 0x7FFFFFFF : static_cast<int>(remaining);
  }

  int peek() override {
    if (cursor_ >= totalSize()) {
      return -1;
    }
    return byteAt(cursor_);
  }

  int read() override {
    if (cursor_ >= totalSize()) {
      return -1;
    }
    return byteAt(cursor_++);
  }

  size_t readBytes(char* buffer, size_t length) override {
    if (!buffer || length == 0 || cursor_ >= totalSize()) {
      return 0;
    }

    const size_t remaining = totalSize() - cursor_;
    const size_t toCopy = length < remaining ? length : remaining;
    size_t copied = 0;

    while (copied < toCopy) {
      if (cursor_ < sizeof(header_)) {
        const size_t headerRemain = sizeof(header_) - cursor_;
        const size_t chunk = (toCopy - copied) < headerRemain ? (toCopy - copied) : headerRemain;
        memcpy(buffer + copied, header_ + cursor_, chunk);
        cursor_ += chunk;
        copied += chunk;
      } else {
        const size_t pcmOffset = cursor_ - sizeof(header_);
        const size_t pcmRemain = pcmSizeBytes_ - pcmOffset;
        const size_t chunk = (toCopy - copied) < pcmRemain ? (toCopy - copied) : pcmRemain;
        memcpy(buffer + copied, pcmBytes_ + pcmOffset, chunk);
        cursor_ += chunk;
        copied += chunk;
      }
    }

    return copied;
  }

  size_t write(uint8_t b) override {
    (void)b;
    return 0;
  }

 private:
  uint8_t byteAt(size_t idx) const {
    if (idx < sizeof(header_)) {
      return header_[idx];
    }
    return pcmBytes_[idx - sizeof(header_)];
  }

  uint8_t header_[44] = {0};
  const uint8_t* pcmBytes_;
  size_t pcmSizeBytes_;
  size_t cursor_;
};

class ByteArrayReadStream : public Stream {
 public:
  ByteArrayReadStream(const uint8_t* data, size_t len) : data_(data), len_(len), pos_(0) {}

  int available() override {
    if (pos_ >= len_) {
      return 0;
    }
    const size_t remaining = len_ - pos_;
    return remaining > 0x7FFFFFFF ? 0x7FFFFFFF : static_cast<int>(remaining);
  }

  int peek() override {
    if (pos_ >= len_) {
      return -1;
    }
    return data_[pos_];
  }

  int read() override {
    if (pos_ >= len_) {
      return -1;
    }
    return data_[pos_++];
  }

  size_t readBytes(char* buffer, size_t length) override {
    if (!buffer || length == 0 || pos_ >= len_) {
      return 0;
    }
    const size_t remaining = len_ - pos_;
    const size_t toCopy = length < remaining ? length : remaining;
    memcpy(buffer, data_ + pos_, toCopy);
    pos_ += toCopy;
    return toCopy;
  }

  size_t write(uint8_t b) override {
    (void)b;
    return 0;
  }

 private:
  const uint8_t* data_;
  size_t len_;
  size_t pos_;
};

class PrefixedReadStream : public Stream {
 public:
  PrefixedReadStream(const uint8_t* prefix, size_t prefixLen, Stream& base)
      : prefix_(prefix), prefixLen_(prefixLen), prefixPos_(0), base_(base) {}

  int available() override {
    const size_t preRemaining = prefixPos_ < prefixLen_ ? (prefixLen_ - prefixPos_) : 0;
    const int baseAvailable = base_.available();
    const size_t total = preRemaining + (baseAvailable > 0 ? static_cast<size_t>(baseAvailable) : 0);
    return total > 0x7FFFFFFF ? 0x7FFFFFFF : static_cast<int>(total);
  }

  int peek() override {
    if (prefixPos_ < prefixLen_) {
      return prefix_[prefixPos_];
    }
    return base_.peek();
  }

  int read() override {
    if (prefixPos_ < prefixLen_) {
      return prefix_[prefixPos_++];
    }
    return base_.read();
  }

  size_t readBytes(char* buffer, size_t length) override {
    if (!buffer || length == 0) {
      return 0;
    }

    size_t copied = 0;
    if (prefixPos_ < prefixLen_) {
      const size_t preRemaining = prefixLen_ - prefixPos_;
      const size_t take = length < preRemaining ? length : preRemaining;
      memcpy(buffer, prefix_ + prefixPos_, take);
      prefixPos_ += take;
      copied += take;
    }

    if (copied < length) {
      copied += base_.readBytes(buffer + copied, length - copied);
    }

    return copied;
  }

  size_t write(uint8_t b) override {
    (void)b;
    return 0;
  }

 private:
  const uint8_t* prefix_;
  size_t prefixLen_;
  size_t prefixPos_;
  Stream& base_;
};

bool beginHttp(HTTPClient& http, WiFiClientSecure& secureClient, const String& url) {
  if (url.startsWith("https://")) {
    secureClient.setInsecure();
    secureClient.setTimeout(STT_IO_TIMEOUT_MS);
    return http.begin(secureClient, url);
  }
  return http.begin(url);
}

bool isRetryableSttTransportError(int code) {
  return code == HTTPC_ERROR_CONNECTION_REFUSED ||
         code == HTTPC_ERROR_SEND_HEADER_FAILED ||
         code == HTTPC_ERROR_SEND_PAYLOAD_FAILED ||
         code == HTTPC_ERROR_NOT_CONNECTED ||
         code == HTTPC_ERROR_CONNECTION_LOST ||
         code == HTTPC_ERROR_READ_TIMEOUT;
}

String sttHttpErrorText(int code) {
  switch (code) {
    case HTTPC_ERROR_CONNECTION_REFUSED:
      return "connection refused";
    case HTTPC_ERROR_SEND_HEADER_FAILED:
      return "send header failed";
    case HTTPC_ERROR_SEND_PAYLOAD_FAILED:
      return "send payload failed";
    case HTTPC_ERROR_NOT_CONNECTED:
      return "not connected";
    case HTTPC_ERROR_CONNECTION_LOST:
      return "connection lost";
    case HTTPC_ERROR_READ_TIMEOUT:
      return "read timeout";
    default:
      return "";
  }
}

bool initSdCard() {
  SD_MMC.setPins(
      PIN_SD_CLK,
      PIN_SD_CMD,
      PIN_SD_D0,
      PIN_SD_D1,
      PIN_SD_D2,
      PIN_SD_D3);

  if (!SD_MMC.begin("/sdcard", true)) {
    return false;
  }

  if (!SD_MMC.exists("/voice")) {
    SD_MMC.mkdir("/voice");
  }

  return true;
}

String nextCapturePath() {
  for (int i = 0; i < 1000; ++i) {
    char path[48];
    snprintf(path, sizeof(path), "/voice/cap_%05lu.wav", static_cast<unsigned long>(captureIndex++));
    if (!SD_MMC.exists(path)) {
      return String(path);
    }
  }
  return String("/voice/cap_fallback.wav");
}

bool saveCaptureToSd(const int16_t* samples, size_t sampleCount, String& outPath, String& outError) {
  if (!sdReady) {
    outError = "SD not ready";
    return false;
  }
  if (!samples || sampleCount == 0) {
    outError = "No samples";
    return false;
  }

  outPath = nextCapturePath();
  File f = SD_MMC.open(outPath, FILE_WRITE);
  if (!f) {
    outError = "Open failed";
    return false;
  }

  const size_t dataBytes = sampleCount * sizeof(int16_t);
  uint8_t wavHeader[44] = {0};
  writeWavHeader(wavHeader, dataBytes, CAPTURE_SAMPLE_RATE);

  size_t wrote = f.write(wavHeader, sizeof(wavHeader));
  if (wrote != sizeof(wavHeader)) {
    f.close();
    outError = "Header write failed";
    return false;
  }

  wrote = f.write(reinterpret_cast<const uint8_t*>(samples), dataBytes);
  f.close();

  if (wrote != dataBytes) {
    outError = "Audio write failed";
    return false;
  }

  return true;
}

bool homeAssistantStt(const int16_t* samples, size_t sampleCount, String& outText, String& outError) {
  if (!isHomeAssistantConfigured()) {
    outError = "HA not configured";
    return false;
  }

  const String haHost = urlHost(String(HA_BASE_URL));
  if (haHost.length() > 0) {
    IPAddress resolved;
    if (!WiFi.hostByName(haHost.c_str(), resolved)) {
      outError = String("STT DNS failed: ") + haHost;
      return false;
    }
  }

  const String endpoint = joinedUrl(HA_BASE_URL, String("/api/stt/") + HA_STT_PROVIDER);
  const String endpointAuthority = urlAuthority(endpoint);
  const uint16_t endpointPort = urlPort(endpoint);

  const auto tryStt = [&](const String& languageTag, String& attemptErr) -> bool {
    const String authHeader = String("Bearer ") + HA_ACCESS_TOKEN;
    const String speechHeader = String("language=") + languageTag +
                                ";format=wav;codec=pcm;bit_rate=16;sample_rate=16000;channel=1";

    for (uint8_t retry = 0; retry <= STT_MAX_RETRIES; ++retry) {
      if (WiFi.status() != WL_CONNECTED) {
        attemptErr = "STT Wi-Fi disconnected";
        return false;
      }

      HTTPClient http;
      WiFiClientSecure secureClient;
      if (!beginHttp(http, secureClient, endpoint)) {
        attemptErr = "HTTP begin failed";
        return false;
      }

      http.setReuse(false);
      http.useHTTP10(true);
      http.setConnectTimeout(STT_CONNECT_TIMEOUT_MS);
      http.setTimeout(STT_IO_TIMEOUT_MS);
      http.addHeader("Authorization", authHeader);
      http.addHeader("Content-Type", "audio/wav");
      http.addHeader("X-Speech-Content", speechHeader);

      const size_t pcmBytes = sampleCount * sizeof(int16_t);
      const size_t wavBytes = pcmBytes + 44;
      int code = -1;

      uint8_t* wavPayload = static_cast<uint8_t*>(malloc(wavBytes));
      if (wavPayload) {
        writeWavHeader(wavPayload, pcmBytes, CAPTURE_SAMPLE_RATE);
        memcpy(wavPayload + 44, samples, pcmBytes);
        code = http.POST(wavPayload, wavBytes);
        free(wavPayload);
      } else {
        // Fallback when heap is tight.
        WavUploadStream wavStream(samples, sampleCount, CAPTURE_SAMPLE_RATE);
        code = http.sendRequest("POST", &wavStream, wavStream.totalSize());
      }

      String response;
      if (code >= 200 && code <= 599) {
        // Avoid blocking body reads on transport-level failures (negative codes).
        response = http.getString();
      }
      http.end();

      if (code >= 200 && code <= 299) {
        DynamicJsonDocument doc(2048);
        if (deserializeJson(doc, response)) {
          attemptErr = "STT JSON parse failed";
          return false;
        }

        const String result = doc["result"] | "";
        outText = doc["text"] | "";
        outText.trim();
        if (result != "success") {
          attemptErr = String("STT result: ") + result;
          return false;
        }
        if (outText.length() == 0) {
          Serial.println(String("[STT] success but empty; raw=") + clipText(response, 180));
        }

        return true;
      }

      attemptErr = String("STT HTTP ") + code;
      const String codeText = sttHttpErrorText(code);
      if (codeText.length() > 0) {
        attemptErr += ": " + codeText;
      }
      if (response.length() > 0) {
        attemptErr += ": " + clipText(response, 80);
      }
      if (code < 0 && endpointAuthority.length() > 0) {
        attemptErr += String(" @") + endpointAuthority +
                      String(" port ") + endpointPort;
      }

      if (isRetryableSttTransportError(code) && retry < STT_MAX_RETRIES) {
        delay(180 + retry * 180);
        continue;
      }

      return false;
    }

    return false;
  };

  const String configuredLang = String(HA_LANGUAGE);
  if (tryStt(configuredLang, outError)) {
    return true;
  }

  if (!outError.startsWith("STT HTTP 415")) {
    return false;
  }

  const String fallbackLang = baseLanguageTag(configuredLang);
  if (fallbackLang.length() == 0 || fallbackLang == configuredLang) {
    return false;
  }

  String retryErr;
  if (tryStt(fallbackLang, retryErr)) {
    return true;
  }

  outError = retryErr;
  return false;
}

bool homeAssistantConversation(const String& text, String& outReply, String& outError) {
  if (!isHomeAssistantConfigured()) {
    outError = "HA not configured";
    return false;
  }

  const String endpoint = joinedUrl(HA_BASE_URL, "/api/conversation/process");
  HTTPClient http;
  WiFiClientSecure secureClient;
  if (!beginHttp(http, secureClient, endpoint)) {
    outError = "HTTP begin failed";
    return false;
  }

  http.setReuse(false);
  const String authHeader = String("Bearer ") + HA_ACCESS_TOKEN;
  http.addHeader("Authorization", authHeader);
  http.addHeader("Content-Type", "application/json");

  DynamicJsonDocument req(512);
  req["text"] = text;
  req["language"] = HA_LANGUAGE;
  if (strlen(HA_AGENT_ID) > 0) {
    req["agent_id"] = HA_AGENT_ID;
  }

  String body;
  serializeJson(req, body);

  const int code = http.POST(body);
  if (code < 200 || code > 299) {
    outError = String("Conversation HTTP ") + code;
    http.end();
    return false;
  }

  const String response = http.getString();
  http.end();

  DynamicJsonDocument doc(4096);
  if (deserializeJson(doc, response)) {
    outError = "Conversation JSON parse failed";
    return false;
  }

  outReply = doc["response"]["speech"]["plain"]["speech"] | "";
  if (outReply.length() == 0 && doc["response"]["speech"].is<const char*>()) {
    outReply = doc["response"]["speech"].as<const char*>();
  }
  if (outReply.length() == 0) {
    outReply = doc["response"]["data"]["message"] | "";
  }
  if (outReply.length() == 0) {
    outReply = "(No speech reply)";
  }

  return true;
}

bool homeAssistantFetchCoreUrls(String& outInternalUrl, String& outExternalUrl, String& outError) {
  outInternalUrl = "";
  outExternalUrl = "";
  outError = "";

  if (!isHomeAssistantConfigured()) {
    outError = "HA not configured";
    return false;
  }

  const String endpoint = joinedUrl(HA_BASE_URL, "/api/config");
  HTTPClient http;
  WiFiClientSecure secureClient;
  if (!beginHttp(http, secureClient, endpoint)) {
    outError = "Config HTTP begin failed";
    return false;
  }

  http.setReuse(false);
  http.setTimeout(8000);
  http.addHeader("Authorization", String("Bearer ") + HA_ACCESS_TOKEN);

  const int code = http.GET();
  if (code < 200 || code > 299) {
    String response = "";
    if (code >= 200 && code <= 599) {
      response = http.getString();
    }
    outError = String("Config HTTP ") + code;
    if (response.length() > 0) {
      outError += ": " + clipText(response, 80);
    }
    http.end();
    return false;
  }

  const String response = http.getString();
  http.end();

  DynamicJsonDocument doc(4096);
  if (deserializeJson(doc, response)) {
    outError = "Config JSON parse failed";
    return false;
  }

  outInternalUrl = doc["internal_url"] | "";
  outExternalUrl = doc["external_url"] | "";
  return true;
}

bool homeAssistantTtsGetUrl(const String& text, String& outUrl, String& outError) {
  if (!isHomeAssistantConfigured()) {
    outError = "HA not configured";
    return false;
  }
  if (strlen(HA_TTS_ENGINE_ID) == 0) {
    outError = "TTS engine not configured";
    return false;
  }

  const String endpoint = joinedUrl(HA_BASE_URL, "/api/tts_get_url");
  const String authHeader = String("Bearer ") + HA_ACCESS_TOKEN;
  const String configuredEngineId = String(HA_TTS_ENGINE_ID);
  const String strippedEngineId =
      configuredEngineId.startsWith("tts.") ? configuredEngineId.substring(4) : configuredEngineId;
  const String configuredLanguage = String(HA_LANGUAGE);
  const bool engineLooksPiper =
      configuredEngineId.indexOf("piper") >= 0 || strippedEngineId.indexOf("piper") >= 0;
  String ttsLanguage = configuredLanguage;
  if (engineLooksPiper && ttsLanguage.indexOf('-') >= 0) {
    // Wyoming Piper reports locale tags with underscore (e.g. en_US).
    ttsLanguage.replace("-", "_");
  }

  auto normalizeReturnedUrl = [&](String& candidate) {
    if (!candidate.startsWith("http://") && !candidate.startsWith("https://")) {
      if (!candidate.startsWith("/")) {
        candidate = "/" + candidate;
      }
      candidate = joinedUrl(HA_BASE_URL, candidate);
    }
  };

  auto attemptPayload = [&](const JsonDocument& req,
                            const char* label,
                            int& outHttpCode,
                            String& outHttpBody,
                            String& outParsedUrl) -> bool {
    outHttpCode = -1;
    outHttpBody = "";
    outParsedUrl = "";

    HTTPClient http;
    WiFiClientSecure secureClient;
    if (!beginHttp(http, secureClient, endpoint)) {
      outHttpCode = -998;
      outHttpBody = "HTTP begin failed";
      return false;
    }

    http.setReuse(false);
    http.setTimeout(12000);
    http.addHeader("Authorization", authHeader);
    http.addHeader("Content-Type", "application/json");

    String body;
    serializeJson(req, body);
    Serial.println(String("[TTS] ") + label + " req=" + clipText(body, 180));

    outHttpCode = http.POST(body);
    if (outHttpCode >= 200 && outHttpCode <= 599) {
      outHttpBody = http.getString();
    }
    http.end();

    if (outHttpCode < 200 || outHttpCode > 299) {
      Serial.println(
          String("[TTS] ") + label +
          String(" -> HTTP ") + outHttpCode +
          (outHttpBody.length() ? String(" body=") + clipText(outHttpBody, 120) : String("")));
      return false;
    }

    DynamicJsonDocument doc(2048);
    if (deserializeJson(doc, outHttpBody)) {
      outHttpBody = "JSON parse failed";
      return false;
    }

    outParsedUrl = doc["url"] | "";
    if (outParsedUrl.length() == 0) {
      outParsedUrl = doc["path"] | "";
    }
    if (outParsedUrl.length() == 0) {
      outHttpBody = "URL missing in response";
      return false;
    }

    normalizeReturnedUrl(outParsedUrl);
    Serial.println(String("[TTS] ") + label + " url=" + clipText(outParsedUrl, 160));
    return true;
  };

  int lastCode = -1;
  String lastBody = "";

  {
    DynamicJsonDocument req(768);
    req["engine_id"] = configuredEngineId;
    req["message"] = text;
    req["language"] = ttsLanguage;
    JsonObject options = req["options"].to<JsonObject>();
    options["preferred_format"] = "wav";

    String candidateUrl;
    if (attemptPayload(req, "engine_id", lastCode, lastBody, candidateUrl)) {
      outUrl = candidateUrl;
      return true;
    }
  }

  if (strippedEngineId.length() > 0 && strippedEngineId != configuredEngineId) {
    DynamicJsonDocument req(768);
    req["engine_id"] = strippedEngineId;
    req["message"] = text;
    req["language"] = ttsLanguage;
    JsonObject options = req["options"].to<JsonObject>();
    options["preferred_format"] = "wav";

    String candidateUrl;
    if (attemptPayload(req, "engine_id stripped", lastCode, lastBody, candidateUrl)) {
      outUrl = candidateUrl;
      return true;
    }
  }

  if (strippedEngineId.length() > 0) {
    DynamicJsonDocument req(768);
    req["platform"] = strippedEngineId;
    req["message"] = text;
    req["language"] = ttsLanguage;
    JsonObject options = req["options"].to<JsonObject>();
    options["preferred_format"] = "wav";

    String candidateUrl;
    if (attemptPayload(req, "platform", lastCode, lastBody, candidateUrl)) {
      outUrl = candidateUrl;
      return true;
    }
  }

  outError = String("TTS URL HTTP ") + lastCode;
  if (lastBody.length() > 0) {
    outError += ": " + clipText(lastBody, 80);
  }

  if (lastCode == 500) {
    String internalUrl;
    String externalUrl;
    String cfgErr;
    if (homeAssistantFetchCoreUrls(internalUrl, externalUrl, cfgErr)) {
      if (internalUrl.length() == 0 && externalUrl.length() == 0) {
        outError += " (HA internal/external URL unset)";
      }
    } else if (cfgErr.length() > 0) {
      outError += " (config check: " + clipText(cfgErr, 42) + ")";
    }
  }

  return false;
}

bool homeAssistantPlayTtsWav(const String& ttsUrl, String& outError) {
  String resolvedUrl = ttsUrl;
  if (!resolvedUrl.startsWith("http://") && !resolvedUrl.startsWith("https://")) {
    if (!resolvedUrl.startsWith("/")) {
      resolvedUrl = "/" + resolvedUrl;
    }
    resolvedUrl = joinedUrl(HA_BASE_URL, resolvedUrl);
  }

  HTTPClient http;
  WiFiClientSecure secureClient;
  if (!beginHttp(http, secureClient, resolvedUrl)) {
    outError = "TTS WAV HTTP begin failed";
    return false;
  }
  http.setReuse(false);
  http.useHTTP10(true);  // Avoid chunked transfer framing in stream parser.
  http.setTimeout(12000);
  http.addHeader("Authorization", String("Bearer ") + HA_ACCESS_TOKEN);

  const int code = http.GET();
  if (code < 200 || code > 299) {
    outError = String("TTS WAV HTTP ") + code;
    http.end();
    return false;
  }

  const String contentType = http.header("Content-Type");
  if (contentType.length() > 0) {
    Serial.println(String("[TTS] content-type: ") + contentType);
  }

  Stream* wavStream = http.getStreamPtr();
  if (!wavStream) {
    outError = "TTS WAV stream missing";
    http.end();
    return false;
  }
  int contentLen = http.getSize();
  if (contentLen <= 0) {
    const String cl = http.header("Content-Length");
    if (cl.length() > 0) {
      contentLen = cl.toInt();
    }
  }
  Serial.println(String("[TTS] size hint: ") + contentLen);

  if (contentLen <= 0) {
    // Unknown body length is common with HA's TTS proxy.
    // Prefetch a small head chunk for smooth startup, then continue streaming live.
    uint8_t* prefetchBytes = allocAudioBuffer(TTS_STREAM_PREFETCH_BYTES);
    if (prefetchBytes) {
      size_t got = 0;
      const uint32_t prefetchStartMs = millis();
      uint32_t lastRxMs = prefetchStartMs;
      while (got < TTS_STREAM_PREFETCH_BYTES) {
        size_t toRead = TTS_STREAM_PREFETCH_BYTES - got;
        if (toRead > 1024) {
          toRead = 1024;
        }
        const size_t n = wavStream->readBytes(
            reinterpret_cast<char*>(prefetchBytes + got),
            toRead);
        if (n > 0) {
          got += n;
          lastRxMs = millis();
        } else {
          if (!http.connected()) {
            break;
          }
          const uint32_t now = millis();
          if ((now - prefetchStartMs) > TTS_STREAM_PREFETCH_MAX_MS &&
              (now - lastRxMs) > TTS_STREAM_PREFETCH_IDLE_MS) {
            break;
          }
          delay(1);
        }
      }

      if (got > 0) {
        Serial.println(String("[TTS] prebuffer bytes: ") + got);
        PrefixedReadStream mixedStream(prefetchBytes, got, *wavStream);
        const bool ok = playWavFromStream(mixedStream, outError);
        free(prefetchBytes);
        http.end();
        return ok;
      }

      free(prefetchBytes);
    } else {
      Serial.println(String("[TTS] prebuffer alloc failed for ") + TTS_STREAM_PREFETCH_BYTES + " bytes");
    }
  }

  // Known size: full buffer to avoid playback glitches from network jitter.
  const bool canBufferKnownLength = contentLen > 0 && contentLen <= static_cast<int>(TTS_BUFFER_MAX_BYTES);
  if (canBufferKnownLength) {
    size_t targetBytes = canBufferKnownLength
                             ? static_cast<size_t>(contentLen)
                             : static_cast<size_t>(0);
    uint8_t* wavBytes = allocAudioBuffer(targetBytes);

    if (wavBytes) {
      size_t got = 0;
      uint32_t lastRxMs = millis();
      const uint32_t idleTimeoutMs = 12000;
      while (got < targetBytes) {
        size_t toRead = targetBytes - got;
        if (toRead > 4096) {
          toRead = 4096;
        }
        const size_t n = wavStream->readBytes(
            reinterpret_cast<char*>(wavBytes + got),
            toRead);
        if (n > 0) {
          got += n;
          lastRxMs = millis();
          if (got >= static_cast<size_t>(contentLen)) {
            break;
          }
          continue;
        }
        if (!http.connected()) {
          break;
        }
        if ((millis() - lastRxMs) > idleTimeoutMs) {
          break;
        }
        delay(1);
      }

      if (got < static_cast<size_t>(contentLen)) {
        free(wavBytes);
        http.end();
        outError = "TTS WAV download timeout";
        return false;
      }

      if (got == 0) {
        free(wavBytes);
      } else {
        Serial.println(String("[TTS] buffered bytes: ") + got);
        ByteArrayReadStream memStream(wavBytes, got);
        const bool ok = playWavFromStream(memStream, outError);
        free(wavBytes);
        http.end();
        return ok;
      }
    } else {
      Serial.println(String("[TTS] buffer alloc failed for ") + targetBytes + " bytes");
    }
  }

  if (contentLen > static_cast<int>(TTS_BUFFER_MAX_BYTES)) {
    Serial.println(String("[TTS] content too large to buffer: ") + contentLen);
  }

  const bool ok = playWavFromStream(*wavStream, outError);
  http.end();
  return ok;
}

bool connectToWifi(const String& ssid, const String& password, uint32_t timeoutMs) {
  if (ssid.length() == 0) {
    return false;
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), password.c_str());

  const unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeoutMs) {
    uiSetWifiMenuStatus(String("Connecting to ") + ssid + " ...");
    uiRefreshNow();
    delay(200);
  }

  if (WiFi.status() == WL_CONNECTED) {
    uiSetWifiMenuStatus(String("Connected: ") + WiFi.localIP().toString());
    return true;
  }

  uiSetWifiMenuStatus("Wi-Fi connect failed");
  return false;
}

void uiUpdateStatus() {
  if (!uiReady) {
    return;
  }

  if (uiLabelState) {
    lv_label_set_text(uiLabelState, stateName(state));
  }

  if (uiLabelDetail) {
    lv_label_set_text(uiLabelDetail, clipText(stateDetail, 80).c_str());
  }

  if (uiLabelWifi) {
    if (WiFi.status() == WL_CONNECTED) {
      const String wifiLine = String("Wi-Fi: ") + WiFi.SSID() + "  " + WiFi.localIP().toString();
      lv_label_set_text(uiLabelWifi, clipText(wifiLine, 80).c_str());
    } else {
      lv_label_set_text(uiLabelWifi, "Wi-Fi: disconnected");
    }
  }
}

void runCapturePipeline(bool bypassWakeWord) {
  const auto stopAndReturn = [&]() -> bool {
    if (!assistantStopRequested) {
      return false;
    }
    assistantStopRequested = false;
    setState(AssistantState::Idle, "Stopped");
    uiSetTalkEnabled(true);
    return true;
  };

  if (stopAndReturn()) {
    return;
  }
  if (state != AssistantState::Idle) {
    return;
  }
  if (!audioReady) {
    setState(AssistantState::Error, "Audio not initialized");
    return;
  }

  const bool manualMode = bypassWakeWord;
  if (manualMode) {
    setState(AssistantState::Listening, "Listening... tap Stop");
  } else {
    setState(AssistantState::Listening, "Capturing 3s audio...");
  }
  uiRefreshNow();

  CaptureMetrics metrics{};
  size_t captureSamples = CAPTURE_SAMPLES;
  bool captured = false;
  const float kI2SDeadPeak = 0.0008f;
  String captureSource = "I2S";

  if (!manualMode && i2sMicReady && micCapture.captureBlocking(captureBuffer, CAPTURE_SAMPLES, metrics)) {
    captured = true;
  }

  if (manualMode) {
    if (i2sMicReady) {
      CaptureMetrics i2sMetrics{};
      size_t manualSamples = 0;
      if (captureI2SMicUntilStop(captureBuffer, captureBufferCapacity, manualSamples, i2sMetrics)) {
        metrics = i2sMetrics;
        captureSamples = manualSamples;
        captured = true;
        captureSource = "I2S";
      }
    }

    const bool i2sLooksDead = !captured || metrics.peakNorm < kI2SDeadPeak;
    if (i2sLooksDead && analogMicReady) {
      CaptureMetrics analogMetrics{};
      size_t manualSamples = 0;
      if (captureAnalogMicUntilStop(captureBuffer, captureBufferCapacity, manualSamples, analogMetrics)) {
        metrics = analogMetrics;
        captureSamples = manualSamples;
        captured = true;
        captureSource = "ADC";
      }
    }
  } else if (analogMicReady) {
    const bool i2sLooksDead = !captured || metrics.peakNorm < kI2SDeadPeak;
    if (i2sLooksDead) {
      CaptureMetrics analogMetrics{};
      if (captureAnalogMicBlocking(captureBuffer, CAPTURE_SAMPLES, analogMetrics)) {
        metrics = analogMetrics;
        captured = true;
        captureSource = "ADC";
      }
    }
  }

  if (!captured) {
    if (stopAndReturn()) {
      return;
    }
    if (manualMode) {
      setState(AssistantState::Idle, "Stopped");
      uiSetTalkEnabled(true);
      return;
    }
    setState(AssistantState::Error, "Mic capture failed (I2S+ADC)");
    uiSetTalkEnabled(true);
    return;
  }
  if (stopAndReturn()) {
    return;
  }

  const float captureGain = chooseCaptureGain(metrics.peakNorm);
  if (captureGain > 1.001f) {
    applyPcmGain(captureBuffer, captureSamples, captureGain);
    metrics = I2SMicCapture::analyze(captureBuffer, captureSamples);
  }

  const size_t preTrimSamples = captureSamples;
  size_t trimmedSamples = 0;
  if (trimPcmSilenceInPlace(captureBuffer, captureSamples, trimmedSamples)) {
    metrics = I2SMicCapture::analyze(captureBuffer, captureSamples);
    Serial.println(
        String("[MIC] trimmed ") + trimmedSamples + " samples (" +
        String((trimmedSamples * 1000UL) / CAPTURE_SAMPLE_RATE) + " ms)");
  }

  String savedPath;
  String saveErr;
  if (saveCaptureToSd(captureBuffer, captureSamples, savedPath, saveErr)) {
    Serial.println(String("[SD] saved: ") + savedPath);
  }

  String levelText = captureSource + String(" ") + buildLevelText(metrics, captureGain);
  if (captureSamples != preTrimSamples) {
    levelText += String(" trim ") +
                 String(((preTrimSamples - captureSamples) * 1000UL) / CAPTURE_SAMPLE_RATE) + "ms";
  }
  {
    size_t nonZero = 0;
    int32_t maxAbs = 0;
    for (size_t i = 0; i < captureSamples; ++i) {
      const int32_t v = captureBuffer[i];
      if (v != 0) {
        nonZero++;
      }
      const int32_t absV = v < 0 ? -v : v;
      if (absV > maxAbs) {
        maxAbs = absV;
      }
    }
    Serial.println(
        String("[MIC] ") + levelText +
        String("  nz=") + nonZero + "/" + captureSamples +
        String("  max=") + maxAbs);
  }
  setState(AssistantState::Thinking, levelText);
  uiRefreshNow();

  if (!isHomeAssistantConfigured() || WiFi.status() != WL_CONNECTED) {
    setState(AssistantState::Idle, "Captured locally");
    uiSetTalkEnabled(true);
    return;
  }

  String transcript;
  String sttErr;
  setState(AssistantState::Thinking, "Transcribing...");
  uiRefreshNow();
  if (!homeAssistantStt(captureBuffer, captureSamples, transcript, sttErr)) {
    Serial.println(String("[STT] error: ") + sttErr);
    handsFreeCooldownUntilMs = millis() + 4500;
    setState(AssistantState::Idle, "STT failed: " + clipText(sttErr, 48));
    uiSetTalkEnabled(true);
    return;
  }
  if (stopAndReturn()) {
    return;
  }
  transcript.trim();
  if (transcript.length() > 0) {
    Serial.println(String("[STT] \"") + clipText(transcript, 120) + "\"");
  } else {
    Serial.println("[STT] <empty transcript>");
  }
  if (transcript.length() == 0) {
    handsFreeCooldownUntilMs = millis() + 2800;
    setState(AssistantState::Idle, "No speech: " + clipText(levelText, 45));
    uiSetTalkEnabled(true);
    return;
  }

  const bool wakeBypassActive = bypassWakeWord || (millis() < wakeBypassUntilMs);
  if (!wakeBypassActive) {
    String matchedWakePhrase;
    if (!transcriptHasWakePhrase(transcript, matchedWakePhrase)) {
      Serial.println(String("[WAKE] miss transcript=\"") + clipText(transcript, 96) + "\"");
      handsFreeCooldownUntilMs = millis() + 2600;
      setState(AssistantState::Idle, "Wake word not heard");
      uiSetTalkEnabled(true);
      return;
    }
    Serial.println(String("[WAKE] matched: ") + matchedWakePhrase);

    const String normalizedTranscript = normalizePhraseForMatch(transcript);
    const String normalizedWake = normalizePhraseForMatch(matchedWakePhrase);
    if (normalizedTranscript == normalizedWake) {
      wakeBypassUntilMs = millis() + 6000;
      setState(AssistantState::Idle, "Wake heard; speak now");
      uiSetTalkEnabled(true);
      return;
    }
  } else {
    Serial.println(bypassWakeWord ? "[WAKE] bypass manual button" : "[WAKE] bypass active");
  }
  wakeBypassUntilMs = 0;
  if (stopAndReturn()) {
    return;
  }

  String reply;
  String convErr;
  setState(AssistantState::Thinking, "Processing intent...");
  uiRefreshNow();
  if (!homeAssistantConversation(transcript, reply, convErr)) {
    handsFreeCooldownUntilMs = millis() + 2200;
    setState(AssistantState::Idle, "Intent failed: " + clipText(convErr, 44));
    uiSetTalkEnabled(true);
    return;
  }
  if (stopAndReturn()) {
    return;
  }

  bool ttsPlayed = false;
  if (strlen(HA_TTS_ENGINE_ID) > 0) {
    String ttsUrl;
    String ttsErr;

    setState(AssistantState::Speaking, "Requesting TTS...");
    uiRefreshNow();
    if (homeAssistantTtsGetUrl(reply, ttsUrl, ttsErr)) {
      setState(AssistantState::Speaking, "Playing reply...");
      uiRefreshNow();
      if (homeAssistantPlayTtsWav(ttsUrl, ttsErr)) {
        ttsPlayed = true;
      } else {
        Serial.println(String("[TTS] playback failed: ") + ttsErr);
        setState(AssistantState::Speaking, "TTS failed; showing text");
      }
    } else {
      Serial.println(String("[TTS] URL request failed: ") + ttsErr);
      setState(AssistantState::Speaking, "TTS unavailable; showing text");
    }
  }
  if (stopAndReturn()) {
    return;
  }

  if (!ttsPlayed) {
    setState(AssistantState::Speaking, "HA: " + clipText(reply, 40));
    delay(1500);
  }

  setState(AssistantState::Idle, "Ready");
  uiSetTalkEnabled(true);
}

void lvglFlushCb(lv_disp_drv_t* disp, const lv_area_t* area, lv_color_t* color_p) {
  const uint16_t w = static_cast<uint16_t>(area->x2 - area->x1 + 1);
  const uint16_t h = static_cast<uint16_t>(area->y2 - area->y1 + 1);

  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.pushColors(reinterpret_cast<uint16_t*>(&color_p->full), static_cast<uint32_t>(w) * h, true);
  tft.endWrite();

  lv_disp_flush_ready(disp);
}

void lvglTouchReadCb(lv_indev_drv_t* drv, lv_indev_data_t* data) {
  (void)drv;
  if (!touchReady) {
    data->state = LV_INDEV_STATE_REL;
    return;
  }

  TouchPoint raw{};
  if (!touch.readPoint(raw)) {
    data->state = LV_INDEV_STATE_REL;
    return;
  }

  const TouchPoint mapped = mapTouchToRotation1(raw);
  data->state = LV_INDEV_STATE_PR;
  data->point.x = mapped.x;
  data->point.y = mapped.y;
}

void uiTalkButtonEventCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }
  if (state == AssistantState::Idle) {
    assistantStopRequested = false;
    bypassWakeWordForNextCapture = true;
    captureRequested = true;
  } else {
    assistantStopRequested = true;
    captureRequested = false;
    setState(state, "Stopping...");
  }
}

void uiWifiScanButtonEventCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }
  wifiScanRequested = true;
}

void uiWifiSaveButtonEventCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }
  wifiSaveRequested = true;
}

void uiWifiConnectButtonEventCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }
  wifiConnectRequested = true;
}

void uiWifiDropdownEventCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED || !uiDdSsid || !uiTaSsid) {
    return;
  }

  char selected[64] = {0};
  lv_dropdown_get_selected_str(uiDdSsid, selected, sizeof(selected));
  lv_textarea_set_text(uiTaSsid, selected);
}

void uiShowKeyboardFor(lv_obj_t* ta) {
  if (!uiKeyboard || !ta) {
    return;
  }
  lv_keyboard_set_textarea(uiKeyboard, ta);
  lv_obj_clear_flag(uiKeyboard, LV_OBJ_FLAG_HIDDEN);
}

void uiHideKeyboard() {
  if (!uiKeyboard) {
    return;
  }
  lv_keyboard_set_textarea(uiKeyboard, nullptr);
  lv_obj_add_flag(uiKeyboard, LV_OBJ_FLAG_HIDDEN);
}

void uiTextareaEventCb(lv_event_t* e) {
  const lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t* ta = lv_event_get_target(e);
  if (code == LV_EVENT_FOCUSED) {
    uiShowKeyboardFor(ta);
  }
}

void uiKeyboardEventCb(lv_event_t* e) {
  const lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_READY || code == LV_EVENT_CANCEL) {
    uiHideKeyboard();
  }
}

void buildUi() {
  lv_obj_t* scr = lv_scr_act();
  lv_obj_set_style_bg_color(scr, lv_color_hex(0x0D1B2A), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_text_color(scr, lv_color_hex(0xEAF2FF), LV_PART_MAIN);

  uiTabview = lv_tabview_create(scr, LV_DIR_TOP, 34);
  lv_obj_set_size(uiTabview, SCREEN_WIDTH, SCREEN_HEIGHT);

  lv_obj_t* tabMain = lv_tabview_add_tab(uiTabview, "Assistant");
  lv_obj_t* tabConfig = lv_tabview_add_tab(uiTabview, "Config");

  lv_obj_set_scrollbar_mode(tabMain, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_scrollbar_mode(tabConfig, LV_SCROLLBAR_MODE_OFF);

  uiLabelState = lv_label_create(tabMain);
  lv_obj_align(uiLabelState, LV_ALIGN_TOP_LEFT, 8, 8);
  lv_label_set_text(uiLabelState, "Booting");

  uiLabelDetail = lv_label_create(tabMain);
  lv_obj_set_width(uiLabelDetail, SCREEN_WIDTH - 16);
  lv_obj_align(uiLabelDetail, LV_ALIGN_TOP_LEFT, 8, 42);
  lv_label_set_long_mode(uiLabelDetail, LV_LABEL_LONG_WRAP);
  lv_label_set_text(uiLabelDetail, "Initializing...");

  uiLabelWifi = lv_label_create(tabMain);
  lv_obj_set_width(uiLabelWifi, SCREEN_WIDTH - 16);
  lv_obj_align(uiLabelWifi, LV_ALIGN_BOTTOM_LEFT, 8, -88);
  lv_label_set_long_mode(uiLabelWifi, LV_LABEL_LONG_WRAP);
  lv_label_set_text(uiLabelWifi, "Wi-Fi: disconnected");

  uiLabelMic = lv_label_create(tabMain);
  lv_obj_set_width(uiLabelMic, SCREEN_WIDTH - 16);
  lv_obj_align(uiLabelMic, LV_ALIGN_BOTTOM_LEFT, 8, -66);
  lv_label_set_long_mode(uiLabelMic, LV_LABEL_LONG_WRAP);
  lv_label_set_text(uiLabelMic, "Mic: probing...");

  uiBtnTalk = lv_btn_create(tabMain);
  lv_obj_set_size(uiBtnTalk, 160, 48);
  lv_obj_align(uiBtnTalk, LV_ALIGN_BOTTOM_MID, 0, -12);
  lv_obj_add_event_cb(uiBtnTalk, uiTalkButtonEventCb, LV_EVENT_CLICKED, nullptr);
  uiLabelTalk = lv_label_create(uiBtnTalk);
  lv_label_set_text(uiLabelTalk, "Listen");
  lv_obj_center(uiLabelTalk);

  lv_obj_t* labelSsid = lv_label_create(tabConfig);
  lv_obj_align(labelSsid, LV_ALIGN_TOP_LEFT, 8, 8);
  lv_label_set_text(labelSsid, "Wi-Fi Network");

  uiDdSsid = lv_dropdown_create(tabConfig);
  lv_obj_set_width(uiDdSsid, 210);
  lv_obj_align(uiDdSsid, LV_ALIGN_TOP_LEFT, 8, 28);
  lv_dropdown_set_options(uiDdSsid, "No scan yet");
  lv_obj_add_event_cb(uiDdSsid, uiWifiDropdownEventCb, LV_EVENT_VALUE_CHANGED, nullptr);

  lv_obj_t* btnScan = lv_btn_create(tabConfig);
  lv_obj_set_size(btnScan, 86, 36);
  lv_obj_align(btnScan, LV_ALIGN_TOP_RIGHT, -8, 28);
  lv_obj_add_event_cb(btnScan, uiWifiScanButtonEventCb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* btnScanLabel = lv_label_create(btnScan);
  lv_label_set_text(btnScanLabel, "Scan");
  lv_obj_center(btnScanLabel);

  lv_obj_t* labelManual = lv_label_create(tabConfig);
  lv_obj_align(labelManual, LV_ALIGN_TOP_LEFT, 8, 70);
  lv_label_set_text(labelManual, "SSID");

  uiTaSsid = lv_textarea_create(tabConfig);
  lv_obj_set_width(uiTaSsid, SCREEN_WIDTH - 16);
  lv_obj_align(uiTaSsid, LV_ALIGN_TOP_LEFT, 8, 90);
  lv_textarea_set_one_line(uiTaSsid, true);
  lv_obj_add_event_cb(uiTaSsid, uiTextareaEventCb, LV_EVENT_FOCUSED, nullptr);

  lv_obj_t* labelPwd = lv_label_create(tabConfig);
  lv_obj_align(labelPwd, LV_ALIGN_TOP_LEFT, 8, 128);
  lv_label_set_text(labelPwd, "Password");

  uiTaPassword = lv_textarea_create(tabConfig);
  lv_obj_set_width(uiTaPassword, SCREEN_WIDTH - 16);
  lv_obj_align(uiTaPassword, LV_ALIGN_TOP_LEFT, 8, 148);
  lv_textarea_set_one_line(uiTaPassword, true);
  lv_textarea_set_password_mode(uiTaPassword, true);
  lv_obj_add_event_cb(uiTaPassword, uiTextareaEventCb, LV_EVENT_FOCUSED, nullptr);

  lv_obj_t* btnSave = lv_btn_create(tabConfig);
  lv_obj_set_size(btnSave, 100, 36);
  lv_obj_align(btnSave, LV_ALIGN_BOTTOM_LEFT, 8, -10);
  lv_obj_add_event_cb(btnSave, uiWifiSaveButtonEventCb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* btnSaveLabel = lv_label_create(btnSave);
  lv_label_set_text(btnSaveLabel, "Save");
  lv_obj_center(btnSaveLabel);

  lv_obj_t* btnConnect = lv_btn_create(tabConfig);
  lv_obj_set_size(btnConnect, 100, 36);
  lv_obj_align(btnConnect, LV_ALIGN_BOTTOM_LEFT, 116, -10);
  lv_obj_add_event_cb(btnConnect, uiWifiConnectButtonEventCb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* btnConnectLabel = lv_label_create(btnConnect);
  lv_label_set_text(btnConnectLabel, "Connect");
  lv_obj_center(btnConnectLabel);

  uiLabelWifiMenuStatus = lv_label_create(tabConfig);
  lv_obj_set_width(uiLabelWifiMenuStatus, 94);
  lv_obj_align(uiLabelWifiMenuStatus, LV_ALIGN_BOTTOM_RIGHT, -8, -11);
  lv_label_set_long_mode(uiLabelWifiMenuStatus, LV_LABEL_LONG_WRAP);
  lv_label_set_text(uiLabelWifiMenuStatus, "Status: idle");

  uiKeyboard = lv_keyboard_create(scr);
  lv_obj_set_size(uiKeyboard, SCREEN_WIDTH, 110);
  lv_obj_align(uiKeyboard, LV_ALIGN_BOTTOM_MID, 0, 0);
  lv_obj_add_flag(uiKeyboard, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_event_cb(uiKeyboard, uiKeyboardEventCb, LV_EVENT_READY, nullptr);
  lv_obj_add_event_cb(uiKeyboard, uiKeyboardEventCb, LV_EVENT_CANCEL, nullptr);

  uiReady = true;
  uiSyncWifiFields();
  uiUpdateStatus();
  uiUpdateTalkButtonLabel();
  uiSetWifiMenuStatus("Set Wi-Fi then Connect");
}

void handleWifiScan() {
  uiSetWifiMenuStatus("Scanning...");
  uiRefreshNow();

  const int count = WiFi.scanNetworks(false, true);
  if (count <= 0) {
    if (uiDdSsid) {
      lv_dropdown_set_options(uiDdSsid, "No networks found");
    }
    uiSetWifiMenuStatus("Scan complete: none");
    WiFi.scanDelete();
    return;
  }

  String options;
  for (int i = 0; i < count; ++i) {
    const String name = WiFi.SSID(i);
    if (name.length() == 0) {
      continue;
    }
    if (options.length() > 0) {
      options += "\n";
    }
    options += name;
  }

  if (options.length() == 0) {
    options = "Hidden SSID only";
  }

  if (uiDdSsid) {
    lv_dropdown_set_options(uiDdSsid, options.c_str());
    char selected[64] = {0};
    lv_dropdown_get_selected_str(uiDdSsid, selected, sizeof(selected));
    if (uiTaSsid) {
      lv_textarea_set_text(uiTaSsid, selected);
    }
  }

  uiSetWifiMenuStatus(String("Found ") + String(count) + " network(s)");
  WiFi.scanDelete();
}

void handleWifiSave() {
  const String ssid = uiTaSsid ? String(lv_textarea_get_text(uiTaSsid)) : "";
  const String password = uiTaPassword ? String(lv_textarea_get_text(uiTaPassword)) : "";

  if (ssid.length() == 0) {
    uiSetWifiMenuStatus("SSID is required");
    return;
  }

  saveWifiConfig(ssid, password);
  uiSetWifiMenuStatus("Saved to flash");
}

void handleWifiConnect() {
  const String ssid = uiTaSsid ? String(lv_textarea_get_text(uiTaSsid)) : "";
  const String password = uiTaPassword ? String(lv_textarea_get_text(uiTaPassword)) : "";

  if (ssid.length() == 0) {
    uiSetWifiMenuStatus("Enter SSID first");
    return;
  }

  setState(AssistantState::ConnectingWiFi, "Connecting...");
  uiRefreshNow();

  if (connectToWifi(ssid, password, 15000)) {
    setState(AssistantState::Idle, "Ready");
    return;
  }

  setState(AssistantState::Idle, "Wi-Fi offline");
}

void pollBootButton() {
  const bool pressedNow = digitalRead(PIN_BOOT_BUTTON) == LOW;
  const unsigned long now = millis();

  if (pressedNow && !bootPressed) {
    bootPressed = true;
    bootPressStartMs = now;
    return;
  }

  if (!pressedNow && bootPressed) {
    const unsigned long heldMs = now - bootPressStartMs;
    bootPressed = false;

    if (heldMs >= BOOT_LONG_PRESS_MS) {
      if (uiTabview) {
        const uint16_t current = lv_tabview_get_tab_act(uiTabview);
        const uint16_t next = current == 0 ? 1 : 0;
        lv_tabview_set_act(uiTabview, next, LV_ANIM_ON);
      }
      uiSetWifiMenuStatus("Long press: toggled tab");
    } else {
      uiSetWifiMenuStatus("Short press: no action");
    }
  }
}

void pollHandsFreeTrigger() {
#if ASSISTANT_ENABLE_HANDS_FREE
  if (!handsFreeEnabled || !audioReady || state != AssistantState::Idle || captureRequested) {
    return;
  }

  const uint32_t now = millis();
  if (now < handsFreeCooldownUntilMs) {
    return;
  }
  if ((now - lastVadPollMs) < 40) {
    return;
  }
  lastVadPollMs = now;

  const VadSnapshot vadNow = sampleVadSnapshot();
  const float rmsNorm = vadNow.rmsNorm;
  const float peakNorm = vadNow.peakNorm;
  lastVadRmsNorm = rmsNorm;
  lastVadPeakNorm = peakNorm;

  if (!vadNoiseFloorReady) {
    vadNoiseFloorNorm = rmsNorm;
    vadNoiseFloorPeakNorm = peakNorm;
    vadNoiseFloorReady = true;
  } else {
    if (rmsNorm < vadNoiseFloorNorm) {
      // Pull floor down quickly if startup sampled a loud moment.
      vadNoiseFloorNorm = vadNoiseFloorNorm * 0.80f + rmsNorm * 0.20f;
    } else {
      // Let floor rise slowly to avoid suppressing actual speech.
      vadNoiseFloorNorm = vadNoiseFloorNorm * 0.995f + rmsNorm * 0.005f;
    }
    if (peakNorm < vadNoiseFloorPeakNorm) {
      vadNoiseFloorPeakNorm = vadNoiseFloorPeakNorm * 0.80f + peakNorm * 0.20f;
    } else {
      vadNoiseFloorPeakNorm = vadNoiseFloorPeakNorm * 0.995f + peakNorm * 0.005f;
    }
  }

  float dynamicVadThreshold = ASSISTANT_VAD_TRIGGER_RMS;
  const float adaptiveThreshold = vadNoiseFloorNorm * 1.02f + 0.0005f;
  if (adaptiveThreshold > dynamicVadThreshold) {
    dynamicVadThreshold = adaptiveThreshold;
  }
  if (dynamicVadThreshold < 0.004f) {
    dynamicVadThreshold = 0.004f;
  }
  if (dynamicVadThreshold > 0.050f) {
    dynamicVadThreshold = 0.050f;
  }

  float peakTriggerThreshold = vadNoiseFloorPeakNorm * 1.35f + 0.010f;
  if (peakTriggerThreshold < 0.018f) {
    peakTriggerThreshold = 0.018f;
  }
  if (peakTriggerThreshold > 0.25f) {
    peakTriggerThreshold = 0.25f;
  }

  const float speechRmsGate = dynamicVadThreshold > 0.022f ? dynamicVadThreshold : 0.022f;
  const float floorDelta = rmsNorm - vadNoiseFloorNorm;
  const float peakDelta = peakNorm - vadNoiseFloorPeakNorm;
  const bool clearlyAboveFloor = floorDelta >= 0.0022f || peakDelta >= 0.025f;
  const bool hotRms = rmsNorm >= speechRmsGate;
  const bool hotPeak = peakNorm >= peakTriggerThreshold;

  if ((hotRms && clearlyAboveFloor) || hotPeak) {
    if (vadActiveFrames < 255) {
      vadActiveFrames++;
    }
  } else if (vadActiveFrames > 1) {
    vadActiveFrames -= 2;
  } else {
    vadActiveFrames = 0;
  }

  if ((now - lastVadDebugMs) > 2400) {
    lastVadDebugMs = now;
    Serial.println(
        String("[VAD] rms=") + String(rmsNorm * 100.0f, 2) +
        String("% thr=") + String(dynamicVadThreshold * 100.0f, 2) +
        String("% gate=") + String(speechRmsGate * 100.0f, 2) +
        String("% floor=") + String(vadNoiseFloorNorm * 100.0f, 2) +
        String("% d=") + String(floorDelta * 100.0f, 2) +
        String("% p=") + String(peakNorm * 100.0f, 2) +
        String("% pthr=") + String(peakTriggerThreshold * 100.0f, 2) +
        String("% pd=") + String(peakDelta * 100.0f, 2) +
        String("% frames=") + String(vadActiveFrames));
  }

  const uint8_t triggerFrames = ASSISTANT_VAD_TRIGGER_FRAMES > 3 ? ASSISTANT_VAD_TRIGGER_FRAMES : 3;
  if (vadActiveFrames >= triggerFrames) {
    vadActiveFrames = 0;
    handsFreeCooldownUntilMs = now + 1700;
    bypassWakeWordForNextCapture = false;
    captureRequested = true;
    Serial.println(
        String("[VAD] trigger rms=") + String(rmsNorm * 100.0f, 2) +
        String("% gate=") + String(speechRmsGate * 100.0f, 2) +
        String("% floor=") + String(vadNoiseFloorNorm * 100.0f, 2) +
        String("% p=") + String(peakNorm * 100.0f, 2) +
        String("% pthr=") + String(peakTriggerThreshold * 100.0f, 2) + "%");
  }
#else
  (void)handsFreeEnabled;
#endif
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("[FW] voice_assistant_es3c28p 2026-03-31a");

  pinMode(PIN_BOOT_BUTTON, INPUT_PULLUP);
  pinMode(PIN_TFT_BL, OUTPUT);
  digitalWrite(PIN_TFT_BL, HIGH);

  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  lv_init();
  lv_disp_draw_buf_init(&drawBuf, drawBufPixels, nullptr, SCREEN_WIDTH * 20);

  static lv_disp_drv_t dispDrv;
  lv_disp_drv_init(&dispDrv);
  dispDrv.hor_res = SCREEN_WIDTH;
  dispDrv.ver_res = SCREEN_HEIGHT;
  dispDrv.flush_cb = lvglFlushCb;
  dispDrv.draw_buf = &drawBuf;
  lv_disp_drv_register(&dispDrv);

  static lv_indev_drv_t indevDrv;
  lv_indev_drv_init(&indevDrv);
  indevDrv.type = LV_INDEV_TYPE_POINTER;
  indevDrv.read_cb = lvglTouchReadCb;
  lv_indev_drv_register(&indevDrv);

  loadWifiConfig();
  buildUi();

  Wire.begin(PIN_TOUCH_SDA, PIN_TOUCH_SCL, 400000);

  touchReady = touch.begin(PIN_TOUCH_SDA, PIN_TOUCH_SCL, PIN_TOUCH_RST, PIN_TOUCH_INT, 400000, false);
  if (!touchReady) {
    uiSetWifiMenuStatus("Touch unavailable; use BOOT");
  }

  if (psramFound()) {
    const size_t desiredSamples = CAPTURE_SAMPLE_RATE * 12;  // up to 12s manual capture
    int16_t* psramCapture = static_cast<int16_t*>(ps_malloc(desiredSamples * sizeof(int16_t)));
    if (psramCapture) {
      captureBuffer = psramCapture;
      captureBufferCapacity = desiredSamples;
      Serial.println(String("[MIC] capture buffer PSRAM samples=") + captureBufferCapacity);
    }
  }

  analogMicReady = probeAnalogMicPin(PIN_ANALOG_MIC);

  const bool codecReady = codec.begin(Wire, 0x18);
  i2sMicReady = codecReady && micCapture.begin(codec, CAPTURE_SAMPLE_RATE);
  Serial.println(String("[MIC] codec i2c ") + (codecReady ? "ok" : "fail"));
  Serial.println(String("[MIC] i2s capture ") + (i2sMicReady ? "ready" : "fail"));
  audioReady = i2sMicReady || analogMicReady;
  if (!audioReady) {
    setState(AssistantState::Error, "Audio init failed");
    uiSetTalkEnabled(false);
  } else {
    if (analogMicReady && !i2sMicReady) {
      Serial.println("[MIC] using ADC fallback on IO14");
    }
    uiSetTalkEnabled(true);
    if (handsFreeEnabled) {
      Serial.println("[VAD] hands-free trigger enabled");
    }
  }

  sdReady = initSdCard();
  if (!sdReady) {
    Serial.println("[SD] init failed");
  }

  if (configuredSsid.length() > 0) {
    setState(AssistantState::ConnectingWiFi, String("Connecting ") + configuredSsid);
    if (connectToWifi(configuredSsid, configuredPassword, 15000)) {
      setState(AssistantState::Idle, "Ready");
    } else {
      setState(AssistantState::Idle, "Wi-Fi offline");
    }
  } else {
    setState(AssistantState::Idle, "Set Wi-Fi in Config tab");
  }

  uiRefreshNow();
}

void loop() {
  static uint32_t lastUiPollMs = 0;

  const uint32_t now = millis();

  lv_timer_handler();
  pollBootButton();
  pollHandsFreeTrigger();

  if (wifiScanRequested) {
    wifiScanRequested = false;
    handleWifiScan();
  }

  if (wifiSaveRequested) {
    wifiSaveRequested = false;
    handleWifiSave();
  }

  if (wifiConnectRequested) {
    wifiConnectRequested = false;
    handleWifiConnect();
  }

  if (captureRequested) {
    const bool bypassWakeWord = bypassWakeWordForNextCapture;
    bypassWakeWordForNextCapture = false;
    captureRequested = false;
    runCapturePipeline(bypassWakeWord);
  }

  if ((now - lastUiPollMs) > 1000) {
    lastUiPollMs = now;
    uiUpdateStatus();
  }
  uiUpdateMicLevelLine();

  delay(5);
}
