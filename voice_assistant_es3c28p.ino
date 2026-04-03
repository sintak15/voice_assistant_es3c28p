
#include "tft_espi_local_setup.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <TFT_eSPI.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include <esp_heap_caps.h>
#include <cstring>
#include <stdlib.h>

#include "board_pins.h"
#include "es8311_codec.h"
#include "i2s_mic_capture.h"
#include "secrets.example.h"
#if __has_include("secrets.h")
#include "secrets.h"
#endif

#define ASSISTANT_HAS_LOCAL_SR 1

enum class AssistantState {
  Booting,
  ConnectingWiFi,
  Idle,
  Listening,
  Transcribing,
  Thinking,
  Speaking,
  Error
};

static const uint16_t SCREEN_WIDTH = 240;
static const uint16_t SCREEN_HEIGHT = 320;

#ifndef ASSISTANT_CAPTURE_MAX_SECONDS
#define ASSISTANT_CAPTURE_MAX_SECONDS 18
#endif

#ifndef ASSISTANT_HANDSFREE_CAPTURE_MAX_SECONDS
#define ASSISTANT_HANDSFREE_CAPTURE_MAX_SECONDS 8
#endif

#ifndef ASSISTANT_VAD_START_RMS
#define ASSISTANT_VAD_START_RMS 0.012f
#endif

#ifndef ASSISTANT_VAD_STOP_RMS
#define ASSISTANT_VAD_STOP_RMS 0.007f
#endif

#ifndef ASSISTANT_VAD_TRAILING_SILENCE_MS
#define ASSISTANT_VAD_TRAILING_SILENCE_MS 900
#endif

#ifndef ASSISTANT_VAD_MAX_WAIT_MS
#define ASSISTANT_VAD_MAX_WAIT_MS 2200
#endif

#ifndef ASSISTANT_VAD_MIN_SPEECH_MS
#define ASSISTANT_VAD_MIN_SPEECH_MS 280
#endif

#ifndef ASSISTANT_VAD_PREROLL_MS
#define ASSISTANT_VAD_PREROLL_MS 260
#endif

#ifndef ASSISTANT_HANDSFREE_VAD_TRAILING_SILENCE_MS
#define ASSISTANT_HANDSFREE_VAD_TRAILING_SILENCE_MS 520
#endif

#ifndef ASSISTANT_HANDSFREE_VAD_MAX_WAIT_MS
#define ASSISTANT_HANDSFREE_VAD_MAX_WAIT_MS 1400
#endif

#ifndef ASSISTANT_HANDSFREE_VAD_MIN_SPEECH_MS
#define ASSISTANT_HANDSFREE_VAD_MIN_SPEECH_MS 220
#endif

#ifndef ASSISTANT_HANDSFREE_VAD_PREROLL_MS
#define ASSISTANT_HANDSFREE_VAD_PREROLL_MS 200
#endif

#ifndef ASSISTANT_BUSY_STATE_TIMEOUT_MS
#define ASSISTANT_BUSY_STATE_TIMEOUT_MS 120000
#endif

#ifndef ASSISTANT_WATCHDOG_MAX_STRIKES
#define ASSISTANT_WATCHDOG_MAX_STRIKES 3
#endif

#ifndef ASSISTANT_LOW_HEAP_THRESHOLD_BYTES
#define ASSISTANT_LOW_HEAP_THRESHOLD_BYTES 12000
#endif

#ifndef ASSISTANT_STT_UPLOAD_MAX_SECONDS
#define ASSISTANT_STT_UPLOAD_MAX_SECONDS 4
#endif

static const uint32_t CAPTURE_SAMPLE_RATE = 16000;
static const uint16_t CAPTURE_SECONDS = 4;
static const size_t CAPTURE_SAMPLES = CAPTURE_SAMPLE_RATE * CAPTURE_SECONDS;
static const float CAPTURE_MIN_RMS = 0.010f;
static const float CAPTURE_MIN_PEAK = 0.030f;

static const uint16_t STT_CONNECT_TIMEOUT_MS = 7000;
static const uint16_t STT_IO_TIMEOUT_MS = 14000;
static const uint8_t STT_MAX_RETRIES = 1;
static const uint16_t TTS_STREAM_READ_TIMEOUT_MS = 120;
static const uint16_t TTS_STREAM_IDLE_EOF_MS = 700;
static const size_t TTS_RAW_HTTP_MAX_WAV_BYTES = 512 * 1024;
static const uint32_t STAGE_TIMEOUT_LISTENING_MS = 30000;
static const uint32_t STAGE_TIMEOUT_TRANSCRIBING_MS = 45000;
static const uint32_t STAGE_TIMEOUT_THINKING_MS = 25000;
static const uint32_t STAGE_TIMEOUT_SPEAKING_MS = 50000;
static const uint16_t AUDIO_RECOVERY_SETTLE_MS = 20;

static const uint32_t WIFI_POLL_INTERVAL_MS = 700;
static const uint32_t WIFI_RETRY_BASE_MS = 2000;
static const uint32_t WIFI_RETRY_MAX_MS = 60000;
static const uint32_t WIFI_CONNECT_TIMEOUT_MS = 12000;
static const uint32_t LOCAL_WAKE_COOLDOWN_MS = 1800;
static const uint32_t HANDSFREE_VAD_POLL_INTERVAL_MS = 120;
static const uint32_t HANDSFREE_VAD_COOLDOWN_MS = 2600;
static const uint32_t RUNTIME_HEALTH_POLL_MS = 1000;
static const uint32_t WATCHDOG_STRIKE_WINDOW_MS = 8UL * 60UL * 1000UL;

#if ASSISTANT_HAS_LOCAL_SR
#include <esp32-hal-sr.h>
enum : int {
  SR_CMD_WAKE_BOB = 1,
};
static const sr_cmd_t kLocalWakeCommands[] = {
    {SR_CMD_WAKE_BOB, "ok mister bob", "bKd MgSTk BnB"},
    {SR_CMD_WAKE_BOB, "okay mister bob", "bKd MgSTk BnB"},
};
#endif

enum class CaptureRequestSource : uint8_t {
  Unknown = 0,
  Boot = 1,
  Vad = 2,
  Wake = 3,
};

static TFT_eSPI tft = TFT_eSPI(SCREEN_WIDTH, SCREEN_HEIGHT);
static TFT_eSprite uiSprite = TFT_eSprite(&tft);
static bool uiSpriteReady = false;
static Es8311Codec codec;
static I2SMicCapture micCapture;

static AssistantState state = AssistantState::Booting;
static String stateDetail = "Starting";

static bool audioReady = false;
static bool captureRequested = false;
static CaptureRequestSource captureSource = CaptureRequestSource::Unknown;
static bool handsFreeEnabled = false;
static bool localWakeEnabled = false;
static bool localWakeReady = false;
static bool localWakePaused = false;
static volatile bool localWakeTriggerPending = false;
static volatile uint32_t localWakeTriggerAtMs = 0;
static volatile int localWakePhraseId = -1;
static uint8_t handsFreeVadActiveFrames = 0;
static uint32_t handsFreeCooldownUntilMs = 0;
static uint32_t handsFreeVadLastPollMs = 0;
static uint32_t handsFreeArmAtMs = 0;
static float handsFreeNoiseRms = 0.0f;
static CaptureMetrics handsFreeLastLevel;

static bool bootPressed = false;
static uint32_t bootPressStartMs = 0;

static String configuredSsid;
static String configuredPassword;
static String reconnectSsid;
static String reconnectPassword;

static bool wifiReconnectPending = false;
static uint8_t wifiReconnectFailures = 0;
static uint32_t wifiNextRetryAtMs = 0;
static uint32_t wifiLastPollMs = 0;
static wl_status_t wifiLastStatus = WL_IDLE_STATUS;
static bool haAuthValidated = false;
static uint32_t haAuthValidatedAtMs = 0;

static String lastTranscript;
static String lastReply;

static bool uiDirty = true;
static uint32_t lastUiDrawMs = 0;

static int16_t captureBufferStorage[CAPTURE_SAMPLES] = {0};
static int16_t* captureBuffer = captureBufferStorage;
static size_t captureBufferCapacity = CAPTURE_SAMPLES;
static bool captureBufferInPsram = false;
static size_t lastCaptureSamples = 0;
static uint32_t lastCaptureMs = 0;
static float lastCaptureRms = 0.0f;
static float lastCapturePeak = 0.0f;
static String haConversationId;
static uint32_t stateEnteredAtMs = 0;
static uint32_t lastHealthPollMs = 0;
static uint32_t watchdogWindowStartMs = 0;
static uint8_t watchdogStrikeCount = 0;
static uint8_t lowHeapConsecutive = 0;
static float listeningFrameRms = 0.0f;
static float listeningFramePeak = 0.0f;
static uint32_t listeningCapturedMs = 0;
static bool listeningSpeechStarted = false;
static uint32_t listeningUiLastRefreshMs = 0;

String clipText(const String& text, size_t maxLen) {
  if (text.length() <= maxLen) {
    return text;
  }
  if (maxLen < 4) {
    return text.substring(0, maxLen);
  }
  return text.substring(0, maxLen - 3) + "...";
}

String tailText(const String& text, size_t maxLen) {
  if (text.length() <= maxLen) {
    return text;
  }
  if (maxLen < 4) {
    return text.substring(text.length() - maxLen);
  }
  return "..." + text.substring(text.length() - (maxLen - 3));
}

void logMemoryStats(const char* phase) {
  const size_t internalFree = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  const size_t internalLargest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
  Serial.println(
      String("[MEM] ") + phase +
      " | int_free=" + internalFree +
      " int_largest=" + internalLargest);

  if (psramFound()) {
    const size_t psramTotal = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    const size_t psramFree = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    const size_t psramLargest = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
    Serial.println(
        String("[MEM] ") + phase +
        " | psram_free=" + psramFree +
        " psram_largest=" + psramLargest +
        " psram_total=" + psramTotal);
  }
}

String trimmedConfig(const char* value) {
  String out(value ? value : "");
  out.trim();
  return out;
}

String normalizedAccessTokenValue() {
  String token = trimmedConfig(HA_ACCESS_TOKEN);
  if (token.length() >= 7) {
    String prefix = token.substring(0, 7);
    prefix.toLowerCase();
    if (prefix == "bearer ") {
      token.remove(0, 7);
    }
  }
  token.trim();
  return token;
}

String authHeaderValue() {
  const String token = normalizedAccessTokenValue();
  if (token.length() == 0) {
    return "";
  }
  return String("Bearer ") + token;
}

void invalidateHaAuthProbeCache() {
  haAuthValidated = false;
  haAuthValidatedAtMs = 0;
}

const char* stateName(AssistantState s) {
  switch (s) {
    case AssistantState::Booting:
      return "Booting";
    case AssistantState::ConnectingWiFi:
      return "Connecting Wi-Fi";
    case AssistantState::Idle:
      return "Idle";
    case AssistantState::Listening:
      return "Listening";
    case AssistantState::Transcribing:
      return "Transcribing";
    case AssistantState::Thinking:
      return "Thinking";
    case AssistantState::Speaking:
      return "Speaking";
    case AssistantState::Error:
      return "Error";
  }
  return "Unknown";
}

template <typename DisplayT>
void renderUiFrame(DisplayT& display, uint32_t nowMs) {
  display.fillScreen(TFT_BLACK);
  display.setTextDatum(TL_DATUM);

  display.setTextColor(TFT_CYAN, TFT_BLACK);
  display.drawString("HA Voice Core", 8, 8, 2);

  display.setTextColor(TFT_WHITE, TFT_BLACK);
  display.drawString(String("State: ") + stateName(state), 8, 34, 2);

  if (state == AssistantState::Error) {
    static const size_t kErrorCharsPerLine = 58;
    display.setTextColor(TFT_RED, TFT_BLACK);
    display.drawString(clipText(stateDetail, kErrorCharsPerLine), 8, 52, 1);
    if (stateDetail.length() > kErrorCharsPerLine) {
      display.drawString(tailText(stateDetail, kErrorCharsPerLine), 8, 62, 1);
    }
  } else {
    display.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    display.drawString(clipText(stateDetail, 46), 8, 52, 2);
  }

  if (WiFi.status() == WL_CONNECTED) {
    display.setTextColor(TFT_GREEN, TFT_BLACK);
    display.drawString(clipText(String("Wi-Fi: ") + WiFi.SSID() + " " + String(WiFi.RSSI()) + " dBm", 50), 8, 74, 2);
  } else {
    display.setTextColor(TFT_ORANGE, TFT_BLACK);
    if (wifiReconnectPending) {
      int32_t waitMs = static_cast<int32_t>(wifiNextRetryAtMs - nowMs);
      if (waitMs < 0) {
        waitMs = 0;
      }
      display.drawString(String("Wi-Fi retry in ") + String((waitMs + 999) / 1000) + "s", 8, 74, 2);
    } else {
      display.drawString("Wi-Fi disconnected", 8, 74, 2);
    }
  }

  display.setTextColor(TFT_YELLOW, TFT_BLACK);
  display.drawString("Transcript", 8, 108, 2);
  display.setTextColor(TFT_WHITE, TFT_BLACK);
  display.drawString(clipText(lastTranscript.length() ? lastTranscript : "(none)", 62), 8, 126, 2);

  display.setTextColor(TFT_YELLOW, TFT_BLACK);
  display.drawString("Reply", 8, 168, 2);
  display.setTextColor(TFT_WHITE, TFT_BLACK);
  display.drawString(clipText(lastReply.length() ? lastReply : "(none)", 62), 8, 186, 2);

  if (state == AssistantState::Listening) {
    const uint16_t panelBg = display.color565(34, 6, 6);
    const uint16_t panelBorder = display.color565(230, 70, 70);
    const uint16_t meterBg = display.color565(60, 60, 60);
    const uint16_t pulseColor = ((nowMs / 180) % 2) == 0 ? TFT_ORANGE : TFT_RED;

    display.fillRoundRect(8, 220, 224, 90, 10, panelBg);
    display.drawRoundRect(8, 220, 224, 90, 10, panelBorder);
    display.fillCircle(24, 238, 7, pulseColor);

    display.setTextColor(TFT_WHITE, panelBg);
    display.drawString("LISTENING", 38, 228, 4);
    display.setTextColor(TFT_LIGHTGREY, panelBg);
    display.drawString(listeningSpeechStarted ? "Speech detected" : "Waiting for speech", 14, 254, 2);

    const int meterX = 14;
    const int meterY = 274;
    const int meterW = 196;
    const int meterH = 10;
    display.fillRect(meterX, meterY, meterW, meterH, meterBg);
    display.drawRect(meterX, meterY, meterW, meterH, TFT_DARKGREY);

    int rmsWidth = static_cast<int>(listeningFrameRms * 3800.0f);
    if (rmsWidth < 0) {
      rmsWidth = 0;
    }
    if (rmsWidth > (meterW - 2)) {
      rmsWidth = meterW - 2;
    }
    if (rmsWidth > 0) {
      display.fillRect(meterX + 1, meterY + 1, rmsWidth, meterH - 2, TFT_CYAN);
    }

    int peakX = meterX + static_cast<int>(listeningFramePeak * (meterW - 2));
    if (peakX < meterX + 1) {
      peakX = meterX + 1;
    }
    if (peakX > meterX + meterW - 2) {
      peakX = meterX + meterW - 2;
    }
    display.drawFastVLine(peakX, meterY - 1, meterH + 2, TFT_YELLOW);

    display.setTextColor(TFT_WHITE, panelBg);
    display.drawString(String("Live: ") + String(listeningCapturedMs / 1000.0f, 1) + "s", 14, 289, 2);
  } else {
    display.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    display.drawString("Press BOOT for capture", 8, 288, 2);

    if (lastCaptureSamples > 0) {
      display.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
      const String capInfo =
          String("Last cap ") + String(lastCaptureMs / 1000.0f, 2) +
          "s pk " + String(lastCapturePeak * 100.0f, 1) + "%";
      display.drawString(clipText(capInfo, 42), 8, 270, 2);
    }
  }
}

void drawScreen(bool force) {
  const uint32_t nowMs = millis();
  const uint32_t minRefreshMs = state == AssistantState::Listening ? 90 : 350;
  if (!force && !uiDirty && (nowMs - lastUiDrawMs) < minRefreshMs) {
    return;
  }

  uiDirty = false;
  lastUiDrawMs = nowMs;

  if (uiSpriteReady) {
    renderUiFrame(uiSprite, nowMs);
    uiSprite.pushSprite(0, 0);
  } else {
    renderUiFrame(tft, nowMs);
  }
}

void listeningCaptureProgressCallback(
    const CaptureMetrics& frameMetrics,
    bool speechStarted,
    size_t capturedSamples,
    void* userData) {
  (void)userData;

  listeningFrameRms = frameMetrics.rmsNorm;
  listeningFramePeak = frameMetrics.peakNorm;
  listeningSpeechStarted = speechStarted;
  listeningCapturedMs = static_cast<uint32_t>((capturedSamples * 1000ULL) / CAPTURE_SAMPLE_RATE);

  const uint32_t nowMs = millis();
  if ((nowMs - listeningUiLastRefreshMs) < 90) {
    return;
  }
  listeningUiLastRefreshMs = nowMs;
  uiDirty = true;
  drawScreen(false);
}

void setState(AssistantState nextState, const String& detail) {
  state = nextState;
  stateDetail = detail;
  stateEnteredAtMs = millis();
  if (nextState != AssistantState::Listening) {
    listeningUiLastRefreshMs = 0;
  }
  uiDirty = true;

  Serial.print("State -> ");
  Serial.print(stateName(nextState));
  Serial.print(" | ");
  Serial.println(detail);
}

void setTranscript(const String& text) {
  lastTranscript = clipText(text, 110);
  uiDirty = true;
}

void setReply(const String& text) {
  lastReply = clipText(text, 120);
  uiDirty = true;
}

bool isHomeAssistantConfigured() {
  return trimmedConfig(HA_BASE_URL).length() > 0 &&
         normalizedAccessTokenValue().length() > 0 &&
         trimmedConfig(HA_STT_PROVIDER).length() > 0;
}

String joinedUrl(const char* base, const String& path) {
  String out(base ? base : "");
  if (out.endsWith("/")) {
    out.remove(out.length() - 1);
  }
  if (path.startsWith("/")) {
    return out + path;
  }
  return out + "/" + path;
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

String ttsLanguageTag(const String& language) {
  String out = language;
  out.trim();
  if (out.length() == 0) {
    return out;
  }
  out.replace("-", "_");
  return out;
}

bool beginHttp(HTTPClient& http, WiFiClient& plainClient, WiFiClientSecure& secureClient, const String& url) {
  if (url.startsWith("https://")) {
    secureClient.setInsecure();
    secureClient.setTimeout(STT_IO_TIMEOUT_MS);
    return http.begin(secureClient, url);
  }
  plainClient.setTimeout(STT_IO_TIMEOUT_MS);
  return http.begin(plainClient, url);
}

bool parseHttpUrl(
    const String& url,
    String& outHost,
    uint16_t& outPort,
    String& outPath,
    bool& outSecure) {
  outHost = "";
  outPort = 80;
  outPath = "/";
  outSecure = false;
  int prefixLen = 0;
  if (url.startsWith("http://")) {
    outSecure = false;
    outPort = 80;
    prefixLen = 7;
  } else if (url.startsWith("https://")) {
    outSecure = true;
    outPort = 443;
    prefixLen = 8;
  } else {
    return false;
  }

  const int slash = url.indexOf('/', prefixLen);
  String hostPort;
  if (slash < 0) {
    hostPort = url.substring(prefixLen);
    outPath = "/";
  } else {
    hostPort = url.substring(prefixLen, slash);
    outPath = url.substring(slash);
  }

  if (hostPort.length() == 0) {
    return false;
  }

  const int colon = hostPort.lastIndexOf(':');
  if (colon > 0 && hostPort.indexOf(']') < 0) {
    outHost = hostPort.substring(0, colon);
    const int parsedPort = hostPort.substring(colon + 1).toInt();
    if (parsedPort <= 0 || parsedPort > 65535) {
      return false;
    }
    outPort = static_cast<uint16_t>(parsedPort);
  } else {
    outHost = hostPort;
  }

  return outHost.length() > 0;
}

bool readExactClientBytes(
    WiFiClient& client,
    uint8_t* out,
    size_t len,
    uint32_t timeoutMs,
    String& outError) {
  if (!out || len == 0) {
    return len == 0;
  }

  size_t total = 0;
  uint32_t lastProgressMs = millis();
  while (total < len) {
    while (client.available() && total < len) {
      const int c = client.read();
      if (c >= 0) {
        out[total++] = static_cast<uint8_t>(c);
        lastProgressMs = millis();
      }
    }
    if (total >= len) {
      return true;
    }
    if (!client.connected() && !client.available()) {
      outError = "HTTP body truncated";
      return false;
    }
    if ((millis() - lastProgressMs) > timeoutMs) {
      outError = "HTTP body read timeout";
      return false;
    }
    delay(1);
  }
  return true;
}

bool readChunkedHttpBody(
    WiFiClient& client,
    String& outBody,
    String& outError) {
  outBody = "";
  outError = "";

  uint32_t lastProgressMs = millis();
  while (true) {
    String sizeLine = client.readStringUntil('\n');
    sizeLine.trim();

    if (sizeLine.length() == 0) {
      if (!client.connected() && !client.available()) {
        outError = "HTTP chunk header missing";
        return false;
      }
      if ((millis() - lastProgressMs) > STT_IO_TIMEOUT_MS) {
        outError = "HTTP chunk header timeout";
        return false;
      }
      continue;
    }

    const int semicolon = sizeLine.indexOf(';');
    if (semicolon >= 0) {
      sizeLine = sizeLine.substring(0, semicolon);
      sizeLine.trim();
    }

    char* endPtr = nullptr;
    const long parsed = strtol(sizeLine.c_str(), &endPtr, 16);
    if (endPtr == sizeLine.c_str() || parsed < 0) {
      outError = "HTTP chunk size parse failed";
      return false;
    }
    const size_t chunkSize = static_cast<size_t>(parsed);

    if (chunkSize == 0) {
      while (true) {
        String trailerLine = client.readStringUntil('\n');
        trailerLine.trim();
        if (trailerLine.length() == 0) {
          return true;
        }
        if ((millis() - lastProgressMs) > STT_IO_TIMEOUT_MS) {
          outError = "HTTP chunk trailer timeout";
          return false;
        }
      }
    }

    const size_t startLen = outBody.length();
    outBody.reserve(startLen + chunkSize);
    size_t remaining = chunkSize;
    while (remaining > 0) {
      while (client.available() && remaining > 0) {
        const int c = client.read();
        if (c >= 0) {
          outBody += static_cast<char>(c);
          remaining--;
          lastProgressMs = millis();
        }
      }
      if (remaining == 0) {
        break;
      }
      if (!client.connected() && !client.available()) {
        outError = "HTTP chunk truncated";
        return false;
      }
      if ((millis() - lastProgressMs) > STT_IO_TIMEOUT_MS) {
        outError = "HTTP chunk read timeout";
        return false;
      }
      delay(1);
    }

    uint8_t crlf[2] = {0};
    if (!readExactClientBytes(client, crlf, sizeof(crlf), STT_IO_TIMEOUT_MS, outError)) {
      return false;
    }
    if (crlf[0] != '\r' || crlf[1] != '\n') {
      outError = "HTTP chunk terminator missing";
      return false;
    }
  }
}

bool readHttpResponseBody(
    WiFiClient& client,
    int contentLength,
    bool chunked,
    String& outBody,
    String& outError) {
  outBody = "";
  outError = "";

  if (chunked) {
    return readChunkedHttpBody(client, outBody, outError);
  }

  uint32_t lastProgressMs = millis();
  if (contentLength > 0) {
    outBody.reserve(contentLength < 1024 ? contentLength : 1024);
  }

  while (true) {
    while (client.available()) {
      const int c = client.read();
      if (c >= 0) {
        outBody += static_cast<char>(c);
      }
      lastProgressMs = millis();
      if (contentLength >= 0 && static_cast<int>(outBody.length()) >= contentLength) {
        return true;
      }
    }

    if (!client.connected() && !client.available()) {
      return true;
    }

    if ((millis() - lastProgressMs) > STT_IO_TIMEOUT_MS) {
      outError = "HTTP response read timeout";
      return false;
    }
    delay(1);
  }
}

bool readHttpResponseHead(
    WiFiClient& client,
    int& outCode,
    int& outContentLength,
    bool& outChunked,
    String& outContentType,
    String& outError) {
  outCode = 0;
  outContentLength = -1;
  outChunked = false;
  outContentType = "";
  outError = "";

  String statusLine = client.readStringUntil('\n');
  statusLine.trim();
  if (!statusLine.startsWith("HTTP/")) {
    outError = "bad HTTP status";
    return false;
  }

  const int firstSpace = statusLine.indexOf(' ');
  const int secondSpace = firstSpace >= 0 ? statusLine.indexOf(' ', firstSpace + 1) : -1;
  if (firstSpace < 0) {
    outError = "status parse failed";
    return false;
  }
  const String codeText = secondSpace > firstSpace
                              ? statusLine.substring(firstSpace + 1, secondSpace)
                              : statusLine.substring(firstSpace + 1);
  outCode = codeText.toInt();

  while (true) {
    String headerLine = client.readStringUntil('\n');
    headerLine.trim();
    if (headerLine.length() == 0) {
      break;
    }

    const int colon = headerLine.indexOf(':');
    if (colon <= 0) {
      continue;
    }

    String name = headerLine.substring(0, colon);
    String value = headerLine.substring(colon + 1);
    name.trim();
    value.trim();
    String loweredName = name;
    String loweredValue = value;
    loweredName.toLowerCase();
    loweredValue.toLowerCase();

    if (loweredName == "content-length") {
      outContentLength = value.toInt();
    } else if (loweredName == "transfer-encoding" && loweredValue.indexOf("chunked") >= 0) {
      outChunked = true;
    } else if (loweredName == "content-type") {
      outContentType = value;
    }
  }

  return true;
}

bool sendRawHttpRequest(
    const String& endpoint,
    const String& method,
    const String* headers,
    size_t headerCount,
    const uint8_t* payload,
    size_t payloadBytes,
    int& outCode,
    String& outContentType,
    String& outBody,
    String& outError) {
  outCode = 0;
  outContentType = "";
  outBody = "";
  outError = "";

  String host;
  uint16_t port = 80;
  String path;
  bool secure = false;
  if (!parseHttpUrl(endpoint, host, port, path, secure)) {
    outCode = HTTPC_ERROR_NOT_CONNECTED;
    outError = "URL parse failed";
    return false;
  }
  if (secure) {
    outCode = HTTPC_ERROR_NOT_CONNECTED;
    outError = "raw HTTP path only";
    return false;
  }

  WiFiClient client;
  client.setTimeout(STT_IO_TIMEOUT_MS);
  if (!client.connect(host.c_str(), port)) {
    outCode = HTTPC_ERROR_CONNECTION_REFUSED;
    outError = "connect failed";
    return false;
  }

  String request =
      method + " " + path + " HTTP/1.1\r\n" +
      "Host: " + host + ":" + String(port) + "\r\n";
  for (size_t i = 0; i < headerCount; ++i) {
    request += headers[i] + "\r\n";
  }
  if (payload && payloadBytes > 0) {
    request += "Content-Length: " + String(payloadBytes) + "\r\n";
  }
  request += "Connection: close\r\n\r\n";

  const size_t headerWritten = client.print(request);
  if (headerWritten != request.length()) {
    client.stop();
    outCode = HTTPC_ERROR_SEND_HEADER_FAILED;
    outError = "send header failed";
    return false;
  }

  if (payload && payloadBytes > 0) {
    size_t sent = 0;
    uint32_t lastProgressMs = millis();
    while (sent < payloadBytes) {
      const size_t remaining = payloadBytes - sent;
      const size_t chunk = remaining > 1024 ? 1024 : remaining;
      const size_t written = client.write(payload + sent, chunk);
      if (written > 0) {
        sent += written;
        lastProgressMs = millis();
        continue;
      }
      if (!client.connected()) {
        client.stop();
        outCode = HTTPC_ERROR_CONNECTION_LOST;
        outError = "connection lost while sending";
        return false;
      }
      if ((millis() - lastProgressMs) > STT_IO_TIMEOUT_MS) {
        client.stop();
        outCode = HTTPC_ERROR_SEND_PAYLOAD_FAILED;
        outError = "send payload timeout";
        return false;
      }
      delay(1);
    }
  }

  int contentLength = -1;
  bool chunked = false;
  if (!readHttpResponseHead(
          client,
          outCode,
          contentLength,
          chunked,
          outContentType,
          outError)) {
    client.stop();
    outCode = HTTPC_ERROR_READ_TIMEOUT;
    return false;
  }

  if (!readHttpResponseBody(client, contentLength, chunked, outBody, outError)) {
    client.stop();
    if (outCode == 0) {
      outCode = HTTPC_ERROR_READ_TIMEOUT;
    }
    return false;
  }

  client.stop();
  return true;
}

bool postSttOverRawHttp(
    const String& endpoint,
    const String& authHeader,
    const String& speechHeader,
    const uint8_t* payload,
    size_t payloadBytes,
    int& outCode,
    String& outBody,
    String& outError) {
  String contentType;
  const String headers[] = {
      String("Authorization: ") + authHeader,
      "Content-Type: audio/wav",
      String("X-Speech-Content: ") + speechHeader,
  };
  if (!sendRawHttpRequest(
          endpoint,
          "POST",
          headers,
          sizeof(headers) / sizeof(headers[0]),
          payload,
          payloadBytes,
          outCode,
          contentType,
          outBody,
          outError)) {
    if (outError.length() > 0) {
      outError = String("STT ") + outError;
    }
    return false;
  }
  return true;
}

bool postJsonOverRawHttp(
    const String& endpoint,
    const String& authHeader,
    const String& jsonBody,
    int& outCode,
    String& outBody,
    String& outError) {
  String contentType;
  const String headers[] = {
      String("Authorization: ") + authHeader,
      "Content-Type: application/json",
      "Accept: application/json",
  };
  if (!sendRawHttpRequest(
          endpoint,
          "POST",
          headers,
          sizeof(headers) / sizeof(headers[0]),
          reinterpret_cast<const uint8_t*>(jsonBody.c_str()),
          jsonBody.length(),
          outCode,
          contentType,
          outBody,
          outError)) {
    if (outError.length() > 0) {
      outError = String("JSON ") + outError;
    }
    return false;
  }
  return true;
}

bool getRawOverHttp(
    const String& endpoint,
    const String& authHeader,
    int& outCode,
    String& outContentType,
    String& outBody,
    String& outError) {
  const String headers[] = {
      String("Authorization: ") + authHeader,
      "Accept: */*",
  };
  if (!sendRawHttpRequest(
          endpoint,
          "GET",
          headers,
          sizeof(headers) / sizeof(headers[0]),
          nullptr,
          0,
          outCode,
          outContentType,
          outBody,
          outError)) {
    if (outError.length() > 0) {
      outError = String("GET ") + outError;
    }
    return false;
  }
  return true;
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
      : pcmBytes_(reinterpret_cast<const uint8_t*>(samples)), pcmSizeBytes_(sampleCount * sizeof(int16_t)), cursor_(0) {
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

class MemoryReadStream : public Stream {
 public:
  MemoryReadStream(const uint8_t* data, size_t len)
      : data_(data), len_(len), cursor_(0) {}

  int available() override {
    const size_t remaining = len_ - cursor_;
    return remaining > 0x7FFFFFFF ? 0x7FFFFFFF : static_cast<int>(remaining);
  }

  int peek() override {
    if (cursor_ >= len_) {
      return -1;
    }
    return data_[cursor_];
  }

  int read() override {
    if (cursor_ >= len_) {
      return -1;
    }
    return data_[cursor_++];
  }

  size_t readBytes(char* buffer, size_t length) override {
    if (!buffer || length == 0 || cursor_ >= len_) {
      return 0;
    }

    const size_t remaining = len_ - cursor_;
    const size_t toCopy = length < remaining ? length : remaining;
    memcpy(buffer, data_ + cursor_, toCopy);
    cursor_ += toCopy;
    return toCopy;
  }

  size_t write(uint8_t b) override {
    (void)b;
    return 0;
  }

 private:
  const uint8_t* data_;
  size_t len_;
  size_t cursor_;
};
uint16_t readLe16(const uint8_t* in) {
  return static_cast<uint16_t>(in[0]) | (static_cast<uint16_t>(in[1]) << 8);
}

uint32_t readLe32(const uint8_t* in) {
  return static_cast<uint32_t>(in[0]) |
         (static_cast<uint32_t>(in[1]) << 8) |
         (static_cast<uint32_t>(in[2]) << 16) |
         (static_cast<uint32_t>(in[3]) << 24);
}

bool readExactFromStream(Stream& stream, uint8_t* out, size_t len, uint32_t timeoutMs) {
  if (!out || len == 0) {
    return len == 0;
  }

  size_t total = 0;
  uint32_t lastProgressMs = millis();
  while (total < len) {
    const size_t n = stream.readBytes(reinterpret_cast<char*>(out + total), len - total);
    if (n > 0) {
      total += n;
      lastProgressMs = millis();
      continue;
    }

    if ((millis() - lastProgressMs) > timeoutMs) {
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

  uint8_t scratch[128] = {0};
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

bool streamWavPcmToSpeaker(Stream& stream, uint32_t dataBytes, uint16_t channels, uint32_t sampleRate, String& outError) {
  if (channels != 1 && channels != 2) {
    outError = "WAV channels unsupported";
    return false;
  }

  const bool unknownDataLen = (dataBytes == 0);
  const size_t frameBytes = channels * sizeof(int16_t);
  static const size_t kChunkBytes = 2048;

  static int16_t inSamples[kChunkBytes / sizeof(int16_t)] = {0};
  static int16_t monoSamples[1024] = {0};

  uint32_t remaining = dataBytes;
  uint32_t lastProgressMs = millis();
  uint32_t resamplePhase = 0;
  size_t resampleOutCount = 0;
  size_t totalStreamed = 0;

  while (unknownDataLen || remaining > 0) {
    size_t toRead = unknownDataLen ? kChunkBytes : (remaining < kChunkBytes ? remaining : kChunkBytes);
    toRead -= (toRead % frameBytes);
    if (toRead == 0) {
      break;
    }

    size_t readBytes = 0;
    if (unknownDataLen) {
      const int availableNow = stream.available();
      if (availableNow <= 0) {
        if ((millis() - lastProgressMs) > TTS_STREAM_IDLE_EOF_MS) {
          break;
        }
        delay(2);
        continue;
      }
      const size_t readable = static_cast<size_t>(availableNow);
      const size_t readTarget = readable < toRead ? readable : toRead;
      readBytes = stream.readBytes(reinterpret_cast<char*>(inSamples), readTarget);
      if (readBytes == 0) {
        if ((millis() - lastProgressMs) > TTS_STREAM_IDLE_EOF_MS) {
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

    if (sampleRate == CAPTURE_SAMPLE_RATE && channels == 1) {
      const size_t sampleCount = playableBytes / sizeof(int16_t);
      if (!micCapture.playPcm16Blocking(
              reinterpret_cast<const uint8_t*>(inSamples),
              sampleCount * sizeof(int16_t),
              CAPTURE_SAMPLE_RATE)) {
        outError = "PCM playback failed";
        return false;
      }
    } else if (sampleRate == CAPTURE_SAMPLE_RATE && channels == 2) {
      const size_t frameCount = playableBytes / (2 * sizeof(int16_t));
      for (size_t i = 0; i < frameCount; ++i) {
        const int32_t mix = static_cast<int32_t>(inSamples[i * 2]) +
                            static_cast<int32_t>(inSamples[i * 2 + 1]);
        monoSamples[i] = static_cast<int16_t>(mix / 2);
      }
      if (!micCapture.playPcm16Blocking(
              reinterpret_cast<const uint8_t*>(monoSamples),
              frameCount * sizeof(int16_t),
              CAPTURE_SAMPLE_RATE)) {
        outError = "Stereo downmix playback failed";
        return false;
      }
    } else {
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

        resamplePhase += CAPTURE_SAMPLE_RATE;
        while (resamplePhase >= sampleRate) {
          monoSamples[resampleOutCount++] = mono;
          resamplePhase -= sampleRate;

          if (resampleOutCount >= (sizeof(monoSamples) / sizeof(monoSamples[0]))) {
            if (!micCapture.playPcm16Blocking(
                    reinterpret_cast<const uint8_t*>(monoSamples),
                    resampleOutCount * sizeof(int16_t),
                    CAPTURE_SAMPLE_RATE)) {
              outError = "Resample playback failed";
              return false;
            }
            resampleOutCount = 0;
          }
        }
      }
    }

    if (!unknownDataLen) {
      remaining -= static_cast<uint32_t>(readBytes);
    }
    yield();
  }

  if (resampleOutCount > 0) {
    if (!micCapture.playPcm16Blocking(
            reinterpret_cast<const uint8_t*>(monoSamples),
            resampleOutCount * sizeof(int16_t),
            CAPTURE_SAMPLE_RATE)) {
      outError = "Resample playback failed";
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
  if (!readExactFromStream(stream, riff, sizeof(riff), 9000)) {
    outError = "WAV header timeout";
    return false;
  }

  if (memcmp(riff + 0, "RIFF", 4) != 0 || memcmp(riff + 8, "WAVE", 4) != 0) {
    outError = "Not a WAV file";
    return false;
  }

  bool fmtSeen = false;
  uint16_t wavFormat = 0;
  uint16_t wavChannels = 0;
  uint16_t wavBitsPerSample = 0;
  uint32_t wavSampleRate = 0;

  while (true) {
    uint8_t chunkHeader[8] = {0};
    if (!readExactFromStream(stream, chunkHeader, sizeof(chunkHeader), 9000)) {
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
      if (!readExactFromStream(stream, fmt, sizeof(fmt), 9000)) {
        outError = "WAV fmt read failed";
        return false;
      }

      wavFormat = readLe16(fmt + 0);
      wavChannels = readLe16(fmt + 2);
      wavSampleRate = readLe32(fmt + 4);
      wavBitsPerSample = readLe16(fmt + 14);
      fmtSeen = true;

      const uint32_t extraFmtBytes = chunkSize - 16;
      if (extraFmtBytes > 0 && !discardStreamBytes(stream, extraFmtBytes, 9000)) {
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
      if (!fmtSeen || wavFormat != 1 || wavBitsPerSample != 16 || (wavChannels != 1 && wavChannels != 2) || wavSampleRate == 0) {
        outError = "WAV format unsupported";
        return false;
      }
      return streamWavPcmToSpeaker(stream, chunkSize, wavChannels, wavSampleRate, outError);
    }

    const uint32_t bytesToSkip = chunkSize + ((chunkSize & 1) ? 1 : 0);
    if (!discardStreamBytes(stream, bytesToSkip, 9000)) {
      outError = "WAV chunk skip timeout";
      return false;
    }
  }
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

bool isRetryableSttTransportError(int code) {
  return code == HTTPC_ERROR_CONNECTION_REFUSED ||
         code == HTTPC_ERROR_SEND_HEADER_FAILED ||
         code == HTTPC_ERROR_SEND_PAYLOAD_FAILED ||
         code == HTTPC_ERROR_NOT_CONNECTED ||
         code == HTTPC_ERROR_CONNECTION_LOST ||
         code == HTTPC_ERROR_READ_TIMEOUT;
}

bool homeAssistantAuthProbe(String& outError) {
  outError = "";

  if (!isHomeAssistantConfigured()) {
    outError = "HA not configured";
    return false;
  }

  const String baseUrl = trimmedConfig(HA_BASE_URL);
  const String authHeader = authHeaderValue();
  if (baseUrl.length() == 0 || authHeader.length() == 0) {
    outError = "HA config missing";
    return false;
  }

  const String endpoint = joinedUrl(baseUrl.c_str(), "/api/");
  int code = 0;
  String response;
  if (endpoint.startsWith("http://")) {
    String contentType;
    String transportError;
    if (!getRawOverHttp(
            endpoint,
            authHeader,
            code,
            contentType,
            response,
            transportError)) {
      if (transportError.length() > 0) {
        outError = String("Auth probe ") + transportError;
      } else {
        outError = "Auth probe GET failed";
      }
      return false;
    }
  } else {
    HTTPClient http;
    WiFiClient plainClient;
    WiFiClientSecure secureClient;
    if (!beginHttp(http, plainClient, secureClient, endpoint)) {
      outError = "Auth probe begin failed";
      return false;
    }

    http.setReuse(false);
    http.useHTTP10(true);
    http.setConnectTimeout(STT_CONNECT_TIMEOUT_MS);
    http.setTimeout(STT_IO_TIMEOUT_MS);
    http.addHeader("Authorization", authHeader);

    code = http.GET();
    response = (code >= 200 && code <= 599) ? http.getString() : "";
    http.end();
  }

  if (code >= 200 && code <= 299) {
    return true;
  }

  outError = String("Auth probe HTTP ") + code;
  if (code == 401 || code == 403) {
    outError += ": token rejected";
    invalidateHaAuthProbeCache();
  }
  if (response.length() > 0) {
    outError += ": " + clipText(response, 72);
  }
  return false;
}

bool ensureHaAuthReady(String& outError) {
  outError = "";

  if (WiFi.status() != WL_CONNECTED) {
    outError = "Wi-Fi disconnected";
    invalidateHaAuthProbeCache();
    return false;
  }

  const uint32_t nowMs = millis();
  if (haAuthValidated && (nowMs - haAuthValidatedAtMs) < 300000UL) {
    return true;
  }

  if (!homeAssistantAuthProbe(outError)) {
    invalidateHaAuthProbeCache();
    return false;
  }

  haAuthValidated = true;
  haAuthValidatedAtMs = nowMs;
  return true;
}

bool homeAssistantStt(const int16_t* samples, size_t sampleCount, String& outText, String& outError) {
  outText = "";
  outError = "";

  if (!isHomeAssistantConfigured()) {
    outError = "HA not configured";
    return false;
  }

  const String baseUrl = trimmedConfig(HA_BASE_URL);
  const String sttProvider = trimmedConfig(HA_STT_PROVIDER);
  const String authHeader = authHeaderValue();
  if (baseUrl.length() == 0 || sttProvider.length() == 0 || authHeader.length() == 0) {
    outError = "HA config missing";
    return false;
  }

  const String endpoint = joinedUrl(baseUrl.c_str(), String("/api/stt/") + sttProvider);

  const auto tryLanguage = [&](const String& language, String& languageError) -> bool {
    const String speechHeader = String("language=") + language +
                                ";format=wav;codec=pcm;bit_rate=16;sample_rate=16000;channel=1";
    const size_t shortSampleLimit = static_cast<size_t>(CAPTURE_SAMPLE_RATE) * 4;
    size_t sendSampleCount = sampleCount;
    bool shortenedPayloadTried = false;

    for (uint8_t retry = 0; retry <= STT_MAX_RETRIES; ++retry) {
      if (WiFi.status() != WL_CONNECTED) {
        languageError = "Wi-Fi disconnected";
        return false;
      }

      const size_t wavDataBytes = sendSampleCount * sizeof(int16_t);
      const size_t wavTotalBytes = 44 + wavDataBytes;
      uint8_t* wavPayload = nullptr;
#if defined(ESP32)
      if (psramFound()) {
        wavPayload = static_cast<uint8_t*>(ps_malloc(wavTotalBytes));
      }
#endif
      if (!wavPayload) {
        wavPayload = static_cast<uint8_t*>(malloc(wavTotalBytes));
      }
      if (!wavPayload) {
        languageError = String("STT payload alloc failed: ") + wavTotalBytes + " bytes";
        return false;
      }

      writeWavHeader(wavPayload, wavDataBytes, CAPTURE_SAMPLE_RATE);
      memcpy(wavPayload + 44, samples, wavDataBytes);
      int code = 0;
      String response;
      bool requestOk = false;

      if (endpoint.startsWith("http://")) {
        String rawError;
        requestOk = postSttOverRawHttp(
            endpoint,
            authHeader,
            speechHeader,
            wavPayload,
            wavTotalBytes,
            code,
            response,
            rawError);
        if (!requestOk && rawError.length() > 0) {
          Serial.println(String("[STT] raw-http error: ") + rawError);
        }
      } else {
        HTTPClient http;
        WiFiClient plainClient;
        WiFiClientSecure secureClient;
        if (!beginHttp(http, plainClient, secureClient, endpoint)) {
          free(wavPayload);
          languageError = "HTTP begin failed";
          return false;
        }

        http.setReuse(false);
        http.useHTTP10(true);
        http.setConnectTimeout(STT_CONNECT_TIMEOUT_MS);
        http.setTimeout(STT_IO_TIMEOUT_MS);
        http.addHeader("Authorization", authHeader);
        http.addHeader("Content-Type", "audio/wav");
        http.addHeader("X-Speech-Content", speechHeader);

        code = http.sendRequest("POST", wavPayload, wavTotalBytes);
        response = (code >= 200 && code <= 599) ? http.getString() : "";
        requestOk = true;
        http.end();
      }

      free(wavPayload);
      if (!requestOk && code == 0) {
        code = HTTPC_ERROR_NOT_CONNECTED;
      }

      if (code >= 200 && code <= 299) {
        DynamicJsonDocument doc(2048);
        if (deserializeJson(doc, response)) {
          languageError = "STT JSON parse failed";
          return false;
        }

        const String result = doc["result"] | "success";
        outText = doc["text"] | "";
        outText.trim();
        if (result == "success" && outText.length() > 0) {
          return true;
        }

        languageError = "STT result empty";
        return false;
      }

      languageError = String("STT HTTP ") + code;
      const String codeText = sttHttpErrorText(code);
      if (codeText.length() > 0) {
        languageError += ": " + codeText;
      }
      if (response.length() > 0) {
        languageError += ": " + clipText(response, 80);
      }
      if (code == 401 || code == 403) {
        invalidateHaAuthProbeCache();
      }

      Serial.println(
          String("[STT] lang=") + language +
          " retry=" + retry +
          " payload_bytes=" + wavTotalBytes +
          " code=" + code +
          " err=" + languageError);

      const bool retryableTransport = isRetryableSttTransportError(code);
      const bool retryableServer = (code >= 500 && code <= 599);

      if ((retryableTransport || retryableServer) &&
          !shortenedPayloadTried &&
          sendSampleCount > shortSampleLimit) {
        shortenedPayloadTried = true;
        sendSampleCount = shortSampleLimit;
        Serial.println(
            String("[STT] retry with short payload samples=") + sendSampleCount +
            " seconds=" + String(sendSampleCount / static_cast<float>(CAPTURE_SAMPLE_RATE), 1));
        delay(120);
        continue;
      }

      if ((retryableTransport || retryableServer) && retry < STT_MAX_RETRIES) {
        delay(200 + retry * 200);
        continue;
      }
      return false;
    }

    return false;
  };

  String preferredLanguage = baseLanguageTag(String(HA_LANGUAGE));
  preferredLanguage.trim();
  if (preferredLanguage.length() == 0) {
    preferredLanguage = String(HA_LANGUAGE);
    preferredLanguage.trim();
  }
  String preferredError;
  if (tryLanguage(preferredLanguage, preferredError)) {
    return true;
  }

  const String fallback = baseLanguageTag(preferredLanguage);
  if (fallback.length() == 0 || fallback == preferredLanguage) {
    outError = preferredError;
    return false;
  }

  String fallbackError;
  if (tryLanguage(fallback, fallbackError)) {
    Serial.println(
        String("[STT] fallback language ") + preferredLanguage + " -> " + fallback);
    return true;
  }

  outError = clipText(
      preferredError + " | fallback(" + fallback + "): " + fallbackError,
      150);
  return false;
}

bool homeAssistantConversation(const String& text, String& outReply, String& outError) {
  outReply = "";
  outError = "";

  if (!isHomeAssistantConfigured()) {
    outError = "HA not configured";
    return false;
  }

  const String baseUrl = trimmedConfig(HA_BASE_URL);
  const String authHeader = authHeaderValue();
  if (baseUrl.length() == 0 || authHeader.length() == 0) {
    outError = "HA config missing";
    return false;
  }

  const String endpoint = joinedUrl(baseUrl.c_str(), "/api/conversation/process");
  DynamicJsonDocument req(768);
  req["text"] = text;
  req["language"] = HA_LANGUAGE;
  if (strlen(HA_AGENT_ID) > 0) {
    req["agent_id"] = HA_AGENT_ID;
  }
  if (haConversationId.length() > 0) {
    req["conversation_id"] = haConversationId;
  }

  String body;
  serializeJson(req, body);

  int code = 0;
  String response;
  String transportError;
  bool requestOk = false;
  if (endpoint.startsWith("http://")) {
    requestOk = postJsonOverRawHttp(
        endpoint,
        authHeader,
        body,
        code,
        response,
        transportError);
    if (!requestOk && transportError.length() > 0) {
      Serial.println(String("[CONV] raw-http error: ") + transportError);
    }
  } else {
    HTTPClient http;
    WiFiClient plainClient;
    WiFiClientSecure secureClient;
    if (!beginHttp(http, plainClient, secureClient, endpoint)) {
      outError = "Conversation HTTP begin failed";
      return false;
    }

    http.setReuse(false);
    http.setConnectTimeout(STT_CONNECT_TIMEOUT_MS);
    http.setTimeout(STT_IO_TIMEOUT_MS);
    http.addHeader("Authorization", authHeader);
    http.addHeader("Content-Type", "application/json");

    code = http.POST(body);
    response = (code >= 200 && code <= 599) ? http.getString() : "";
    requestOk = true;
    http.end();
  }
  if (!requestOk && code == 0) {
    code = HTTPC_ERROR_NOT_CONNECTED;
  }

  if (code < 200 || code > 299) {
    outError = String("Conversation HTTP ") + code;
    const String codeText = sttHttpErrorText(code);
    if (codeText.length() > 0) {
      outError += ": " + codeText;
    }
    if (!requestOk && transportError.length() > 0) {
      outError += ": " + clipText(transportError, 64);
    }
    if (code == 401 || code == 403) {
      invalidateHaAuthProbeCache();
    }
    if (response.length() > 0) {
      outError += ": " + clipText(response, 80);
    }
    return false;
  }

  DynamicJsonDocument doc(4096);
  if (deserializeJson(doc, response)) {
    outError = "Conversation JSON parse failed";
    return false;
  }

  String nextConversationId = doc["conversation_id"] | "";
  nextConversationId.trim();
  if (nextConversationId.length() > 0) {
    haConversationId = nextConversationId;
  }

  outReply = doc["response"]["speech"]["plain"]["speech"] | "";
  if (outReply.length() == 0 && doc["response"]["speech"].is<const char*>()) {
    outReply = doc["response"]["speech"].as<const char*>();
  }
  if (outReply.length() == 0) {
    outReply = doc["response"]["data"]["message"] | "";
  }

  outReply.trim();
  if (outReply.length() == 0) {
    outError = "Conversation reply empty";
    return false;
  }

  return true;
}

bool normalizeUrl(String& url) {
  if (url.length() == 0) {
    return false;
  }
  const String baseUrl = trimmedConfig(HA_BASE_URL);
  if (baseUrl.length() == 0) {
    return false;
  }
  if (!url.startsWith("http://") && !url.startsWith("https://")) {
    if (!url.startsWith("/")) {
      url = "/" + url;
    }
    url = joinedUrl(baseUrl.c_str(), url);
  }
  return true;
}

bool homeAssistantTtsGetUrl(const String& text, String& outUrl, String& outError) {
  outUrl = "";
  outError = "";

  if (!isHomeAssistantConfigured()) {
    outError = "HA not configured";
    return false;
  }
  const String baseUrl = trimmedConfig(HA_BASE_URL);
  const String authHeader = authHeaderValue();
  const String ttsEngine = trimmedConfig(HA_TTS_ENGINE_ID);
  if (baseUrl.length() == 0 || authHeader.length() == 0) {
    outError = "HA config missing";
    return false;
  }

  if (ttsEngine.length() == 0) {
    outError = "TTS engine not configured";
    return false;
  }

  const String endpoint = joinedUrl(baseUrl.c_str(), "/api/tts_get_url");
  const String configuredLang = trimmedConfig(HA_LANGUAGE);
  const String preferredLang = ttsLanguageTag(configuredLang);

  const auto fetchUrl = [&](const String& language, bool preferWav, String& attemptError) -> bool {
    DynamicJsonDocument req(768);
    req["engine_id"] = ttsEngine;
    req["message"] = text;
    if (language.length() > 0) {
      req["language"] = language;
    }
    if (preferWav) {
      req["options"]["preferred_format"] = "wav";
    }

    String body;
    serializeJson(req, body);

    int code = 0;
    String response;
    String transportError;
    bool requestOk = false;
    if (endpoint.startsWith("http://")) {
      requestOk = postJsonOverRawHttp(
          endpoint,
          authHeader,
          body,
          code,
          response,
          transportError);
      if (!requestOk && transportError.length() > 0) {
        Serial.println(String("[TTS] raw-http error: ") + transportError);
      }
    } else {
      HTTPClient http;
      WiFiClient plainClient;
      WiFiClientSecure secureClient;
      if (!beginHttp(http, plainClient, secureClient, endpoint)) {
        attemptError = "TTS URL begin failed";
        return false;
      }

      http.setReuse(false);
      http.setConnectTimeout(STT_CONNECT_TIMEOUT_MS);
      http.setTimeout(STT_IO_TIMEOUT_MS);
      http.addHeader("Authorization", authHeader);
      http.addHeader("Content-Type", "application/json");

      code = http.POST(body);
      response = (code >= 200 && code <= 599) ? http.getString() : "";
      requestOk = true;
      http.end();
    }
    if (!requestOk && code == 0) {
      code = HTTPC_ERROR_NOT_CONNECTED;
    }

    if (code < 200 || code > 299) {
      attemptError = String("TTS URL HTTP ") + code;
      const String codeText = sttHttpErrorText(code);
      if (codeText.length() > 0) {
        attemptError += ": " + codeText;
      }
      if (!requestOk && transportError.length() > 0) {
        attemptError += ": " + clipText(transportError, 64);
      }
      if (code == 401 || code == 403) {
        invalidateHaAuthProbeCache();
      }
      if (response.length() > 0) {
        attemptError += ": " + clipText(response, 80);
      }
      return false;
    }

    DynamicJsonDocument doc(2048);
    if (deserializeJson(doc, response)) {
      attemptError = "TTS URL JSON parse failed";
      return false;
    }

    outUrl = doc["url"] | "";
    if (outUrl.length() == 0) {
      outUrl = doc["path"] | "";
    }
    if (!normalizeUrl(outUrl)) {
      attemptError = "TTS URL missing";
      return false;
    }

    String loweredUrl = outUrl;
    loweredUrl.toLowerCase();
    if (loweredUrl.endsWith(".mp3")) {
      attemptError = "TTS returned MP3 URL";
      return false;
    }

    return true;
  };

  String attemptError;
  if (fetchUrl(preferredLang, true, attemptError)) {
    return true;
  }

  if (configuredLang.length() > 0 && configuredLang != preferredLang) {
    if (fetchUrl(configuredLang, true, attemptError)) {
      return true;
    }
  }

  if (fetchUrl("", true, attemptError)) {
    return true;
  }

  outError = attemptError;
  return false;
}

bool homeAssistantPlayTtsWav(const String& ttsUrl, String& outError) {
  outError = "";
  const String authHeader = authHeaderValue();
  if (authHeader.length() == 0) {
    outError = "HA token missing";
    return false;
  }

  String resolved = ttsUrl;
  if (!normalizeUrl(resolved)) {
    outError = "TTS URL invalid";
    return false;
  }

  if (resolved.startsWith("http://")) {
    int code = 0;
    String contentType;
    String wavBody;
    String transportError;
    if (!getRawOverHttp(
            resolved,
            authHeader,
            code,
            contentType,
            wavBody,
            transportError)) {
      outError = String("TTS WAV raw GET failed");
      if (transportError.length() > 0) {
        outError += ": " + clipText(transportError, 72);
      }
      return false;
    }

    if (code < 200 || code > 299) {
      outError = String("TTS WAV HTTP ") + code;
      if (code == 401 || code == 403) {
        invalidateHaAuthProbeCache();
      }
      if (wavBody.length() > 0) {
        outError += ": " + clipText(wavBody, 80);
      }
      return false;
    }

    if (contentType.length() > 0 &&
        contentType.indexOf("audio/wav") < 0 &&
        contentType.indexOf("audio/x-wav") < 0 &&
        contentType.indexOf("audio/wave") < 0) {
      outError = "TTS content-type not WAV: " + clipText(contentType, 34);
      return false;
    }

    if (wavBody.length() == 0) {
      outError = "TTS WAV body empty";
      return false;
    }
    if (wavBody.length() > TTS_RAW_HTTP_MAX_WAV_BYTES) {
      outError = String("TTS WAV too large: ") + wavBody.length();
      return false;
    }

    MemoryReadStream wavStream(
        reinterpret_cast<const uint8_t*>(wavBody.c_str()),
        wavBody.length());
    const uint32_t playStartMs = millis();
    const bool ok = playWavFromStream(wavStream, outError);
    Serial.println(
        String("[TTS] raw_wav_bytes=") + wavBody.length() +
        " play_ms=" + (millis() - playStartMs) +
        " ok=" + (ok ? "1" : "0"));
    return ok;
  }

  HTTPClient http;
  WiFiClient plainClient;
  WiFiClientSecure secureClient;
  if (!beginHttp(http, plainClient, secureClient, resolved)) {
    outError = "TTS WAV begin failed";
    return false;
  }

  http.setReuse(false);
  http.useHTTP10(true);
  http.setConnectTimeout(STT_CONNECT_TIMEOUT_MS);
  http.setTimeout(STT_IO_TIMEOUT_MS);
  http.addHeader("Authorization", authHeader);
  const char* wantedHeaders[] = {"Content-Type"};
  http.collectHeaders(wantedHeaders, 1);

  const int code = http.GET();
  if (code < 200 || code > 299) {
    outError = String("TTS WAV HTTP ") + code;
    if (code == 401 || code == 403) {
      invalidateHaAuthProbeCache();
    }
    const String body = (code >= 200 && code <= 599) ? http.getString() : "";
    if (body.length() > 0) {
      outError += ": " + clipText(body, 80);
    }
    http.end();
    return false;
  }

  const String contentType = http.header("Content-Type");
  if (contentType.length() > 0 &&
      contentType.indexOf("audio/wav") < 0 &&
      contentType.indexOf("audio/x-wav") < 0 &&
      contentType.indexOf("audio/wave") < 0) {
    outError = "TTS content-type not WAV: " + clipText(contentType, 34);
    http.end();
    return false;
  }

  Stream* wavStream = http.getStreamPtr();
  if (!wavStream) {
    outError = "TTS stream missing";
    http.end();
    return false;
  }

  wavStream->setTimeout(TTS_STREAM_READ_TIMEOUT_MS);
  const uint32_t playStartMs = millis();
  const bool ok = playWavFromStream(*wavStream, outError);
  Serial.println(
      String("[TTS] wav_play_ms=") + (millis() - playStartMs) +
      " ok=" + (ok ? "1" : "0"));
  http.end();
  return ok;
}
void loadWifiConfig() {
  Preferences prefs;
  prefs.begin("voice_cfg", true);
  configuredSsid = prefs.getString("wifi_ssid", WIFI_SSID);
  configuredPassword = prefs.getString("wifi_pwd", WIFI_PASSWORD);
  prefs.end();
}

bool connectToWifi(const String& ssid, const String& password, uint32_t timeoutMs) {
  if (ssid.length() == 0) {
    return false;
  }

  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);

  if (WiFi.status() == WL_CONNECTED && WiFi.SSID() == ssid) {
    return true;
  }

  WiFi.begin(ssid.c_str(), password.c_str());

  const uint32_t startMs = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - startMs) < timeoutMs) {
    drawScreen(false);
    delay(150);
  }

  return WiFi.status() == WL_CONNECTED;
}

uint32_t wifiReconnectBackoffMs(uint8_t failures) {
  uint32_t delayMs = WIFI_RETRY_BASE_MS;
  uint8_t steps = failures > 0 ? static_cast<uint8_t>(failures - 1) : 0;
  if (steps > 5) {
    steps = 5;
  }

  while (steps > 0 && delayMs < WIFI_RETRY_MAX_MS) {
    if (delayMs > (WIFI_RETRY_MAX_MS / 2)) {
      delayMs = WIFI_RETRY_MAX_MS;
      break;
    }
    delayMs *= 2;
    steps--;
  }

  if (delayMs > WIFI_RETRY_MAX_MS) {
    delayMs = WIFI_RETRY_MAX_MS;
  }
  return delayMs;
}

void scheduleWifiReconnect(uint32_t nowMs, bool immediate, const String& reason) {
  if (reconnectSsid.length() == 0) {
    wifiReconnectPending = false;
    return;
  }

  const uint32_t delayMs = immediate ? 1200 : wifiReconnectBackoffMs(wifiReconnectFailures);
  wifiReconnectPending = true;
  wifiNextRetryAtMs = nowMs + delayMs;
  uiDirty = true;

  Serial.println(String("[WIFI] retry in ") + delayMs + " ms | " + reason);
}

bool stateBusy() {
  return state == AssistantState::Listening ||
         state == AssistantState::Transcribing ||
         state == AssistantState::Thinking ||
         state == AssistantState::Speaking;
}

uint32_t stateTimeoutMs(AssistantState s) {
  switch (s) {
    case AssistantState::Listening:
      return STAGE_TIMEOUT_LISTENING_MS;
    case AssistantState::Transcribing:
      return STAGE_TIMEOUT_TRANSCRIBING_MS;
    case AssistantState::Thinking:
      return STAGE_TIMEOUT_THINKING_MS;
    case AssistantState::Speaking:
      return STAGE_TIMEOUT_SPEAKING_MS;
    default:
      return ASSISTANT_BUSY_STATE_TIMEOUT_MS;
  }
}

void pollWifiConnection(uint32_t nowMs) {
  if ((nowMs - wifiLastPollMs) < WIFI_POLL_INTERVAL_MS) {
    return;
  }
  wifiLastPollMs = nowMs;

  const wl_status_t current = WiFi.status();
  if (current != wifiLastStatus) {
    Serial.println(String("[WIFI] status ") + static_cast<int>(wifiLastStatus) + " -> " + static_cast<int>(current));
    wifiLastStatus = current;
    uiDirty = true;

    if (current == WL_CONNECTED) {
      wifiReconnectPending = false;
      wifiReconnectFailures = 0;
      wifiNextRetryAtMs = 0;
      invalidateHaAuthProbeCache();
      if (state == AssistantState::ConnectingWiFi) {
        setState(AssistantState::Idle, "Wi-Fi connected");
      }
    } else if (reconnectSsid.length() > 0) {
      scheduleWifiReconnect(nowMs, true, "link lost");
      if (state == AssistantState::Idle) {
        setState(AssistantState::Idle, "Wi-Fi reconnecting");
      }
    }
  }

  if (!wifiReconnectPending || reconnectSsid.length() == 0 || WiFi.status() == WL_CONNECTED) {
    return;
  }

  if (stateBusy()) {
    return;
  }

  if (static_cast<int32_t>(wifiNextRetryAtMs - nowMs) > 0) {
    return;
  }

  setState(AssistantState::ConnectingWiFi, String("Wi-Fi retry #") + String(wifiReconnectFailures + 1));
  if (connectToWifi(reconnectSsid, reconnectPassword, WIFI_CONNECT_TIMEOUT_MS)) {
    wifiReconnectFailures = 0;
    wifiReconnectPending = false;
    wifiNextRetryAtMs = 0;
    setState(AssistantState::Idle, "Wi-Fi connected");
    return;
  }

  wifiReconnectFailures++;
  setState(AssistantState::Idle, "Wi-Fi offline");
  scheduleWifiReconnect(nowMs, false, "retry failed");
}

const char* captureSourceName(CaptureRequestSource source) {
  switch (source) {
    case CaptureRequestSource::Boot:
      return "BOOT";
    case CaptureRequestSource::Vad:
      return "VAD";
    case CaptureRequestSource::Wake:
      return "WAKE";
    default:
      return "UNKNOWN";
  }
}

void requestCapture(CaptureRequestSource source, const char* sourceLabel) {
  if (stateBusy()) {
    return;
  }
  captureSource = source;
  captureRequested = true;
  setState(AssistantState::Idle, String("Capture requested: ") + sourceLabel);
}

void requestCapture(const char* sourceLabel) {
  requestCapture(CaptureRequestSource::Unknown, sourceLabel);
}

void pollBootButton(uint32_t nowMs) {
  const bool pressedNow = digitalRead(PIN_BOOT_BUTTON) == LOW;

  if (pressedNow && !bootPressed) {
    bootPressed = true;
    bootPressStartMs = nowMs;
    return;
  }

  if (!pressedNow && bootPressed) {
    const uint32_t heldMs = nowMs - bootPressStartMs;
    bootPressed = false;

    if (heldMs < 1200) {
      requestCapture(CaptureRequestSource::Boot, "BOOT");
    }
  }
}

#if ASSISTANT_HAS_LOCAL_SR
esp_err_t localWakeFillCb(void* arg, void* out, size_t len, size_t* bytesRead, uint32_t timeoutMs) {
  (void)arg;
  if (bytesRead) {
    *bytesRead = 0;
  }
  if (!out || len == 0 || !bytesRead) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!handsFreeEnabled || !localWakeReady || localWakePaused) {
    vTaskDelay(pdMS_TO_TICKS(20));
    return ESP_FAIL;
  }
  if (!audioReady || state != AssistantState::Idle || captureRequested) {
    vTaskDelay(pdMS_TO_TICKS(12));
    return ESP_FAIL;
  }

  const uint32_t effectiveTimeout = timeoutMs == 0 || timeoutMs == UINT32_MAX ? 80 : timeoutMs;
  return micCapture.readForSr(out, len, bytesRead, effectiveTimeout);
}

void localWakeEventCb(void* arg, sr_event_t event, int commandId, int phraseId) {
  (void)arg;
  if (event != SR_EVENT_COMMAND || commandId != SR_CMD_WAKE_BOB) {
    return;
  }
  localWakePhraseId = phraseId;
  localWakeTriggerAtMs = millis();
  localWakeTriggerPending = true;
}
#endif

bool startLocalWakeEngine(String& outError) {
  outError = "";
  localWakeTriggerPending = false;
  localWakePhraseId = -1;

  if (!localWakeEnabled || !handsFreeEnabled) {
    return false;
  }

#if !ASSISTANT_HAS_LOCAL_SR
  outError = "ESP-SR unavailable";
  return false;
#else
  if (!audioReady) {
    outError = "audio not ready";
    return false;
  }
  if (localWakeReady) {
    return true;
  }
  if (!micCapture.prepareCaptureClock()) {
    outError = "capture clock setup failed";
    return false;
  }

  const esp_err_t err = sr_start(
      localWakeFillCb,
      nullptr,
      SR_CHANNELS_STEREO,
      SR_MODE_COMMAND,
      "MM",
      kLocalWakeCommands,
      sizeof(kLocalWakeCommands) / sizeof(kLocalWakeCommands[0]),
      localWakeEventCb,
      nullptr);
  if (err != ESP_OK) {
    outError = String("sr_start failed: ") + err;
    localWakeReady = false;
    return false;
  }

  localWakePaused = false;
  localWakeReady = true;
  Serial.println("[WAKE] local wake detector ready");
  return true;
#endif
}

void stopLocalWakeEngine() {
#if ASSISTANT_HAS_LOCAL_SR
  if (localWakeReady) {
    sr_stop();
  }
#endif
  localWakeReady = false;
  localWakePaused = false;
  localWakeTriggerPending = false;
  localWakePhraseId = -1;
}

bool pauseLocalWakeEngine() {
#if ASSISTANT_HAS_LOCAL_SR
  if (!localWakeReady || localWakePaused) {
    return false;
  }
  if (sr_pause() == ESP_OK) {
    localWakePaused = true;
    return true;
  }
#endif
  return false;
}

void resumeLocalWakeEngine() {
#if ASSISTANT_HAS_LOCAL_SR
  if (!localWakeReady || !localWakePaused) {
    return;
  }
  if (!micCapture.prepareCaptureClock()) {
    Serial.println("[WAKE] resume failed: capture clock");
    return;
  }
  if (sr_resume() == ESP_OK) {
    localWakePaused = false;
    localWakeTriggerPending = false;
  }
#endif
}

void pollLocalWakeTrigger(uint32_t nowMs) {
  if (!handsFreeEnabled || !localWakeEnabled || !localWakeReady) {
    return;
  }
  if (!localWakeTriggerPending || captureRequested || state != AssistantState::Idle) {
    return;
  }
  if (nowMs < handsFreeCooldownUntilMs) {
    return;
  }

  localWakeTriggerPending = false;
  handsFreeCooldownUntilMs = nowMs + LOCAL_WAKE_COOLDOWN_MS;
  handsFreeVadActiveFrames = 0;
  Serial.println(String("[WAKE] local trigger phrase=") + localWakePhraseId);
  requestCapture(CaptureRequestSource::Wake, "WAKE");
}

void pollHandsFreeTrigger(uint32_t nowMs) {
  if (!handsFreeEnabled || !audioReady || captureRequested || state != AssistantState::Idle) {
    return;
  }
  if (nowMs < handsFreeArmAtMs) {
    return;
  }
  if (nowMs < handsFreeCooldownUntilMs) {
    return;
  }

  if (localWakeEnabled && localWakeReady) {
#if ASSISTANT_LOCAL_WAKE_USE_VAD_FALLBACK
    // Keep VAD active as a fallback if requested.
#else
    return;
#endif
  }

  if ((nowMs - handsFreeVadLastPollMs) < HANDSFREE_VAD_POLL_INTERVAL_MS) {
    return;
  }
  handsFreeVadLastPollMs = nowMs;

  CaptureMetrics level;
  if (!micCapture.probeLevel(level, 128)) {
    handsFreeVadActiveFrames = 0;
    return;
  }
  handsFreeLastLevel = level;

  if (handsFreeNoiseRms <= 0.0f) {
    handsFreeNoiseRms = level.rmsNorm;
  } else {
    handsFreeNoiseRms = (handsFreeNoiseRms * 0.96f) + (level.rmsNorm * 0.04f);
  }

  float rmsTrigger = ASSISTANT_VAD_TRIGGER_RMS > 0.001f ? ASSISTANT_VAD_TRIGGER_RMS : 0.001f;
  const float adaptiveTrigger = handsFreeNoiseRms * 2.6f;
  if (adaptiveTrigger > rmsTrigger) {
    rmsTrigger = adaptiveTrigger;
  }
  const float peakTrigger = rmsTrigger * 2.8f;
  const bool hot = level.rmsNorm >= rmsTrigger || level.peakNorm >= peakTrigger;
  if (hot) {
    if (handsFreeVadActiveFrames < 20) {
      handsFreeVadActiveFrames++;
    }
  } else if (handsFreeVadActiveFrames > 0) {
    handsFreeVadActiveFrames--;
  }

  const uint8_t neededFrames = ASSISTANT_VAD_TRIGGER_FRAMES > 0 ? ASSISTANT_VAD_TRIGGER_FRAMES : 1;
  if (handsFreeVadActiveFrames >= neededFrames) {
    handsFreeVadActiveFrames = 0;
    handsFreeCooldownUntilMs = nowMs + HANDSFREE_VAD_COOLDOWN_MS;
    Serial.println(
        String("[VAD] hands-free trigger rms=") + String(level.rmsNorm, 4) +
        " peak=" + String(level.peakNorm, 4));
    requestCapture(CaptureRequestSource::Vad, "VAD");
  }
}

void registerWatchdogStrike(const String& reason) {
  const uint32_t nowMs = millis();
  if (watchdogWindowStartMs == 0 || (nowMs - watchdogWindowStartMs) > WATCHDOG_STRIKE_WINDOW_MS) {
    watchdogWindowStartMs = nowMs;
    watchdogStrikeCount = 0;
  }
  watchdogStrikeCount++;
  Serial.println(String("[WDT] strike ") + watchdogStrikeCount + "/" + ASSISTANT_WATCHDOG_MAX_STRIKES + " | " + reason);

  if (watchdogStrikeCount >= ASSISTANT_WATCHDOG_MAX_STRIKES) {
    Serial.println("[WDT] max strikes reached, restarting");
    delay(80);
    ESP.restart();
  }
}

bool recoverAudioPipeline(const String& reason, bool restartWakeEngine) {
  Serial.println(String("[AUDIO] recover start | ") + reason);

  stopLocalWakeEngine();
  bool ended = true;
  bool codecReady = true;
  bool usedDriverRestart = false;

  if (micCapture.isReady()) {
    usedDriverRestart = micCapture.restartDriver();
    if (usedDriverRestart) {
      audioReady = true;
      if (!micCapture.prepareCaptureClock()) {
        audioReady = false;
      }
    }
  }

  if (!audioReady) {
    ended = micCapture.end();
    delay(AUDIO_RECOVERY_SETTLE_MS);
    codecReady = codec.begin(Wire, 0x18);
    const bool micReady = codecReady && micCapture.begin(codec, CAPTURE_SAMPLE_RATE);
    audioReady = micReady;
    if (audioReady) {
      micCapture.flushIoBuffers();
      if (!micCapture.prepareCaptureClock()) {
        audioReady = false;
      }
    }
  }

  if (!audioReady) {
    Serial.println(
        String("[AUDIO] recover failed | end=") + (ended ? "1" : "0") +
        " codec=" + (codecReady ? "1" : "0") +
        " driver_restart=" + (usedDriverRestart ? "1" : "0"));
    return false;
  }

  if (restartWakeEngine && localWakeEnabled) {
    String wakeErr;
    if (!startLocalWakeEngine(wakeErr)) {
      Serial.println(String("[WAKE] restart failed: ") + wakeErr);
      localWakeEnabled = false;
    }
  }

  Serial.println(
      String("[AUDIO] recover ok | end=") + (ended ? "1" : "0") +
      " driver_restart=" + (usedDriverRestart ? "1" : "0"));
  return true;
}

void softRecoverRuntime(const String& reason) {
  registerWatchdogStrike(reason);
  captureRequested = false;
  captureSource = CaptureRequestSource::Unknown;
  invalidateHaAuthProbeCache();
  const bool audioRecovered = recoverAudioPipeline(reason, true);
  if (!audioRecovered) {
    setState(AssistantState::Error, "Recovery failed: audio pipeline");
    return;
  }
  setState(AssistantState::Idle, String("Recovered: ") + clipText(reason, 28));
}

void pollRuntimeHealth(uint32_t nowMs) {
  if ((nowMs - lastHealthPollMs) < RUNTIME_HEALTH_POLL_MS) {
    return;
  }
  lastHealthPollMs = nowMs;

  if (stateBusy()) {
    const uint32_t busyMs = nowMs - stateEnteredAtMs;
    const uint32_t stageTimeoutMs = stateTimeoutMs(state);
    if (busyMs > stageTimeoutMs) {
      softRecoverRuntime(
          String("state timeout ") + stateName(state) +
          " " + String(busyMs) + "/" + String(stageTimeoutMs) + "ms");
      return;
    }
  }

  const size_t internalFree = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  if (internalFree < ASSISTANT_LOW_HEAP_THRESHOLD_BYTES) {
    if (lowHeapConsecutive < 10) {
      lowHeapConsecutive++;
    }
  } else if (lowHeapConsecutive > 0) {
    lowHeapConsecutive--;
  }

  if (lowHeapConsecutive >= 3) {
    lowHeapConsecutive = 0;
    softRecoverRuntime(String("low heap ") + internalFree);
  }
}

void runAssistantCycle(CaptureRequestSource triggerSource) {
  if (!audioReady) {
    setState(AssistantState::Error, "Audio not initialized");
    return;
  }

  if (WiFi.status() != WL_CONNECTED) {
    setState(AssistantState::Error, "Wi-Fi disconnected");
    scheduleWifiReconnect(millis(), true, "capture while offline");
    return;
  }

  if (!isHomeAssistantConfigured()) {
    setState(AssistantState::Error, "HA not configured in secrets.h");
    return;
  }

  String authErr;
  if (!ensureHaAuthReady(authErr)) {
    setState(AssistantState::Error, "HA auth failed: " + clipText(authErr, 45));
    return;
  }

  struct LocalWakePauseGuard {
    bool paused = false;
    LocalWakePauseGuard() { paused = pauseLocalWakeEngine(); }
    ~LocalWakePauseGuard() {
      if (paused) {
        resumeLocalWakeEngine();
      }
    }
  } localWakePauseGuard;

  const auto stageExpired = [&](AssistantState stage, uint32_t startedAtMs, const char* label) -> bool {
    const uint32_t elapsedMs = millis() - startedAtMs;
    const uint32_t timeoutMs = stateTimeoutMs(stage);
    if (elapsedMs <= timeoutMs) {
      return false;
    }
    softRecoverRuntime(
        String(label) + " timeout " + elapsedMs + "/" + timeoutMs + "ms");
    return true;
  };

  const bool handsFreeTriggered =
      triggerSource == CaptureRequestSource::Vad || triggerSource == CaptureRequestSource::Wake;

  listeningFrameRms = 0.0f;
  listeningFramePeak = 0.0f;
  listeningCapturedMs = 0;
  listeningSpeechStarted = false;
  listeningUiLastRefreshMs = 0;
  setState(AssistantState::Listening, "Capturing audio");
  const uint32_t listeningStartedMs = millis();
  drawScreen(true);

  size_t captureMaxSamples = captureBufferCapacity;
  size_t captureMaxSeconds = ASSISTANT_CAPTURE_MAX_SECONDS;
  if (handsFreeTriggered && ASSISTANT_HANDSFREE_CAPTURE_MAX_SECONDS > 0) {
    captureMaxSeconds = ASSISTANT_HANDSFREE_CAPTURE_MAX_SECONDS;
  }
  const size_t configuredMaxSamples =
      static_cast<size_t>(CAPTURE_SAMPLE_RATE) * captureMaxSeconds;
  if (configuredMaxSamples > 0 && configuredMaxSamples < captureMaxSamples) {
    captureMaxSamples = configuredMaxSamples;
  }
  if (captureMaxSamples < CAPTURE_SAMPLE_RATE / 2) {
    captureMaxSamples = CAPTURE_SAMPLE_RATE / 2;
  }

  CaptureVadConfig vad;
  vad.startRmsNorm = ASSISTANT_VAD_START_RMS;
  vad.stopRmsNorm = ASSISTANT_VAD_STOP_RMS;
  vad.frameMs = 20;
  vad.maxWaitMs = handsFreeTriggered ? ASSISTANT_HANDSFREE_VAD_MAX_WAIT_MS : ASSISTANT_VAD_MAX_WAIT_MS;
  vad.trailingSilenceMs = handsFreeTriggered ? ASSISTANT_HANDSFREE_VAD_TRAILING_SILENCE_MS : ASSISTANT_VAD_TRAILING_SILENCE_MS;
  vad.minSpeechMs = handsFreeTriggered ? ASSISTANT_HANDSFREE_VAD_MIN_SPEECH_MS : ASSISTANT_VAD_MIN_SPEECH_MS;
  vad.preRollMs = handsFreeTriggered ? ASSISTANT_HANDSFREE_VAD_PREROLL_MS : ASSISTANT_VAD_PREROLL_MS;

  Serial.println(
      String("[MIC] trigger=") + captureSourceName(triggerSource) +
      " max_seconds=" + String(captureMaxSamples / static_cast<float>(CAPTURE_SAMPLE_RATE), 1) +
      " vad_wait_ms=" + vad.maxWaitMs +
      " vad_tail_ms=" + vad.trailingSilenceMs);

  CaptureResult captureResult;
  if (!micCapture.captureVoiceCommand(
          captureBuffer,
          captureMaxSamples,
          vad,
          captureResult,
          listeningCaptureProgressCallback,
          nullptr)) {
    if (recoverAudioPipeline("capture stall", true)) {
      setState(AssistantState::Idle, "Recovered: mic capture stall");
    } else {
      setState(AssistantState::Error, "Microphone capture failed");
    }
    return;
  }

  if (stageExpired(AssistantState::Listening, listeningStartedMs, "listening")) {
    return;
  }

  lastCaptureSamples = captureResult.sampleCount;
  lastCaptureMs = captureResult.speechMs;
  lastCaptureRms = captureResult.metrics.rmsNorm;
  lastCapturePeak = captureResult.metrics.peakNorm;
  Serial.println(
      String("[MIC] samples=") + lastCaptureSamples +
      " ms=" + lastCaptureMs +
      " rms=" + String(lastCaptureRms, 4) +
      " peak=" + String(lastCapturePeak, 4));

  if (!captureResult.speechDetected || captureResult.sampleCount == 0) {
    setState(AssistantState::Idle, "No speech detected");
    return;
  }
  if (captureResult.metrics.rmsNorm < CAPTURE_MIN_RMS &&
      captureResult.metrics.peakNorm < CAPTURE_MIN_PEAK) {
    setState(AssistantState::Idle, "Speech too quiet");
    return;
  }

  setState(AssistantState::Transcribing, "Sending to STT");
  const uint32_t transcribingStartedMs = millis();
  drawScreen(true);

  size_t sttSampleCount = captureResult.sampleCount;
  const size_t sttMaxSamples =
      static_cast<size_t>(CAPTURE_SAMPLE_RATE) * static_cast<size_t>(ASSISTANT_STT_UPLOAD_MAX_SECONDS);
  if (sttMaxSamples > 0 && sttSampleCount > sttMaxSamples) {
    Serial.println(
        String("[STT] trimming samples ") + sttSampleCount +
        " -> " + sttMaxSamples +
        " (" + String(sttMaxSamples / static_cast<float>(CAPTURE_SAMPLE_RATE), 1) + "s)");
    sttSampleCount = sttMaxSamples;
  }

  String transcript;
  String sttErr;
  if (!homeAssistantStt(captureBuffer, sttSampleCount, transcript, sttErr)) {
    setState(AssistantState::Error, "STT failed: " + sttErr);
    return;
  }
  if (stageExpired(AssistantState::Transcribing, transcribingStartedMs, "transcribing")) {
    return;
  }

  setTranscript(transcript);
  setState(AssistantState::Thinking, "Calling conversation");
  const uint32_t thinkingStartedMs = millis();
  drawScreen(true);

  String reply;
  String convErr;
  if (!homeAssistantConversation(transcript, reply, convErr)) {
    setState(AssistantState::Error, "Conversation failed: " + clipText(convErr, 48));
    return;
  }
  if (stageExpired(AssistantState::Thinking, thinkingStartedMs, "thinking")) {
    return;
  }

  setReply(reply);

  if (strlen(HA_TTS_ENGINE_ID) == 0) {
    setState(AssistantState::Idle, "Reply ready (TTS disabled)");
    return;
  }

  setState(AssistantState::Speaking, "Playing TTS");
  const uint32_t speakingStartedMs = millis();
  drawScreen(true);

  String ttsUrl;
  String ttsErr;
  if (!homeAssistantTtsGetUrl(reply, ttsUrl, ttsErr)) {
    setState(AssistantState::Error, "TTS URL failed: " + clipText(ttsErr, 50));
    return;
  }
  if (stageExpired(AssistantState::Speaking, speakingStartedMs, "speaking(tts-url)")) {
    return;
  }

  if (!homeAssistantPlayTtsWav(ttsUrl, ttsErr)) {
    const bool likelyAudioStall =
        ttsErr.indexOf("playback failed") >= 0 ||
        ttsErr.indexOf("WAV stream timeout") >= 0;
    if (likelyAudioStall && recoverAudioPipeline("tts playback stall", true)) {
      setState(AssistantState::Idle, "Recovered: TTS playback stall");
      return;
    }
    setState(AssistantState::Error, "TTS playback failed: " + clipText(ttsErr, 44));
    return;
  }
  if (stageExpired(AssistantState::Speaking, speakingStartedMs, "speaking(playback)")) {
    return;
  }

  setState(AssistantState::Idle, "Ready");
}
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("[FW] HA Voice Core clean restart");
  Serial.println(
      String("[CFG] base_len=") + trimmedConfig(HA_BASE_URL).length() +
      " token_len=" + normalizedAccessTokenValue().length() +
      " stt_len=" + trimmedConfig(HA_STT_PROVIDER).length() +
      " tts_len=" + trimmedConfig(HA_TTS_ENGINE_ID).length());

  pinMode(PIN_BOOT_BUTTON, INPUT_PULLUP);
  pinMode(PIN_TFT_BL, OUTPUT);
  digitalWrite(PIN_TFT_BL, HIGH);

  new (&tft) TFT_eSPI(SCREEN_WIDTH, SCREEN_HEIGHT);
  tft.begin();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  uiSprite.setColorDepth(16);
  uiSpriteReady = uiSprite.createSprite(SCREEN_WIDTH, SCREEN_HEIGHT) != nullptr;
  if (!uiSpriteReady) {
    Serial.println("[UI] Sprite allocate failed; using direct TFT rendering");
  }

  loadWifiConfig();
  reconnectSsid = configuredSsid;
  reconnectPassword = configuredPassword;

  Wire.begin(PIN_TOUCH_SDA, PIN_TOUCH_SCL, 400000);

  const bool codecReady = codec.begin(Wire, 0x18);
  audioReady = codecReady && micCapture.begin(codec, CAPTURE_SAMPLE_RATE);
  if (!audioReady) {
    setState(AssistantState::Error, "Audio init failed");
  }
  logMemoryStats("boot");

  if (psramFound()) {
    const size_t psramFree = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    const size_t reserveBytes = 1024 * 1024;
    const size_t preferredSamples =
        static_cast<size_t>(CAPTURE_SAMPLE_RATE) * static_cast<size_t>(ASSISTANT_CAPTURE_MAX_SECONDS);
    size_t targetBytes = preferredSamples * sizeof(int16_t);
    const size_t usableBytes = psramFree > reserveBytes ? (psramFree - reserveBytes) : (psramFree / 2);
    if (targetBytes > usableBytes) {
      targetBytes = usableBytes;
    }
    targetBytes -= (targetBytes % sizeof(int16_t));

    int16_t* psramCapture = nullptr;
    if (targetBytes >= (CAPTURE_SAMPLES * sizeof(int16_t) * 2)) {
      psramCapture = static_cast<int16_t*>(ps_malloc(targetBytes));
    }
    if (psramCapture) {
      captureBuffer = psramCapture;
      captureBufferCapacity = targetBytes / sizeof(int16_t);
      captureBufferInPsram = true;
      Serial.println(
          String("[MIC] PSRAM capture samples=") + captureBufferCapacity +
          " seconds=" + String(captureBufferCapacity / static_cast<float>(CAPTURE_SAMPLE_RATE), 1));
    }
  }
  Serial.println(
      String("[AI] capture_mode=VAD max_seconds=") +
      String(captureBufferCapacity / static_cast<float>(CAPTURE_SAMPLE_RATE), 1) +
      " psram=" + (captureBufferInPsram ? "yes" : "no"));
  logMemoryStats("post-buffer");

  handsFreeEnabled = (ASSISTANT_ENABLE_HANDS_FREE != 0);
  localWakeEnabled = handsFreeEnabled && (ASSISTANT_LOCAL_WAKE_ENABLED != 0);
  handsFreeArmAtMs = millis() + 8000;
  Serial.println(String("[WAKE] hands-free ") + (handsFreeEnabled ? "enabled" : "disabled"));
  Serial.println(String("[WAKE] local detector ") + (localWakeEnabled ? "enabled" : "disabled"));
#if ASSISTANT_HAS_LOCAL_SR
  Serial.println("[WAKE] phrases: ok mister bob | okay mister bob");
#endif
  if (localWakeEnabled) {
    String wakeErr;
    if (!startLocalWakeEngine(wakeErr)) {
      localWakeEnabled = false;
      localWakeReady = false;
      Serial.println(String("[WAKE] local disabled: ") + wakeErr);
    }
  }

  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);
  wifiLastStatus = WiFi.status();

  if (configuredSsid.length() > 0) {
    setState(AssistantState::ConnectingWiFi, String("Connecting ") + configuredSsid);
    if (connectToWifi(configuredSsid, configuredPassword, WIFI_CONNECT_TIMEOUT_MS)) {
      setState(AssistantState::Idle, "Ready");
    } else {
      setState(AssistantState::Idle, "Wi-Fi offline");
      scheduleWifiReconnect(millis(), true, "boot connect failed");
    }
  } else {
    setState(AssistantState::Error, "Set Wi-Fi in secrets.h or saved config");
  }

  if (!isHomeAssistantConfigured()) {
    setState(AssistantState::Error, "Set HA_BASE_URL/HA_ACCESS_TOKEN/HA_STT_PROVIDER");
  }

  drawScreen(true);
}

void loop() {
  const uint32_t nowMs = millis();

  pollBootButton(nowMs);
  pollLocalWakeTrigger(nowMs);
  pollHandsFreeTrigger(nowMs);
  pollWifiConnection(nowMs);
  pollRuntimeHealth(nowMs);

  if (captureRequested && (state == AssistantState::Idle || state == AssistantState::Error)) {
    const CaptureRequestSource triggerSource = captureSource;
    captureRequested = false;
    captureSource = CaptureRequestSource::Unknown;
    runAssistantCycle(triggerSource);
  }

  drawScreen(false);
  delay(6);
}
