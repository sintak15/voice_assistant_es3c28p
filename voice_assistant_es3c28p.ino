
#include "tft_espi_local_setup.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <TFT_eSPI.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include <cstring>

#include "board_pins.h"
#include "es8311_codec.h"
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
  Transcribing,
  Thinking,
  Speaking,
  Error
};

static const uint16_t SCREEN_WIDTH = 240;
static const uint16_t SCREEN_HEIGHT = 320;

static const uint32_t CAPTURE_SAMPLE_RATE = 16000;
static const uint16_t CAPTURE_SECONDS = 4;
static const size_t CAPTURE_SAMPLES = CAPTURE_SAMPLE_RATE * CAPTURE_SECONDS;
static const float CAPTURE_MIN_RMS = 0.010f;
static const float CAPTURE_MIN_PEAK = 0.030f;

static const uint16_t STT_CONNECT_TIMEOUT_MS = 7000;
static const uint16_t STT_IO_TIMEOUT_MS = 14000;
static const uint8_t STT_MAX_RETRIES = 2;

static const uint32_t WIFI_POLL_INTERVAL_MS = 700;
static const uint32_t WIFI_RETRY_BASE_MS = 2000;
static const uint32_t WIFI_RETRY_MAX_MS = 60000;
static const uint32_t WIFI_CONNECT_TIMEOUT_MS = 12000;

static TFT_eSPI tft = TFT_eSPI(SCREEN_WIDTH, SCREEN_HEIGHT);
static Es8311Codec codec;
static I2SMicCapture micCapture;

static AssistantState state = AssistantState::Booting;
static String stateDetail = "Starting";

static bool audioReady = false;
static bool captureRequested = false;

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

static String lastTranscript;
static String lastReply;

static bool uiDirty = true;
static uint32_t lastUiDrawMs = 0;

static int16_t captureBufferStorage[CAPTURE_SAMPLES] = {0};
static int16_t* captureBuffer = captureBufferStorage;
static size_t captureBufferCapacity = CAPTURE_SAMPLES;

String clipText(const String& text, size_t maxLen) {
  if (text.length() <= maxLen) {
    return text;
  }
  if (maxLen < 4) {
    return text.substring(0, maxLen);
  }
  return text.substring(0, maxLen - 3) + "...";
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

void drawScreen(bool force) {
  const uint32_t nowMs = millis();
  if (!force && !uiDirty && (nowMs - lastUiDrawMs) < 350) {
    return;
  }

  uiDirty = false;
  lastUiDrawMs = nowMs;

  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(TL_DATUM);

  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawString("HA Voice Core", 8, 8, 2);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(String("State: ") + stateName(state), 8, 34, 2);

  uint16_t detailColor = TFT_LIGHTGREY;
  if (state == AssistantState::Error) {
    detailColor = TFT_RED;
  }
  tft.setTextColor(detailColor, TFT_BLACK);
  tft.drawString(clipText(stateDetail, 46), 8, 52, 2);

  if (WiFi.status() == WL_CONNECTED) {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.drawString(clipText(String("Wi-Fi: ") + WiFi.SSID() + " " + String(WiFi.RSSI()) + " dBm", 50), 8, 74, 2);
  } else {
    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    if (wifiReconnectPending) {
      int32_t waitMs = static_cast<int32_t>(wifiNextRetryAtMs - nowMs);
      if (waitMs < 0) {
        waitMs = 0;
      }
      tft.drawString(String("Wi-Fi retry in ") + String((waitMs + 999) / 1000) + "s", 8, 74, 2);
    } else {
      tft.drawString("Wi-Fi disconnected", 8, 74, 2);
    }
  }

  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.drawString("Transcript", 8, 108, 2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(clipText(lastTranscript.length() ? lastTranscript : "(none)", 62), 8, 126, 2);

  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.drawString("Reply", 8, 168, 2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(clipText(lastReply.length() ? lastReply : "(none)", 62), 8, 186, 2);

  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString("Press BOOT for capture", 8, 288, 2);
}

void setState(AssistantState nextState, const String& detail) {
  state = nextState;
  stateDetail = detail;
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
  return strlen(HA_BASE_URL) > 0 && strlen(HA_ACCESS_TOKEN) > 0 && strlen(HA_STT_PROVIDER) > 0;
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

bool beginHttp(HTTPClient& http, WiFiClientSecure& secureClient, const String& url) {
  if (url.startsWith("https://")) {
    secureClient.setInsecure();
    secureClient.setTimeout(STT_IO_TIMEOUT_MS);
    return http.begin(secureClient, url);
  }
  return http.begin(url);
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
      readBytes = stream.readBytes(reinterpret_cast<char*>(inSamples), toRead);
      if (readBytes == 0) {
        if ((millis() - lastProgressMs) > 1500) {
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
bool homeAssistantStt(const int16_t* samples, size_t sampleCount, String& outText, String& outError) {
  outText = "";
  outError = "";

  if (!isHomeAssistantConfigured()) {
    outError = "HA not configured";
    return false;
  }

  const String endpoint = joinedUrl(HA_BASE_URL, String("/api/stt/") + HA_STT_PROVIDER);
  const String authHeader = String("Bearer ") + HA_ACCESS_TOKEN;

  const auto tryLanguage = [&](const String& language, String& languageError) -> bool {
    const String speechHeader = String("language=") + language +
                                ";format=wav;codec=pcm;bit_rate=16;sample_rate=16000;channel=1";

    for (uint8_t retry = 0; retry <= STT_MAX_RETRIES; ++retry) {
      if (WiFi.status() != WL_CONNECTED) {
        languageError = "Wi-Fi disconnected";
        return false;
      }

      HTTPClient http;
      WiFiClientSecure secureClient;
      if (!beginHttp(http, secureClient, endpoint)) {
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

      WavUploadStream wav(samples, sampleCount, CAPTURE_SAMPLE_RATE);
      const int code = http.sendRequest("POST", &wav, wav.totalSize());
      const String response = (code >= 200 && code <= 599) ? http.getString() : "";
      http.end();

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

      if (isRetryableSttTransportError(code) && retry < STT_MAX_RETRIES) {
        delay(200 + retry * 200);
        continue;
      }
      return false;
    }

    return false;
  };

  if (tryLanguage(String(HA_LANGUAGE), outError)) {
    return true;
  }

  if (!outError.startsWith("STT HTTP 415")) {
    return false;
  }

  const String fallback = baseLanguageTag(String(HA_LANGUAGE));
  if (fallback.length() == 0 || fallback == String(HA_LANGUAGE)) {
    return false;
  }

  String fallbackError;
  if (tryLanguage(fallback, fallbackError)) {
    return true;
  }

  outError = fallbackError;
  return false;
}

bool homeAssistantConversation(const String& text, String& outReply, String& outError) {
  outReply = "";
  outError = "";

  if (!isHomeAssistantConfigured()) {
    outError = "HA not configured";
    return false;
  }

  const String endpoint = joinedUrl(HA_BASE_URL, "/api/conversation/process");
  HTTPClient http;
  WiFiClientSecure secureClient;
  if (!beginHttp(http, secureClient, endpoint)) {
    outError = "Conversation HTTP begin failed";
    return false;
  }

  http.setReuse(false);
  http.setConnectTimeout(STT_CONNECT_TIMEOUT_MS);
  http.setTimeout(STT_IO_TIMEOUT_MS);
  http.addHeader("Authorization", String("Bearer ") + HA_ACCESS_TOKEN);
  http.addHeader("Content-Type", "application/json");

  DynamicJsonDocument req(768);
  req["text"] = text;
  req["language"] = HA_LANGUAGE;
  if (strlen(HA_AGENT_ID) > 0) {
    req["agent_id"] = HA_AGENT_ID;
  }

  String body;
  serializeJson(req, body);

  const int code = http.POST(body);
  const String response = (code >= 200 && code <= 599) ? http.getString() : "";
  http.end();

  if (code < 200 || code > 299) {
    outError = String("Conversation HTTP ") + code;
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
  if (!url.startsWith("http://") && !url.startsWith("https://")) {
    if (!url.startsWith("/")) {
      url = "/" + url;
    }
    url = joinedUrl(HA_BASE_URL, url);
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
  if (strlen(HA_TTS_ENGINE_ID) == 0) {
    outError = "TTS engine not configured";
    return false;
  }

  const String endpoint = joinedUrl(HA_BASE_URL, "/api/tts_get_url");
  HTTPClient http;
  WiFiClientSecure secureClient;
  if (!beginHttp(http, secureClient, endpoint)) {
    outError = "TTS URL begin failed";
    return false;
  }

  http.setReuse(false);
  http.setConnectTimeout(STT_CONNECT_TIMEOUT_MS);
  http.setTimeout(STT_IO_TIMEOUT_MS);
  http.addHeader("Authorization", String("Bearer ") + HA_ACCESS_TOKEN);
  http.addHeader("Content-Type", "application/json");

  DynamicJsonDocument req(768);
  req["engine_id"] = HA_TTS_ENGINE_ID;
  req["message"] = text;
  req["language"] = HA_LANGUAGE;
  req["options"]["preferred_format"] = "wav";

  String body;
  serializeJson(req, body);

  const int code = http.POST(body);
  const String response = (code >= 200 && code <= 599) ? http.getString() : "";
  http.end();

  if (code < 200 || code > 299) {
    outError = String("TTS URL HTTP ") + code;
    if (response.length() > 0) {
      outError += ": " + clipText(response, 80);
    }
    return false;
  }

  DynamicJsonDocument doc(2048);
  if (deserializeJson(doc, response)) {
    outError = "TTS URL JSON parse failed";
    return false;
  }

  outUrl = doc["url"] | "";
  if (outUrl.length() == 0) {
    outUrl = doc["path"] | "";
  }

  if (!normalizeUrl(outUrl)) {
    outError = "TTS URL missing";
    return false;
  }

  return true;
}

bool homeAssistantPlayTtsWav(const String& ttsUrl, String& outError) {
  outError = "";
  String resolved = ttsUrl;
  if (!normalizeUrl(resolved)) {
    outError = "TTS URL invalid";
    return false;
  }

  HTTPClient http;
  WiFiClientSecure secureClient;
  if (!beginHttp(http, secureClient, resolved)) {
    outError = "TTS WAV begin failed";
    return false;
  }

  http.setReuse(false);
  http.useHTTP10(true);
  http.setConnectTimeout(STT_CONNECT_TIMEOUT_MS);
  http.setTimeout(STT_IO_TIMEOUT_MS);
  http.addHeader("Authorization", String("Bearer ") + HA_ACCESS_TOKEN);

  const int code = http.GET();
  if (code < 200 || code > 299) {
    outError = String("TTS WAV HTTP ") + code;
    const String body = (code >= 200 && code <= 599) ? http.getString() : "";
    if (body.length() > 0) {
      outError += ": " + clipText(body, 80);
    }
    http.end();
    return false;
  }

  Stream* wavStream = http.getStreamPtr();
  if (!wavStream) {
    outError = "TTS stream missing";
    http.end();
    return false;
  }

  const bool ok = playWavFromStream(*wavStream, outError);
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

void requestCapture(const char* source) {
  if (stateBusy()) {
    return;
  }
  captureRequested = true;
  setState(AssistantState::Idle, String("Capture requested: ") + source);
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
      requestCapture("BOOT");
    }
  }
}

void runAssistantCycle() {
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

  setState(AssistantState::Listening, "Capturing audio");
  drawScreen(true);

  const size_t captureSamples = captureBufferCapacity < CAPTURE_SAMPLES ? captureBufferCapacity : CAPTURE_SAMPLES;
  CaptureMetrics metrics;
  if (!micCapture.captureBlocking(captureBuffer, captureSamples, metrics)) {
    setState(AssistantState::Error, "Microphone capture failed");
    return;
  }

  if (metrics.rmsNorm < CAPTURE_MIN_RMS && metrics.peakNorm < CAPTURE_MIN_PEAK) {
    setState(AssistantState::Idle, "No speech detected");
    return;
  }

  setState(AssistantState::Transcribing, "Sending to STT");
  drawScreen(true);

  String transcript;
  String sttErr;
  if (!homeAssistantStt(captureBuffer, captureSamples, transcript, sttErr)) {
    setState(AssistantState::Error, "STT failed: " + clipText(sttErr, 54));
    return;
  }

  setTranscript(transcript);
  setState(AssistantState::Thinking, "Calling conversation");
  drawScreen(true);

  String reply;
  String convErr;
  if (!homeAssistantConversation(transcript, reply, convErr)) {
    setState(AssistantState::Error, "Conversation failed: " + clipText(convErr, 48));
    return;
  }

  setReply(reply);

  if (strlen(HA_TTS_ENGINE_ID) == 0) {
    setState(AssistantState::Idle, "Reply ready (TTS disabled)");
    return;
  }

  setState(AssistantState::Speaking, "Playing TTS");
  drawScreen(true);

  String ttsUrl;
  String ttsErr;
  if (!homeAssistantTtsGetUrl(reply, ttsUrl, ttsErr)) {
    setState(AssistantState::Error, "TTS URL failed: " + clipText(ttsErr, 50));
    return;
  }

  if (!homeAssistantPlayTtsWav(ttsUrl, ttsErr)) {
    setState(AssistantState::Error, "TTS playback failed: " + clipText(ttsErr, 44));
    return;
  }

  setState(AssistantState::Idle, "Ready");
}
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("[FW] HA Voice Core clean restart");

  pinMode(PIN_BOOT_BUTTON, INPUT_PULLUP);
  pinMode(PIN_TFT_BL, OUTPUT);
  digitalWrite(PIN_TFT_BL, HIGH);

  new (&tft) TFT_eSPI(SCREEN_WIDTH, SCREEN_HEIGHT);
  tft.begin();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);

  loadWifiConfig();
  reconnectSsid = configuredSsid;
  reconnectPassword = configuredPassword;

  Wire.begin(PIN_TOUCH_SDA, PIN_TOUCH_SCL, 400000);

  const bool codecReady = codec.begin(Wire, 0x18);
  audioReady = codecReady && micCapture.begin(codec, CAPTURE_SAMPLE_RATE);
  if (!audioReady) {
    setState(AssistantState::Error, "Audio init failed");
  }

  if (psramFound()) {
    const size_t desiredSamples = CAPTURE_SAMPLE_RATE * 6;
    int16_t* psramCapture = static_cast<int16_t*>(ps_malloc(desiredSamples * sizeof(int16_t)));
    if (psramCapture) {
      captureBuffer = psramCapture;
      captureBufferCapacity = desiredSamples;
      Serial.println(String("[MIC] PSRAM capture samples=") + captureBufferCapacity);
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
  pollWifiConnection(nowMs);

  if (captureRequested && (state == AssistantState::Idle || state == AssistantState::Error)) {
    captureRequested = false;
    runAssistantCycle();
  }

  drawScreen(false);
  delay(6);
}
