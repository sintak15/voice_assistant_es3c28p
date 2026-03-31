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

static TFT_eSPI tft = TFT_eSPI(SCREEN_WIDTH, SCREEN_HEIGHT);
static lv_disp_draw_buf_t drawBuf;
static lv_color_t drawBufPixels[SCREEN_WIDTH * 20];

Ft6336Touch touch;
Es8311Codec codec;
I2SMicCapture micCapture;

AssistantState state = AssistantState::Booting;
String stateDetail = "Starting";

bool touchReady = false;
bool sdReady = false;
bool uiReady = false;

bool captureRequested = false;
bool wifiScanRequested = false;
bool wifiSaveRequested = false;
bool wifiConnectRequested = false;

bool bootPressed = false;
unsigned long bootPressStartMs = 0;

String configuredSsid;
String configuredPassword;

int16_t captureBuffer[CAPTURE_SAMPLES] = {0};
uint32_t captureIndex = 0;

lv_obj_t* uiTabview = nullptr;
lv_obj_t* uiLabelState = nullptr;
lv_obj_t* uiLabelDetail = nullptr;
lv_obj_t* uiLabelWifi = nullptr;
lv_obj_t* uiBtnTalk = nullptr;

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

void setState(AssistantState nextState, const String& detail) {
  state = nextState;
  stateDetail = detail;

  Serial.print("State -> ");
  Serial.print(stateName(nextState));
  Serial.print(" | ");
  Serial.println(detail);

  uiUpdateStatus();
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

bool beginHttp(HTTPClient& http, WiFiClientSecure& secureClient, const String& url) {
  if (url.startsWith("https://")) {
    secureClient.setInsecure();
    return http.begin(secureClient, url);
  }
  return http.begin(url);
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

  const size_t dataBytes = sampleCount * sizeof(int16_t);
  const size_t wavBytes = 44 + dataBytes;
  uint8_t* wav = static_cast<uint8_t*>(malloc(wavBytes));
  if (!wav) {
    outError = "No memory for WAV";
    return false;
  }

  writeWavHeader(wav, dataBytes, CAPTURE_SAMPLE_RATE);
  memcpy(wav + 44, samples, dataBytes);

  const String endpoint = joinedUrl(HA_BASE_URL, String("/api/stt/") + HA_STT_PROVIDER);
  HTTPClient http;
  WiFiClientSecure secureClient;
  if (!beginHttp(http, secureClient, endpoint)) {
    free(wav);
    outError = "HTTP begin failed";
    return false;
  }

  const String authHeader = String("Bearer ") + HA_ACCESS_TOKEN;
  const String speechHeader = String("language=") + HA_LANGUAGE +
                              "; format=wav; codec=pcm; bit_rate=16; sample_rate=16000; channel=1";

  http.addHeader("Authorization", authHeader);
  http.addHeader("Content-Type", "audio/wav");
  http.addHeader("X-Speech-Content", speechHeader);

  const int code = http.POST(wav, wavBytes);
  free(wav);

  if (code < 200 || code > 299) {
    outError = String("STT HTTP ") + code;
    http.end();
    return false;
  }

  const String response = http.getString();
  http.end();

  DynamicJsonDocument doc(2048);
  if (deserializeJson(doc, response)) {
    outError = "STT JSON parse failed";
    return false;
  }

  const String result = doc["result"] | "";
  outText = doc["text"] | "";
  if (result != "success" || outText.length() == 0) {
    outError = String("STT result: ") + result;
    return false;
  }

  return true;
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

void runCapturePipeline() {
  if (state != AssistantState::Idle) {
    return;
  }

  uiSetTalkEnabled(false);
  setState(AssistantState::Listening, "Capturing 3s audio...");
  uiRefreshNow();

  CaptureMetrics metrics{};
  if (!micCapture.captureBlocking(captureBuffer, CAPTURE_SAMPLES, metrics)) {
    setState(AssistantState::Error, "Mic capture failed");
    uiSetTalkEnabled(true);
    return;
  }

  String savedPath;
  String saveErr;
  if (saveCaptureToSd(captureBuffer, CAPTURE_SAMPLES, savedPath, saveErr)) {
    Serial.println(String("[SD] saved: ") + savedPath);
  }

  const float rmsDb = 20.0f * log10f(metrics.rmsNorm > 0.000001f ? metrics.rmsNorm : 0.000001f);
  setState(AssistantState::Thinking, String("RMS ") + String(rmsDb, 1) + " dBFS");
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
  if (!homeAssistantStt(captureBuffer, CAPTURE_SAMPLES, transcript, sttErr)) {
    setState(AssistantState::Error, "STT failed: " + sttErr);
    uiSetTalkEnabled(true);
    return;
  }

  String reply;
  String convErr;
  setState(AssistantState::Thinking, "Processing intent...");
  uiRefreshNow();
  if (!homeAssistantConversation(transcript, reply, convErr)) {
    setState(AssistantState::Error, "Intent failed: " + convErr);
    uiSetTalkEnabled(true);
    return;
  }

  setState(AssistantState::Speaking, "HA: " + clipText(reply, 40));
  delay(1500);
  setState(AssistantState::Idle, "Ready");
  uiSetTalkEnabled(true);
}
