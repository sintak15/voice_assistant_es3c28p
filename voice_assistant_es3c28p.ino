
#include "tft_espi_local_setup.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <TFT_eSPI.h>
#include <lvgl.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include <esp_heap_caps.h>
#include <cstring>
#include <math.h>
#include <stdlib.h>

#include "board_pins.h"
#include "es8311_codec.h"
#include "ft6336_touch.h"
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
#define ASSISTANT_LOW_HEAP_THRESHOLD_BYTES 7000
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
static const uint16_t UI_TAB_INDEX_LAUNCHER = 0;
static const uint16_t UI_TAB_INDEX_ASSISTANT = 1;
static const uint16_t UI_TAB_INDEX_CONTROLS = 2;
static const uint16_t UI_TAB_INDEX_DIAGNOSTICS = 3;
static const uint16_t UI_TAB_INDEX_LOGS = 4;
static const uint16_t UI_TAB_COUNT = 5;

#if ASSISTANT_HAS_LOCAL_SR
#include <esp32-hal-sr.h>
enum : int {
  SR_CMD_WAKE_BOB = 1,
  SR_CMD_CANCEL = 2,
  SR_CMD_REPEAT = 3,
  SR_CMD_STATUS = 4,
};
static const sr_cmd_t kLocalWakeCommands[] = {
    {SR_CMD_WAKE_BOB, "ok mister bob", "bKd MgSTk BnB"},
    {SR_CMD_WAKE_BOB, "okay mister bob", "bKd MgSTk BnB"},
    {SR_CMD_WAKE_BOB, "ok mr bob", "bKd MgSTk BnB"},
    {SR_CMD_WAKE_BOB, "okay mr bob", "bKd MgSTk BnB"},
};
static const sr_mode_t LOCAL_WAKE_SR_MODE = SR_MODE_COMMAND;
#endif

enum class CaptureRequestSource : uint8_t {
  Unknown = 0,
  Boot = 1,
  Vad = 2,
  Wake = 3,
};

static TFT_eSPI tft = TFT_eSPI(SCREEN_WIDTH, SCREEN_HEIGHT);
static lv_disp_draw_buf_t lvDrawBuf;
static lv_color_t* lvDrawPixels = nullptr;
static Ft6336Touch touch;
static bool touchReady = false;
static bool uiReady = false;
static lv_obj_t* uiTabview = nullptr;
static lv_obj_t* uiLabelLauncherState = nullptr;
static lv_obj_t* uiLabelLauncherWifi = nullptr;
static lv_obj_t* uiLabelAssistantState = nullptr;
static lv_obj_t* uiLabelAssistantDetail = nullptr;
static lv_obj_t* uiLabelAssistantWifi = nullptr;
static lv_obj_t* uiLabelAssistantMic = nullptr;
static lv_obj_t* uiLabelAssistantHint = nullptr;
static lv_obj_t* uiLabelControlsStatus = nullptr;
static lv_obj_t* uiBtnControlCapture = nullptr;
static lv_obj_t* uiBtnControlRepeat = nullptr;
static lv_obj_t* uiBtnControlWifi = nullptr;
static lv_obj_t* uiBtnControlRecover = nullptr;
static lv_obj_t* uiLabelDiagnosticsRun = nullptr;
static lv_obj_t* uiLabelDiagnosticsStageA = nullptr;
static lv_obj_t* uiLabelDiagnosticsStageB = nullptr;
static lv_obj_t* uiLabelDiagnosticsMic = nullptr;
static lv_obj_t* uiLabelDiagnosticsRuntime = nullptr;
static lv_obj_t* uiTaTranscript = nullptr;
static lv_obj_t* uiTaReply = nullptr;
static bool uiActionCapture = false;
static bool uiActionRepeat = false;
static bool uiActionWifiRetry = false;
static bool uiActionRecoverAudio = false;
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
static volatile bool localWakeStartInProgress = false;
static volatile bool localWakeStopRequested = false;
static volatile bool localWakeTriggerPending = false;
static volatile uint32_t localWakeTriggerAtMs = 0;
static volatile int localWakeCommandId = -1;
static volatile int localWakePhraseId = -1;
static uint8_t handsFreeVadActiveFrames = 0;
static uint32_t handsFreeCooldownUntilMs = 0;
static uint32_t handsFreeVadLastPollMs = 0;
static uint32_t handsFreeArmAtMs = 0;
static float handsFreeNoiseRms = 0.0f;
static CaptureMetrics handsFreeLastLevel;
static uint32_t handsFreeLastLevelMs = 0;
static volatile float localWakeLiveRms = 0.0f;
static volatile float localWakeLivePeak = 0.0f;
static volatile uint32_t localWakeLiveUpdatedMs = 0;
static volatile uint32_t localWakeFillOkCount = 0;
static volatile uint32_t localWakeFillFailCount = 0;

static bool bootPressed = false;
static uint32_t bootLastTriggerMs = 0;

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
static uint32_t lastWakeStatsLogMs = 0;
static bool listeningSpeechStarted = false;
static uint32_t listeningUiLastRefreshMs = 0;
static uint32_t successfulCycleCount = 0;
static uint32_t errorStateCount = 0;
static uint32_t lastCycleTotalMs = 0;
static uint32_t lastCycleListeningMs = 0;
static uint32_t lastCycleSttMs = 0;
static uint32_t lastCycleThinkingMs = 0;
static uint32_t lastCycleTtsUrlMs = 0;
static uint32_t lastCyclePlaybackMs = 0;
static String lastCycleResult = "(none)";

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

String formatMsCompact(uint32_t ms) {
  if (ms == 0) {
    return "-";
  }
  if (ms >= 10000) {
    return String(ms / 1000) + "s";
  }
  return String(ms / 1000.0f, 1) + "s";
}

void updateLiveMicStatsFromPcm16(const int16_t* samples, size_t sampleCount) {
  if (!samples || sampleCount == 0) {
    return;
  }

  uint32_t peak = 0;
  double sumSq = 0.0;
  for (size_t i = 0; i < sampleCount; ++i) {
    const int32_t v = static_cast<int32_t>(samples[i]);
    const uint32_t absV = static_cast<uint32_t>(v < 0 ? -v : v);
    if (absV > peak) {
      peak = absV;
    }
    sumSq += static_cast<double>(v) * static_cast<double>(v);
  }

  localWakeLiveRms = static_cast<float>(sqrt(sumSq / static_cast<double>(sampleCount)) / 32768.0);
  localWakeLivePeak = static_cast<float>(peak) / 32768.0f;
  localWakeLiveUpdatedMs = millis();
}

void uiUpdateStatus(uint32_t nowMs);

void drawScreen(bool force) {
  const uint32_t nowMs = millis();
  const uint32_t minRefreshMs = state == AssistantState::Listening ? 85 : 260;

  if (uiReady) {
    if (force || uiDirty || (nowMs - lastUiDrawMs) >= minRefreshMs) {
      uiUpdateStatus(nowMs);
      uiDirty = false;
      lastUiDrawMs = nowMs;
    }
    lv_timer_handler();
    return;
  }

  if (!force && !uiDirty && (nowMs - lastUiDrawMs) < 450) {
    return;
  }
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("UI offline", 8, 8, 2);
  tft.drawString(String("State: ") + stateName(state), 8, 30, 2);
  tft.drawString(clipText(stateDetail, 40), 8, 52, 2);
  uiDirty = false;
  lastUiDrawMs = nowMs;
}

lv_color_t uiStateColor(AssistantState s) {
  switch (s) {
    case AssistantState::Booting:
      return lv_color_hex(0x93C5FD);
    case AssistantState::ConnectingWiFi:
      return lv_color_hex(0x60A5FA);
    case AssistantState::Idle:
      return lv_color_hex(0x34D399);
    case AssistantState::Listening:
      return lv_color_hex(0xF59E0B);
    case AssistantState::Transcribing:
      return lv_color_hex(0xFBBF24);
    case AssistantState::Thinking:
      return lv_color_hex(0xC4B5FD);
    case AssistantState::Speaking:
      return lv_color_hex(0x2DD4BF);
    case AssistantState::Error:
      return lv_color_hex(0xF87171);
  }
  return lv_color_hex(0xE5E7EB);
}

const char* uiStateHint(AssistantState s) {
  switch (s) {
    case AssistantState::Booting:
      return "Bringing up display, audio, and network";
    case AssistantState::ConnectingWiFi:
      return "Joining Wi-Fi and validating cloud";
    case AssistantState::Idle:
      return "Tap Capture, or press BOOT to talk";
    case AssistantState::Listening:
      return "Listening with VAD and live levels";
    case AssistantState::Transcribing:
      return "Uploading audio to STT";
    case AssistantState::Thinking:
      return "Waiting for conversation response";
    case AssistantState::Speaking:
      return "Streaming TTS playback";
    case AssistantState::Error:
      return "Use Recover Audio, then retry";
  }
  return "";
}

TouchPoint mapTouchToRotation0(const TouchPoint& raw) {
  int mappedX = raw.x;
  int mappedY = raw.y;
  if (mappedX < 0) {
    mappedX = 0;
  }
  if (mappedX >= SCREEN_WIDTH) {
    mappedX = SCREEN_WIDTH - 1;
  }
  if (mappedY < 0) {
    mappedY = 0;
  }
  if (mappedY >= SCREEN_HEIGHT) {
    mappedY = SCREEN_HEIGHT - 1;
  }
  return {
      static_cast<uint16_t>(mappedX),
      static_cast<uint16_t>(mappedY),
  };
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

  const TouchPoint mapped = mapTouchToRotation0(raw);
  data->state = LV_INDEV_STATE_PR;
  data->point.x = mapped.x;
  data->point.y = mapped.y;
}

void uiSetControlsStatus(const String& text) {
  if (uiLabelControlsStatus) {
    lv_label_set_text(uiLabelControlsStatus, clipText(text, 70).c_str());
  }
  uiDirty = true;
}

void uiOpenLauncherTabEventCb(lv_event_t* e) {
  if (lv_event_get_code(e) == LV_EVENT_CLICKED && uiTabview) {
    lv_tabview_set_act(uiTabview, UI_TAB_INDEX_LAUNCHER, LV_ANIM_ON);
  }
}

void uiOpenAssistantTabEventCb(lv_event_t* e) {
  if (lv_event_get_code(e) == LV_EVENT_CLICKED && uiTabview) {
    lv_tabview_set_act(uiTabview, UI_TAB_INDEX_ASSISTANT, LV_ANIM_ON);
  }
}

void uiOpenControlsTabEventCb(lv_event_t* e) {
  if (lv_event_get_code(e) == LV_EVENT_CLICKED && uiTabview) {
    lv_tabview_set_act(uiTabview, UI_TAB_INDEX_CONTROLS, LV_ANIM_ON);
  }
}

void uiOpenDiagnosticsTabEventCb(lv_event_t* e) {
  if (lv_event_get_code(e) == LV_EVENT_CLICKED && uiTabview) {
    lv_tabview_set_act(uiTabview, UI_TAB_INDEX_DIAGNOSTICS, LV_ANIM_ON);
  }
}

void uiOpenLogsTabEventCb(lv_event_t* e) {
  if (lv_event_get_code(e) == LV_EVENT_CLICKED && uiTabview) {
    lv_tabview_set_act(uiTabview, UI_TAB_INDEX_LOGS, LV_ANIM_ON);
  }
}

void uiCaptureButtonEventCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }
  uiActionCapture = true;
  uiSetControlsStatus("Capture requested");
}

void uiRepeatButtonEventCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }
  uiActionRepeat = true;
  uiSetControlsStatus("Repeat requested");
}

void uiWifiRetryButtonEventCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }
  uiActionWifiRetry = true;
  uiSetControlsStatus("Wi-Fi reconnect requested");
}

void uiRecoverAudioButtonEventCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }
  uiActionRecoverAudio = true;
  uiSetControlsStatus("Audio recovery requested");
}

void uiStylePanel(lv_obj_t* panel, uint32_t bgColor, uint32_t borderColor) {
  lv_obj_set_style_bg_color(panel, lv_color_hex(bgColor), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(panel, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_width(panel, 1, LV_PART_MAIN);
  lv_obj_set_style_border_color(panel, lv_color_hex(borderColor), LV_PART_MAIN);
  lv_obj_set_style_radius(panel, 10, LV_PART_MAIN);
}

void uiStyleLauncherButton(lv_obj_t* button, uint32_t borderColor, uint32_t bgColor) {
  lv_obj_set_style_radius(button, 10, LV_PART_MAIN);
  lv_obj_set_style_bg_color(button, lv_color_hex(bgColor), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(button, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_width(button, 1, LV_PART_MAIN);
  lv_obj_set_style_border_color(button, lv_color_hex(borderColor), LV_PART_MAIN);
  lv_obj_set_style_shadow_width(button, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(button, 5, LV_PART_MAIN);
}

void uiBuildSubmenuHeader(lv_obj_t* tab, const char* title) {
  lv_obj_t* nav = lv_obj_create(tab);
  lv_obj_set_size(nav, SCREEN_WIDTH, 34);
  lv_obj_align(nav, LV_ALIGN_TOP_MID, 0, 0);
  lv_obj_set_style_bg_color(nav, lv_color_hex(0x0F172A), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(nav, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_width(nav, 0, LV_PART_MAIN);
  lv_obj_set_style_radius(nav, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(nav, 0, LV_PART_MAIN);
  lv_obj_set_scrollbar_mode(nav, LV_SCROLLBAR_MODE_OFF);
  lv_obj_clear_flag(nav, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t* backBtn = lv_btn_create(nav);
  lv_obj_set_size(backBtn, 56, 24);
  lv_obj_align(backBtn, LV_ALIGN_LEFT_MID, 4, 0);
  lv_obj_set_style_radius(backBtn, 6, LV_PART_MAIN);
  lv_obj_set_style_bg_color(backBtn, lv_color_hex(0x1E293B), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(backBtn, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_add_event_cb(backBtn, uiOpenLauncherTabEventCb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* backLabel = lv_label_create(backBtn);
  lv_label_set_text(backLabel, "Back");
  lv_obj_center(backLabel);

  lv_obj_t* titleLabel = lv_label_create(nav);
  lv_label_set_text(titleLabel, title);
  lv_obj_set_style_text_color(titleLabel, lv_color_hex(0xCBD5E1), LV_PART_MAIN);
  lv_obj_align(titleLabel, LV_ALIGN_CENTER, 0, 0);
}

void buildUi() {
  lv_obj_t* scr = lv_scr_act();
  lv_obj_set_style_bg_color(scr, lv_color_hex(0x05070B), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_text_color(scr, lv_color_hex(0xE5E7EB), LV_PART_MAIN);

  uiTabview = lv_tabview_create(scr, LV_DIR_TOP, 0);
  lv_obj_set_size(uiTabview, SCREEN_WIDTH, SCREEN_HEIGHT);
  lv_obj_set_style_bg_color(uiTabview, lv_color_hex(0x05070B), LV_PART_MAIN);
  lv_obj_set_style_border_width(uiTabview, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(uiTabview, 0, LV_PART_MAIN);

  lv_obj_t* tabBtns = lv_tabview_get_tab_btns(uiTabview);
  if (tabBtns) {
    lv_obj_add_flag(tabBtns, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_size(tabBtns, 1, 1);
  }
  lv_obj_t* tabContent = lv_tabview_get_content(uiTabview);
  if (tabContent) {
    // Keep tab switching button-driven only; disable user swipe between tabs.
    lv_obj_clear_flag(tabContent, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(tabContent, LV_OBJ_FLAG_SCROLL_CHAIN_HOR);
    lv_obj_clear_flag(tabContent, LV_OBJ_FLAG_SCROLL_ELASTIC);
    lv_obj_clear_flag(tabContent, LV_OBJ_FLAG_SCROLL_MOMENTUM);
    lv_obj_set_scroll_dir(tabContent, LV_DIR_NONE);
    lv_obj_set_scrollbar_mode(tabContent, LV_SCROLLBAR_MODE_OFF);
  }

  lv_obj_t* tabLauncher = lv_tabview_add_tab(uiTabview, "Launcher");
  lv_obj_t* tabAssistant = lv_tabview_add_tab(uiTabview, "Assistant");
  lv_obj_t* tabControls = lv_tabview_add_tab(uiTabview, "Controls");
  lv_obj_t* tabDiagnostics = lv_tabview_add_tab(uiTabview, "Diagnostics");
  lv_obj_t* tabLogs = lv_tabview_add_tab(uiTabview, "Logs");
  lv_obj_t* tabs[UI_TAB_COUNT] = {
      tabLauncher,
      tabAssistant,
      tabControls,
      tabDiagnostics,
      tabLogs,
  };
  for (uint16_t i = 0; i < UI_TAB_COUNT; ++i) {
    lv_obj_set_scrollbar_mode(tabs[i], LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_pad_all(tabs[i], 0, LV_PART_MAIN);
    lv_obj_set_style_bg_color(tabs[i], lv_color_hex(0x05070B), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(tabs[i], LV_OPA_COVER, LV_PART_MAIN);
  }

  lv_obj_t* launchHeader = lv_obj_create(tabLauncher);
  lv_obj_set_size(launchHeader, SCREEN_WIDTH - 12, 58);
  lv_obj_align(launchHeader, LV_ALIGN_TOP_MID, 0, 6);
  uiStylePanel(launchHeader, 0x0B1220, 0x253147);
  lv_obj_set_style_pad_all(launchHeader, 7, LV_PART_MAIN);
  lv_obj_set_scrollbar_mode(launchHeader, LV_SCROLLBAR_MODE_OFF);
  lv_obj_clear_flag(launchHeader, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t* launchTitle = lv_label_create(launchHeader);
  lv_label_set_text(launchTitle, "Voice Launcher");
  lv_obj_set_style_text_color(launchTitle, lv_color_hex(0x7DD3FC), LV_PART_MAIN);
  lv_obj_align(launchTitle, LV_ALIGN_TOP_LEFT, 0, 0);

  uiLabelLauncherState = lv_label_create(launchHeader);
  lv_label_set_text(uiLabelLauncherState, "State: Booting");
  lv_obj_set_width(uiLabelLauncherState, SCREEN_WIDTH - 30);
  lv_obj_align(uiLabelLauncherState, LV_ALIGN_TOP_LEFT, 0, 18);

  uiLabelLauncherWifi = lv_label_create(launchHeader);
  lv_label_set_text(uiLabelLauncherWifi, "Wi-Fi: offline");
  lv_obj_set_width(uiLabelLauncherWifi, SCREEN_WIDTH - 30);
  lv_obj_align(uiLabelLauncherWifi, LV_ALIGN_TOP_LEFT, 0, 34);

  lv_obj_t* launchGrid = lv_obj_create(tabLauncher);
  lv_obj_set_size(launchGrid, SCREEN_WIDTH - 12, 246);
  lv_obj_align(launchGrid, LV_ALIGN_TOP_MID, 0, 68);
  uiStylePanel(launchGrid, 0x090E19, 0x202C3F);
  const int16_t launchGridPad = 10;
  const int16_t launchGap = 10;
  lv_obj_set_style_pad_all(launchGrid, launchGridPad, LV_PART_MAIN);
  lv_obj_set_style_pad_row(launchGrid, launchGap, LV_PART_MAIN);
  lv_obj_set_style_pad_column(launchGrid, launchGap, LV_PART_MAIN);
  lv_obj_set_scrollbar_mode(launchGrid, LV_SCROLLBAR_MODE_OFF);
  lv_obj_clear_flag(launchGrid, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_clip_corner(launchGrid, true, LV_PART_MAIN);
  lv_obj_set_layout(launchGrid, LV_LAYOUT_FLEX);
  lv_obj_set_flex_flow(launchGrid, LV_FLEX_FLOW_ROW_WRAP);
  lv_obj_set_flex_align(
      launchGrid,
      LV_FLEX_ALIGN_CENTER,
      LV_FLEX_ALIGN_CENTER,
      LV_FLEX_ALIGN_CENTER);

  const int16_t launchBtnW = 96;
  const int16_t launchBtnH = 96;

  lv_obj_t* btnAssistant = lv_btn_create(launchGrid);
  lv_obj_set_size(btnAssistant, launchBtnW, launchBtnH);
  uiStyleLauncherButton(btnAssistant, 0x0EA5E9, 0x0D2236);
  lv_obj_add_event_cb(btnAssistant, uiOpenAssistantTabEventCb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* lblAssistant = lv_label_create(btnAssistant);
  lv_label_set_text(lblAssistant, "Assistant\nStatus");
  lv_label_set_long_mode(lblAssistant, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(lblAssistant, 92);
  lv_obj_set_style_text_align(lblAssistant, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_center(lblAssistant);

  lv_obj_t* btnControls = lv_btn_create(launchGrid);
  lv_obj_set_size(btnControls, launchBtnW, launchBtnH);
  uiStyleLauncherButton(btnControls, 0x22C55E, 0x0D2A1D);
  lv_obj_add_event_cb(btnControls, uiOpenControlsTabEventCb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* lblControls = lv_label_create(btnControls);
  lv_label_set_text(lblControls, "Controls");
  lv_obj_set_style_text_align(lblControls, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_center(lblControls);

  lv_obj_t* btnDiagnostics = lv_btn_create(launchGrid);
  lv_obj_set_size(btnDiagnostics, launchBtnW, launchBtnH);
  uiStyleLauncherButton(btnDiagnostics, 0xF59E0B, 0x2C210C);
  lv_obj_add_event_cb(btnDiagnostics, uiOpenDiagnosticsTabEventCb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* lblDiagnostics = lv_label_create(btnDiagnostics);
  lv_label_set_text(lblDiagnostics, "Diagnostics");
  lv_obj_set_style_text_align(lblDiagnostics, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_center(lblDiagnostics);

  lv_obj_t* btnLogs = lv_btn_create(launchGrid);
  lv_obj_set_size(btnLogs, launchBtnW, launchBtnH);
  uiStyleLauncherButton(btnLogs, 0xA78BFA, 0x231C38);
  lv_obj_add_event_cb(btnLogs, uiOpenLogsTabEventCb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* lblLogs = lv_label_create(btnLogs);
  lv_label_set_text(lblLogs, "Logs");
  lv_obj_set_style_text_align(lblLogs, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_center(lblLogs);

  uiBuildSubmenuHeader(tabAssistant, "Assistant");
  lv_obj_t* assistantPanel = lv_obj_create(tabAssistant);
  lv_obj_set_size(assistantPanel, SCREEN_WIDTH - 12, SCREEN_HEIGHT - 46);
  lv_obj_align(assistantPanel, LV_ALIGN_TOP_MID, 0, 40);
  uiStylePanel(assistantPanel, 0x090D14, 0x263245);
  lv_obj_set_style_pad_all(assistantPanel, 8, LV_PART_MAIN);
  lv_obj_set_scrollbar_mode(assistantPanel, LV_SCROLLBAR_MODE_OFF);
  lv_obj_clear_flag(assistantPanel, LV_OBJ_FLAG_SCROLLABLE);

  uiLabelAssistantState = lv_label_create(assistantPanel);
  lv_label_set_text(uiLabelAssistantState, "State: Booting");
  lv_obj_set_width(uiLabelAssistantState, SCREEN_WIDTH - 30);
  lv_obj_align(uiLabelAssistantState, LV_ALIGN_TOP_LEFT, 0, 0);

  uiLabelAssistantDetail = lv_label_create(assistantPanel);
  lv_label_set_text(uiLabelAssistantDetail, "Starting...");
  lv_obj_set_width(uiLabelAssistantDetail, SCREEN_WIDTH - 30);
  lv_obj_align(uiLabelAssistantDetail, LV_ALIGN_TOP_LEFT, 0, 24);
  lv_label_set_long_mode(uiLabelAssistantDetail, LV_LABEL_LONG_WRAP);

  uiLabelAssistantWifi = lv_label_create(assistantPanel);
  lv_label_set_text(uiLabelAssistantWifi, "Wi-Fi: offline");
  lv_obj_set_width(uiLabelAssistantWifi, SCREEN_WIDTH - 30);
  lv_obj_align(uiLabelAssistantWifi, LV_ALIGN_TOP_LEFT, 0, 82);

  uiLabelAssistantMic = lv_label_create(assistantPanel);
  lv_label_set_text(uiLabelAssistantMic, "Mic: idle");
  lv_obj_set_width(uiLabelAssistantMic, SCREEN_WIDTH - 30);
  lv_obj_align(uiLabelAssistantMic, LV_ALIGN_TOP_LEFT, 0, 106);

  uiLabelAssistantHint = lv_label_create(assistantPanel);
  lv_label_set_text(uiLabelAssistantHint, "Tap Capture from Controls");
  lv_obj_set_width(uiLabelAssistantHint, SCREEN_WIDTH - 30);
  lv_obj_align(uiLabelAssistantHint, LV_ALIGN_TOP_LEFT, 0, 134);
  lv_label_set_long_mode(uiLabelAssistantHint, LV_LABEL_LONG_WRAP);
  lv_obj_set_style_text_color(uiLabelAssistantHint, lv_color_hex(0x94A3B8), LV_PART_MAIN);

  uiBuildSubmenuHeader(tabControls, "Controls");
  lv_obj_t* controlsPanel = lv_obj_create(tabControls);
  lv_obj_set_size(controlsPanel, SCREEN_WIDTH - 12, SCREEN_HEIGHT - 46);
  lv_obj_align(controlsPanel, LV_ALIGN_TOP_MID, 0, 40);
  uiStylePanel(controlsPanel, 0x0A1017, 0x243447);
  lv_obj_set_style_pad_all(controlsPanel, 8, LV_PART_MAIN);
  lv_obj_set_scrollbar_mode(controlsPanel, LV_SCROLLBAR_MODE_OFF);
  lv_obj_clear_flag(controlsPanel, LV_OBJ_FLAG_SCROLLABLE);

  uiBtnControlCapture = lv_btn_create(controlsPanel);
  lv_obj_set_size(uiBtnControlCapture, 104, 78);
  lv_obj_align(uiBtnControlCapture, LV_ALIGN_TOP_LEFT, 0, 0);
  uiStyleLauncherButton(uiBtnControlCapture, 0x38BDF8, 0x0D2236);
  lv_obj_add_event_cb(uiBtnControlCapture, uiCaptureButtonEventCb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* captureLabel = lv_label_create(uiBtnControlCapture);
  lv_label_set_text(captureLabel, "Capture");
  lv_obj_center(captureLabel);

  uiBtnControlRepeat = lv_btn_create(controlsPanel);
  lv_obj_set_size(uiBtnControlRepeat, 104, 78);
  lv_obj_align(uiBtnControlRepeat, LV_ALIGN_TOP_RIGHT, 0, 0);
  uiStyleLauncherButton(uiBtnControlRepeat, 0x2DD4BF, 0x102721);
  lv_obj_add_event_cb(uiBtnControlRepeat, uiRepeatButtonEventCb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* repeatLabel = lv_label_create(uiBtnControlRepeat);
  lv_label_set_text(repeatLabel, "Repeat");
  lv_obj_center(repeatLabel);

  uiBtnControlWifi = lv_btn_create(controlsPanel);
  lv_obj_set_size(uiBtnControlWifi, 104, 78);
  lv_obj_align(uiBtnControlWifi, LV_ALIGN_TOP_LEFT, 0, 88);
  uiStyleLauncherButton(uiBtnControlWifi, 0xF59E0B, 0x2E230F);
  lv_obj_add_event_cb(uiBtnControlWifi, uiWifiRetryButtonEventCb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* wifiLabel = lv_label_create(uiBtnControlWifi);
  lv_label_set_text(wifiLabel, "Reconnect");
  lv_obj_center(wifiLabel);

  uiBtnControlRecover = lv_btn_create(controlsPanel);
  lv_obj_set_size(uiBtnControlRecover, 104, 78);
  lv_obj_align(uiBtnControlRecover, LV_ALIGN_TOP_RIGHT, 0, 88);
  uiStyleLauncherButton(uiBtnControlRecover, 0xEF4444, 0x30161A);
  lv_obj_add_event_cb(uiBtnControlRecover, uiRecoverAudioButtonEventCb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* recoverLabel = lv_label_create(uiBtnControlRecover);
  lv_label_set_text(recoverLabel, "Recover");
  lv_obj_center(recoverLabel);

  uiLabelControlsStatus = lv_label_create(controlsPanel);
  lv_label_set_text(uiLabelControlsStatus, "Controls ready");
  lv_obj_set_width(uiLabelControlsStatus, SCREEN_WIDTH - 30);
  lv_obj_align(uiLabelControlsStatus, LV_ALIGN_BOTTOM_LEFT, 0, -6);
  lv_label_set_long_mode(uiLabelControlsStatus, LV_LABEL_LONG_WRAP);
  lv_obj_set_style_text_color(uiLabelControlsStatus, lv_color_hex(0x9FB3CF), LV_PART_MAIN);

  uiBuildSubmenuHeader(tabDiagnostics, "Diagnostics");
  lv_obj_t* diagPanel = lv_obj_create(tabDiagnostics);
  lv_obj_set_size(diagPanel, SCREEN_WIDTH - 12, SCREEN_HEIGHT - 46);
  lv_obj_align(diagPanel, LV_ALIGN_TOP_MID, 0, 40);
  uiStylePanel(diagPanel, 0x0A0C10, 0x3A2E1C);
  lv_obj_set_style_pad_all(diagPanel, 8, LV_PART_MAIN);
  lv_obj_set_scrollbar_mode(diagPanel, LV_SCROLLBAR_MODE_OFF);
  lv_obj_clear_flag(diagPanel, LV_OBJ_FLAG_SCROLLABLE);

  uiLabelDiagnosticsRun = lv_label_create(diagPanel);
  lv_label_set_text(uiLabelDiagnosticsRun, "Last run: (none)");
  lv_obj_set_width(uiLabelDiagnosticsRun, SCREEN_WIDTH - 30);
  lv_obj_align(uiLabelDiagnosticsRun, LV_ALIGN_TOP_LEFT, 0, 0);

  uiLabelDiagnosticsStageA = lv_label_create(diagPanel);
  lv_label_set_text(uiLabelDiagnosticsStageA, "L - | STT - | AI -");
  lv_obj_set_width(uiLabelDiagnosticsStageA, SCREEN_WIDTH - 30);
  lv_obj_align(uiLabelDiagnosticsStageA, LV_ALIGN_TOP_LEFT, 0, 24);

  uiLabelDiagnosticsStageB = lv_label_create(diagPanel);
  lv_label_set_text(uiLabelDiagnosticsStageB, "TU - | TP - | OK 0 | ER 0");
  lv_obj_set_width(uiLabelDiagnosticsStageB, SCREEN_WIDTH - 30);
  lv_obj_align(uiLabelDiagnosticsStageB, LV_ALIGN_TOP_LEFT, 0, 48);

  uiLabelDiagnosticsMic = lv_label_create(diagPanel);
  lv_label_set_text(uiLabelDiagnosticsMic, "Mic: pending");
  lv_obj_set_width(uiLabelDiagnosticsMic, SCREEN_WIDTH - 30);
  lv_obj_align(uiLabelDiagnosticsMic, LV_ALIGN_TOP_LEFT, 0, 78);
  lv_obj_set_style_text_color(uiLabelDiagnosticsMic, lv_color_hex(0xA5F3FC), LV_PART_MAIN);

  uiLabelDiagnosticsRuntime = lv_label_create(diagPanel);
  lv_label_set_text(uiLabelDiagnosticsRuntime, "Heap: pending");
  lv_obj_set_width(uiLabelDiagnosticsRuntime, SCREEN_WIDTH - 30);
  lv_obj_align(uiLabelDiagnosticsRuntime, LV_ALIGN_TOP_LEFT, 0, 106);
  lv_label_set_long_mode(uiLabelDiagnosticsRuntime, LV_LABEL_LONG_WRAP);
  lv_obj_set_style_text_color(uiLabelDiagnosticsRuntime, lv_color_hex(0x9FB3CF), LV_PART_MAIN);

  uiBuildSubmenuHeader(tabLogs, "Logs");
  lv_obj_t* logsPanel = lv_obj_create(tabLogs);
  lv_obj_set_size(logsPanel, SCREEN_WIDTH - 12, SCREEN_HEIGHT - 46);
  lv_obj_align(logsPanel, LV_ALIGN_TOP_MID, 0, 40);
  uiStylePanel(logsPanel, 0x0A0F1A, 0x2D3650);
  lv_obj_set_style_pad_all(logsPanel, 8, LV_PART_MAIN);
  lv_obj_set_scrollbar_mode(logsPanel, LV_SCROLLBAR_MODE_OFF);
  lv_obj_clear_flag(logsPanel, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t* transcriptLabel = lv_label_create(logsPanel);
  lv_label_set_text(transcriptLabel, "Transcript");
  lv_obj_set_style_text_color(transcriptLabel, lv_color_hex(0xFDE68A), LV_PART_MAIN);
  lv_obj_align(transcriptLabel, LV_ALIGN_TOP_LEFT, 0, 0);

  uiTaTranscript = lv_textarea_create(logsPanel);
  lv_obj_set_size(uiTaTranscript, SCREEN_WIDTH - 30, 102);
  lv_obj_align(uiTaTranscript, LV_ALIGN_TOP_LEFT, 0, 18);
  lv_textarea_set_text(uiTaTranscript, "(none)");
  lv_obj_set_style_radius(uiTaTranscript, 8, LV_PART_MAIN);
  lv_obj_set_style_bg_color(uiTaTranscript, lv_color_hex(0x101928), LV_PART_MAIN);
  lv_obj_set_style_text_color(uiTaTranscript, lv_color_hex(0xEAF2FF), LV_PART_MAIN);
  lv_obj_clear_flag(uiTaTranscript, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_clear_flag(uiTaTranscript, LV_OBJ_FLAG_CLICK_FOCUSABLE);
  lv_obj_set_scrollbar_mode(uiTaTranscript, LV_SCROLLBAR_MODE_ACTIVE);

  lv_obj_t* replyLabel = lv_label_create(logsPanel);
  lv_label_set_text(replyLabel, "Reply");
  lv_obj_set_style_text_color(replyLabel, lv_color_hex(0xFDE68A), LV_PART_MAIN);
  lv_obj_align(replyLabel, LV_ALIGN_TOP_LEFT, 0, 126);

  uiTaReply = lv_textarea_create(logsPanel);
  lv_obj_set_size(uiTaReply, SCREEN_WIDTH - 30, 112);
  lv_obj_align(uiTaReply, LV_ALIGN_TOP_LEFT, 0, 144);
  lv_textarea_set_text(uiTaReply, "(none)");
  lv_obj_set_style_radius(uiTaReply, 8, LV_PART_MAIN);
  lv_obj_set_style_bg_color(uiTaReply, lv_color_hex(0x101928), LV_PART_MAIN);
  lv_obj_set_style_text_color(uiTaReply, lv_color_hex(0xEAF2FF), LV_PART_MAIN);
  lv_obj_clear_flag(uiTaReply, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_clear_flag(uiTaReply, LV_OBJ_FLAG_CLICK_FOCUSABLE);
  lv_obj_set_scrollbar_mode(uiTaReply, LV_SCROLLBAR_MODE_ACTIVE);

  lv_tabview_set_act(uiTabview, UI_TAB_INDEX_LAUNCHER, LV_ANIM_OFF);
  uiSetControlsStatus("Controls ready");
}

void uiUpdateStatus(uint32_t nowMs) {
  const uint32_t stateAgeSec = (nowMs - stateEnteredAtMs) / 1000;
  const bool busy =
      state == AssistantState::Listening ||
      state == AssistantState::Transcribing ||
      state == AssistantState::Thinking ||
      state == AssistantState::Speaking;

  String wifiLine;
  if (WiFi.status() == WL_CONNECTED) {
    wifiLine = clipText(String("Wi-Fi: ") + WiFi.SSID() + "  " + String(WiFi.RSSI()) + " dBm", 48);
  } else if (wifiReconnectPending) {
    int32_t waitMs = static_cast<int32_t>(wifiNextRetryAtMs - nowMs);
    if (waitMs < 0) {
      waitMs = 0;
    }
    wifiLine = String("Wi-Fi retry in ") + String((waitMs + 999) / 1000) + "s";
  } else {
    wifiLine = "Wi-Fi: disconnected";
  }

  const String stateLine = String("State: ") + stateName(state) + "  " + stateAgeSec + "s";
  if (uiLabelLauncherState) {
    lv_label_set_text(uiLabelLauncherState, clipText(stateLine, 48).c_str());
    lv_obj_set_style_text_color(uiLabelLauncherState, uiStateColor(state), LV_PART_MAIN);
  }
  if (uiLabelLauncherWifi) {
    lv_label_set_text(uiLabelLauncherWifi, clipText(wifiLine, 48).c_str());
    lv_obj_set_style_text_color(
        uiLabelLauncherWifi,
        WiFi.status() == WL_CONNECTED ? lv_color_hex(0x86EFAC) : lv_color_hex(0xFCA5A5),
        LV_PART_MAIN);
  }
  if (uiLabelAssistantState) {
    lv_label_set_text(uiLabelAssistantState, clipText(stateLine, 60).c_str());
    lv_obj_set_style_text_color(uiLabelAssistantState, uiStateColor(state), LV_PART_MAIN);
  }
  if (uiLabelAssistantDetail) {
    lv_label_set_text(uiLabelAssistantDetail, clipText(String("Detail: ") + stateDetail, 110).c_str());
    lv_obj_set_style_text_color(
        uiLabelAssistantDetail,
        state == AssistantState::Error ? lv_color_hex(0xF87171) : lv_color_hex(0xCBD5E1),
        LV_PART_MAIN);
  }
  if (uiLabelAssistantWifi) {
    lv_label_set_text(uiLabelAssistantWifi, clipText(wifiLine, 60).c_str());
    lv_obj_set_style_text_color(
        uiLabelAssistantWifi,
        WiFi.status() == WL_CONNECTED ? lv_color_hex(0x86EFAC) : lv_color_hex(0xFCA5A5),
        LV_PART_MAIN);
  }
  if (uiLabelAssistantMic) {
    String micLine;
    if (state == AssistantState::Listening) {
      micLine =
          String("Mic live: ") +
          String(listeningCapturedMs / 1000.0f, 1) + "s  rms " + String(listeningFrameRms, 4) +
          "  pk " + String(listeningFramePeak, 4);
    } else {
      micLine =
          String("Last cap: ") +
          formatMsCompact(lastCaptureMs) +
          "  rms " + String(lastCaptureRms, 4) +
          "  pk " + String(lastCapturePeak, 4);
    }
    lv_label_set_text(uiLabelAssistantMic, clipText(micLine, 64).c_str());
    lv_obj_set_style_text_color(uiLabelAssistantMic, lv_color_hex(0xA5F3FC), LV_PART_MAIN);
  }
  if (uiLabelAssistantHint) {
    String hint = String(uiStateHint(state));
    if (state == AssistantState::Idle) {
      hint += String(" | HF ") + (handsFreeEnabled ? "on" : "off");
    }
    lv_label_set_text(uiLabelAssistantHint, clipText(hint, 84).c_str());
  }

  if (uiBtnControlCapture) {
    if (busy) {
      lv_obj_add_state(uiBtnControlCapture, LV_STATE_DISABLED);
    } else {
      lv_obj_clear_state(uiBtnControlCapture, LV_STATE_DISABLED);
    }
  }
  if (uiBtnControlRepeat) {
    if (busy || lastReply.length() == 0) {
      lv_obj_add_state(uiBtnControlRepeat, LV_STATE_DISABLED);
    } else {
      lv_obj_clear_state(uiBtnControlRepeat, LV_STATE_DISABLED);
    }
  }
  if (uiBtnControlWifi) {
    if (busy) {
      lv_obj_add_state(uiBtnControlWifi, LV_STATE_DISABLED);
    } else {
      lv_obj_clear_state(uiBtnControlWifi, LV_STATE_DISABLED);
    }
  }
  if (uiBtnControlRecover) {
    if (busy) {
      lv_obj_add_state(uiBtnControlRecover, LV_STATE_DISABLED);
    } else {
      lv_obj_clear_state(uiBtnControlRecover, LV_STATE_DISABLED);
    }
  }

  if (uiLabelDiagnosticsRun) {
    const String runLine =
        String("Last run: ") +
        clipText(lastCycleResult, 18) +
        "  total " + formatMsCompact(lastCycleTotalMs);
    lv_label_set_text(uiLabelDiagnosticsRun, clipText(runLine, 62).c_str());
  }
  if (uiLabelDiagnosticsStageA) {
    const String stageA =
        String("L ") + formatMsCompact(lastCycleListeningMs) +
        "  STT " + formatMsCompact(lastCycleSttMs) +
        "  AI " + formatMsCompact(lastCycleThinkingMs);
    lv_label_set_text(uiLabelDiagnosticsStageA, clipText(stageA, 62).c_str());
  }
  if (uiLabelDiagnosticsStageB) {
    const String stageB =
        String("TU ") + formatMsCompact(lastCycleTtsUrlMs) +
        "  TP " + formatMsCompact(lastCyclePlaybackMs) +
        "  OK " + successfulCycleCount +
        "  ER " + errorStateCount;
    lv_label_set_text(uiLabelDiagnosticsStageB, clipText(stageB, 62).c_str());
  }
  if (uiLabelDiagnosticsMic) {
    float micRms = 0.0f;
    float micPeak = 0.0f;
    uint32_t micAgeMs = 0;
    const char* micSource = "none";
    const uint32_t srFillOk = localWakeFillOkCount;
    const uint32_t srFillFail = localWakeFillFailCount;

    if (state == AssistantState::Listening) {
      micSource = "listen";
      micRms = listeningFrameRms;
      micPeak = listeningFramePeak;
      micAgeMs = 0;
    } else {
      const uint32_t wakeUpdatedMs = localWakeLiveUpdatedMs;
      if (wakeUpdatedMs > 0) {
        micSource = "wake";
        micRms = localWakeLiveRms;
        micPeak = localWakeLivePeak;
        micAgeMs = nowMs - wakeUpdatedMs;
      } else if (handsFreeLastLevelMs > 0) {
        micSource = "vad";
        micRms = handsFreeLastLevel.rmsNorm;
        micPeak = handsFreeLastLevel.peakNorm;
        micAgeMs = nowMs - handsFreeLastLevelMs;
      } else if (lastCaptureMs > 0) {
        micSource = "last";
        micRms = lastCaptureRms;
        micPeak = lastCapturePeak;
        micAgeMs = nowMs - lastCaptureMs;
      }
    }

    String micLine = "Mic live: pending";
    if (strcmp(micSource, "none") != 0) {
      micLine =
          String("Mic ") + micSource +
          "  rms " + String(micRms, 4) +
          "  pk " + String(micPeak, 4) +
          "  age " + formatMsCompact(micAgeMs) +
          "  sr " + srFillOk + "/" + srFillFail;
    } else if (localWakeEnabled) {
      micLine = String("Mic wake waiting  sr ") + srFillOk + "/" + srFillFail;
    }
    lv_label_set_text(uiLabelDiagnosticsMic, clipText(micLine, 74).c_str());
    lv_obj_set_style_text_color(
        uiLabelDiagnosticsMic,
        strcmp(micSource, "none") == 0 ? lv_color_hex(0x94A3B8) : lv_color_hex(0xA5F3FC),
        LV_PART_MAIN);
  }
  if (uiLabelDiagnosticsRuntime) {
    String runtimeLine =
        String("Heap ") + (heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024) + "k";
    if (psramFound()) {
      runtimeLine += String(" | PS ") + (heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024) + "k";
    }
    runtimeLine += String(" | Wake ") + ((localWakeEnabled && localWakeReady) ? "on" : "off");
    lv_label_set_text(uiLabelDiagnosticsRuntime, clipText(runtimeLine, 78).c_str());
  }

  static String shownTranscript;
  static String shownReply;
  const String transcript = lastTranscript.length() ? lastTranscript : "(none)";
  const String reply = lastReply.length() ? lastReply : "(none)";
  if (uiTaTranscript && transcript != shownTranscript) {
    lv_textarea_set_text(uiTaTranscript, transcript.c_str());
    shownTranscript = transcript;
  }
  if (uiTaReply && reply != shownReply) {
    lv_textarea_set_text(uiTaReply, reply.c_str());
    shownReply = reply;
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
  if (nextState == AssistantState::Error && state != AssistantState::Error) {
    errorStateCount++;
  }
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
    if (nowMs - bootLastTriggerMs >= 220) {
      bootLastTriggerMs = nowMs;
      requestCapture(CaptureRequestSource::Boot, "BOOT");
    }
    return;
  }

  if (!pressedNow && bootPressed) {
    bootPressed = false;
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
  const bool wakeEngineLive = localWakeReady || localWakeStartInProgress;
  if (!handsFreeEnabled || !wakeEngineLive || localWakePaused) {
    vTaskDelay(pdMS_TO_TICKS(20));
    return ESP_FAIL;
  }
  // During startup, allow Booting/Connecting so sr_start can consume frames
  // and finish initialization before we mark localWakeReady.
  const bool wakeAcceptingState =
      state == AssistantState::Booting ||
      state == AssistantState::ConnectingWiFi ||
      state == AssistantState::Idle ||
      state == AssistantState::Error;
  if (!audioReady || !wakeAcceptingState || captureRequested) {
    vTaskDelay(pdMS_TO_TICKS(12));
    return ESP_FAIL;
  }

  const uint32_t effectiveTimeout = timeoutMs == 0 || timeoutMs == UINT32_MAX ? 80 : timeoutMs;
  const esp_err_t fillErr = micCapture.readForSr(out, len, bytesRead, effectiveTimeout);
  if (fillErr == ESP_OK && *bytesRead >= sizeof(int16_t)) {
    localWakeFillOkCount++;
    updateLiveMicStatsFromPcm16(
        static_cast<const int16_t*>(out),
        *bytesRead / sizeof(int16_t));
  } else {
    localWakeFillFailCount++;
  }
  return fillErr;
}

void localWakeEventCb(void* arg, sr_event_t event, int commandId, int phraseId) {
  (void)arg;
  if (event == SR_EVENT_TIMEOUT) {
    Serial.println("[WAKE] event timeout");
    if (localWakeReady && !localWakePaused) {
      sr_set_mode(LOCAL_WAKE_SR_MODE);
    }
    return;
  }
  if (event == SR_EVENT_WAKEWORD || event == SR_EVENT_WAKEWORD_CHANNEL) {
    Serial.println(
        String("[WAKE] event wakeword cmd=") + commandId +
        " phrase=" + phraseId);
    localWakeCommandId = SR_CMD_WAKE_BOB;
    localWakePhraseId = phraseId;
    localWakeTriggerAtMs = millis();
    localWakeTriggerPending = true;
    return;
  }
  if (event != SR_EVENT_COMMAND) {
    return;
  }
  Serial.println(
      String("[WAKE] event command cmd=") + commandId +
      " phrase=" + phraseId);
  localWakeCommandId = commandId;
  localWakePhraseId = phraseId;
  localWakeTriggerAtMs = millis();
  localWakeTriggerPending = true;
  if (localWakeReady && !localWakePaused) {
    sr_set_mode(LOCAL_WAKE_SR_MODE);
  }
}

#endif

bool startLocalWakeEngine(String& outError) {
  outError = "";
  localWakeTriggerPending = false;
  localWakeCommandId = -1;
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

  localWakePaused = false;
  localWakeStopRequested = false;
  localWakeReady = true;
  // Mark wake ready before sr_start so the fill callback can supply frames if
  // sr_start blocks during internal initialization.
  Serial.println("[WAKE] local wake detector ready");
  Serial.println("[WAKE] sr_start begin");
  const esp_err_t err = sr_start(
      localWakeFillCb,
      nullptr,
      SR_CHANNELS_STEREO,
      LOCAL_WAKE_SR_MODE,
      "MM",
      kLocalWakeCommands,
      sizeof(kLocalWakeCommands) / sizeof(kLocalWakeCommands[0]),
      localWakeEventCb,
      nullptr);
  const bool stopRequested = localWakeStopRequested;
  localWakeStopRequested = false;

  if (stopRequested) {
    localWakeReady = false;
    localWakePaused = false;
    localWakeTriggerPending = false;
    localWakeCommandId = -1;
    localWakePhraseId = -1;
    return true;
  }

  if (err != ESP_OK) {
    localWakeReady = false;
    localWakePaused = false;
    localWakeTriggerPending = false;
    localWakeCommandId = -1;
    localWakePhraseId = -1;
    outError = String("sr_start failed err: ") + err;
    return false;
  }
  // If sr_start returned without an explicit stop request, verify SR is still
  // active. Non-blocking implementations should pass this; unexpected exits fail.
  const esp_err_t pauseErr = sr_pause();
  if (pauseErr != ESP_OK) {
    localWakeReady = false;
    localWakePaused = false;
    localWakeTriggerPending = false;
    localWakeCommandId = -1;
    localWakePhraseId = -1;
    outError = "sr exited unexpectedly";
    return false;
  }
  sr_resume();
  sr_set_mode(LOCAL_WAKE_SR_MODE);
  Serial.println("[WAKE] sr_start active");
  return true;
#endif
}

void stopLocalWakeEngine() {
#if ASSISTANT_HAS_LOCAL_SR
  localWakeStopRequested = true;
  if (localWakeReady) {
    sr_stop();
  }
#endif
  localWakeReady = false;
  localWakePaused = false;
  localWakeTriggerPending = false;
  localWakeCommandId = -1;
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
    localWakeCommandId = -1;
    localWakePhraseId = -1;
    sr_set_mode(LOCAL_WAKE_SR_MODE);
  }
#endif
}

void queueLocalWakeStart(const char* reason) {
#if ASSISTANT_HAS_LOCAL_SR
  if (!localWakeEnabled || !handsFreeEnabled || localWakeReady || localWakeStartInProgress) {
    return;
  }
  localWakeStartInProgress = true;
  Serial.println(String("[WAKE] local start begin: ") + reason);
  String wakeErr;
  const bool started = startLocalWakeEngine(wakeErr);
  localWakeStartInProgress = false;
  if (!started) {
    localWakeEnabled = false;
    localWakeReady = false;
    Serial.println(
        String("[WAKE] local disabled: ") +
        (wakeErr.length() ? wakeErr : "start failed"));
    return;
  }
  Serial.println(String("[WAKE] local start ready: ") + reason);
#else
  (void)reason;
#endif
}

void handleLocalRepeatCommand() {
  if (lastReply.length() == 0) {
    setState(AssistantState::Idle, "Repeat: no last reply");
    return;
  }
  if (strlen(HA_TTS_ENGINE_ID) == 0) {
    setState(AssistantState::Idle, "Repeat: TTS disabled");
    return;
  }
  if (WiFi.status() != WL_CONNECTED) {
    setState(AssistantState::Idle, "Repeat: Wi-Fi offline");
    scheduleWifiReconnect(millis(), true, "local repeat while offline");
    return;
  }

  String authErr;
  if (!ensureHaAuthReady(authErr)) {
    setState(AssistantState::Idle, "Repeat auth failed");
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

  setState(AssistantState::Speaking, "Repeating last reply");
  drawScreen(true);

  String ttsUrl;
  String ttsErr;
  if (!homeAssistantTtsGetUrl(lastReply, ttsUrl, ttsErr)) {
    setState(AssistantState::Idle, "Repeat TTS URL failed");
    return;
  }
  if (!homeAssistantPlayTtsWav(ttsUrl, ttsErr)) {
    setState(AssistantState::Idle, "Repeat playback failed");
    return;
  }

  setState(AssistantState::Idle, "Ready");
}

void handleLocalStatusCommand() {
  String detail = "Wi-Fi ";
  if (WiFi.status() == WL_CONNECTED) {
    detail += "up ";
    detail += String(WiFi.RSSI());
    detail += "dBm";
  } else {
    detail += "down";
  }
  detail += " HF ";
  detail += (handsFreeEnabled ? "on" : "off");
  setState(AssistantState::Idle, clipText(detail, 48));
}

void pollLocalWakeTrigger(uint32_t nowMs) {
  if (!handsFreeEnabled || !localWakeEnabled || !localWakeReady) {
    return;
  }
  const bool wakeAcceptingState =
      state == AssistantState::Idle || state == AssistantState::Error;
  if (!localWakeTriggerPending || captureRequested || !wakeAcceptingState) {
    return;
  }
  const int commandId = localWakeCommandId;
  const int phraseId = localWakePhraseId;
  localWakeTriggerPending = false;
  localWakeCommandId = -1;
  localWakePhraseId = -1;

  if (nowMs < handsFreeCooldownUntilMs) {
    return;
  }
  handsFreeCooldownUntilMs = nowMs + LOCAL_WAKE_COOLDOWN_MS;
  handsFreeVadActiveFrames = 0;

  Serial.println(
      String("[WAKE] local cmd=") + commandId +
      " phrase=" + phraseId);

  switch (commandId) {
    case SR_CMD_WAKE_BOB:
      requestCapture(CaptureRequestSource::Wake, "WAKE");
      return;
    case SR_CMD_CANCEL:
      captureRequested = false;
      captureSource = CaptureRequestSource::Unknown;
      setState(AssistantState::Idle, "Canceled");
      return;
    case SR_CMD_REPEAT:
      handleLocalRepeatCommand();
      return;
    case SR_CMD_STATUS:
      handleLocalStatusCommand();
      return;
    default:
      setState(AssistantState::Idle, "Unknown local command");
      return;
  }
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
  handsFreeLastLevelMs = nowMs;

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
    queueLocalWakeStart("recover");
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
  const bool lowHeap = internalFree < ASSISTANT_LOW_HEAP_THRESHOLD_BYTES;
  // Local SR keeps baseline internal heap lower while idle; only trigger
  // recovery from low-heap conditions when actively in a busy stage.
  if (lowHeap && stateBusy()) {
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
  const uint32_t cycleStartedMs = millis();
  lastCycleTotalMs = 0;
  lastCycleListeningMs = 0;
  lastCycleSttMs = 0;
  lastCycleThinkingMs = 0;
  lastCycleTtsUrlMs = 0;
  lastCyclePlaybackMs = 0;
  lastCycleResult = "running";

  const auto finishCycle = [&](const String& result, bool success) {
    lastCycleTotalMs = millis() - cycleStartedMs;
    lastCycleResult = clipText(result, 26);
    if (success) {
      successfulCycleCount++;
    }
    uiDirty = true;
  };

  if (!audioReady) {
    setState(AssistantState::Error, "Audio not initialized");
    finishCycle("audio not ready", false);
    return;
  }

  if (WiFi.status() != WL_CONNECTED) {
    setState(AssistantState::Error, "Wi-Fi disconnected");
    scheduleWifiReconnect(millis(), true, "capture while offline");
    finishCycle("wifi disconnected", false);
    return;
  }

  if (!isHomeAssistantConfigured()) {
    setState(AssistantState::Error, "HA not configured in secrets.h");
    finishCycle("ha not configured", false);
    return;
  }

  String authErr;
  if (!ensureHaAuthReady(authErr)) {
    setState(AssistantState::Error, "HA auth failed: " + clipText(authErr, 45));
    finishCycle("ha auth failed", false);
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
    finishCycle(String(label) + " timeout", false);
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
      finishCycle("mic stall recovered", false);
    } else {
      setState(AssistantState::Error, "Microphone capture failed");
      finishCycle("mic capture failed", false);
    }
    return;
  }

  if (stageExpired(AssistantState::Listening, listeningStartedMs, "listening")) {
    return;
  }
  lastCycleListeningMs = millis() - listeningStartedMs;

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
    finishCycle("no speech", false);
    return;
  }
  if (captureResult.metrics.rmsNorm < CAPTURE_MIN_RMS &&
      captureResult.metrics.peakNorm < CAPTURE_MIN_PEAK) {
    setState(AssistantState::Idle, "Speech too quiet");
    finishCycle("speech too quiet", false);
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
    lastCycleSttMs = millis() - transcribingStartedMs;
    finishCycle("stt failed", false);
    return;
  }
  if (stageExpired(AssistantState::Transcribing, transcribingStartedMs, "transcribing")) {
    return;
  }
  lastCycleSttMs = millis() - transcribingStartedMs;

  setTranscript(transcript);
  setState(AssistantState::Thinking, "Calling conversation");
  const uint32_t thinkingStartedMs = millis();
  drawScreen(true);

  String reply;
  String convErr;
  if (!homeAssistantConversation(transcript, reply, convErr)) {
    setState(AssistantState::Error, "Conversation failed: " + clipText(convErr, 48));
    lastCycleThinkingMs = millis() - thinkingStartedMs;
    finishCycle("conversation failed", false);
    return;
  }
  if (stageExpired(AssistantState::Thinking, thinkingStartedMs, "thinking")) {
    return;
  }
  lastCycleThinkingMs = millis() - thinkingStartedMs;

  setReply(reply);

  if (strlen(HA_TTS_ENGINE_ID) == 0) {
    setState(AssistantState::Idle, "Reply ready (TTS disabled)");
    finishCycle("ok (tts off)", true);
    return;
  }

  setState(AssistantState::Speaking, "Playing TTS");
  const uint32_t speakingStartedMs = millis();
  drawScreen(true);

  String ttsUrl;
  String ttsErr;
  const uint32_t ttsUrlStartedMs = millis();
  if (!homeAssistantTtsGetUrl(reply, ttsUrl, ttsErr)) {
    setState(AssistantState::Error, "TTS URL failed: " + clipText(ttsErr, 50));
    lastCycleTtsUrlMs = millis() - ttsUrlStartedMs;
    finishCycle("tts url failed", false);
    return;
  }
  lastCycleTtsUrlMs = millis() - ttsUrlStartedMs;
  if (stageExpired(AssistantState::Speaking, speakingStartedMs, "speaking(tts-url)")) {
    return;
  }

  const uint32_t playbackStartedMs = millis();
  if (!homeAssistantPlayTtsWav(ttsUrl, ttsErr)) {
    lastCyclePlaybackMs = millis() - playbackStartedMs;
    const bool likelyAudioStall =
        ttsErr.indexOf("playback failed") >= 0 ||
        ttsErr.indexOf("WAV stream timeout") >= 0;
    if (likelyAudioStall && recoverAudioPipeline("tts playback stall", true)) {
      setState(AssistantState::Idle, "Recovered: TTS playback stall");
      finishCycle("tts stall recovered", false);
      return;
    }
    setState(AssistantState::Error, "TTS playback failed: " + clipText(ttsErr, 44));
    finishCycle("tts playback failed", false);
    return;
  }
  lastCyclePlaybackMs = millis() - playbackStartedMs;
  if (stageExpired(AssistantState::Speaking, speakingStartedMs, "speaking(playback)")) {
    return;
  }

  setState(AssistantState::Idle, "Ready");
  finishCycle("ok", true);
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
  lv_init();
  const size_t lvDrawPixelCount = SCREEN_WIDTH * 20;
  const size_t lvDrawBytes = lvDrawPixelCount * sizeof(lv_color_t);
  lvDrawPixels = static_cast<lv_color_t*>(
      heap_caps_malloc(lvDrawBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (!lvDrawPixels) {
    lvDrawPixels = static_cast<lv_color_t*>(
        heap_caps_malloc(lvDrawBytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
  }
  if (!lvDrawPixels) {
    Serial.println("[UI] LVGL draw buffer allocation failed");
    setState(AssistantState::Error, "UI draw buffer alloc failed");
    while (true) {
      delay(1000);
    }
  }
  Serial.println(
      String("[UI] lvbuf bytes=") + lvDrawBytes +
      " psram=" + (psramFound() ? "yes" : "no"));
  lv_disp_draw_buf_init(&lvDrawBuf, lvDrawPixels, nullptr, SCREEN_WIDTH * 20);

  static lv_disp_drv_t dispDrv;
  lv_disp_drv_init(&dispDrv);
  dispDrv.hor_res = SCREEN_WIDTH;
  dispDrv.ver_res = SCREEN_HEIGHT;
  dispDrv.flush_cb = lvglFlushCb;
  dispDrv.draw_buf = &lvDrawBuf;
  lv_disp_drv_register(&dispDrv);

  static lv_indev_drv_t indevDrv;
  lv_indev_drv_init(&indevDrv);
  indevDrv.type = LV_INDEV_TYPE_POINTER;
  indevDrv.read_cb = lvglTouchReadCb;
  lv_indev_drv_register(&indevDrv);

  loadWifiConfig();
  reconnectSsid = configuredSsid;
  reconnectPassword = configuredPassword;

  touchReady = touch.begin(PIN_TOUCH_SDA, PIN_TOUCH_SCL, PIN_TOUCH_RST, PIN_TOUCH_INT, 400000, false);
  if (!touchReady) {
    Serial.println("[UI] Touch init failed; launcher will be display-only");
  }
  buildUi();
  uiReady = true;
  uiDirty = true;
  drawScreen(true);

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
  Serial.println("[WAKE] phrases: ok mister bob");
  Serial.println("[WAKE] mode: command");
#endif

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

  if (localWakeEnabled) {
    // Start local SR after Wi-Fi boot work to reduce internal-RAM contention.
    queueLocalWakeStart("post-boot");
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
  if (localWakeEnabled && (nowMs - lastWakeStatsLogMs) >= 3000) {
    lastWakeStatsLogMs = nowMs;
    Serial.println(
        String("[WAKE] stats ready=") + (localWakeReady ? "1" : "0") +
        " paused=" + (localWakePaused ? "1" : "0") +
        " ok=" + localWakeFillOkCount +
        " fail=" + localWakeFillFailCount +
        " rms=" + String(localWakeLiveRms, 4) +
        " pk=" + String(localWakeLivePeak, 4));
  }

  if (uiActionCapture) {
    uiActionCapture = false;
    if (stateBusy()) {
      uiSetControlsStatus("Busy; wait for current cycle");
    } else {
      requestCapture(CaptureRequestSource::Unknown, "UI");
      uiSetControlsStatus("Capture queued");
    }
  }

  if (uiActionRepeat) {
    uiActionRepeat = false;
    if (stateBusy()) {
      uiSetControlsStatus("Busy; cannot repeat now");
    } else {
      uiSetControlsStatus("Repeating last reply");
      handleLocalRepeatCommand();
    }
  }

  if (uiActionWifiRetry) {
    uiActionWifiRetry = false;
    if (stateBusy()) {
      uiSetControlsStatus("Busy; reconnect deferred");
    } else {
      if (WiFi.status() == WL_CONNECTED) {
        WiFi.disconnect();
      }
      scheduleWifiReconnect(nowMs, true, "manual UI reconnect");
      setState(AssistantState::ConnectingWiFi, "Manual Wi-Fi reconnect");
      uiSetControlsStatus("Wi-Fi reconnect scheduled");
    }
  }

  if (uiActionRecoverAudio) {
    uiActionRecoverAudio = false;
    if (stateBusy()) {
      uiSetControlsStatus("Busy; recover after cycle");
    } else {
      uiSetControlsStatus("Recovering audio pipeline");
      softRecoverRuntime("manual UI recovery");
    }
  }

  if (captureRequested && (state == AssistantState::Idle || state == AssistantState::Error)) {
    const CaptureRequestSource triggerSource = captureSource;
    captureRequested = false;
    captureSource = CaptureRequestSource::Unknown;
    runAssistantCycle(triggerSource);
  }

  drawScreen(false);
  delay(6);
}
