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
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <math.h>
#include <new>
#include <time.h>

#if __has_include(<esp32-hal-sr.h>)
#include <esp32-hal-sr.h>
#endif
#if __has_include(<ESP_SR.h>)
#include <ESP_SR.h>
#endif

#include "board_pins.h"
#include "es8311_codec.h"
#include "ft6336_touch.h"
#include "i2s_mic_capture.h"
#include "secrets.example.h"
#if __has_include("secrets.h")
#include "secrets.h"
#endif

#if defined(SR_MODE_COMMAND) && defined(SR_EVENT_COMMAND)
#define ASSISTANT_HAS_LOCAL_SR 1
#else
#define ASSISTANT_HAS_LOCAL_SR 0
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

enum class CaptureMode {
  Assistant,
  Dictation
};

static const uint16_t SCREEN_WIDTH = 240;
static const uint16_t SCREEN_HEIGHT = 320;
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
static const uint32_t MANUAL_CAPTURE_MAX_MS = 8000;
static const uint32_t DICTATION_CAPTURE_MAX_MS = 5000;
static const uint8_t DICTATION_END_SILENT_CHUNKS = 3;
static const float DICTATION_SPEECH_START_RMS = 0.014f;
static const float DICTATION_SPEECH_START_PEAK = 0.040f;
static const float DICTATION_SILENCE_RMS = 0.009f;
static const float DICTATION_SILENCE_PEAK = 0.028f;
static const float PRE_STT_MIN_SPEECH_PERCENT = 8.0f;
static const size_t PRE_STT_FRAME_SAMPLES = 320;
static const float TTS_OUTPUT_GAIN = 1.00f;
static const size_t TTS_BUFFER_MAX_BYTES = 700 * 1024;
static const uint32_t TTS_BUFFER_IDLE_TIMEOUT_MS = 300;
static const size_t TTS_STREAM_PREFETCH_BYTES = 12 * 1024;
static const uint32_t TTS_STREAM_PREFETCH_MAX_MS = 240;
static const uint32_t TTS_STREAM_PREFETCH_IDLE_MS = 90;
static const size_t NOTES_MAX_CHARS = 6000;
static const uint16_t TAB_INDEX_HOME = 0;
static const uint16_t TAB_INDEX_SETTINGS = 1;
static const uint16_t TAB_INDEX_WIFI = 2;
static const uint16_t TAB_INDEX_PERFORMANCE = 3;
static const uint16_t TAB_INDEX_NOTES = 4;
static const uint16_t TAB_INDEX_POCKET_PET = 5;
static const uint16_t TAB_INDEX_CALENDAR = 6;
static const uint16_t TAB_INDEX_SYNC_TASKS = 7;
static const uint16_t TAB_COUNT = 8;
static const size_t MAX_CALENDAR_EVENTS = 20;
static const size_t MAX_SYNC_TASKS = 24;
static const uint16_t CALENDAR_ALERT_LEAD_MINUTES = 10;
static const uint32_t CLOUD_SYNC_RETRY_MS = 5UL * 60UL * 1000UL;
static const size_t MAX_TODO_ENTITY_OPTIONS = 12;
static const lv_coord_t NAV_BAR_HEIGHT = 44;
static const lv_coord_t NAV_CLEARANCE = 52;
static const uint32_t POCKET_PET_UPDATE_INTERVAL_MS = 60UL * 1000UL;
static const uint32_t POCKET_PET_AUTOSAVE_INTERVAL_MS = 5UL * 60UL * 1000UL;
static const uint32_t LOCAL_WAKE_BYPASS_MS = 6500;
static const uint32_t LOCAL_WAKE_COOLDOWN_MS = 1600;
static const size_t TTS_ASYNC_RING_SAMPLES = CAPTURE_SAMPLE_RATE * 2;
static const size_t TTS_ASYNC_CHUNK_SAMPLES = 512;
static const BaseType_t TTS_ASYNC_CORE = 0;
static const uint32_t TTS_ASYNC_PUSH_TIMEOUT_MS = 3000;
static const uint32_t PERF_REFRESH_INTERVAL_MS = 1000;

#ifndef CLOUD_CALENDAR_EVENTS_URL
#define CLOUD_CALENDAR_EVENTS_URL ""
#endif

#ifndef CLOUD_TASKS_LIST_URL
#define CLOUD_TASKS_LIST_URL ""
#endif

#ifndef CLOUD_TASK_COMPLETE_WEBHOOK_URL
#define CLOUD_TASK_COMPLETE_WEBHOOK_URL ""
#endif

#ifndef CLOUD_API_BEARER_TOKEN
#define CLOUD_API_BEARER_TOKEN ""
#endif

#ifndef CLOUD_SYNC_DAYS_AHEAD
#define CLOUD_SYNC_DAYS_AHEAD 3
#endif

#ifndef HA_CALENDAR_ENTITY_ID
#define HA_CALENDAR_ENTITY_ID ""
#endif

#ifndef HA_TODO_ENTITY_ID
#define HA_TODO_ENTITY_ID ""
#endif

#ifndef DEVICE_TZ
#define DEVICE_TZ "CST6CDT,M3.2.0/2,M11.1.0/2"
#endif

#ifndef ASSISTANT_LOCAL_WAKE_ENABLED
#define ASSISTANT_LOCAL_WAKE_ENABLED 1
#endif

#ifndef ASSISTANT_LOCAL_WAKE_USE_VAD_FALLBACK
#define ASSISTANT_LOCAL_WAKE_USE_VAD_FALLBACK 0
#endif

static TFT_eSPI tft = TFT_eSPI(SCREEN_WIDTH, SCREEN_HEIGHT);
static lv_disp_draw_buf_t drawBuf;
static lv_color_t drawBufPixels[SCREEN_WIDTH * 20];
static lv_color_t drawBufPixelsAlt[SCREEN_WIDTH * 20];

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
bool localWakeEnabled = false;
bool localWakeReady = false;
bool localWakePaused = false;
volatile bool localWakeTriggerPending = false;
volatile uint32_t localWakeTriggerAtMs = 0;
volatile int localWakePhraseId = -1;

bool captureRequested = false;
bool wifiScanRequested = false;
bool wifiSaveRequested = false;
bool wifiConnectRequested = false;
bool assistantStopRequested = false;
bool micMuted = false;
bool bypassWakeWordForNextCapture = false;
CaptureMode pendingCaptureMode = CaptureMode::Assistant;
CaptureMode activeCaptureMode = CaptureMode::Assistant;
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
float loopCpuUsagePercent = 0.0f;
String latestMicStatsLine = "Mic n/a";
String lastAssistantReplyText;

static const size_t NAV_HISTORY_DEPTH = 12;
uint16_t navHistory[NAV_HISTORY_DEPTH] = {0};
size_t navHistoryCount = 0;
uint16_t navCurrentTab = TAB_INDEX_HOME;
bool navIgnoreTabChange = false;

bool bootPressed = false;
unsigned long bootPressStartMs = 0;

String configuredSsid;
String configuredPassword;
String configuredTodoEntity;

int16_t captureBufferStorage[CAPTURE_SAMPLES] = {0};
int16_t* captureBuffer = captureBufferStorage;
size_t captureBufferCapacity = CAPTURE_SAMPLES;
uint32_t captureIndex = 0;

lv_obj_t* uiTabview = nullptr;
lv_obj_t* uiLabelState = nullptr;
lv_obj_t* uiLabelDetail = nullptr;
lv_obj_t* uiLabelHint = nullptr;
lv_obj_t* uiLabelResponse = nullptr;
lv_obj_t* uiLabelWifi = nullptr;
lv_obj_t* uiIconState = nullptr;
lv_obj_t* uiIconWifi = nullptr;
lv_obj_t* uiIconSd = nullptr;
lv_obj_t* uiIconMic = nullptr;
lv_obj_t* uiBtnTalk = nullptr;
lv_obj_t* uiLabelTalk = nullptr;
lv_obj_t* uiBtnMicMute = nullptr;
lv_obj_t* uiLabelMicMute = nullptr;

lv_obj_t* uiDdSsid = nullptr;
lv_obj_t* uiTaSsid = nullptr;
lv_obj_t* uiTaPassword = nullptr;
lv_obj_t* uiLabelWifiMenuStatus = nullptr;
lv_obj_t* uiKeyboard = nullptr;

lv_obj_t* uiTaNotes = nullptr;
lv_obj_t* uiBtnDictate = nullptr;
lv_obj_t* uiLabelDictate = nullptr;
lv_obj_t* uiBtnNotesDictate = nullptr;
lv_obj_t* uiLabelNotesDictate = nullptr;
lv_obj_t* uiBtnClearNotes = nullptr;
lv_obj_t* uiLabelNotesStatus = nullptr;
lv_obj_t* uiBtnOpenNotes = nullptr;
lv_obj_t* uiBtnOpenSettings = nullptr;
lv_obj_t* uiBtnOpenPet = nullptr;
lv_obj_t* uiLabelPetFace = nullptr;
lv_obj_t* uiLabelPetSummary = nullptr;
lv_obj_t* uiLabelPetDetail = nullptr;
lv_obj_t* uiBarPetHunger = nullptr;
lv_obj_t* uiBarPetHappiness = nullptr;
lv_obj_t* uiBarPetHealth = nullptr;
lv_obj_t* uiBarPetEnergy = nullptr;
lv_obj_t* uiBarPetCleanliness = nullptr;
lv_obj_t* uiLabelPetSleepButton = nullptr;
lv_obj_t* uiBtnOpenCalendar = nullptr;
lv_obj_t* uiBtnOpenSyncTasks = nullptr;
lv_obj_t* uiListCalendar = nullptr;
lv_obj_t* uiLabelCalendarStatus = nullptr;
lv_obj_t* uiListSyncTasks = nullptr;
lv_obj_t* uiLabelSyncTasksStatus = nullptr;
lv_obj_t* uiDdTodoEntity = nullptr;
lv_obj_t* uiTaPerformance = nullptr;
lv_obj_t* uiLabelPerfStatus = nullptr;

String notesText;

enum class PocketPetMode : uint8_t {
  Normal = 0,
  Eating = 1,
  Playing = 2,
  Sleeping = 3,
  Sick = 4,
  Hungry = 5,
  Dead = 6
};

struct PocketPetState {
  uint8_t hunger = 50;       // Tamapetchi-compatible: higher means more full.
  uint8_t happiness = 50;
  uint8_t health = 80;
  uint8_t energy = 100;
  uint8_t cleanliness = 80;
  uint32_t ageMinutes = 0;
  bool isAlive = true;
  PocketPetMode mode = PocketPetMode::Normal;
  uint32_t lastTickMs = 0;
  uint32_t stateChangeMs = 0;
  uint32_t lastInteractionMs = 0;
  uint32_t wakeMessageUntilMs = 0;
  uint32_t lastPersistMs = 0;
};

PocketPetState pocketPet;
String pocketPetBannerMessage;
uint32_t pocketPetBannerUntilMs = 0;

struct CalendarEventItem {
  String id;
  String title;
  String startLabel;
  String endLabel;
  time_t startEpoch = 0;
  time_t endEpoch = 0;
  bool alerted = false;
};

struct SyncedTaskItem {
  String id;
  String title;
  String dueLabel;
  bool completed = false;
};

CalendarEventItem calendarEvents[MAX_CALENDAR_EVENTS];
size_t calendarEventCount = 0;
SyncedTaskItem syncedTasks[MAX_SYNC_TASKS];
size_t syncedTaskCount = 0;

bool cloudSyncRequested = false;
bool cloudDataLoadedFromCache = false;
uint32_t lastCloudSyncAttemptMs = 0;
int32_t lastCloudSyncDayKey = -1;
bool todoEntityRefreshRequested = false;
String todoEntityOptions[MAX_TODO_ENTITY_OPTIONS];
size_t todoEntityOptionCount = 0;

#if ASSISTANT_HAS_LOCAL_SR
enum : int {
  SR_CMD_WAKE_BOB = 1,
};

static const sr_cmd_t kLocalWakeCommands[] = {
    {SR_CMD_WAKE_BOB, "ok bob", "bKd BnB"},
    {SR_CMD_WAKE_BOB, "okay bob", "bKd BnB"},
};

#if __has_include(<ESP_SR.h>)
ESP_SR_Class* const gEspSrAnchor = &ESP_SR;
#endif
#endif

struct AsyncPcmPlaybackContext;

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

lv_color_t uiStateColor(AssistantState s) {
  switch (s) {
    case AssistantState::Booting:
      return lv_color_hex(0x93C5FD);
    case AssistantState::ConnectingWiFi:
      return lv_color_hex(0x60A5FA);
    case AssistantState::Idle:
      return lv_color_hex(0x34D399);
    case AssistantState::Listening:
      return lv_color_hex(0xFBBF24);
    case AssistantState::Thinking:
      return lv_color_hex(0xC4B5FD);
    case AssistantState::Speaking:
      return lv_color_hex(0x2DD4BF);
    case AssistantState::Error:
      return lv_color_hex(0xF87171);
  }
  return lv_color_hex(0xEAF2FF);
}

const char* uiStateHint(AssistantState s) {
  switch (s) {
    case AssistantState::Booting:
      return "Starting hardware and services";
    case AssistantState::ConnectingWiFi:
      return "Joining network, please wait";
    case AssistantState::Idle:
      return "Tap Listen or Dictate to begin";
    case AssistantState::Listening:
      return "Speak naturally; tap Stop to cancel";
    case AssistantState::Thinking:
      return "Transcribing and preparing a response";
    case AssistantState::Speaking:
      return "Playing assistant reply";
    case AssistantState::Error:
      return "Check detail text for the last failure";
  }
  return "";
}

TouchPoint mapTouchToRotation1(const TouchPoint& raw) {
  const int mappedX = raw.x;
  const int mappedY = raw.y;

  TouchPoint mapped = {
      static_cast<uint16_t>(mappedX < 0 ? 0 : (mappedX > (SCREEN_WIDTH - 1) ? (SCREEN_WIDTH - 1) : mappedX)),
      static_cast<uint16_t>(mappedY < 0 ? 0 : (mappedY > (SCREEN_HEIGHT - 1) ? (SCREEN_HEIGHT - 1) : mappedY)),
  };

  return mapped;
}

void uiUpdateStatus();
void buildUi();
void pollBootButton();
void pollHandsFreeTrigger();
void pollLocalWakeTrigger();
bool startLocalWakeEngine(String& outError);
void stopLocalWakeEngine();
bool pauseLocalWakeEngine();
void resumeLocalWakeEngine();
void handleWifiScan();
void handleWifiSave();
void handleWifiConnect();
void uiShowKeyboardFor(lv_obj_t* ta);
void uiHideKeyboard();
void uiUpdateTalkButtonLabel();
void uiUpdateDictateButtonLabel();
void uiUpdateMicMuteButton();
void uiSetAssistantReplyText(const String& text);
void uiSetNotesStatus(const String& text);
void uiAppendDictationNote(const String& text);
void uiSetCalendarStatus(const String& text);
void uiSetSyncTasksStatus(const String& text);
void uiRefreshCalendarList();
void uiRefreshSyncTasksList();
void uiRefreshPerformanceMetrics();
void uiUpdatePocketPet();
void updatePocketPetLoop(uint32_t nowMs);
void loadPocketPetState();
void savePocketPetState();
void pocketPetSetBanner(const String& message, uint32_t durationMs);
bool syncCloudData(bool force, String& outError);
bool loadCloudCache(String& outError);
bool saveCloudCache(String& outError);
bool calibrateVoiceNoiseFloor(String& outSummary);
void maybeRunScheduledCloudSync(uint32_t nowMs);
void checkCalendarAlerts(uint32_t nowMs);
bool playAlertChime();
void syncClockIfNeeded();
bool fetchSyncedTasksFromHomeAssistant(String& outError);
bool updateTodoItemInHomeAssistant(const SyncedTaskItem& task, bool completed, String& outError);
String activeTodoEntity();
void saveTodoEntityConfig(const String& entityId);
bool fetchTodoEntityOptionsFromHomeAssistant(String& outError);
void uiRefreshTodoEntityDropdown();
void requestCapture(CaptureMode mode, bool bypassWakeWord);
bool homeAssistantTtsGetUrl(const String& text, String& outUrl, String& outError);
bool homeAssistantPlayTtsWav(const String& ttsUrl, String& outError);

void lvglFlushCb(lv_disp_drv_t* disp, const lv_area_t* area, lv_color_t* color_p);
void lvglTouchReadCb(lv_indev_drv_t* drv, lv_indev_data_t* data);

void uiTalkButtonEventCb(lv_event_t* e);
void uiMicMuteToggleButtonEventCb(lv_event_t* e);
void uiWifiScanButtonEventCb(lv_event_t* e);
void uiWifiSaveButtonEventCb(lv_event_t* e);
void uiWifiConnectButtonEventCb(lv_event_t* e);
void uiWifiDropdownEventCb(lv_event_t* e);
void uiTodoEntityDropdownEventCb(lv_event_t* e);
void uiTodoEntityRefreshButtonEventCb(lv_event_t* e);
void uiTextareaEventCb(lv_event_t* e);
void uiKeyboardEventCb(lv_event_t* e);
void uiDictateButtonEventCb(lv_event_t* e);
void uiClearNotesButtonEventCb(lv_event_t* e);
void uiOpenNotesButtonEventCb(lv_event_t* e);
void uiOpenSettingsButtonEventCb(lv_event_t* e);
void uiOpenWifiMenuButtonEventCb(lv_event_t* e);
void uiOpenPerformanceButtonEventCb(lv_event_t* e);
void uiSettingsSyncButtonEventCb(lv_event_t* e);
void uiCalibrateMicButtonEventCb(lv_event_t* e);
void uiOpenPocketPetButtonEventCb(lv_event_t* e);
void uiOpenCalendarButtonEventCb(lv_event_t* e);
void uiOpenSyncTasksButtonEventCb(lv_event_t* e);
void uiCloudSyncButtonEventCb(lv_event_t* e);
void uiSyncedTaskToggleEventCb(lv_event_t* e);
void uiNavHomeButtonEventCb(lv_event_t* e);
void uiNavBackButtonEventCb(lv_event_t* e);
void uiTabviewChangedEventCb(lv_event_t* e);
void uiPocketPetExitButtonEventCb(lv_event_t* e);
void uiPocketPetFeedButtonEventCb(lv_event_t* e);
void uiPocketPetPlayButtonEventCb(lv_event_t* e);
void uiPocketPetCleanButtonEventCb(lv_event_t* e);
void uiPocketPetSleepButtonEventCb(lv_event_t* e);
void uiPocketPetHealButtonEventCb(lv_event_t* e);
void uiPocketPetResetButtonEventCb(lv_event_t* e);

void setState(AssistantState nextState, const String& detail) {
  state = nextState;
  stateDetail = detail;

  Serial.print("State -> ");
  Serial.print(stateName(nextState));
  Serial.print(" | ");
  Serial.println(detail);

  uiUpdateStatus();
  uiUpdateTalkButtonLabel();
  uiUpdateDictateButtonLabel();
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

void uiSetNotesStatus(const String& text) {
  if (!uiLabelNotesStatus) {
    return;
  }
  lv_label_set_text(uiLabelNotesStatus, clipText(text, 88).c_str());
}

void uiSetCalendarStatus(const String& text) {
  if (!uiLabelCalendarStatus) {
    return;
  }
  lv_label_set_text(uiLabelCalendarStatus, clipText(text, 90).c_str());
}

void uiSetSyncTasksStatus(const String& text) {
  if (!uiLabelSyncTasksStatus) {
    return;
  }
  lv_label_set_text(uiLabelSyncTasksStatus, clipText(text, 90).c_str());
}

static String formatBytesCompact(uint64_t bytes) {
  const char* units[] = {"B", "KB", "MB", "GB"};
  double value = static_cast<double>(bytes);
  size_t idx = 0;
  while (value >= 1024.0 && idx < 3) {
    value /= 1024.0;
    idx++;
  }
  if (idx == 0) {
    return String(static_cast<uint32_t>(value)) + " " + units[idx];
  }
  return String(value, value >= 100.0 ? 0 : 1) + " " + units[idx];
}

static uint32_t usagePercentU32(uint32_t used, uint32_t total) {
  if (total == 0) {
    return 0;
  }
  return static_cast<uint32_t>((static_cast<uint64_t>(used) * 100ULL) / total);
}

void uiRefreshPerformanceMetrics() {
  if (!uiTaPerformance) {
    return;
  }

  const uint32_t heapTotal = ESP.getHeapSize();
  const uint32_t heapFree = ESP.getFreeHeap();
  const uint32_t heapUsed = heapTotal > heapFree ? (heapTotal - heapFree) : 0;

  const uint32_t psramTotal = ESP.getPsramSize();
  const uint32_t psramFree = ESP.getFreePsram();
  const uint32_t psramUsed = psramTotal > psramFree ? (psramTotal - psramFree) : 0;

  const uint32_t sketchUsed = ESP.getSketchSize();
  const uint32_t sketchRegion = sketchUsed + ESP.getFreeSketchSpace();

  uint64_t sdTotal = 0;
  uint64_t sdUsed = 0;
  uint64_t sdFree = 0;
  if (sdReady) {
    sdTotal = SD_MMC.totalBytes();
    sdUsed = SD_MMC.usedBytes();
    sdFree = sdTotal > sdUsed ? (sdTotal - sdUsed) : 0;
  }

  String metrics;
  metrics.reserve(520);
  metrics += "CPU (loop est): ";
  metrics += String(loopCpuUsagePercent, 1);
  metrics += "% @ ";
  metrics += String(ESP.getCpuFreqMHz());
  metrics += " MHz\n";
  metrics += "RAM (heap): ";
  metrics += formatBytesCompact(heapUsed);
  metrics += " / ";
  metrics += formatBytesCompact(heapTotal);
  metrics += " (";
  metrics += String(usagePercentU32(heapUsed, heapTotal));
  metrics += "%)\n";
  metrics += "PSRAM: ";
  metrics += formatBytesCompact(psramUsed);
  metrics += " / ";
  metrics += formatBytesCompact(psramTotal);
  metrics += " (";
  metrics += String(usagePercentU32(psramUsed, psramTotal));
  metrics += "%)\n";
  metrics += "ROM (sketch): ";
  metrics += formatBytesCompact(sketchUsed);
  metrics += " / ";
  metrics += formatBytesCompact(sketchRegion);
  metrics += " (";
  metrics += String(usagePercentU32(sketchUsed, sketchRegion));
  metrics += "%)\n";
  if (sdReady && sdTotal > 0) {
    metrics += "SD Card: ";
    metrics += formatBytesCompact(sdUsed);
    metrics += " / ";
    metrics += formatBytesCompact(sdTotal);
    metrics += " free ";
    metrics += formatBytesCompact(sdFree);
    metrics += "\n";
  } else {
    metrics += "SD Card: not mounted\n";
  }
  metrics += "Wi-Fi: ";
  metrics += WiFi.status() == WL_CONNECTED ? "connected" : "disconnected";
  metrics += "\n";
  metrics += micMuted ? String("Mic muted (capture + wake blocked)") : latestMicStatsLine;

  lv_textarea_set_text(uiTaPerformance, metrics.c_str());
  if (uiLabelPerfStatus) {
    String status = String("Updated: ") + String(millis() / 1000UL) + "s uptime";
    lv_label_set_text(uiLabelPerfStatus, clipText(status, 90).c_str());
  }
}

static uint8_t clampPercentInt(int value) {
  if (value < 0) {
    return 0;
  }
  if (value > 100) {
    return 100;
  }
  return static_cast<uint8_t>(value);
}

const char* pocketPetModeName(PocketPetMode mode) {
  switch (mode) {
    case PocketPetMode::Normal:
      return "Normal";
    case PocketPetMode::Eating:
      return "Eating";
    case PocketPetMode::Playing:
      return "Playing";
    case PocketPetMode::Sleeping:
      return "Sleeping";
    case PocketPetMode::Sick:
      return "Sick";
    case PocketPetMode::Hungry:
      return "Hungry";
    case PocketPetMode::Dead:
      return "Dead";
  }
  return "Normal";
}

void pocketPetSetBanner(const String& message, uint32_t durationMs) {
  pocketPetBannerMessage = message;
  if (durationMs == 0) {
    pocketPetBannerUntilMs = 0;
    return;
  }
  pocketPetBannerUntilMs = millis() + durationMs;
}

void loadPocketPetState() {
  Preferences prefs;
  prefs.begin("pet_cfg", true);
  pocketPet.hunger = clampPercentInt(static_cast<int>(prefs.getUChar("h", 50)));
  pocketPet.happiness = clampPercentInt(static_cast<int>(prefs.getUChar("hap", 50)));
  pocketPet.health = clampPercentInt(static_cast<int>(prefs.getUChar("hl", 80)));
  pocketPet.energy = clampPercentInt(static_cast<int>(prefs.getUChar("en", 100)));
  pocketPet.cleanliness = clampPercentInt(static_cast<int>(prefs.getUChar("cln", 80)));
  pocketPet.ageMinutes = prefs.getUInt("age", 0);
  pocketPet.isAlive = prefs.getBool("alive", true);
  uint8_t modeRaw = prefs.getUChar("mode", static_cast<uint8_t>(PocketPetMode::Normal));
  prefs.end();

  if (modeRaw > static_cast<uint8_t>(PocketPetMode::Dead)) {
    modeRaw = static_cast<uint8_t>(PocketPetMode::Normal);
  }
  pocketPet.mode = static_cast<PocketPetMode>(modeRaw);
  if (!pocketPet.isAlive) {
    pocketPet.mode = PocketPetMode::Dead;
  }

  const uint32_t now = millis();
  pocketPet.lastTickMs = now;
  pocketPet.stateChangeMs = now;
  pocketPet.lastInteractionMs = now;
  pocketPet.wakeMessageUntilMs = 0;
  pocketPet.lastPersistMs = now;
  pocketPetBannerMessage = "";
  pocketPetBannerUntilMs = 0;
}

void savePocketPetState() {
  Preferences prefs;
  prefs.begin("pet_cfg", false);
  prefs.putUChar("h", pocketPet.hunger);
  prefs.putUChar("hap", pocketPet.happiness);
  prefs.putUChar("hl", pocketPet.health);
  prefs.putUChar("en", pocketPet.energy);
  prefs.putUChar("cln", pocketPet.cleanliness);
  prefs.putUInt("age", pocketPet.ageMinutes);
  prefs.putBool("alive", pocketPet.isAlive);
  prefs.putUChar("mode", static_cast<uint8_t>(pocketPet.mode));
  prefs.end();
  pocketPet.lastPersistMs = millis();
}

void uiUpdatePocketPet() {
  if (!uiLabelPetFace && !uiBarPetHunger && !uiBarPetHappiness && !uiBarPetHealth &&
      !uiBarPetEnergy && !uiBarPetCleanliness && !uiLabelPetDetail) {
    return;
  }

  const uint32_t now = millis();
  if (pocketPetBannerUntilMs != 0 && static_cast<int32_t>(now - pocketPetBannerUntilMs) >= 0) {
    pocketPetBannerUntilMs = 0;
    pocketPetBannerMessage = "";
  }

  if (!pocketPet.isAlive) {
    pocketPet.mode = PocketPetMode::Dead;
  }

  const char* face = "(^_^)";
  String summary = "Happy and alert";
  lv_color_t summaryColor = lv_color_hex(0x93C5FD);
  switch (pocketPet.mode) {
    case PocketPetMode::Eating:
      face = "(^o^)";
      summary = "Eating";
      summaryColor = lv_color_hex(0x86EFAC);
      break;
    case PocketPetMode::Playing:
      face = "(^_^)!";
      summary = "Playing";
      summaryColor = lv_color_hex(0x67E8F9);
      break;
    case PocketPetMode::Sleeping:
      face = "(-_-) zZ";
      summary = "Sleeping";
      summaryColor = lv_color_hex(0xC4B5FD);
      break;
    case PocketPetMode::Sick:
      face = "(._.)";
      summary = "Feeling sick";
      summaryColor = lv_color_hex(0xFCA5A5);
      break;
    case PocketPetMode::Hungry:
      face = "(o_o)";
      summary = "Very hungry";
      summaryColor = lv_color_hex(0xFDBA74);
      break;
    case PocketPetMode::Dead:
      face = "(x_x)";
      summary = "Pet has passed away";
      summaryColor = lv_color_hex(0xF87171);
      break;
    case PocketPetMode::Normal:
    default:
      if (pocketPet.happiness < 35) {
        face = "(._.)";
        summary = "Needs attention";
        summaryColor = lv_color_hex(0xFDE68A);
      } else if (pocketPet.health < 40) {
        face = "(-_-)";
        summary = "Needs care";
        summaryColor = lv_color_hex(0xFCA5A5);
      }
      break;
  }

  if (uiLabelPetFace) {
    lv_label_set_text(uiLabelPetFace, face);
  }
  if (uiLabelPetSummary) {
    lv_label_set_text(uiLabelPetSummary, summary.c_str());
    lv_obj_set_style_text_color(uiLabelPetSummary, summaryColor, LV_PART_MAIN);
  }
  if (uiBarPetHunger) {
    lv_bar_set_value(uiBarPetHunger, pocketPet.hunger, LV_ANIM_ON);
  }
  if (uiBarPetHappiness) {
    lv_bar_set_value(uiBarPetHappiness, pocketPet.happiness, LV_ANIM_ON);
  }
  if (uiBarPetHealth) {
    lv_bar_set_value(uiBarPetHealth, pocketPet.health, LV_ANIM_ON);
  }
  if (uiBarPetEnergy) {
    lv_bar_set_value(uiBarPetEnergy, pocketPet.energy, LV_ANIM_ON);
  }
  if (uiBarPetCleanliness) {
    lv_bar_set_value(uiBarPetCleanliness, pocketPet.cleanliness, LV_ANIM_ON);
  }
  if (uiLabelPetDetail) {
    String detail;
    if (pocketPetBannerMessage.length() > 0) {
      detail = pocketPetBannerMessage;
    } else {
      detail = String("Age ") + String(pocketPet.ageMinutes) + "m | " + pocketPetModeName(pocketPet.mode);
    }
    lv_label_set_text(uiLabelPetDetail, detail.c_str());
    lv_obj_set_style_text_color(uiLabelPetDetail, lv_color_hex(0x9FB3CF), LV_PART_MAIN);
  }
  if (uiLabelPetSleepButton) {
    lv_label_set_text(uiLabelPetSleepButton, pocketPet.mode == PocketPetMode::Sleeping ? "Wake" : "Sleep");
  }
}

void updatePocketPetLoop(uint32_t nowMs) {
  bool redraw = false;
  bool saveNeeded = false;

  if (pocketPet.lastTickMs == 0) {
    pocketPet.lastTickMs = nowMs;
    redraw = true;
  }

  if (pocketPetBannerUntilMs != 0 && static_cast<int32_t>(nowMs - pocketPetBannerUntilMs) >= 0) {
    pocketPetBannerUntilMs = 0;
    pocketPetBannerMessage = "";
    redraw = true;
  }

  // Eating/playing are short-lived action states.
  if ((pocketPet.mode == PocketPetMode::Eating || pocketPet.mode == PocketPetMode::Playing) &&
      pocketPet.stateChangeMs != 0 &&
      static_cast<int32_t>(nowMs - pocketPet.stateChangeMs) >= 5000) {
    pocketPet.mode = PocketPetMode::Normal;
    redraw = true;
    saveNeeded = true;
  }

  if (nowMs > pocketPet.lastTickMs) {
    const uint32_t elapsedMs = nowMs - pocketPet.lastTickMs;
    if (elapsedMs >= POCKET_PET_UPDATE_INTERVAL_MS) {
      const uint32_t ticks = elapsedMs / POCKET_PET_UPDATE_INTERVAL_MS;
      pocketPet.lastTickMs += ticks * POCKET_PET_UPDATE_INTERVAL_MS;

      for (uint32_t i = 0; i < ticks; i++) {
        if (!pocketPet.isAlive) {
          pocketPet.mode = PocketPetMode::Dead;
          break;
        }

        if (pocketPet.mode == PocketPetMode::Sleeping) {
          pocketPet.hunger = clampPercentInt(static_cast<int>(pocketPet.hunger) - 2);
          pocketPet.happiness = clampPercentInt(static_cast<int>(pocketPet.happiness) - 1);
          pocketPet.energy = clampPercentInt(static_cast<int>(pocketPet.energy) + 10);
          pocketPet.cleanliness = clampPercentInt(static_cast<int>(pocketPet.cleanliness) - 2);

          if (pocketPet.energy >= 100) {
            pocketPet.mode = PocketPetMode::Normal;
            pocketPet.stateChangeMs = nowMs;
            pocketPetSetBanner("Woke up fully rested", 2500);
          }
        } else {
          pocketPet.hunger = clampPercentInt(static_cast<int>(pocketPet.hunger) - 5);
          pocketPet.happiness = clampPercentInt(static_cast<int>(pocketPet.happiness) - 3);
          pocketPet.energy = clampPercentInt(static_cast<int>(pocketPet.energy) - 2);
          pocketPet.cleanliness = clampPercentInt(static_cast<int>(pocketPet.cleanliness) - 4);
        }

        if (pocketPet.hunger < 20 || pocketPet.cleanliness < 20) {
          pocketPet.health = clampPercentInt(static_cast<int>(pocketPet.health) - 5);
        }

        if (pocketPet.health == 0) {
          pocketPet.isAlive = false;
          pocketPet.mode = PocketPetMode::Dead;
          pocketPetSetBanner("Pet has passed away. Reset to begin again.", 4500);
        } else if (pocketPet.mode != PocketPetMode::Dead && pocketPet.mode != PocketPetMode::Sleeping) {
          if (pocketPet.health < 30 && pocketPet.mode != PocketPetMode::Sick) {
            pocketPet.mode = PocketPetMode::Sick;
            pocketPet.stateChangeMs = nowMs;
          } else if (pocketPet.hunger < 20 && pocketPet.mode != PocketPetMode::Sick && pocketPet.mode != PocketPetMode::Hungry) {
            pocketPet.mode = PocketPetMode::Hungry;
            pocketPet.stateChangeMs = nowMs;
          } else if (pocketPet.mode == PocketPetMode::Hungry && pocketPet.hunger >= 20) {
            pocketPet.mode = PocketPetMode::Normal;
          } else if (pocketPet.mode == PocketPetMode::Sick && pocketPet.health >= 30) {
            pocketPet.mode = PocketPetMode::Normal;
          }
        }

        pocketPet.ageMinutes++;
        saveNeeded = true;
      }
      redraw = true;
    }
  }

  if (saveNeeded ||
      (pocketPet.lastPersistMs != 0 && nowMs > pocketPet.lastPersistMs &&
       (nowMs - pocketPet.lastPersistMs) >= POCKET_PET_AUTOSAVE_INTERVAL_MS)) {
    savePocketPetState();
  }

  if (redraw) {
    uiUpdatePocketPet();
  }
}

void uiAppendDictationNote(const String& text) {
  String cleaned = text;
  cleaned.trim();
  if (cleaned.length() == 0) {
    return;
  }

  if (notesText.length() > 0 && !notesText.endsWith("\n")) {
    notesText += "\n";
  }
  notesText += cleaned;
  notesText += "\n";

  if (notesText.length() > NOTES_MAX_CHARS) {
    const int overflow = static_cast<int>(notesText.length() - NOTES_MAX_CHARS);
    notesText.remove(0, overflow);
    const int firstBreak = notesText.indexOf('\n');
    if (firstBreak >= 0) {
      notesText.remove(0, firstBreak + 1);
    }
  }

  if (uiTaNotes) {
    lv_textarea_set_text(uiTaNotes, notesText.c_str());
    lv_textarea_set_cursor_pos(uiTaNotes, LV_TEXTAREA_CURSOR_LAST);
  }
}

void requestCapture(CaptureMode mode, bool bypassWakeWord) {
  if (micMuted) {
    setState(AssistantState::Idle, "Mic muted");
    if (mode == CaptureMode::Dictation) {
      uiSetNotesStatus("Mic is muted");
    }
    return;
  }
  assistantStopRequested = false;
  pendingCaptureMode = mode;
  bypassWakeWordForNextCapture = bypassWakeWord;
  captureRequested = true;
}

void uiSetTalkEnabled(bool enabled) {
  if (!uiBtnTalk) {
    return;
  }
  if (enabled) {
    lv_obj_clear_state(uiBtnTalk, LV_STATE_DISABLED);
    if (uiBtnDictate) {
      lv_obj_clear_state(uiBtnDictate, LV_STATE_DISABLED);
    }
    if (uiBtnNotesDictate) {
      lv_obj_clear_state(uiBtnNotesDictate, LV_STATE_DISABLED);
    }
  } else {
    lv_obj_add_state(uiBtnTalk, LV_STATE_DISABLED);
    if (uiBtnDictate) {
      lv_obj_add_state(uiBtnDictate, LV_STATE_DISABLED);
    }
    if (uiBtnNotesDictate) {
      lv_obj_add_state(uiBtnNotesDictate, LV_STATE_DISABLED);
    }
  }
}

void uiSetAssistantReplyText(const String& text) {
  lastAssistantReplyText = clipText(text, 220);
  if (!uiLabelResponse) {
    return;
  }
  const String shown = lastAssistantReplyText.length() > 0 ? lastAssistantReplyText : String("No response yet.");
  lv_label_set_text(uiLabelResponse, shown.c_str());
}

void uiUpdateMicMuteButton() {
  if (!uiBtnMicMute || !uiLabelMicMute) {
    return;
  }

  lv_label_set_text(uiLabelMicMute, micMuted ? LV_SYMBOL_MUTE " Mic Off" : LV_SYMBOL_AUDIO " Mic On");
  lv_obj_center(uiLabelMicMute);

  const lv_color_t border = micMuted ? lv_color_hex(0xF97316) : lv_color_hex(0x22C55E);
  lv_obj_set_style_bg_color(uiBtnMicMute, lv_color_hex(0x161616), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(uiBtnMicMute, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_width(uiBtnMicMute, 1, LV_PART_MAIN);
  lv_obj_set_style_border_color(uiBtnMicMute, border, LV_PART_MAIN);
  lv_obj_set_style_text_color(uiLabelMicMute, border, LV_PART_MAIN);
}

void uiUpdateTalkButtonLabel() {
  if (!uiLabelTalk || !uiBtnTalk) {
    return;
  }

  const bool busy = state != AssistantState::Idle;
  const bool assistantActive = busy && activeCaptureMode == CaptureMode::Assistant;
  const bool disabled = lv_obj_has_state(uiBtnTalk, LV_STATE_DISABLED) || micMuted;
  if (assistantActive) {
    lv_label_set_text(uiLabelTalk, LV_SYMBOL_STOP " Stop");
  } else if (micMuted) {
    lv_label_set_text(uiLabelTalk, LV_SYMBOL_MUTE " Mic Off");
  } else {
    lv_label_set_text(uiLabelTalk, LV_SYMBOL_AUDIO " Listen");
  }
  lv_obj_set_style_text_align(uiLabelTalk, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_center(uiLabelTalk);

  lv_color_t btnColor = assistantActive ? lv_color_hex(0x2A1111) : lv_color_hex(0x161616);
  lv_color_t borderColor = assistantActive ? lv_color_hex(0xF87171) : lv_color_hex(0x00FF88);
  if (disabled) {
    btnColor = lv_color_hex(0x101010);
    borderColor = lv_color_hex(0x4B5563);
  }

  lv_obj_set_style_bg_color(uiBtnTalk, btnColor, LV_PART_MAIN);
  lv_obj_set_style_bg_opa(uiBtnTalk, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_width(uiBtnTalk, assistantActive ? 2 : 1, LV_PART_MAIN);
  lv_obj_set_style_border_color(uiBtnTalk, borderColor, LV_PART_MAIN);
  lv_obj_set_style_radius(uiBtnTalk, 8, LV_PART_MAIN);
  lv_obj_set_style_shadow_width(uiBtnTalk, 0, LV_PART_MAIN);
  lv_obj_set_style_text_color(uiLabelTalk, disabled ? lv_color_hex(0x6B7280) : borderColor, LV_PART_MAIN);
}

void uiUpdateDictateButtonLabel() {
  if (!uiLabelDictate || !uiBtnDictate) {
    return;
  }

  const bool busy = state != AssistantState::Idle;
  const bool dictationActive = busy && activeCaptureMode == CaptureMode::Dictation;
  const bool disabled = lv_obj_has_state(uiBtnDictate, LV_STATE_DISABLED) || micMuted;
  if (dictationActive) {
    lv_label_set_text(uiLabelDictate, LV_SYMBOL_STOP " Stop");
  } else if (micMuted) {
    lv_label_set_text(uiLabelDictate, LV_SYMBOL_MUTE " Muted");
  } else {
    lv_label_set_text(uiLabelDictate, LV_SYMBOL_EDIT " Dictate");
  }
  lv_obj_set_style_text_align(uiLabelDictate, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_center(uiLabelDictate);

  lv_color_t btnColor = dictationActive ? lv_color_hex(0x2A1111) : lv_color_hex(0x161616);
  lv_color_t borderColor = dictationActive ? lv_color_hex(0xF87171) : lv_color_hex(0x60A5FA);
  if (disabled) {
    btnColor = lv_color_hex(0x101010);
    borderColor = lv_color_hex(0x4B5563);
  }

  lv_obj_set_style_bg_color(uiBtnDictate, btnColor, LV_PART_MAIN);
  lv_obj_set_style_bg_opa(uiBtnDictate, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_width(uiBtnDictate, dictationActive ? 2 : 1, LV_PART_MAIN);
  lv_obj_set_style_border_color(uiBtnDictate, borderColor, LV_PART_MAIN);
  lv_obj_set_style_radius(uiBtnDictate, 8, LV_PART_MAIN);
  lv_obj_set_style_shadow_width(uiBtnDictate, 0, LV_PART_MAIN);
  lv_obj_set_style_text_color(uiLabelDictate, disabled ? lv_color_hex(0x6B7280) : borderColor, LV_PART_MAIN);

  if (uiLabelNotesDictate && uiBtnNotesDictate) {
    lv_label_set_text(uiLabelNotesDictate, dictationActive ? "Stop" : (micMuted ? "Muted" : "Dictate"));
    lv_obj_center(uiLabelNotesDictate);
    lv_obj_set_style_bg_color(uiBtnNotesDictate, dictationActive ? lv_color_hex(0x2A1111) : lv_color_hex(0x111827), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(uiBtnNotesDictate, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(uiBtnNotesDictate, 12, LV_PART_MAIN);
    lv_obj_set_style_border_width(uiBtnNotesDictate, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(uiBtnNotesDictate, dictationActive ? lv_color_hex(0xF87171) : lv_color_hex(0x60A5FA), LV_PART_MAIN);
  }
}

void loadWifiConfig() {
  Preferences prefs;
  prefs.begin("voice_cfg", true);
  configuredSsid = prefs.getString("wifi_ssid", WIFI_SSID);
  configuredPassword = prefs.getString("wifi_pwd", WIFI_PASSWORD);
  String todoDefault = strlen(HA_TODO_ENTITY_ID) > 0 ? String(HA_TODO_ENTITY_ID) : String("todo.to_do_list");
  configuredTodoEntity = prefs.getString("ha_todo_entity", todoDefault);
  prefs.end();
}

void saveWifiConfig(const String& ssid, const String& password) {
  Preferences prefs;
  prefs.begin("voice_cfg", false);
  prefs.putString("wifi_ssid", ssid);
  prefs.putString("wifi_pwd", password);
  if (configuredTodoEntity.length() > 0) {
    prefs.putString("ha_todo_entity", configuredTodoEntity);
  }
  prefs.end();

  configuredSsid = ssid;
  configuredPassword = password;
}

void saveTodoEntityConfig(const String& entityId) {
  String cleaned = entityId;
  cleaned.trim();
  if (cleaned.length() == 0) {
    return;
  }
  configuredTodoEntity = cleaned;
  Preferences prefs;
  prefs.begin("voice_cfg", false);
  prefs.putString("ha_todo_entity", configuredTodoEntity);
  prefs.end();
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
  return strlen(HA_BASE_URL) > 0 && strlen(HA_ACCESS_TOKEN) > 0;
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

  // Always include primary wake phrases first, even if config drifts.
  String configured = String("ok bob,okay bob,") + String(HA_WAKE_PHRASE);
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

String urlEncode(const String& input) {
  String out;
  out.reserve(input.length() * 3);
  const char* hex = "0123456789ABCDEF";
  for (size_t i = 0; i < input.length(); ++i) {
    const uint8_t c = static_cast<uint8_t>(input[i]);
    if ((c >= 'a' && c <= 'z') ||
        (c >= 'A' && c <= 'Z') ||
        (c >= '0' && c <= '9') ||
        c == '-' || c == '_' || c == '.' || c == '~') {
      out += static_cast<char>(c);
      continue;
    }
    out += '%';
    out += hex[c >> 4];
    out += hex[c & 0x0F];
  }
  return out;
}

String isoUtc(time_t epoch) {
  struct tm tmUtc {};
  gmtime_r(&epoch, &tmUtc);
  char buf[24] = {0};
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tmUtc);
  return String(buf);
}

String formatTimeLabel(time_t epoch, bool includeDate) {
  if (epoch <= 0) {
    return "";
  }
  struct tm tmLocal {};
  localtime_r(&epoch, &tmLocal);
  char buf[32] = {0};
  strftime(buf, sizeof(buf), includeDate ? "%a %H:%M" : "%H:%M", &tmLocal);
  return String(buf);
}

String formatDateLabel(time_t epoch) {
  if (epoch <= 0) {
    return "";
  }
  struct tm tmLocal {};
  localtime_r(&epoch, &tmLocal);
  char buf[40] = {0};
  strftime(buf, sizeof(buf), "%a, %b %d", &tmLocal);
  return String(buf);
}

time_t timegmCompat(const struct tm* value) {
  if (!value) {
    return 0;
  }
  struct tm tmp = *value;
  const char* oldTz = getenv("TZ");
  String oldTzValue = oldTz ? String(oldTz) : "";

  setenv("TZ", "UTC0", 1);
  tzset();
  const time_t utc = mktime(&tmp);

  if (oldTzValue.length() > 0) {
    setenv("TZ", oldTzValue.c_str(), 1);
  } else {
    unsetenv("TZ");
  }
  tzset();
  return utc;
}

bool parseIsoTimestamp(const String& value, time_t& outEpoch) {
  outEpoch = 0;
  String src = value;
  src.trim();
  if (src.length() < 10) {
    return false;
  }

  int year = 0;
  int month = 0;
  int day = 0;
  int hour = 0;
  int minute = 0;
  int second = 0;
  int tzSign = 0;
  int tzHour = 0;
  int tzMinute = 0;
  bool hasTime = src.indexOf('T') >= 0;
  bool isZulu = src.endsWith("Z");
  bool hasOffset = false;

  if (!hasTime) {
    if (sscanf(src.c_str(), "%d-%d-%d", &year, &month, &day) != 3) {
      return false;
    }
  } else {
    int dotPos = src.indexOf('.');
    if (dotPos >= 0) {
      int cutPos = dotPos + 1;
      while (cutPos < src.length()) {
        const char ch = src[cutPos];
        if (ch == 'Z' || ch == '+' || ch == '-') {
          break;
        }
        cutPos++;
      }
      src.remove(dotPos, cutPos - dotPos);
    }

    int tzPos = src.lastIndexOf('+');
    int negPos = src.lastIndexOf('-');
    const int tPos = src.indexOf('T');
    if (negPos > tPos) {
      tzPos = negPos;
    }
    if (tzPos > tPos) {
      hasOffset = true;
      const String offset = src.substring(tzPos);
      src = src.substring(0, tzPos);
      if (offset.length() >= 6) {
        tzSign = offset[0] == '-' ? -1 : 1;
        tzHour = offset.substring(1, 3).toInt();
        tzMinute = offset.substring(4, 6).toInt();
      }
    } else if (src.endsWith("Z")) {
      isZulu = true;
      src.remove(src.length() - 1);
    }

    const int parts = sscanf(src.c_str(), "%d-%d-%dT%d:%d:%d", &year, &month, &day, &hour, &minute, &second);
    if (parts < 5) {
      return false;
    }
    if (parts == 5) {
      second = 0;
    }
  }

  struct tm tmValue {};
  tmValue.tm_year = year - 1900;
  tmValue.tm_mon = month - 1;
  tmValue.tm_mday = day;
  tmValue.tm_hour = hour;
  tmValue.tm_min = minute;
  tmValue.tm_sec = second;
  tmValue.tm_isdst = -1;

  if (!hasTime) {
    outEpoch = mktime(&tmValue);
    return outEpoch > 0;
  }

  if (isZulu || hasOffset) {
    time_t utcBase = timegmCompat(&tmValue);
    if (hasOffset) {
      const int32_t offsetSeconds = (tzHour * 3600) + (tzMinute * 60);
      utcBase -= (tzSign * offsetSeconds);
    }
    outEpoch = utcBase;
    return outEpoch > 0;
  }

  outEpoch = mktime(&tmValue);
  return outEpoch > 0;
}

int32_t localDayKey(time_t epoch) {
  if (epoch <= 0) {
    return -1;
  }
  struct tm tmLocal {};
  localtime_r(&epoch, &tmLocal);
  return (tmLocal.tm_year + 1900) * 1000 + tmLocal.tm_yday;
}

String cloudAuthTokenForUrl(const String& url) {
  if (strlen(CLOUD_API_BEARER_TOKEN) > 0) {
    return String(CLOUD_API_BEARER_TOKEN);
  }
  if (strlen(HA_ACCESS_TOKEN) > 0 && strlen(HA_BASE_URL) > 0) {
    const String base = String(HA_BASE_URL);
    if (url.startsWith(base)) {
      return String(HA_ACCESS_TOKEN);
    }
  }
  return "";
}

bool httpGetJsonWithAuth(const String& url, String& outBody, String& outError) {
  outBody = "";
  outError = "";
  HTTPClient http;
  WiFiClientSecure secureClient;
  if (!beginHttp(http, secureClient, url)) {
    outError = "HTTP begin failed";
    return false;
  }

  http.setConnectTimeout(STT_CONNECT_TIMEOUT_MS);
  http.setTimeout(STT_IO_TIMEOUT_MS);
  http.addHeader("Accept", "application/json");
  const String authToken = cloudAuthTokenForUrl(url);
  if (authToken.length() > 0) {
    http.addHeader("Authorization", String("Bearer ") + authToken);
  }

  const int code = http.GET();
  if (code <= 0) {
    outError = String("HTTP ") + code;
    http.end();
    return false;
  }
  outBody = http.getString();
  http.end();
  if (code < 200 || code >= 300) {
    outError = String("HTTP ") + code;
    return false;
  }
  return true;
}

bool httpPostJsonWithAuth(const String& url, const String& payload, String& outError) {
  outError = "";
  HTTPClient http;
  WiFiClientSecure secureClient;
  if (!beginHttp(http, secureClient, url)) {
    outError = "HTTP begin failed";
    return false;
  }

  http.setConnectTimeout(STT_CONNECT_TIMEOUT_MS);
  http.setTimeout(STT_IO_TIMEOUT_MS);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Accept", "application/json");
  const String authToken = cloudAuthTokenForUrl(url);
  if (authToken.length() > 0) {
    http.addHeader("Authorization", String("Bearer ") + authToken);
  }

  const int code = http.POST(payload);
  if (code <= 0) {
    outError = String("HTTP ") + code;
    http.end();
    return false;
  }
  http.end();
  if (code < 200 || code >= 300) {
    outError = String("HTTP ") + code;
    return false;
  }
  return true;
}

void sortCalendarEvents() {
  if (calendarEventCount < 2) {
    return;
  }
  for (size_t i = 0; i < calendarEventCount - 1; ++i) {
    for (size_t j = i + 1; j < calendarEventCount; ++j) {
      if (calendarEvents[j].startEpoch < calendarEvents[i].startEpoch) {
        CalendarEventItem tmp = calendarEvents[i];
        calendarEvents[i] = calendarEvents[j];
        calendarEvents[j] = tmp;
      }
    }
  }
}

void sortSyncedTasks() {
  if (syncedTaskCount < 2) {
    return;
  }
  for (size_t i = 0; i < syncedTaskCount - 1; ++i) {
    for (size_t j = i + 1; j < syncedTaskCount; ++j) {
      if (syncedTasks[i].completed != syncedTasks[j].completed) {
        if (syncedTasks[i].completed) {
          SyncedTaskItem tmp = syncedTasks[i];
          syncedTasks[i] = syncedTasks[j];
          syncedTasks[j] = tmp;
        }
      }
    }
  }
}

String jsonStringOrEmpty(JsonVariantConst v) {
  if (v.is<const char*>()) {
    return String(v.as<const char*>());
  }
  if (v.is<String>()) {
    return v.as<String>();
  }
  return "";
}

bool parseCalendarPayload(const String& payload, String& outError) {
  outError = "";
  DynamicJsonDocument doc(24576);
  const auto jsonErr = deserializeJson(doc, payload);
  if (jsonErr) {
    outError = "Calendar JSON parse failed";
    return false;
  }

  JsonArray arr;
  if (doc.is<JsonArray>()) {
    arr = doc.as<JsonArray>();
  } else if (doc["items"].is<JsonArray>()) {
    arr = doc["items"].as<JsonArray>();
  } else if (doc["events"].is<JsonArray>()) {
    arr = doc["events"].as<JsonArray>();
  } else {
    outError = "Calendar payload missing array";
    return false;
  }

  calendarEventCount = 0;
  for (JsonVariantConst item : arr) {
    if (calendarEventCount >= MAX_CALENDAR_EVENTS) {
      break;
    }

    String title = jsonStringOrEmpty(item["summary"]);
    if (title.length() == 0) {
      title = jsonStringOrEmpty(item["title"]);
    }
    if (title.length() == 0) {
      title = jsonStringOrEmpty(item["name"]);
    }
    if (title.length() == 0) {
      title = "Untitled";
    }

    String id = jsonStringOrEmpty(item["id"]);
    if (id.length() == 0) {
      id = title;
    }

    String startRaw;
    String endRaw;
    if (item["start"].is<JsonObjectConst>()) {
      startRaw = jsonStringOrEmpty(item["start"]["dateTime"]);
      if (startRaw.length() == 0) {
        startRaw = jsonStringOrEmpty(item["start"]["date"]);
      }
    } else {
      startRaw = jsonStringOrEmpty(item["start"]);
    }
    if (item["end"].is<JsonObjectConst>()) {
      endRaw = jsonStringOrEmpty(item["end"]["dateTime"]);
      if (endRaw.length() == 0) {
        endRaw = jsonStringOrEmpty(item["end"]["date"]);
      }
    } else {
      endRaw = jsonStringOrEmpty(item["end"]);
    }

    time_t startEpoch = 0;
    if (!parseIsoTimestamp(startRaw, startEpoch)) {
      continue;
    }
    time_t endEpoch = 0;
    if (!parseIsoTimestamp(endRaw, endEpoch)) {
      endEpoch = startEpoch + 3600;
    }

    CalendarEventItem& ev = calendarEvents[calendarEventCount++];
    ev.id = id;
    ev.title = title;
    ev.startEpoch = startEpoch;
    ev.endEpoch = endEpoch;
    ev.startLabel = formatTimeLabel(startEpoch, true);
    ev.endLabel = formatTimeLabel(endEpoch, false);
    ev.alerted = false;
  }

  sortCalendarEvents();
  return true;
}

bool parseSyncedTasksPayload(const String& payload, String& outError) {
  outError = "";
  DynamicJsonDocument doc(24576);
  const auto jsonErr = deserializeJson(doc, payload);
  if (jsonErr) {
    outError = "Tasks JSON parse failed";
    return false;
  }

  JsonArray arr;
  if (doc.is<JsonArray>()) {
    arr = doc.as<JsonArray>();
  } else if (doc["items"].is<JsonArray>()) {
    arr = doc["items"].as<JsonArray>();
  } else if (doc["tasks"].is<JsonArray>()) {
    arr = doc["tasks"].as<JsonArray>();
  } else {
    outError = "Tasks payload missing array";
    return false;
  }

  syncedTaskCount = 0;
  for (JsonVariantConst item : arr) {
    if (syncedTaskCount >= MAX_SYNC_TASKS) {
      break;
    }

    String title = jsonStringOrEmpty(item["title"]);
    if (title.length() == 0) {
      title = jsonStringOrEmpty(item["name"]);
    }
    if (title.length() == 0) {
      title = jsonStringOrEmpty(item["text"]);
    }
    if (title.length() == 0) {
      continue;
    }

    String id = jsonStringOrEmpty(item["id"]);
    if (id.length() == 0) {
      id = String("task_") + syncedTaskCount;
    }

    bool completed = false;
    if (item["completed"].is<bool>()) {
      completed = item["completed"].as<bool>();
    } else {
      const String status = jsonStringOrEmpty(item["status"]);
      completed = status.equalsIgnoreCase("completed") || status.equalsIgnoreCase("done");
    }

    String due = jsonStringOrEmpty(item["due"]);
    if (due.length() == 0) {
      due = jsonStringOrEmpty(item["due_date"]);
    }
    if (due.length() == 0) {
      due = jsonStringOrEmpty(item["dueDate"]);
    }

    time_t dueEpoch = 0;
    String dueLabel = "";
    if (parseIsoTimestamp(due, dueEpoch)) {
      dueLabel = formatDateLabel(dueEpoch);
    } else {
      dueLabel = due;
    }

    SyncedTaskItem& t = syncedTasks[syncedTaskCount++];
    t.id = id;
    t.title = title;
    t.dueLabel = dueLabel;
    t.completed = completed;
  }

  sortSyncedTasks();
  return true;
}

String resolveCalendarEndpoint() {
  if (strlen(CLOUD_CALENDAR_EVENTS_URL) > 0) {
    return String(CLOUD_CALENDAR_EVENTS_URL);
  }
  if (strlen(HA_CALENDAR_ENTITY_ID) > 0 && isHomeAssistantConfigured()) {
    String endpoint = joinedUrl(HA_BASE_URL, String("/api/calendars/") + String(HA_CALENDAR_ENTITY_ID));
    const time_t nowEpoch = time(nullptr);
    if (nowEpoch > 100000) {
      const time_t endEpoch = nowEpoch + (static_cast<time_t>(CLOUD_SYNC_DAYS_AHEAD) * 86400);
      endpoint += "?start=" + urlEncode(isoUtc(nowEpoch));
      endpoint += "&end=" + urlEncode(isoUtc(endEpoch));
    }
    return endpoint;
  }
  return "";
}

String resolveTasksEndpoint() {
  if (strlen(CLOUD_TASKS_LIST_URL) > 0) {
    return String(CLOUD_TASKS_LIST_URL);
  }
  return "";
}

String activeTodoEntity() {
  if (configuredTodoEntity.length() > 0) {
    return configuredTodoEntity;
  }
  if (strlen(HA_TODO_ENTITY_ID) > 0) {
    return String(HA_TODO_ENTITY_ID);
  }
  return String("todo.to_do_list");
}

void uiRefreshTodoEntityDropdown() {
  if (!uiDdTodoEntity) {
    return;
  }

  String options;
  if (todoEntityOptionCount == 0) {
    options = activeTodoEntity();
  } else {
    for (size_t i = 0; i < todoEntityOptionCount; ++i) {
      if (i > 0) {
        options += "\n";
      }
      options += todoEntityOptions[i];
    }
  }
  if (options.length() == 0) {
    options = "todo.to_do_list";
  }
  lv_dropdown_set_options(uiDdTodoEntity, options.c_str());

  const String selectedEntity = activeTodoEntity();
  uint16_t selectedIndex = 0;
  bool found = false;
  for (size_t i = 0; i < todoEntityOptionCount; ++i) {
    if (todoEntityOptions[i].equalsIgnoreCase(selectedEntity)) {
      selectedIndex = static_cast<uint16_t>(i);
      found = true;
      break;
    }
  }
  if (!found && todoEntityOptionCount > 0) {
    configuredTodoEntity = todoEntityOptions[0];
    saveTodoEntityConfig(configuredTodoEntity);
    selectedIndex = 0;
  }
  lv_dropdown_set_selected(uiDdTodoEntity, selectedIndex);
}

bool fetchTodoEntityOptionsFromHomeAssistant(String& outError) {
  outError = "";
  if (!isHomeAssistantConfigured()) {
    outError = "HA not configured";
    return false;
  }

  const String endpoint = joinedUrl(HA_BASE_URL, "/api/template");
  HTTPClient http;
  WiFiClientSecure secureClient;
  if (!beginHttp(http, secureClient, endpoint)) {
    outError = "HA todo-entity query begin failed";
    return false;
  }
  http.setConnectTimeout(STT_CONNECT_TIMEOUT_MS);
  http.setTimeout(STT_IO_TIMEOUT_MS);
  http.addHeader("Authorization", String("Bearer ") + HA_ACCESS_TOKEN);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Accept", "application/json");

  DynamicJsonDocument req(512);
  req["template"] = "{{ states.todo | map(attribute='entity_id') | list | tojson }}";
  String payload;
  serializeJson(req, payload);

  const int code = http.POST(payload);
  if (code <= 0) {
    outError = String("HA template HTTP ") + code;
    http.end();
    return false;
  }

  String response = http.getString();
  http.end();
  if (code < 200 || code >= 300) {
    outError = String("HA template HTTP ") + code + ": " + clipText(response, 80);
    return false;
  }

  DynamicJsonDocument doc(4096);
  if (deserializeJson(doc, response)) {
    outError = "HA todo-entity parse failed";
    return false;
  }
  if (!doc.is<JsonArray>()) {
    outError = "HA todo-entity response not array";
    return false;
  }

  todoEntityOptionCount = 0;
  for (JsonVariant item : doc.as<JsonArray>()) {
    if (todoEntityOptionCount >= MAX_TODO_ENTITY_OPTIONS) {
      break;
    }
    String entity = jsonStringOrEmpty(item);
    entity.trim();
    if (!entity.startsWith("todo.")) {
      continue;
    }
    bool duplicate = false;
    for (size_t i = 0; i < todoEntityOptionCount; ++i) {
      if (todoEntityOptions[i].equalsIgnoreCase(entity)) {
        duplicate = true;
        break;
      }
    }
    if (!duplicate) {
      todoEntityOptions[todoEntityOptionCount++] = entity;
    }
  }

  if (todoEntityOptionCount == 0) {
    outError = "No todo.* entities found";
    return false;
  }
  return true;
}

bool fetchCalendarFromCloud(String& outError) {
  outError = "";
  const String endpoint = resolveCalendarEndpoint();
  if (endpoint.length() == 0) {
    outError = "Calendar endpoint not configured";
    return false;
  }

  String body;
  if (!httpGetJsonWithAuth(endpoint, body, outError)) {
    return false;
  }

  return parseCalendarPayload(body, outError);
}

bool fetchSyncedTasksFromCloud(String& outError) {
  outError = "";
  const String endpoint = resolveTasksEndpoint();
  if (endpoint.length() > 0) {
    String body;
    if (!httpGetJsonWithAuth(endpoint, body, outError)) {
      return false;
    }
    return parseSyncedTasksPayload(body, outError);
  }

  if (activeTodoEntity().startsWith("todo.") && isHomeAssistantConfigured()) {
    return fetchSyncedTasksFromHomeAssistant(outError);
  }

  outError = "Task source not configured";
  return false;
}

bool fetchSyncedTasksFromHomeAssistant(String& outError) {
  outError = "";
  if (!isHomeAssistantConfigured()) {
    outError = "HA not configured";
    return false;
  }
  const String todoEntity = activeTodoEntity();
  if (!todoEntity.startsWith("todo.")) {
    outError = "HA to-do entity not set";
    return false;
  }

  const String endpoint = joinedUrl(HA_BASE_URL, "/api/services/todo/get_items?return_response");
  HTTPClient http;
  WiFiClientSecure secureClient;
  if (!beginHttp(http, secureClient, endpoint)) {
    outError = "HA to-do request begin failed";
    return false;
  }
  http.setConnectTimeout(STT_CONNECT_TIMEOUT_MS);
  http.setTimeout(STT_IO_TIMEOUT_MS);
  http.addHeader("Authorization", String("Bearer ") + HA_ACCESS_TOKEN);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Accept", "application/json");

  DynamicJsonDocument req(512);
  req["entity_id"] = todoEntity;
  JsonArray statuses = req.createNestedArray("status");
  statuses.add("needs_action");
  statuses.add("completed");
  String payload;
  serializeJson(req, payload);

  const int code = http.POST(payload);
  if (code <= 0) {
    outError = String("HA to-do HTTP ") + code;
    http.end();
    return false;
  }

  const String response = http.getString();
  http.end();
  if (code < 200 || code >= 300) {
    outError = String("HA to-do HTTP ") + code + ": " + clipText(response, 80);
    return false;
  }

  DynamicJsonDocument doc(16384);
  if (deserializeJson(doc, response)) {
    outError = "HA to-do JSON parse failed";
    return false;
  }

  JsonVariant serviceResponse = doc["service_response"];
  if (serviceResponse.isNull()) {
    serviceResponse = doc.as<JsonVariant>();
  }

  JsonArray itemsArray;
  if (serviceResponse[todoEntity]["items"].is<JsonArray>()) {
    itemsArray = serviceResponse[todoEntity]["items"].as<JsonArray>();
  } else if (serviceResponse["items"].is<JsonArray>()) {
    itemsArray = serviceResponse["items"].as<JsonArray>();
  } else if (serviceResponse.is<JsonObject>()) {
    for (JsonPair kv : serviceResponse.as<JsonObject>()) {
      if (kv.value()["items"].is<JsonArray>()) {
        itemsArray = kv.value()["items"].as<JsonArray>();
        break;
      }
    }
  }

  if (itemsArray.isNull()) {
    outError = "HA to-do response missing items";
    return false;
  }

  syncedTaskCount = 0;
  for (JsonVariant item : itemsArray) {
    if (syncedTaskCount >= MAX_SYNC_TASKS) {
      break;
    }

    const String title = jsonStringOrEmpty(item["summary"]);
    if (title.length() == 0) {
      continue;
    }

    String uid = jsonStringOrEmpty(item["uid"]);
    if (uid.length() == 0) {
      uid = title;
    }

    const String status = jsonStringOrEmpty(item["status"]);
    const bool completed = status.equalsIgnoreCase("completed");

    String dueLabel = jsonStringOrEmpty(item["due_date"]);
    if (dueLabel.length() == 0) {
      dueLabel = jsonStringOrEmpty(item["due_datetime"]);
    }
    time_t dueEpoch = 0;
    if (parseIsoTimestamp(dueLabel, dueEpoch)) {
      dueLabel = formatDateLabel(dueEpoch);
    }

    SyncedTaskItem& t = syncedTasks[syncedTaskCount++];
    t.id = uid;
    t.title = title;
    t.dueLabel = dueLabel;
    t.completed = completed;
  }

  sortSyncedTasks();
  return true;
}

bool saveCloudCache(String& outError) {
  outError = "";
  if (!sdReady) {
    outError = "SD unavailable";
    return false;
  }

  DynamicJsonDocument doc(24576);
  doc["saved_at"] = static_cast<int64_t>(time(nullptr));
  doc["calendar_count"] = static_cast<uint32_t>(calendarEventCount);
  doc["tasks_count"] = static_cast<uint32_t>(syncedTaskCount);

  JsonArray cal = doc.createNestedArray("calendar");
  for (size_t i = 0; i < calendarEventCount; ++i) {
    JsonObject obj = cal.createNestedObject();
    obj["id"] = calendarEvents[i].id;
    obj["title"] = calendarEvents[i].title;
    obj["start_epoch"] = static_cast<int64_t>(calendarEvents[i].startEpoch);
    obj["end_epoch"] = static_cast<int64_t>(calendarEvents[i].endEpoch);
    obj["alerted"] = calendarEvents[i].alerted;
  }

  JsonArray tasks = doc.createNestedArray("tasks");
  for (size_t i = 0; i < syncedTaskCount; ++i) {
    JsonObject obj = tasks.createNestedObject();
    obj["id"] = syncedTasks[i].id;
    obj["title"] = syncedTasks[i].title;
    obj["due"] = syncedTasks[i].dueLabel;
    obj["completed"] = syncedTasks[i].completed;
  }

  if (SD_MMC.exists("/voice/dashboard_cache.json")) {
    SD_MMC.remove("/voice/dashboard_cache.json");
  }
  File f = SD_MMC.open("/voice/dashboard_cache.json", FILE_WRITE);
  if (!f) {
    outError = "Cache open failed";
    return false;
  }
  if (serializeJson(doc, f) == 0) {
    f.close();
    outError = "Cache write failed";
    return false;
  }
  f.flush();
  f.close();
  cloudDataLoadedFromCache = true;
  return true;
}

bool loadCloudCache(String& outError) {
  outError = "";
  if (!sdReady) {
    outError = "SD unavailable";
    return false;
  }
  if (!SD_MMC.exists("/voice/dashboard_cache.json")) {
    outError = "No cache file";
    return false;
  }

  File f = SD_MMC.open("/voice/dashboard_cache.json", FILE_READ);
  if (!f) {
    outError = "Cache open failed";
    return false;
  }

  DynamicJsonDocument doc(24576);
  const auto jsonErr = deserializeJson(doc, f);
  f.close();
  if (jsonErr) {
    outError = "Cache parse failed";
    return false;
  }

  calendarEventCount = 0;
  if (doc["calendar"].is<JsonArray>()) {
    for (JsonVariantConst item : doc["calendar"].as<JsonArray>()) {
      if (calendarEventCount >= MAX_CALENDAR_EVENTS) {
        break;
      }
      CalendarEventItem& ev = calendarEvents[calendarEventCount++];
      ev.id = jsonStringOrEmpty(item["id"]);
      ev.title = jsonStringOrEmpty(item["title"]);
      ev.startEpoch = static_cast<time_t>(item["start_epoch"] | 0);
      ev.endEpoch = static_cast<time_t>(item["end_epoch"] | 0);
      ev.alerted = item["alerted"] | false;
      ev.startLabel = formatTimeLabel(ev.startEpoch, true);
      ev.endLabel = formatTimeLabel(ev.endEpoch, false);
    }
    sortCalendarEvents();
  }

  syncedTaskCount = 0;
  if (doc["tasks"].is<JsonArray>()) {
    for (JsonVariantConst item : doc["tasks"].as<JsonArray>()) {
      if (syncedTaskCount >= MAX_SYNC_TASKS) {
        break;
      }
      SyncedTaskItem& task = syncedTasks[syncedTaskCount++];
      task.id = jsonStringOrEmpty(item["id"]);
      task.title = jsonStringOrEmpty(item["title"]);
      task.dueLabel = jsonStringOrEmpty(item["due"]);
      task.completed = item["completed"] | false;
    }
    sortSyncedTasks();
  }

  cloudDataLoadedFromCache = true;
  return true;
}

void uiRefreshCalendarList() {
  if (!uiListCalendar) {
    return;
  }
  lv_obj_clean(uiListCalendar);

  if (calendarEventCount == 0) {
    lv_list_add_text(uiListCalendar, "No cached events");
    return;
  }

  const int32_t todayKey = localDayKey(time(nullptr));
  for (size_t i = 0; i < calendarEventCount; ++i) {
    const CalendarEventItem& ev = calendarEvents[i];
    const int32_t eventDayKey = localDayKey(ev.startEpoch);
    const String dayPrefix = eventDayKey == todayKey ? "Today " : formatDateLabel(ev.startEpoch) + " ";
    String line = dayPrefix + formatTimeLabel(ev.startEpoch, false) + "  " + clipText(ev.title, 38);
    lv_obj_t* btn = lv_list_add_btn(uiListCalendar, LV_SYMBOL_RIGHT, line.c_str());

    const time_t nowEpoch = time(nullptr);
    if (nowEpoch > 100000 && ev.startEpoch > nowEpoch && (ev.startEpoch - nowEpoch) <= (CALENDAR_ALERT_LEAD_MINUTES * 60)) {
      lv_obj_set_style_text_color(btn, lv_color_hex(0xFBBF24), LV_PART_MAIN);
    } else {
      lv_obj_set_style_text_color(btn, lv_color_hex(0xD1D5DB), LV_PART_MAIN);
    }
  }
}

void uiRefreshSyncTasksList() {
  if (!uiListSyncTasks) {
    return;
  }
  lv_obj_clean(uiListSyncTasks);

  if (syncedTaskCount == 0) {
    lv_list_add_text(uiListSyncTasks, "No synced tasks");
    return;
  }

  for (size_t i = 0; i < syncedTaskCount; ++i) {
    const SyncedTaskItem& task = syncedTasks[i];
    String line = String(task.completed ? "[x] " : "[ ] ") + clipText(task.title, 34);
    if (task.dueLabel.length() > 0) {
      line += " (" + clipText(task.dueLabel, 12) + ")";
    }
    lv_obj_t* btn = lv_list_add_btn(uiListSyncTasks, task.completed ? LV_SYMBOL_OK : LV_SYMBOL_MINUS, line.c_str());
    lv_obj_add_event_cb(
        btn,
        uiSyncedTaskToggleEventCb,
        LV_EVENT_CLICKED,
        reinterpret_cast<void*>(static_cast<uintptr_t>(i + 1)));
    lv_obj_set_style_text_color(btn, task.completed ? lv_color_hex(0x9CA3AF) : lv_color_hex(0xE5E7EB), LV_PART_MAIN);
  }
}

bool postTaskCompletionWebhook(const SyncedTaskItem& task, bool completed, String& outError) {
  outError = "";
  if (strlen(CLOUD_TASK_COMPLETE_WEBHOOK_URL) == 0) {
    outError = "Task completion webhook not set";
    return false;
  }

  DynamicJsonDocument doc(512);
  doc["id"] = task.id;
  doc["title"] = task.title;
  doc["completed"] = completed;
  doc["status"] = completed ? "completed" : "needsAction";
  String payload;
  serializeJson(doc, payload);
  return httpPostJsonWithAuth(String(CLOUD_TASK_COMPLETE_WEBHOOK_URL), payload, outError);
}

bool updateTodoItemInHomeAssistant(const SyncedTaskItem& task, bool completed, String& outError) {
  outError = "";
  if (!isHomeAssistantConfigured()) {
    outError = "HA not configured";
    return false;
  }
  const String todoEntity = activeTodoEntity();
  if (!todoEntity.startsWith("todo.")) {
    outError = "HA to-do entity not set";
    return false;
  }

  const String endpoint = joinedUrl(HA_BASE_URL, "/api/services/todo/update_item");
  auto sendUpdate = [&](const String& itemRef, String& err) -> bool {
    if (itemRef.length() == 0) {
      err = "Empty item reference";
      return false;
    }

    HTTPClient http;
    WiFiClientSecure secureClient;
    if (!beginHttp(http, secureClient, endpoint)) {
      err = "HA to-do update begin failed";
      return false;
    }
    http.setConnectTimeout(STT_CONNECT_TIMEOUT_MS);
    http.setTimeout(STT_IO_TIMEOUT_MS);
    http.addHeader("Authorization", String("Bearer ") + HA_ACCESS_TOKEN);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Accept", "application/json");

    DynamicJsonDocument req(512);
    req["entity_id"] = todoEntity;
    req["item"] = itemRef;
    req["status"] = completed ? "completed" : "needs_action";

    String payload;
    serializeJson(req, payload);
    const int code = http.POST(payload);
    const String response = http.getString();
    http.end();
    if (code >= 200 && code < 300) {
      return true;
    }
    err = String("HA to-do HTTP ") + code + ": " + clipText(response, 80);
    return false;
  };

  String attemptErr;
  if (sendUpdate(task.id, attemptErr)) {
    return true;
  }

  if (task.title != task.id && task.title.length() > 0) {
    String fallbackErr;
    if (sendUpdate(task.title, fallbackErr)) {
      return true;
    }
    outError = fallbackErr;
    return false;
  }

  outError = attemptErr;
  return false;
}

bool playAlertChime() {
  if (!i2sMicReady) {
    return false;
  }

  static int16_t toneBuf[4096];
  size_t idx = 0;
  const auto appendTone = [&](float freqHz, uint16_t ms, float gain) {
    const size_t count = static_cast<size_t>((CAPTURE_SAMPLE_RATE * ms) / 1000UL);
    for (size_t i = 0; i < count && idx < (sizeof(toneBuf) / sizeof(toneBuf[0])); ++i) {
      const float phase = (2.0f * PI * freqHz * static_cast<float>(i)) / static_cast<float>(CAPTURE_SAMPLE_RATE);
      const float sample = sinf(phase) * gain;
      toneBuf[idx++] = static_cast<int16_t>(sample * 32767.0f);
    }
    const size_t gap = CAPTURE_SAMPLE_RATE / 100;
    for (size_t i = 0; i < gap && idx < (sizeof(toneBuf) / sizeof(toneBuf[0])); ++i) {
      toneBuf[idx++] = 0;
    }
  };

  appendTone(880.0f, 90, 0.20f);
  appendTone(1174.0f, 90, 0.18f);
  appendTone(1568.0f, 120, 0.16f);
  if (idx == 0) {
    return false;
  }
  return micCapture.playPcm16Blocking(
      reinterpret_cast<const uint8_t*>(toneBuf),
      idx * sizeof(int16_t),
      CAPTURE_SAMPLE_RATE);
}

void checkCalendarAlerts(uint32_t nowMs) {
  static uint32_t lastCheckMs = 0;
  if (nowMs - lastCheckMs < 4000) {
    return;
  }
  lastCheckMs = nowMs;

  const time_t nowEpoch = time(nullptr);
  if (nowEpoch < 100000 || calendarEventCount == 0) {
    return;
  }

  for (size_t i = 0; i < calendarEventCount; ++i) {
    CalendarEventItem& ev = calendarEvents[i];
    if (ev.alerted) {
      continue;
    }
    if (ev.startEpoch <= nowEpoch) {
      continue;
    }
    const time_t secondsUntil = ev.startEpoch - nowEpoch;
    if (secondsUntil > (CALENDAR_ALERT_LEAD_MINUTES * 60)) {
      continue;
    }

    ev.alerted = true;
    const uint32_t mins = static_cast<uint32_t>((secondsUntil + 59) / 60);
    uiSetCalendarStatus(String("Upcoming: ") + clipText(ev.title, 28) + " in " + mins + "m");
    playAlertChime();
    String cacheErr;
    saveCloudCache(cacheErr);
    return;
  }
}

void syncClockIfNeeded() {
  static bool tzSet = false;
  static uint32_t lastNtpTryMs = 0;
  const uint32_t nowMs = millis();

  if (!tzSet) {
    setenv("TZ", DEVICE_TZ, 1);
    tzset();
    tzSet = true;
  }

  const time_t nowEpoch = time(nullptr);
  if (nowEpoch > 1700000000) {
    return;
  }
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }
  if ((nowMs - lastNtpTryMs) < 30000) {
    return;
  }
  lastNtpTryMs = nowMs;

  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  struct tm tmNow {};
  for (int i = 0; i < 6; ++i) {
    if (getLocalTime(&tmNow, 800)) {
      return;
    }
    delay(100);
  }
}

bool syncCloudData(bool force, String& outError) {
  outError = "";
  const uint32_t nowMs = millis();
  if (!force && (nowMs - lastCloudSyncAttemptMs) < CLOUD_SYNC_RETRY_MS) {
    outError = "Sync cooling down";
    return false;
  }
  lastCloudSyncAttemptMs = nowMs;

  syncClockIfNeeded();

  if (WiFi.status() != WL_CONNECTED) {
    String cacheErr;
    if (loadCloudCache(cacheErr)) {
      uiSetCalendarStatus("Offline: showing cached data");
      uiSetSyncTasksStatus("Offline: showing cached tasks");
      uiRefreshCalendarList();
      uiRefreshSyncTasksList();
      return true;
    }
    outError = "Wi-Fi disconnected";
    return false;
  }

  bool anySuccess = false;
  String errCalendar;
  if (resolveCalendarEndpoint().length() > 0) {
    if (fetchCalendarFromCloud(errCalendar)) {
      anySuccess = true;
      uiSetCalendarStatus("Calendar synced");
      uiRefreshCalendarList();
    } else {
      uiSetCalendarStatus("Calendar sync failed: " + clipText(errCalendar, 40));
    }
  } else {
    uiSetCalendarStatus("Calendar endpoint not configured");
  }

  String errTasks;
  const bool hasTasksSource =
      resolveTasksEndpoint().length() > 0 || (activeTodoEntity().startsWith("todo.") && isHomeAssistantConfigured());
  if (hasTasksSource) {
    if (fetchSyncedTasksFromCloud(errTasks)) {
      anySuccess = true;
      uiSetSyncTasksStatus("Tasks synced");
      uiRefreshSyncTasksList();
    } else {
      uiSetSyncTasksStatus("Task sync failed: " + clipText(errTasks, 40));
    }
  } else {
    uiSetSyncTasksStatus("Task source not configured");
  }

  if (!anySuccess) {
    outError = errCalendar.length() > 0 ? errCalendar : errTasks;
    if (outError.length() == 0) {
      outError = "No sync sources configured";
    }
    return false;
  }

  String cacheErr;
  if (!saveCloudCache(cacheErr)) {
    Serial.println(String("[CACHE] save failed: ") + cacheErr);
  }
  return true;
}

void maybeRunScheduledCloudSync(uint32_t nowMs) {
  syncClockIfNeeded();

  const time_t nowEpoch = time(nullptr);
  if (nowEpoch <= 100000) {
    return;
  }

  struct tm tmLocal {};
  localtime_r(&nowEpoch, &tmLocal);
  const int32_t dayKey = localDayKey(nowEpoch);
  if (tmLocal.tm_hour == 6 && tmLocal.tm_min < 6 && dayKey != lastCloudSyncDayKey) {
    lastCloudSyncDayKey = dayKey;
    cloudSyncRequested = true;
  }
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

bool hasLikelySpeechInCapture(
    const int16_t* samples,
    size_t sampleCount,
    float& outSpeechPercent,
    float& outRmsGate,
    float& outPeakGate) {
  outSpeechPercent = 0.0f;

  if (!samples || sampleCount < (PRE_STT_FRAME_SAMPLES / 2)) {
    outRmsGate = 0.0f;
    outPeakGate = 0.0f;
    return false;
  }

  const float baseRms = vadNoiseFloorReady ? vadNoiseFloorNorm : 0.0040f;
  const float basePeak = vadNoiseFloorReady ? vadNoiseFloorPeakNorm : 0.0140f;

  outRmsGate = baseRms * 1.45f + 0.0010f;
  if (outRmsGate < 0.0080f) {
    outRmsGate = 0.0080f;
  }
  if (outRmsGate > 0.0500f) {
    outRmsGate = 0.0500f;
  }

  outPeakGate = basePeak * 1.55f + 0.0120f;
  if (outPeakGate < 0.0350f) {
    outPeakGate = 0.0350f;
  }
  if (outPeakGate > 0.2800f) {
    outPeakGate = 0.2800f;
  }

  size_t frames = 0;
  size_t hotFrames = 0;
  for (size_t i = 0; i < sampleCount; i += PRE_STT_FRAME_SAMPLES) {
    const size_t n = (sampleCount - i) < PRE_STT_FRAME_SAMPLES ? (sampleCount - i) : PRE_STT_FRAME_SAMPLES;
    if (n < 80) {
      break;
    }
    CaptureMetrics fm = I2SMicCapture::analyze(samples + i, n);
    frames++;
    if (fm.rmsNorm >= outRmsGate || fm.peakNorm >= outPeakGate) {
      hotFrames++;
    }
  }

  if (frames == 0) {
    return false;
  }

  outSpeechPercent = (100.0f * static_cast<float>(hotFrames)) / static_cast<float>(frames);
  float requiredPercent = PRE_STT_MIN_SPEECH_PERCENT;
  if (sampleCount < CAPTURE_SAMPLE_RATE) {
    requiredPercent = 5.0f;
  }
  return outSpeechPercent >= requiredPercent;
}

void uiUpdateMicLevelLine() {
  if (state != AssistantState::Idle) {
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
  latestMicStatsLine = line;

  static uint32_t lastMicSerialMs = 0;
  if ((now - lastMicSerialMs) > 2000) {
    lastMicSerialMs = now;
    Serial.println(String("[MICDBG] ") + line);
  }
}

bool calibrateVoiceNoiseFloor(String& outSummary) {
  outSummary = "";
  if (!audioReady) {
    outSummary = "Audio not ready";
    return false;
  }
  if (micMuted) {
    outSummary = "Mic is muted";
    return false;
  }

  const bool pausedWake = pauseLocalWakeEngine();
  const uint32_t startMs = millis();
  float rmsFloor = 0.0f;
  float peakFloor = 0.0f;
  uint16_t count = 0;

  while ((millis() - startMs) < 1400) {
    VadSnapshot s = sampleVadSnapshot();
    if (s.rmsNorm > 0.0f || s.peakNorm > 0.0f) {
      if (count == 0) {
        rmsFloor = s.rmsNorm;
        peakFloor = s.peakNorm;
      } else {
        rmsFloor = rmsFloor * 0.86f + s.rmsNorm * 0.14f;
        peakFloor = peakFloor * 0.86f + s.peakNorm * 0.14f;
      }
      count++;
    }
    lv_timer_handler();
    delay(35);
  }

  if (pausedWake) {
    resumeLocalWakeEngine();
  }

  if (count < 4) {
    outSummary = "Sample failed";
    return false;
  }

  vadNoiseFloorNorm = rmsFloor;
  vadNoiseFloorPeakNorm = peakFloor;
  vadNoiseFloorReady = true;

  outSummary = String(normToDbFs(vadNoiseFloorNorm), 1) + " dBFS floor";
  Serial.println(String("[VAD] calibrated floor rms=") + String(vadNoiseFloorNorm * 100.0f, 3) +
                 "% peak=" + String(vadNoiseFloorPeakNorm * 100.0f, 3) + "%");
  return true;
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
    CaptureMetrics& metrics,
    bool autoStopOnSilence = false) {
  outSamples = 0;
  if (!out || capacitySamples == 0 || !i2sMicReady) {
    return false;
  }

  const size_t chunkSamples = CAPTURE_SAMPLE_RATE / 5;  // ~200ms chunks for responsive stop.
  const uint32_t startMs = millis();
  const uint32_t maxMs = autoStopOnSilence ? DICTATION_CAPTURE_MAX_MS : MANUAL_CAPTURE_MAX_MS;
  bool speechDetected = false;
  uint8_t silentChunks = 0;

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

    if (autoStopOnSilence) {
      const bool chunkHasSpeech =
          chunkMetrics.rmsNorm >= DICTATION_SPEECH_START_RMS ||
          chunkMetrics.peakNorm >= DICTATION_SPEECH_START_PEAK;

      if (chunkHasSpeech) {
        speechDetected = true;
        silentChunks = 0;
      } else if (speechDetected) {
        const bool chunkIsSilent =
            chunkMetrics.rmsNorm <= DICTATION_SILENCE_RMS &&
            chunkMetrics.peakNorm <= DICTATION_SILENCE_PEAK;
        if (chunkIsSilent) {
          if (silentChunks < 255) {
            silentChunks++;
          }
          if (silentChunks >= DICTATION_END_SILENT_CHUNKS) {
            Serial.println(String("[MIC] dictation auto-stop silence after ") + outSamples + " samples");
            break;
          }
        } else {
          silentChunks = 0;
        }
      }
    }

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
    CaptureMetrics& metrics,
    bool shortWindow = false) {
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
  const uint32_t maxMs = shortWindow ? DICTATION_CAPTURE_MAX_MS : MANUAL_CAPTURE_MAX_MS;

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

struct AsyncPcmPlaybackContext {
  int16_t* ringSamples = nullptr;
  size_t ringCapacity = 0;
  size_t readPos = 0;
  size_t writePos = 0;
  size_t buffered = 0;
  uint32_t sampleRate = CAPTURE_SAMPLE_RATE;
  volatile bool producerDone = false;
  volatile bool abortRequested = false;
  volatile bool consumerFailed = false;
  volatile bool taskFinished = false;
  SemaphoreHandle_t mutex = nullptr;
  TaskHandle_t taskHandle = nullptr;
};

void asyncPcmPlaybackTask(void* arg) {
  AsyncPcmPlaybackContext* ctx = static_cast<AsyncPcmPlaybackContext*>(arg);
  if (!ctx) {
    vTaskDelete(nullptr);
    return;
  }

  int16_t chunk[TTS_ASYNC_CHUNK_SAMPLES] = {0};
  while (true) {
    if (ctx->abortRequested) {
      break;
    }

    size_t toPlay = 0;
    bool drainDone = false;
    if (xSemaphoreTake(ctx->mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      if (ctx->buffered > 0) {
        toPlay = ctx->buffered < TTS_ASYNC_CHUNK_SAMPLES ? ctx->buffered : TTS_ASYNC_CHUNK_SAMPLES;
        const size_t first = (ctx->readPos + toPlay <= ctx->ringCapacity) ? toPlay : (ctx->ringCapacity - ctx->readPos);
        memcpy(chunk, ctx->ringSamples + ctx->readPos, first * sizeof(int16_t));
        if (toPlay > first) {
          memcpy(chunk + first, ctx->ringSamples, (toPlay - first) * sizeof(int16_t));
        }
        ctx->readPos = (ctx->readPos + toPlay) % ctx->ringCapacity;
        ctx->buffered -= toPlay;
      }
      drainDone = ctx->producerDone && ctx->buffered == 0;
      xSemaphoreGive(ctx->mutex);
    }

    if (toPlay > 0) {
      if (!micCapture.playPcm16Blocking(
              reinterpret_cast<const uint8_t*>(chunk),
              toPlay * sizeof(int16_t),
              ctx->sampleRate)) {
        ctx->consumerFailed = true;
        ctx->abortRequested = true;
        break;
      }
      continue;
    }

    if (drainDone) {
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  ctx->taskFinished = true;
  ctx->taskHandle = nullptr;
  vTaskDelete(nullptr);
}

bool asyncPcmPlaybackBegin(AsyncPcmPlaybackContext& ctx, uint32_t sampleRate, size_t ringSamples) {
  memset(&ctx, 0, sizeof(ctx));
  if (ringSamples < TTS_ASYNC_CHUNK_SAMPLES * 2) {
    ringSamples = TTS_ASYNC_CHUNK_SAMPLES * 2;
  }
  ctx.ringSamples = reinterpret_cast<int16_t*>(allocAudioBuffer(ringSamples * sizeof(int16_t)));
  if (!ctx.ringSamples) {
    return false;
  }
  ctx.ringCapacity = ringSamples;
  ctx.sampleRate = sampleRate;
  ctx.mutex = xSemaphoreCreateMutex();
  if (!ctx.mutex) {
    free(ctx.ringSamples);
    ctx.ringSamples = nullptr;
    return false;
  }
  if (xTaskCreatePinnedToCore(
          asyncPcmPlaybackTask,
          "tts_play",
          4096,
          &ctx,
          3,
          &ctx.taskHandle,
          TTS_ASYNC_CORE) != pdPASS) {
    vSemaphoreDelete(ctx.mutex);
    ctx.mutex = nullptr;
    free(ctx.ringSamples);
    ctx.ringSamples = nullptr;
    return false;
  }
  return true;
}

void asyncPcmPlaybackAbort(AsyncPcmPlaybackContext& ctx) {
  ctx.abortRequested = true;
  ctx.producerDone = true;
}

bool asyncPcmPlaybackPush(
    AsyncPcmPlaybackContext& ctx,
    const int16_t* samples,
    size_t sampleCount,
    String& outError) {
  if (!samples || sampleCount == 0) {
    return true;
  }
  if (!ctx.taskHandle || !ctx.ringSamples || ctx.ringCapacity == 0) {
    outError = "TTS playback task unavailable";
    return false;
  }

  size_t pushed = 0;
  uint32_t waitStart = millis();
  while (pushed < sampleCount) {
    if (ctx.abortRequested || ctx.consumerFailed) {
      outError = "TTS playback aborted";
      return false;
    }

    size_t copied = 0;
    if (xSemaphoreTake(ctx.mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      const size_t freeSamples = ctx.ringCapacity - ctx.buffered;
      if (freeSamples > 0) {
        copied = (sampleCount - pushed) < freeSamples ? (sampleCount - pushed) : freeSamples;
        const size_t first = (ctx.writePos + copied <= ctx.ringCapacity) ? copied : (ctx.ringCapacity - ctx.writePos);
        memcpy(ctx.ringSamples + ctx.writePos, samples + pushed, first * sizeof(int16_t));
        if (copied > first) {
          memcpy(ctx.ringSamples, samples + pushed + first, (copied - first) * sizeof(int16_t));
        }
        ctx.writePos = (ctx.writePos + copied) % ctx.ringCapacity;
        ctx.buffered += copied;
      }
      xSemaphoreGive(ctx.mutex);
    }

    if (copied == 0) {
      if ((millis() - waitStart) > TTS_ASYNC_PUSH_TIMEOUT_MS) {
        outError = "TTS ring buffer timeout";
        return false;
      }
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    pushed += copied;
    waitStart = millis();
  }
  return true;
}

bool asyncPcmPlaybackFinish(AsyncPcmPlaybackContext& ctx, String& outError) {
  ctx.producerDone = true;
  const uint32_t waitStart = millis();
  while (!ctx.taskFinished && (millis() - waitStart) < 20000) {
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  if (!ctx.taskFinished) {
    if (ctx.taskHandle) {
      vTaskDelete(ctx.taskHandle);
      ctx.taskHandle = nullptr;
    }
    outError = "TTS playback task timeout";
  } else if (ctx.consumerFailed) {
    outError = "I2S playback task failed";
  }

  if (ctx.mutex) {
    vSemaphoreDelete(ctx.mutex);
    ctx.mutex = nullptr;
  }
  if (ctx.ringSamples) {
    free(ctx.ringSamples);
    ctx.ringSamples = nullptr;
  }

  return outError.length() == 0;
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

  AsyncPcmPlaybackContext asyncPlayback{};
  if (!asyncPcmPlaybackBegin(asyncPlayback, playbackRate, TTS_ASYNC_RING_SAMPLES)) {
    outError = "TTS async buffer init failed";
    return false;
  }
  const auto abortAndFinish = [&](const String& err) -> bool {
    outError = err;
    asyncPcmPlaybackAbort(asyncPlayback);
    String finishErr;
    asyncPcmPlaybackFinish(asyncPlayback, finishErr);
    return false;
  };
  const auto pushPcm = [&](const int16_t* samples, size_t sampleCount, const char* fallbackErr) -> bool {
    String pushErr;
    if (!asyncPcmPlaybackPush(asyncPlayback, samples, sampleCount, pushErr)) {
      if (pushErr.length() == 0) {
        pushErr = fallbackErr;
      }
      return abortAndFinish(pushErr);
    }
    return true;
  };

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
        return abortAndFinish("WAV stream timeout");
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
      if (!pushPcm(inSamples, sampleCount, "I2S mono playback failed")) {
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

      if (!pushPcm(monoSamples, frameCount, "I2S stereo downmix failed")) {
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
            if (!pushPcm(monoSamples, resampleOutFrames, "I2S resample playback failed")) {
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
    if (!pushPcm(monoSamples, resampleOutFrames, "I2S resample playback failed")) {
      return false;
    }
  }

  if (totalStreamed == 0) {
    return abortAndFinish("WAV has no data");
  }

  String finishErr;
  if (!asyncPcmPlaybackFinish(asyncPlayback, finishErr)) {
    outError = finishErr;
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

bool homeAssistantStt(
    const int16_t* samples,
    size_t sampleCount,
    String& outText,
    String& outError,
    uint8_t maxRetries = STT_MAX_RETRIES) {
  if (!isHomeAssistantConfigured()) {
    outError = "HA not configured";
    return false;
  }
  if (strlen(HA_STT_PROVIDER) == 0) {
    outError = "STT provider not configured";
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

    for (uint8_t retry = 0; retry <= maxRetries; ++retry) {
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

      if (isRetryableSttTransportError(code) && retry < maxRetries) {
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
    const String stateLine = String("Status: ") + stateName(state);
    lv_label_set_text(uiLabelState, stateLine.c_str());
    lv_obj_set_style_text_color(uiLabelState, uiStateColor(state), LV_PART_MAIN);
  }
  if (uiIconState) {
    const char* icon = LV_SYMBOL_PLAY;
    if (state == AssistantState::Listening) {
      icon = LV_SYMBOL_AUDIO;
    } else if (state == AssistantState::Thinking) {
      icon = LV_SYMBOL_REFRESH;
    } else if (state == AssistantState::Speaking) {
      icon = LV_SYMBOL_VOLUME_MAX;
    } else if (state == AssistantState::Error) {
      icon = LV_SYMBOL_WARNING;
    } else if (state == AssistantState::Booting || state == AssistantState::ConnectingWiFi) {
      icon = LV_SYMBOL_LOOP;
    }
    lv_label_set_text(uiIconState, icon);
    lv_obj_set_style_text_color(uiIconState, uiStateColor(state), LV_PART_MAIN);
  }

  if (uiLabelDetail) {
    lv_label_set_text(uiLabelDetail, clipText(stateDetail, 52).c_str());
  }

  if (uiLabelHint) {
    lv_label_set_text(uiLabelHint, clipText(uiStateHint(state), 52).c_str());
    lv_obj_set_style_text_color(uiLabelHint, lv_color_hex(0x94A3B8), LV_PART_MAIN);
  }

  if (WiFi.status() == WL_CONNECTED) {
    if (uiLabelWifi) {
      const String wifiLine = String("Wi-Fi: ") + WiFi.SSID() + "  " + String(WiFi.RSSI()) + " dBm";
      lv_label_set_text(uiLabelWifi, clipText(wifiLine, 52).c_str());
      lv_obj_set_style_text_color(uiLabelWifi, lv_color_hex(0x86EFAC), LV_PART_MAIN);
    }
    if (uiIconWifi) {
      lv_obj_set_style_text_color(uiIconWifi, lv_color_hex(0x86EFAC), LV_PART_MAIN);
    }
  } else {
    if (uiLabelWifi) {
      lv_label_set_text(uiLabelWifi, "Wi-Fi: disconnected");
      lv_obj_set_style_text_color(uiLabelWifi, lv_color_hex(0xFCA5A5), LV_PART_MAIN);
    }
    if (uiIconWifi) {
      lv_obj_set_style_text_color(uiIconWifi, lv_color_hex(0xF87171), LV_PART_MAIN);
    }
  }

  if (uiIconSd) {
    lv_obj_set_style_text_color(uiIconSd, sdReady ? lv_color_hex(0x86EFAC) : lv_color_hex(0x6B7280), LV_PART_MAIN);
  }
  if (uiIconMic) {
    lv_label_set_text(uiIconMic, micMuted ? LV_SYMBOL_MUTE : LV_SYMBOL_AUDIO);
    if (micMuted) {
      lv_obj_set_style_text_color(uiIconMic, lv_color_hex(0xF97316), LV_PART_MAIN);
    } else {
      lv_obj_set_style_text_color(uiIconMic, audioReady ? lv_color_hex(0x67E8F9) : lv_color_hex(0xF87171), LV_PART_MAIN);
    }
  }
  uiUpdateMicMuteButton();
}

void runCapturePipeline(bool bypassWakeWord, CaptureMode captureMode) {
  const bool dictationMode = captureMode == CaptureMode::Dictation;
  struct LocalWakePauseGuard {
    bool shouldResume = false;
    LocalWakePauseGuard() { shouldResume = pauseLocalWakeEngine(); }
    ~LocalWakePauseGuard() {
      if (shouldResume) {
        resumeLocalWakeEngine();
      }
    }
  } localWakePauseGuard;

  const auto stopAndReturn = [&]() -> bool {
    if (!assistantStopRequested) {
      return false;
    }
    assistantStopRequested = false;
    setState(AssistantState::Idle, "Stopped");
    if (dictationMode) {
      uiSetNotesStatus("Dictation stopped");
    }
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
    setState(AssistantState::Listening, dictationMode ? "Dictating... tap Stop" : "Listening... tap Stop");
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
      if (captureI2SMicUntilStop(
              captureBuffer,
              captureBufferCapacity,
              manualSamples,
              i2sMetrics,
              dictationMode)) {
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
      if (captureAnalogMicUntilStop(
              captureBuffer,
              captureBufferCapacity,
              manualSamples,
              analogMetrics,
              dictationMode)) {
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
      if (dictationMode) {
        uiSetNotesStatus("Dictation stopped");
      }
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

  float localSpeechPct = 0.0f;
  float speechGateRms = 0.0f;
  float speechGatePeak = 0.0f;
  const bool likelySpeech = hasLikelySpeechInCapture(captureBuffer, captureSamples, localSpeechPct, speechGateRms, speechGatePeak);
  if (!likelySpeech) {
    handsFreeCooldownUntilMs = millis() + 2200;
    Serial.println(
        String("[VAD] pre-STT reject speech=") + String(localSpeechPct, 1) +
        String("% gateRMS=") + String(speechGateRms * 100.0f, 2) +
        String("% gatePeak=") + String(speechGatePeak * 100.0f, 2) + "%");
    setState(AssistantState::Idle, "No speech: " + clipText(levelText, 45));
    if (dictationMode) {
      uiSetNotesStatus("No speech detected");
    }
    uiSetTalkEnabled(true);
    return;
  }

  if (!isHomeAssistantConfigured() || WiFi.status() != WL_CONNECTED) {
    setState(AssistantState::Idle, "Captured locally");
    if (dictationMode) {
      uiSetNotesStatus("Dictation requires Home Assistant STT");
    }
    uiSetTalkEnabled(true);
    return;
  }

  String transcript;
  String sttErr;
  setState(AssistantState::Thinking, "Transcribing...");
  uiRefreshNow();
  const uint8_t sttRetries = dictationMode ? 0 : STT_MAX_RETRIES;
  if (!homeAssistantStt(captureBuffer, captureSamples, transcript, sttErr, sttRetries)) {
    Serial.println(String("[STT] error: ") + sttErr);
    handsFreeCooldownUntilMs = millis() + 4500;
    setState(AssistantState::Idle, "STT failed: " + clipText(sttErr, 48));
    if (dictationMode) {
      uiSetNotesStatus("STT failed: " + clipText(sttErr, 34));
    }
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
    if (dictationMode) {
      uiSetNotesStatus("No speech detected");
    }
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

  if (dictationMode) {
    uiAppendDictationNote(transcript);
    uiSetNotesStatus("Saved to notes");
    setState(AssistantState::Idle, "Note captured");
    uiSetTalkEnabled(true);
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
  uiSetAssistantReplyText(String("Response: ") + clipText(reply, 180));
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
    requestCapture(CaptureMode::Assistant, true);
    return;
  }
  assistantStopRequested = true;
  captureRequested = false;
  setState(state, "Stopping...");
}

void uiMicMuteToggleButtonEventCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }

  micMuted = !micMuted;
  if (micMuted) {
    assistantStopRequested = true;
    captureRequested = false;
    localWakeTriggerPending = false;
    pauseLocalWakeEngine();
    setState(AssistantState::Idle, "Mic muted");
    uiSetNotesStatus("Mic muted");
  } else {
    assistantStopRequested = false;
    resumeLocalWakeEngine();
    setState(AssistantState::Idle, "Mic unmuted");
    uiSetNotesStatus("Ready for dictation");
  }

  uiUpdateMicMuteButton();
  uiUpdateTalkButtonLabel();
  uiUpdateDictateButtonLabel();
  uiRefreshPerformanceMetrics();
}

void uiDictateButtonEventCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }
  if (state == AssistantState::Idle) {
    if (uiTabview) {
      lv_tabview_set_act(uiTabview, TAB_INDEX_NOTES, LV_ANIM_OFF);
    }
    uiSetNotesStatus("Dictating... tap Stop");
    requestCapture(CaptureMode::Dictation, true);
    return;
  }
  assistantStopRequested = true;
  captureRequested = false;
  if (activeCaptureMode == CaptureMode::Dictation) {
    uiSetNotesStatus("Stopping dictation...");
  }
  setState(state, "Stopping...");
}

void uiClearNotesButtonEventCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }
  notesText = "";
  if (uiTaNotes) {
    lv_textarea_set_text(uiTaNotes, "");
  }
  uiSetNotesStatus("Notes cleared");
}

void uiOpenNotesButtonEventCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }
  if (uiTabview) {
    lv_tabview_set_act(uiTabview, TAB_INDEX_NOTES, LV_ANIM_OFF);
  }
}

void uiOpenSettingsButtonEventCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }
  if (uiTabview) {
    lv_tabview_set_act(uiTabview, TAB_INDEX_SETTINGS, LV_ANIM_OFF);
  }
}

void uiOpenWifiMenuButtonEventCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }
  if (uiTabview) {
    lv_tabview_set_act(uiTabview, TAB_INDEX_WIFI, LV_ANIM_OFF);
  }
}

void uiOpenPerformanceButtonEventCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }
  if (uiTabview) {
    lv_tabview_set_act(uiTabview, TAB_INDEX_PERFORMANCE, LV_ANIM_OFF);
  }
  uiRefreshPerformanceMetrics();
}

void uiSettingsSyncButtonEventCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }
  cloudSyncRequested = true;
  uiSetCalendarStatus("Sync requested...");
  uiSetSyncTasksStatus("Sync requested...");
}

void uiCalibrateMicButtonEventCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }
  if (micMuted) {
    setState(AssistantState::Idle, "Unmute mic to calibrate");
    return;
  }

  setState(AssistantState::Thinking, "Calibrating mic floor...");
  uiRefreshNow();
  String summary;
  if (calibrateVoiceNoiseFloor(summary)) {
    setState(AssistantState::Idle, String("Calibration done: ") + summary);
  } else {
    setState(AssistantState::Idle, String("Calibration failed: ") + summary);
  }
  uiRefreshPerformanceMetrics();
}

void uiOpenPocketPetButtonEventCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }
  if (uiTabview) {
    lv_tabview_set_act(uiTabview, TAB_INDEX_POCKET_PET, LV_ANIM_OFF);
  }
}

void uiOpenCalendarButtonEventCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }
  if (uiTabview) {
    lv_tabview_set_act(uiTabview, TAB_INDEX_CALENDAR, LV_ANIM_OFF);
  }
}

void uiOpenSyncTasksButtonEventCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }
  if (uiTabview) {
    lv_tabview_set_act(uiTabview, TAB_INDEX_SYNC_TASKS, LV_ANIM_OFF);
  }
}

void uiCloudSyncButtonEventCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }
  cloudSyncRequested = true;
  uiSetCalendarStatus("Sync requested...");
  uiSetSyncTasksStatus("Sync requested...");
}

void uiNavHomeButtonEventCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED || !uiTabview) {
    return;
  }
  navIgnoreTabChange = true;
  lv_tabview_set_act(uiTabview, TAB_INDEX_HOME, LV_ANIM_OFF);
  navCurrentTab = TAB_INDEX_HOME;
  navIgnoreTabChange = false;
}

void uiNavBackButtonEventCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED || !uiTabview) {
    return;
  }
  if (navHistoryCount > 0) {
    const uint16_t target = navHistory[--navHistoryCount];
    navIgnoreTabChange = true;
    lv_tabview_set_act(uiTabview, target, LV_ANIM_OFF);
    navCurrentTab = target;
    navIgnoreTabChange = false;
    return;
  }

  if (navCurrentTab != TAB_INDEX_HOME) {
    navIgnoreTabChange = true;
    lv_tabview_set_act(uiTabview, TAB_INDEX_HOME, LV_ANIM_OFF);
    navCurrentTab = TAB_INDEX_HOME;
    navIgnoreTabChange = false;
  }
}

void uiTabviewChangedEventCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED || !uiTabview) {
    return;
  }
  const uint16_t nextTab = lv_tabview_get_tab_act(uiTabview);
  if (nextTab == navCurrentTab) {
    return;
  }
  if (!navIgnoreTabChange) {
    if (navHistoryCount < NAV_HISTORY_DEPTH) {
      navHistory[navHistoryCount++] = navCurrentTab;
    } else {
      for (size_t i = 1; i < NAV_HISTORY_DEPTH; ++i) {
        navHistory[i - 1] = navHistory[i];
      }
      navHistory[NAV_HISTORY_DEPTH - 1] = navCurrentTab;
    }
  }
  navCurrentTab = nextTab;
}

void uiSyncedTaskToggleEventCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }

  const uintptr_t rawIndex = reinterpret_cast<uintptr_t>(lv_event_get_user_data(e));
  if (rawIndex == 0) {
    return;
  }
  const size_t idx = static_cast<size_t>(rawIndex - 1);
  if (idx >= syncedTaskCount) {
    return;
  }

  SyncedTaskItem& task = syncedTasks[idx];
  task.completed = !task.completed;
  const bool nowCompleted = task.completed;
  SyncedTaskItem updatedTask = task;
  sortSyncedTasks();
  uiRefreshSyncTasksList();

  String postErr;
  bool cloudUpdated = false;
  const bool hasHaTodo = activeTodoEntity().startsWith("todo.") && isHomeAssistantConfigured();
  if (WiFi.status() == WL_CONNECTED) {
    if (hasHaTodo) {
      cloudUpdated = updateTodoItemInHomeAssistant(updatedTask, nowCompleted, postErr);
    } else if (strlen(CLOUD_TASK_COMPLETE_WEBHOOK_URL) > 0) {
      cloudUpdated = postTaskCompletionWebhook(updatedTask, nowCompleted, postErr);
    }
  }

  String cacheErr;
  saveCloudCache(cacheErr);

  if (cloudUpdated) {
    uiSetSyncTasksStatus("Task updated in cloud");
  } else if (hasHaTodo || strlen(CLOUD_TASK_COMPLETE_WEBHOOK_URL) > 0) {
    uiSetSyncTasksStatus("Local update only: " + clipText(postErr, 36));
  } else {
    uiSetSyncTasksStatus("Saved locally (set HA to-do source to sync)");
  }
}

void uiPocketPetExitButtonEventCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }
  if (uiTabview) {
    lv_tabview_set_act(uiTabview, TAB_INDEX_HOME, LV_ANIM_OFF);
  }
}

void uiPocketPetFeedButtonEventCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }
  if (!pocketPet.isAlive) {
    pocketPetSetBanner("Pet is not alive", 2200);
    uiUpdatePocketPet();
    return;
  }
  const bool wasVeryHungry = pocketPet.hunger < 20;
  pocketPet.hunger = clampPercentInt(static_cast<int>(pocketPet.hunger) + 20);
  pocketPet.energy = clampPercentInt(static_cast<int>(pocketPet.energy) - 5);
  if (wasVeryHungry) {
    pocketPet.health = clampPercentInt(static_cast<int>(pocketPet.health) + 5);
  }
  pocketPet.mode = PocketPetMode::Eating;
  pocketPet.stateChangeMs = millis();
  pocketPet.lastInteractionMs = pocketPet.stateChangeMs;
  pocketPetSetBanner("Fed your pet", 2200);
  savePocketPetState();
  uiUpdatePocketPet();
}

void uiPocketPetPlayButtonEventCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }
  if (!pocketPet.isAlive) {
    pocketPetSetBanner("Pet is not alive", 2200);
    uiUpdatePocketPet();
    return;
  }
  if (pocketPet.energy < 10) {
    pocketPetSetBanner("Too tired to play", 2200);
    uiUpdatePocketPet();
    return;
  }
  pocketPet.happiness = clampPercentInt(static_cast<int>(pocketPet.happiness) + 15);
  pocketPet.energy = clampPercentInt(static_cast<int>(pocketPet.energy) - 15);
  pocketPet.hunger = clampPercentInt(static_cast<int>(pocketPet.hunger) - 10);
  pocketPet.mode = PocketPetMode::Playing;
  pocketPet.stateChangeMs = millis();
  pocketPet.lastInteractionMs = pocketPet.stateChangeMs;
  pocketPetSetBanner("Play time", 2200);
  savePocketPetState();
  uiUpdatePocketPet();
}

void uiPocketPetCleanButtonEventCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }
  if (!pocketPet.isAlive) {
    pocketPetSetBanner("Pet is not alive", 2200);
    uiUpdatePocketPet();
    return;
  }
  pocketPet.cleanliness = 100;
  pocketPet.health = clampPercentInt(static_cast<int>(pocketPet.health) + 5);
  pocketPet.lastInteractionMs = millis();
  pocketPetSetBanner("Fresh and clean", 2200);
  savePocketPetState();
  uiUpdatePocketPet();
}

void uiPocketPetSleepButtonEventCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }
  if (!pocketPet.isAlive) {
    pocketPetSetBanner("Pet is not alive", 2200);
    uiUpdatePocketPet();
    return;
  }
  const bool sleeping = pocketPet.mode == PocketPetMode::Sleeping;
  if (sleeping) {
    pocketPet.mode = PocketPetMode::Normal;
    pocketPetSetBanner("Pet is awake", 2200);
  } else {
    pocketPet.mode = PocketPetMode::Sleeping;
    pocketPet.stateChangeMs = millis();
    pocketPetSetBanner("Pet is sleeping", 2200);
  }
  pocketPet.lastInteractionMs = millis();
  savePocketPetState();
  uiUpdatePocketPet();
}

void uiPocketPetHealButtonEventCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }
  if (!pocketPet.isAlive) {
    pocketPetSetBanner("Pet is not alive", 2200);
    uiUpdatePocketPet();
    return;
  }
  pocketPet.health = 100;
  pocketPet.mode = PocketPetMode::Normal;
  pocketPet.stateChangeMs = millis();
  pocketPet.lastInteractionMs = pocketPet.stateChangeMs;
  pocketPetSetBanner("Health restored", 2200);
  savePocketPetState();
  uiUpdatePocketPet();
}

void uiPocketPetResetButtonEventCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }
  pocketPet.hunger = 50;
  pocketPet.happiness = 50;
  pocketPet.health = 80;
  pocketPet.energy = 100;
  pocketPet.cleanliness = 80;
  pocketPet.ageMinutes = 0;
  pocketPet.isAlive = true;
  pocketPet.mode = PocketPetMode::Normal;
  pocketPet.stateChangeMs = millis();
  pocketPet.lastInteractionMs = pocketPet.stateChangeMs;
  pocketPet.lastTickMs = millis();
  pocketPetSetBanner("New pet adopted", 2600);
  savePocketPetState();
  uiUpdatePocketPet();
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

void uiTodoEntityDropdownEventCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED || !uiDdTodoEntity) {
    return;
  }

  char selected[96] = {0};
  lv_dropdown_get_selected_str(uiDdTodoEntity, selected, sizeof(selected));
  String entity = String(selected);
  entity.trim();
  if (!entity.startsWith("todo.")) {
    return;
  }

  saveTodoEntityConfig(entity);
  uiSetSyncTasksStatus(String("Selected list: ") + clipText(entity, 30));
  cloudSyncRequested = true;
}

void uiTodoEntityRefreshButtonEventCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }
  todoEntityRefreshRequested = true;
  uiSetSyncTasksStatus("Refreshing to-do lists...");
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
  lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_text_color(scr, lv_color_hex(0xE5E7EB), LV_PART_MAIN);

  uiTabview = lv_tabview_create(scr, LV_DIR_TOP, 0);
  lv_obj_set_size(uiTabview, SCREEN_WIDTH, SCREEN_HEIGHT);
  lv_obj_set_style_bg_color(uiTabview, lv_color_hex(0x000000), LV_PART_MAIN);
  lv_obj_set_style_border_width(uiTabview, 0, LV_PART_MAIN);
  lv_obj_clear_flag(uiTabview, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_t* tabContent = lv_tabview_get_content(uiTabview);
  if (tabContent) {
    lv_obj_clear_flag(tabContent, LV_OBJ_FLAG_SCROLLABLE);
  }
  lv_obj_add_event_cb(uiTabview, uiTabviewChangedEventCb, LV_EVENT_VALUE_CHANGED, nullptr);
  navHistoryCount = 0;
  navCurrentTab = TAB_INDEX_HOME;
  navIgnoreTabChange = false;

  lv_obj_t* tabBtns = lv_tabview_get_tab_btns(uiTabview);
  lv_obj_add_flag(tabBtns, LV_OBJ_FLAG_HIDDEN);
  lv_obj_set_size(tabBtns, 1, 1);

  lv_obj_t* tabMain = lv_tabview_add_tab(uiTabview, "Assistant");
  lv_obj_t* tabSettings = lv_tabview_add_tab(uiTabview, "Settings");
  lv_obj_t* tabWifi = lv_tabview_add_tab(uiTabview, "Wi-Fi");
  lv_obj_t* tabPerformance = lv_tabview_add_tab(uiTabview, "Performance");
  lv_obj_t* tabNotes = lv_tabview_add_tab(uiTabview, "Notes");
  lv_obj_t* tabPocketPet = lv_tabview_add_tab(uiTabview, "Pocket Pet");
  lv_obj_t* tabCalendar = lv_tabview_add_tab(uiTabview, "Calendar");
  lv_obj_t* tabSyncTasks = lv_tabview_add_tab(uiTabview, "Sync Tasks");

  lv_obj_set_scrollbar_mode(tabMain, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_scrollbar_mode(tabSettings, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_scrollbar_mode(tabWifi, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_scrollbar_mode(tabPerformance, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_scrollbar_mode(tabNotes, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_scrollbar_mode(tabPocketPet, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_scrollbar_mode(tabCalendar, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_scrollbar_mode(tabSyncTasks, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_style_pad_all(tabMain, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(tabSettings, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(tabWifi, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(tabPerformance, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(tabNotes, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(tabPocketPet, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(tabCalendar, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(tabSyncTasks, 0, LV_PART_MAIN);
  lv_obj_set_style_bg_color(tabMain, lv_color_hex(0x000000), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(tabMain, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_bg_color(tabSettings, lv_color_hex(0x000000), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(tabSettings, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_bg_color(tabWifi, lv_color_hex(0x000000), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(tabWifi, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_bg_color(tabPerformance, lv_color_hex(0x000000), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(tabPerformance, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_bg_color(tabNotes, lv_color_hex(0x000000), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(tabNotes, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_bg_color(tabPocketPet, lv_color_hex(0x000000), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(tabPocketPet, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_bg_color(tabCalendar, lv_color_hex(0x000000), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(tabCalendar, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_bg_color(tabSyncTasks, lv_color_hex(0x000000), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(tabSyncTasks, LV_OPA_COVER, LV_PART_MAIN);

  auto addStandardNav = [&](lv_obj_t* tab, const char* title) {
    (void)title;
    lv_obj_t* nav = lv_obj_create(tab);
    lv_obj_set_size(nav, SCREEN_WIDTH, NAV_BAR_HEIGHT);
    lv_obj_align(nav, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(nav, lv_color_hex(0x0B1220), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(nav, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(nav, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(nav, lv_color_hex(0x1E293B), LV_PART_MAIN);
    lv_obj_set_style_radius(nav, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_top(nav, 4, LV_PART_MAIN);
    lv_obj_set_style_pad_bottom(nav, 4, LV_PART_MAIN);
    lv_obj_set_style_pad_left(nav, 4, LV_PART_MAIN);
    lv_obj_set_style_pad_right(nav, 4, LV_PART_MAIN);
    lv_obj_clear_flag(nav, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* btnBack = lv_btn_create(nav);
    lv_obj_set_size(btnBack, 104, 34);
    lv_obj_align(btnBack, LV_ALIGN_LEFT_MID, 4, 0);
    lv_obj_set_style_radius(btnBack, 8, LV_PART_MAIN);
    lv_obj_set_style_bg_color(btnBack, lv_color_hex(0x1E293B), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(btnBack, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(btnBack, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(btnBack, lv_color_hex(0x334155), LV_PART_MAIN);
    lv_obj_add_event_cb(btnBack, uiNavBackButtonEventCb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t* backLabel = lv_label_create(btnBack);
    lv_label_set_text(backLabel, LV_SYMBOL_LEFT " Back");
    lv_obj_center(backLabel);

    lv_obj_t* btnHome = lv_btn_create(nav);
    lv_obj_set_size(btnHome, 104, 34);
    lv_obj_align(btnHome, LV_ALIGN_RIGHT_MID, -4, 0);
    lv_obj_set_style_radius(btnHome, 8, LV_PART_MAIN);
    lv_obj_set_style_bg_color(btnHome, lv_color_hex(0x0F766E), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(btnHome, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(btnHome, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(btnHome, lv_color_hex(0x2DD4BF), LV_PART_MAIN);
    lv_obj_add_event_cb(btnHome, uiNavHomeButtonEventCb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t* homeLabel = lv_label_create(btnHome);
    lv_label_set_text(homeLabel, LV_SYMBOL_HOME " Home");
    lv_obj_center(homeLabel);
  };

  addStandardNav(tabMain, "Home");
  addStandardNav(tabSettings, "Settings");
  addStandardNav(tabWifi, "Wi-Fi");
  addStandardNav(tabPerformance, "Performance");
  addStandardNav(tabNotes, "Notes");
  addStandardNav(tabPocketPet, "Pocket Pet");
  addStandardNav(tabCalendar, "Calendar");
  addStandardNav(tabSyncTasks, "To-Dos");

  lv_obj_t* statusBar = lv_obj_create(tabMain);
  lv_obj_set_size(statusBar, SCREEN_WIDTH - 12, 56);
  lv_obj_align(statusBar, LV_ALIGN_TOP_MID, 0, 6);
  lv_obj_set_style_bg_color(statusBar, lv_color_hex(0x0E0E0E), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(statusBar, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_width(statusBar, 1, LV_PART_MAIN);
  lv_obj_set_style_border_color(statusBar, lv_color_hex(0x2F2F2F), LV_PART_MAIN);
  lv_obj_set_style_radius(statusBar, 8, LV_PART_MAIN);
  lv_obj_set_style_pad_all(statusBar, 6, LV_PART_MAIN);
  lv_obj_set_scrollbar_mode(statusBar, LV_SCROLLBAR_MODE_OFF);
  lv_obj_clear_flag(statusBar, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t* statusTop = lv_obj_create(statusBar);
  lv_obj_set_size(statusTop, SCREEN_WIDTH - 28, 18);
  lv_obj_align(statusTop, LV_ALIGN_TOP_LEFT, 0, 0);
  lv_obj_set_style_bg_opa(statusTop, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_border_width(statusTop, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(statusTop, 0, LV_PART_MAIN);
  lv_obj_set_style_radius(statusTop, 0, LV_PART_MAIN);
  lv_obj_set_scrollbar_mode(statusTop, LV_SCROLLBAR_MODE_OFF);
  lv_obj_clear_flag(statusTop, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_layout(statusTop, LV_LAYOUT_FLEX);
  lv_obj_set_flex_flow(statusTop, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(statusTop, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

  uiLabelState = lv_label_create(statusTop);
  lv_obj_set_width(uiLabelState, SCREEN_WIDTH - 130);
  lv_label_set_long_mode(uiLabelState, LV_LABEL_LONG_DOT);
  lv_obj_set_style_text_color(uiLabelState, lv_color_hex(0x34D399), LV_PART_MAIN);
  lv_label_set_text(uiLabelState, "Ready");

  lv_obj_t* statusIcons = lv_obj_create(statusTop);
  lv_obj_set_size(statusIcons, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
  lv_obj_set_style_bg_opa(statusIcons, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_border_width(statusIcons, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(statusIcons, 0, LV_PART_MAIN);
  lv_obj_set_style_radius(statusIcons, 0, LV_PART_MAIN);
  lv_obj_set_scrollbar_mode(statusIcons, LV_SCROLLBAR_MODE_OFF);
  lv_obj_clear_flag(statusIcons, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_layout(statusIcons, LV_LAYOUT_FLEX);
  lv_obj_set_flex_flow(statusIcons, LV_FLEX_FLOW_ROW);
  lv_obj_set_style_pad_column(statusIcons, 6, LV_PART_MAIN);

  uiIconState = lv_label_create(statusIcons);
  lv_label_set_text(uiIconState, LV_SYMBOL_PLAY);
  lv_obj_set_style_text_color(uiIconState, lv_color_hex(0x9CA3AF), LV_PART_MAIN);

  uiIconWifi = lv_label_create(statusIcons);
  lv_label_set_text(uiIconWifi, LV_SYMBOL_WIFI);
  lv_obj_set_style_text_color(uiIconWifi, lv_color_hex(0x9CA3AF), LV_PART_MAIN);

  uiIconSd = lv_label_create(statusIcons);
  lv_label_set_text(uiIconSd, LV_SYMBOL_SD_CARD);
  lv_obj_set_style_text_color(uiIconSd, lv_color_hex(0x9CA3AF), LV_PART_MAIN);

  uiIconMic = lv_label_create(statusIcons);
  lv_label_set_text(uiIconMic, LV_SYMBOL_AUDIO);
  lv_obj_set_style_text_color(uiIconMic, lv_color_hex(0x9CA3AF), LV_PART_MAIN);

  uiLabelDetail = lv_label_create(statusBar);
  lv_obj_set_width(uiLabelDetail, SCREEN_WIDTH - 28);
  lv_obj_align(uiLabelDetail, LV_ALIGN_TOP_LEFT, 0, 22);
  lv_label_set_long_mode(uiLabelDetail, LV_LABEL_LONG_DOT);
  lv_obj_set_style_text_color(uiLabelDetail, lv_color_hex(0x9CA3AF), LV_PART_MAIN);
  lv_label_set_text(uiLabelDetail, "Initializing...");

  lv_obj_t* responsePanel = lv_obj_create(tabMain);
  lv_obj_set_size(responsePanel, SCREEN_WIDTH - 12, 34);
  lv_obj_align(responsePanel, LV_ALIGN_TOP_MID, 0, 66);
  lv_obj_set_style_bg_color(responsePanel, lv_color_hex(0x0F172A), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(responsePanel, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_width(responsePanel, 1, LV_PART_MAIN);
  lv_obj_set_style_border_color(responsePanel, lv_color_hex(0x263449), LV_PART_MAIN);
  lv_obj_set_style_radius(responsePanel, 8, LV_PART_MAIN);
  lv_obj_set_style_pad_left(responsePanel, 6, LV_PART_MAIN);
  lv_obj_set_style_pad_right(responsePanel, 6, LV_PART_MAIN);
  lv_obj_set_style_pad_top(responsePanel, 4, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(responsePanel, 4, LV_PART_MAIN);
  lv_obj_set_scrollbar_mode(responsePanel, LV_SCROLLBAR_MODE_OFF);
  lv_obj_clear_flag(responsePanel, LV_OBJ_FLAG_SCROLLABLE);

  uiLabelResponse = lv_label_create(responsePanel);
  lv_obj_set_width(uiLabelResponse, SCREEN_WIDTH - 30);
  lv_label_set_long_mode(uiLabelResponse, LV_LABEL_LONG_SCROLL_CIRCULAR);
  lv_label_set_text(uiLabelResponse, "Response: waiting...");
  lv_obj_set_style_text_color(uiLabelResponse, lv_color_hex(0xBFDBFE), LV_PART_MAIN);
  lv_obj_align(uiLabelResponse, LV_ALIGN_LEFT_MID, 0, 0);

  lv_obj_t* grid = lv_obj_create(tabMain);
  lv_obj_set_size(grid, SCREEN_WIDTH - 16, SCREEN_HEIGHT - 164);
  lv_obj_align(grid, LV_ALIGN_TOP_MID, 0, 106);
  lv_obj_set_style_bg_color(grid, lv_color_hex(0x0B0B0B), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(grid, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_width(grid, 1, LV_PART_MAIN);
  lv_obj_set_style_border_color(grid, lv_color_hex(0x2A2A2A), LV_PART_MAIN);
  lv_obj_set_style_radius(grid, 8, LV_PART_MAIN);
  lv_obj_set_style_pad_all(grid, 8, LV_PART_MAIN);
  lv_obj_set_style_pad_row(grid, 6, LV_PART_MAIN);
  lv_obj_set_style_pad_column(grid, 6, LV_PART_MAIN);
  lv_obj_set_scrollbar_mode(grid, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_layout(grid, LV_LAYOUT_GRID);
  static lv_coord_t launcherCols[] = {LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
  static lv_coord_t launcherRows[] = {LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
  lv_obj_set_grid_dsc_array(grid, launcherCols, launcherRows);

  auto styleLauncherTile = [](lv_obj_t* button, uint32_t borderColor) {
    lv_obj_set_style_radius(button, 8, LV_PART_MAIN);
    lv_obj_set_style_bg_color(button, lv_color_hex(0x161616), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(button, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(button, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(button, lv_color_hex(borderColor), LV_PART_MAIN);
    lv_obj_set_style_shadow_width(button, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(button, 4, LV_PART_MAIN);
  };
  auto styleLauncherLabel = [](lv_obj_t* label, uint32_t textColor) {
    lv_obj_set_width(label, LV_PCT(100));
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_color(label, lv_color_hex(textColor), LV_PART_MAIN);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_center(label);
  };

  uiBtnTalk = lv_btn_create(grid);
  lv_obj_set_grid_cell(uiBtnTalk, LV_GRID_ALIGN_STRETCH, 0, 1, LV_GRID_ALIGN_STRETCH, 0, 1);
  styleLauncherTile(uiBtnTalk, 0x00FF88);
  lv_obj_add_event_cb(uiBtnTalk, uiTalkButtonEventCb, LV_EVENT_CLICKED, nullptr);
  uiLabelTalk = lv_label_create(uiBtnTalk);
  lv_label_set_text(uiLabelTalk, LV_SYMBOL_AUDIO " Listen");
  styleLauncherLabel(uiLabelTalk, 0x00FF88);

  uiBtnDictate = lv_btn_create(grid);
  lv_obj_set_grid_cell(uiBtnDictate, LV_GRID_ALIGN_STRETCH, 1, 1, LV_GRID_ALIGN_STRETCH, 0, 1);
  styleLauncherTile(uiBtnDictate, 0x60A5FA);
  lv_obj_add_event_cb(uiBtnDictate, uiDictateButtonEventCb, LV_EVENT_CLICKED, nullptr);
  uiLabelDictate = lv_label_create(uiBtnDictate);
  lv_label_set_text(uiLabelDictate, LV_SYMBOL_EDIT " Dictate");
  styleLauncherLabel(uiLabelDictate, 0x60A5FA);

  uiBtnOpenCalendar = lv_btn_create(grid);
  lv_obj_set_grid_cell(uiBtnOpenCalendar, LV_GRID_ALIGN_STRETCH, 2, 1, LV_GRID_ALIGN_STRETCH, 0, 1);
  styleLauncherTile(uiBtnOpenCalendar, 0xF59E0B);
  lv_obj_add_event_cb(uiBtnOpenCalendar, uiOpenCalendarButtonEventCb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* uiLabelOpenCalendar = lv_label_create(uiBtnOpenCalendar);
  lv_label_set_text(uiLabelOpenCalendar, LV_SYMBOL_BELL " Calendar");
  styleLauncherLabel(uiLabelOpenCalendar, 0xFDE68A);

  uiBtnOpenSyncTasks = lv_btn_create(grid);
  lv_obj_set_grid_cell(uiBtnOpenSyncTasks, LV_GRID_ALIGN_STRETCH, 0, 1, LV_GRID_ALIGN_STRETCH, 1, 1);
  styleLauncherTile(uiBtnOpenSyncTasks, 0x38BDF8);
  lv_obj_add_event_cb(uiBtnOpenSyncTasks, uiOpenSyncTasksButtonEventCb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* uiLabelOpenSyncTasks = lv_label_create(uiBtnOpenSyncTasks);
  lv_label_set_text(uiLabelOpenSyncTasks, LV_SYMBOL_OK " To-Dos");
  styleLauncherLabel(uiLabelOpenSyncTasks, 0x7DD3FC);

  uiBtnOpenPet = lv_btn_create(grid);
  lv_obj_set_grid_cell(uiBtnOpenPet, LV_GRID_ALIGN_STRETCH, 1, 1, LV_GRID_ALIGN_STRETCH, 1, 1);
  styleLauncherTile(uiBtnOpenPet, 0x6EE7B7);
  lv_obj_add_event_cb(uiBtnOpenPet, uiOpenPocketPetButtonEventCb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* uiLabelOpenPet = lv_label_create(uiBtnOpenPet);
  lv_label_set_text(uiLabelOpenPet, LV_SYMBOL_HOME " Pet");
  styleLauncherLabel(uiLabelOpenPet, 0x6EE7B7);

  uiBtnMicMute = lv_btn_create(grid);
  lv_obj_set_grid_cell(uiBtnMicMute, LV_GRID_ALIGN_STRETCH, 2, 1, LV_GRID_ALIGN_STRETCH, 1, 1);
  styleLauncherTile(uiBtnMicMute, 0x22C55E);
  lv_obj_add_event_cb(uiBtnMicMute, uiMicMuteToggleButtonEventCb, LV_EVENT_CLICKED, nullptr);
  uiLabelMicMute = lv_label_create(uiBtnMicMute);
  lv_label_set_text(uiLabelMicMute, LV_SYMBOL_AUDIO " Mic On");
  styleLauncherLabel(uiLabelMicMute, 0x22C55E);

  uiBtnOpenNotes = lv_btn_create(grid);
  lv_obj_set_grid_cell(uiBtnOpenNotes, LV_GRID_ALIGN_STRETCH, 0, 1, LV_GRID_ALIGN_STRETCH, 2, 1);
  styleLauncherTile(uiBtnOpenNotes, 0xF9A8D4);
  lv_obj_add_event_cb(uiBtnOpenNotes, uiOpenNotesButtonEventCb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* uiLabelOpenNotes = lv_label_create(uiBtnOpenNotes);
  lv_label_set_text(uiLabelOpenNotes, LV_SYMBOL_FILE " Notes");
  styleLauncherLabel(uiLabelOpenNotes, 0xF9A8D4);

  uiBtnOpenSettings = lv_btn_create(grid);
  lv_obj_set_grid_cell(uiBtnOpenSettings, LV_GRID_ALIGN_STRETCH, 1, 1, LV_GRID_ALIGN_STRETCH, 2, 1);
  styleLauncherTile(uiBtnOpenSettings, 0xFBBF24);
  lv_obj_add_event_cb(uiBtnOpenSettings, uiOpenSettingsButtonEventCb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* uiLabelOpenSettings = lv_label_create(uiBtnOpenSettings);
  lv_label_set_text(uiLabelOpenSettings, LV_SYMBOL_SETTINGS " Settings");
  styleLauncherLabel(uiLabelOpenSettings, 0xFBBF24);

  lv_obj_t* settingsTitle = lv_label_create(tabSettings);
  lv_obj_align(settingsTitle, LV_ALIGN_TOP_LEFT, 8, 36);
  lv_label_set_text(settingsTitle, "Settings");
  lv_obj_set_style_text_color(settingsTitle, lv_color_hex(0xFDE68A), LV_PART_MAIN);

  lv_obj_t* settingsHint = lv_label_create(tabSettings);
  lv_obj_set_width(settingsHint, SCREEN_WIDTH - 16);
  lv_obj_align(settingsHint, LV_ALIGN_TOP_LEFT, 8, 56);
  lv_label_set_long_mode(settingsHint, LV_LABEL_LONG_WRAP);
  lv_label_set_text(settingsHint, "Manage Wi-Fi, sync, performance, and mic calibration.");
  lv_obj_set_style_text_color(settingsHint, lv_color_hex(0x9FB3CF), LV_PART_MAIN);

  lv_obj_t* settingsWifiBtn = lv_btn_create(tabSettings);
  lv_obj_set_size(settingsWifiBtn, SCREEN_WIDTH - 16, 40);
  lv_obj_align(settingsWifiBtn, LV_ALIGN_TOP_LEFT, 8, 88);
  lv_obj_set_style_radius(settingsWifiBtn, 10, LV_PART_MAIN);
  lv_obj_set_style_bg_color(settingsWifiBtn, lv_color_hex(0x1D4ED8), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(settingsWifiBtn, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_add_event_cb(settingsWifiBtn, uiOpenWifiMenuButtonEventCb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* settingsWifiLabel = lv_label_create(settingsWifiBtn);
  lv_label_set_text(settingsWifiLabel, LV_SYMBOL_WIFI "  Wi-Fi");
  lv_obj_center(settingsWifiLabel);

  lv_obj_t* settingsPerfBtn = lv_btn_create(tabSettings);
  lv_obj_set_size(settingsPerfBtn, SCREEN_WIDTH - 16, 40);
  lv_obj_align(settingsPerfBtn, LV_ALIGN_TOP_LEFT, 8, 132);
  lv_obj_set_style_radius(settingsPerfBtn, 10, LV_PART_MAIN);
  lv_obj_set_style_bg_color(settingsPerfBtn, lv_color_hex(0x7C3AED), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(settingsPerfBtn, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_add_event_cb(settingsPerfBtn, uiOpenPerformanceButtonEventCb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* settingsPerfLabel = lv_label_create(settingsPerfBtn);
  lv_label_set_text(settingsPerfLabel, LV_SYMBOL_DRIVE "  Performance");
  lv_obj_center(settingsPerfLabel);

  lv_obj_t* settingsSyncBtn = lv_btn_create(tabSettings);
  lv_obj_set_size(settingsSyncBtn, SCREEN_WIDTH - 16, 40);
  lv_obj_align(settingsSyncBtn, LV_ALIGN_TOP_LEFT, 8, 176);
  lv_obj_set_style_radius(settingsSyncBtn, 10, LV_PART_MAIN);
  lv_obj_set_style_bg_color(settingsSyncBtn, lv_color_hex(0x15803D), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(settingsSyncBtn, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_add_event_cb(settingsSyncBtn, uiSettingsSyncButtonEventCb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* settingsSyncLabel = lv_label_create(settingsSyncBtn);
  lv_label_set_text(settingsSyncLabel, LV_SYMBOL_REFRESH "  Sync Now");
  lv_obj_center(settingsSyncLabel);

  lv_obj_t* settingsCalBtn = lv_btn_create(tabSettings);
  lv_obj_set_size(settingsCalBtn, SCREEN_WIDTH - 16, 40);
  lv_obj_align(settingsCalBtn, LV_ALIGN_TOP_LEFT, 8, 220);
  lv_obj_set_style_radius(settingsCalBtn, 10, LV_PART_MAIN);
  lv_obj_set_style_bg_color(settingsCalBtn, lv_color_hex(0x0F766E), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(settingsCalBtn, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_add_event_cb(settingsCalBtn, uiCalibrateMicButtonEventCb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* settingsCalLabel = lv_label_create(settingsCalBtn);
  lv_label_set_text(settingsCalLabel, LV_SYMBOL_AUDIO "  Calibrate Mic");
  lv_obj_center(settingsCalLabel);

  lv_obj_t* labelSsid = lv_label_create(tabWifi);
  lv_obj_align(labelSsid, LV_ALIGN_TOP_LEFT, 8, 36);
  lv_label_set_text(labelSsid, "Wi-Fi Networks");

  uiDdSsid = lv_dropdown_create(tabWifi);
  lv_obj_set_width(uiDdSsid, 206);
  lv_obj_align(uiDdSsid, LV_ALIGN_TOP_LEFT, 8, 58);
  lv_obj_set_style_radius(uiDdSsid, 10, LV_PART_MAIN);
  lv_obj_set_style_bg_color(uiDdSsid, lv_color_hex(0x10233C), LV_PART_MAIN);
  lv_obj_set_style_text_color(uiDdSsid, lv_color_hex(0xEAF2FF), LV_PART_MAIN);
  lv_dropdown_set_options(uiDdSsid, "No scan yet");
  lv_obj_add_event_cb(uiDdSsid, uiWifiDropdownEventCb, LV_EVENT_VALUE_CHANGED, nullptr);

  lv_obj_t* btnScan = lv_btn_create(tabWifi);
  lv_obj_set_size(btnScan, 94, 36);
  lv_obj_align(btnScan, LV_ALIGN_TOP_RIGHT, -8, 58);
  lv_obj_set_style_radius(btnScan, 12, LV_PART_MAIN);
  lv_obj_set_style_bg_color(btnScan, lv_color_hex(0x2563EB), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(btnScan, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_add_event_cb(btnScan, uiWifiScanButtonEventCb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* btnScanLabel = lv_label_create(btnScan);
  lv_label_set_text(btnScanLabel, "Scan");
  lv_obj_center(btnScanLabel);

  lv_obj_t* labelManual = lv_label_create(tabWifi);
  lv_obj_align(labelManual, LV_ALIGN_TOP_LEFT, 8, 102);
  lv_label_set_text(labelManual, "SSID");

  uiTaSsid = lv_textarea_create(tabWifi);
  lv_obj_set_width(uiTaSsid, SCREEN_WIDTH - 16);
  lv_obj_align(uiTaSsid, LV_ALIGN_TOP_LEFT, 8, 122);
  lv_textarea_set_one_line(uiTaSsid, true);
  lv_textarea_set_max_length(uiTaSsid, 63);
  lv_textarea_set_placeholder_text(uiTaSsid, "Enter SSID");
  lv_obj_set_style_radius(uiTaSsid, 10, LV_PART_MAIN);
  lv_obj_set_style_bg_color(uiTaSsid, lv_color_hex(0x10233C), LV_PART_MAIN);
  lv_obj_set_style_text_color(uiTaSsid, lv_color_hex(0xEAF2FF), LV_PART_MAIN);
  lv_obj_add_event_cb(uiTaSsid, uiTextareaEventCb, LV_EVENT_FOCUSED, nullptr);

  lv_obj_t* labelPwd = lv_label_create(tabWifi);
  lv_obj_align(labelPwd, LV_ALIGN_TOP_LEFT, 8, 160);
  lv_label_set_text(labelPwd, "Password");

  uiTaPassword = lv_textarea_create(tabWifi);
  lv_obj_set_width(uiTaPassword, SCREEN_WIDTH - 16);
  lv_obj_align(uiTaPassword, LV_ALIGN_TOP_LEFT, 8, 180);
  lv_textarea_set_one_line(uiTaPassword, true);
  lv_textarea_set_max_length(uiTaPassword, 63);
  lv_textarea_set_placeholder_text(uiTaPassword, "Enter password");
  lv_textarea_set_password_mode(uiTaPassword, true);
  lv_obj_set_style_radius(uiTaPassword, 10, LV_PART_MAIN);
  lv_obj_set_style_bg_color(uiTaPassword, lv_color_hex(0x10233C), LV_PART_MAIN);
  lv_obj_set_style_text_color(uiTaPassword, lv_color_hex(0xEAF2FF), LV_PART_MAIN);
  lv_obj_add_event_cb(uiTaPassword, uiTextareaEventCb, LV_EVENT_FOCUSED, nullptr);

  lv_obj_t* btnSave = lv_btn_create(tabWifi);
  lv_obj_set_size(btnSave, 100, 36);
  lv_obj_align(btnSave, LV_ALIGN_BOTTOM_LEFT, 8, -NAV_CLEARANCE);
  lv_obj_set_style_radius(btnSave, 12, LV_PART_MAIN);
  lv_obj_set_style_bg_color(btnSave, lv_color_hex(0x0F766E), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(btnSave, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_add_event_cb(btnSave, uiWifiSaveButtonEventCb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* btnSaveLabel = lv_label_create(btnSave);
  lv_label_set_text(btnSaveLabel, "Save");
  lv_obj_center(btnSaveLabel);

  lv_obj_t* btnConnect = lv_btn_create(tabWifi);
  lv_obj_set_size(btnConnect, 100, 36);
  lv_obj_align(btnConnect, LV_ALIGN_BOTTOM_LEFT, 116, -NAV_CLEARANCE);
  lv_obj_set_style_radius(btnConnect, 12, LV_PART_MAIN);
  lv_obj_set_style_bg_color(btnConnect, lv_color_hex(0x1D4ED8), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(btnConnect, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_add_event_cb(btnConnect, uiWifiConnectButtonEventCb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* btnConnectLabel = lv_label_create(btnConnect);
  lv_label_set_text(btnConnectLabel, "Connect");
  lv_obj_center(btnConnectLabel);

  uiLabelWifiMenuStatus = lv_label_create(tabWifi);
  lv_obj_set_width(uiLabelWifiMenuStatus, 94);
  lv_obj_align(uiLabelWifiMenuStatus, LV_ALIGN_BOTTOM_RIGHT, -8, -(NAV_CLEARANCE + 1));
  lv_label_set_long_mode(uiLabelWifiMenuStatus, LV_LABEL_LONG_WRAP);
  lv_label_set_text(uiLabelWifiMenuStatus, "Status: idle");
  lv_obj_set_style_text_color(uiLabelWifiMenuStatus, lv_color_hex(0x9FB3CF), LV_PART_MAIN);

  lv_obj_t* perfTitle = lv_label_create(tabPerformance);
  lv_obj_align(perfTitle, LV_ALIGN_TOP_LEFT, 8, 36);
  lv_label_set_text(perfTitle, "Performance");
  lv_obj_set_style_text_color(perfTitle, lv_color_hex(0xC4B5FD), LV_PART_MAIN);

  uiTaPerformance = lv_textarea_create(tabPerformance);
  lv_obj_set_size(uiTaPerformance, SCREEN_WIDTH - 16, 188);
  lv_obj_align(uiTaPerformance, LV_ALIGN_TOP_LEFT, 8, 58);
  lv_textarea_set_one_line(uiTaPerformance, false);
  lv_textarea_set_text(uiTaPerformance, "Collecting metrics...");
  lv_obj_set_style_radius(uiTaPerformance, 10, LV_PART_MAIN);
  lv_obj_set_style_bg_color(uiTaPerformance, lv_color_hex(0x0B1727), LV_PART_MAIN);
  lv_obj_set_style_text_color(uiTaPerformance, lv_color_hex(0xEAF2FF), LV_PART_MAIN);
  lv_textarea_set_cursor_click_pos(uiTaPerformance, false);
  lv_obj_clear_flag(uiTaPerformance, LV_OBJ_FLAG_CLICK_FOCUSABLE);
  lv_obj_set_scrollbar_mode(uiTaPerformance, LV_SCROLLBAR_MODE_ACTIVE);

  uiLabelPerfStatus = lv_label_create(tabPerformance);
  lv_obj_set_width(uiLabelPerfStatus, SCREEN_WIDTH - 16);
  lv_obj_align(uiLabelPerfStatus, LV_ALIGN_BOTTOM_LEFT, 8, -(NAV_CLEARANCE - 6));
  lv_label_set_long_mode(uiLabelPerfStatus, LV_LABEL_LONG_DOT);
  lv_label_set_text(uiLabelPerfStatus, "Waiting for first sample");
  lv_obj_set_style_text_color(uiLabelPerfStatus, lv_color_hex(0x9FB3CF), LV_PART_MAIN);

  lv_obj_t* notesTitle = lv_label_create(tabNotes);
  lv_obj_align(notesTitle, LV_ALIGN_TOP_LEFT, 8, 36);
  lv_label_set_text(notesTitle, "Dictation Notes");

  uiTaNotes = lv_textarea_create(tabNotes);
  lv_obj_set_size(uiTaNotes, SCREEN_WIDTH - 16, 146);
  lv_obj_align(uiTaNotes, LV_ALIGN_TOP_LEFT, 8, 56);
  lv_textarea_set_one_line(uiTaNotes, false);
  lv_textarea_set_text(uiTaNotes, "");
  lv_textarea_set_placeholder_text(uiTaNotes, "Dictated notes will appear here...");
  lv_obj_set_style_radius(uiTaNotes, 10, LV_PART_MAIN);
  lv_obj_set_style_bg_color(uiTaNotes, lv_color_hex(0x10233C), LV_PART_MAIN);
  lv_obj_set_style_text_color(uiTaNotes, lv_color_hex(0xEAF2FF), LV_PART_MAIN);
  lv_obj_set_scrollbar_mode(uiTaNotes, LV_SCROLLBAR_MODE_ACTIVE);

  uiBtnNotesDictate = lv_btn_create(tabNotes);
  lv_obj_set_size(uiBtnNotesDictate, 100, 36);
  lv_obj_align(uiBtnNotesDictate, LV_ALIGN_BOTTOM_LEFT, 8, -NAV_CLEARANCE);
  lv_obj_set_style_radius(uiBtnNotesDictate, 12, LV_PART_MAIN);
  lv_obj_set_style_bg_color(uiBtnNotesDictate, lv_color_hex(0x1D4ED8), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(uiBtnNotesDictate, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_add_event_cb(uiBtnNotesDictate, uiDictateButtonEventCb, LV_EVENT_CLICKED, nullptr);
  uiLabelNotesDictate = lv_label_create(uiBtnNotesDictate);
  lv_label_set_text(uiLabelNotesDictate, "Dictate");
  lv_obj_center(uiLabelNotesDictate);

  uiBtnClearNotes = lv_btn_create(tabNotes);
  lv_obj_set_size(uiBtnClearNotes, 100, 36);
  lv_obj_align(uiBtnClearNotes, LV_ALIGN_BOTTOM_LEFT, 116, -NAV_CLEARANCE);
  lv_obj_set_style_radius(uiBtnClearNotes, 12, LV_PART_MAIN);
  lv_obj_set_style_bg_color(uiBtnClearNotes, lv_color_hex(0x475569), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(uiBtnClearNotes, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_add_event_cb(uiBtnClearNotes, uiClearNotesButtonEventCb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* clearNotesLabel = lv_label_create(uiBtnClearNotes);
  lv_label_set_text(clearNotesLabel, "Clear");
  lv_obj_center(clearNotesLabel);

  uiLabelNotesStatus = lv_label_create(tabNotes);
  lv_obj_set_width(uiLabelNotesStatus, 94);
  lv_obj_align(uiLabelNotesStatus, LV_ALIGN_BOTTOM_RIGHT, -8, -(NAV_CLEARANCE + 1));
  lv_label_set_long_mode(uiLabelNotesStatus, LV_LABEL_LONG_WRAP);
  lv_label_set_text(uiLabelNotesStatus, "Ready");
  lv_obj_set_style_text_color(uiLabelNotesStatus, lv_color_hex(0x9FB3CF), LV_PART_MAIN);

  lv_obj_t* petTitle = lv_label_create(tabPocketPet);
  lv_obj_align(petTitle, LV_ALIGN_TOP_LEFT, 8, 38);
  lv_label_set_text(petTitle, "Pocket Pet (TamaPetchi)");
  lv_obj_set_style_text_color(petTitle, lv_color_hex(0x86EFAC), LV_PART_MAIN);

  lv_obj_t* btnPetExit = lv_btn_create(tabPocketPet);
  lv_obj_set_size(btnPetExit, 74, 30);
  lv_obj_align(btnPetExit, LV_ALIGN_TOP_RIGHT, -8, 34);
  lv_obj_set_style_radius(btnPetExit, 8, LV_PART_MAIN);
  lv_obj_set_style_bg_color(btnPetExit, lv_color_hex(0x334155), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(btnPetExit, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_add_event_cb(btnPetExit, uiPocketPetExitButtonEventCb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* petExitLabel = lv_label_create(btnPetExit);
  lv_label_set_text(petExitLabel, "Exit");
  lv_obj_center(petExitLabel);

  lv_obj_t* petPanel = lv_obj_create(tabPocketPet);
  lv_obj_set_size(petPanel, SCREEN_WIDTH - 16, 210);
  lv_obj_align(petPanel, LV_ALIGN_TOP_MID, 0, 62);
  lv_obj_set_style_bg_color(petPanel, lv_color_hex(0x101010), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(petPanel, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_color(petPanel, lv_color_hex(0x2E5A46), LV_PART_MAIN);
  lv_obj_set_style_border_width(petPanel, 1, LV_PART_MAIN);
  lv_obj_set_style_radius(petPanel, 10, LV_PART_MAIN);
  lv_obj_set_style_pad_all(petPanel, 8, LV_PART_MAIN);
  lv_obj_set_scrollbar_mode(petPanel, LV_SCROLLBAR_MODE_OFF);
  lv_obj_clear_flag(petPanel, LV_OBJ_FLAG_SCROLLABLE);

  uiLabelPetFace = lv_label_create(petPanel);
  lv_obj_align(uiLabelPetFace, LV_ALIGN_TOP_MID, 0, 2);
  lv_label_set_text(uiLabelPetFace, "(^_^)");
  lv_obj_set_style_text_color(uiLabelPetFace, lv_color_hex(0xA7F3D0), LV_PART_MAIN);

  uiLabelPetSummary = lv_label_create(petPanel);
  lv_obj_align(uiLabelPetSummary, LV_ALIGN_TOP_MID, 0, 24);
  lv_label_set_text(uiLabelPetSummary, "Ready");
  lv_obj_set_style_text_color(uiLabelPetSummary, lv_color_hex(0x93C5FD), LV_PART_MAIN);

  lv_obj_t* petLabelHunger = lv_label_create(petPanel);
  lv_obj_align(petLabelHunger, LV_ALIGN_TOP_LEFT, 4, 42);
  lv_label_set_text(petLabelHunger, "Hunger");
  lv_obj_set_style_text_color(petLabelHunger, lv_color_hex(0xFCA5A5), LV_PART_MAIN);

  uiBarPetHunger = lv_bar_create(petPanel);
  lv_obj_set_size(uiBarPetHunger, 126, 10);
  lv_obj_align(uiBarPetHunger, LV_ALIGN_TOP_RIGHT, -4, 44);
  lv_bar_set_range(uiBarPetHunger, 0, 100);
  lv_obj_set_style_bg_color(uiBarPetHunger, lv_color_hex(0x2A2A2A), LV_PART_MAIN);
  lv_obj_set_style_bg_color(uiBarPetHunger, lv_color_hex(0xF97316), LV_PART_INDICATOR);

  lv_obj_t* petLabelHappiness = lv_label_create(petPanel);
  lv_obj_align(petLabelHappiness, LV_ALIGN_TOP_LEFT, 4, 58);
  lv_label_set_text(petLabelHappiness, "Happy");
  lv_obj_set_style_text_color(petLabelHappiness, lv_color_hex(0xFDE68A), LV_PART_MAIN);

  uiBarPetHappiness = lv_bar_create(petPanel);
  lv_obj_set_size(uiBarPetHappiness, 126, 10);
  lv_obj_align(uiBarPetHappiness, LV_ALIGN_TOP_RIGHT, -4, 60);
  lv_bar_set_range(uiBarPetHappiness, 0, 100);
  lv_obj_set_style_bg_color(uiBarPetHappiness, lv_color_hex(0x2A2A2A), LV_PART_MAIN);
  lv_obj_set_style_bg_color(uiBarPetHappiness, lv_color_hex(0xCA8A04), LV_PART_INDICATOR);

  lv_obj_t* petLabelHealth = lv_label_create(petPanel);
  lv_obj_align(petLabelHealth, LV_ALIGN_TOP_LEFT, 4, 74);
  lv_label_set_text(petLabelHealth, "Health");
  lv_obj_set_style_text_color(petLabelHealth, lv_color_hex(0x86EFAC), LV_PART_MAIN);

  uiBarPetHealth = lv_bar_create(petPanel);
  lv_obj_set_size(uiBarPetHealth, 126, 10);
  lv_obj_align(uiBarPetHealth, LV_ALIGN_TOP_RIGHT, -4, 76);
  lv_bar_set_range(uiBarPetHealth, 0, 100);
  lv_obj_set_style_bg_color(uiBarPetHealth, lv_color_hex(0x2A2A2A), LV_PART_MAIN);
  lv_obj_set_style_bg_color(uiBarPetHealth, lv_color_hex(0x16A34A), LV_PART_INDICATOR);

  lv_obj_t* petLabelEnergy = lv_label_create(petPanel);
  lv_obj_align(petLabelEnergy, LV_ALIGN_TOP_LEFT, 4, 90);
  lv_label_set_text(petLabelEnergy, "Energy");
  lv_obj_set_style_text_color(petLabelEnergy, lv_color_hex(0x93C5FD), LV_PART_MAIN);

  uiBarPetEnergy = lv_bar_create(petPanel);
  lv_obj_set_size(uiBarPetEnergy, 126, 10);
  lv_obj_align(uiBarPetEnergy, LV_ALIGN_TOP_RIGHT, -4, 92);
  lv_bar_set_range(uiBarPetEnergy, 0, 100);
  lv_obj_set_style_bg_color(uiBarPetEnergy, lv_color_hex(0x2A2A2A), LV_PART_MAIN);
  lv_obj_set_style_bg_color(uiBarPetEnergy, lv_color_hex(0x2563EB), LV_PART_INDICATOR);

  lv_obj_t* petLabelClean = lv_label_create(petPanel);
  lv_obj_align(petLabelClean, LV_ALIGN_TOP_LEFT, 4, 106);
  lv_label_set_text(petLabelClean, "Clean");
  lv_obj_set_style_text_color(petLabelClean, lv_color_hex(0x67E8F9), LV_PART_MAIN);

  uiBarPetCleanliness = lv_bar_create(petPanel);
  lv_obj_set_size(uiBarPetCleanliness, 126, 10);
  lv_obj_align(uiBarPetCleanliness, LV_ALIGN_TOP_RIGHT, -4, 108);
  lv_bar_set_range(uiBarPetCleanliness, 0, 100);
  lv_obj_set_style_bg_color(uiBarPetCleanliness, lv_color_hex(0x2A2A2A), LV_PART_MAIN);
  lv_obj_set_style_bg_color(uiBarPetCleanliness, lv_color_hex(0x0891B2), LV_PART_INDICATOR);

  uiLabelPetDetail = lv_label_create(petPanel);
  lv_obj_set_width(uiLabelPetDetail, SCREEN_WIDTH - 40);
  lv_obj_align(uiLabelPetDetail, LV_ALIGN_TOP_LEFT, 4, 122);
  lv_label_set_long_mode(uiLabelPetDetail, LV_LABEL_LONG_DOT);
  lv_label_set_text(uiLabelPetDetail, "Age 0m | Normal");
  lv_obj_set_style_text_color(uiLabelPetDetail, lv_color_hex(0x9FB3CF), LV_PART_MAIN);

  lv_obj_t* btnPetFeed = lv_btn_create(petPanel);
  lv_obj_set_size(btnPetFeed, 66, 28);
  lv_obj_align(btnPetFeed, LV_ALIGN_TOP_LEFT, 4, 142);
  lv_obj_set_style_radius(btnPetFeed, 8, LV_PART_MAIN);
  lv_obj_set_style_bg_color(btnPetFeed, lv_color_hex(0x0F766E), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(btnPetFeed, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_add_event_cb(btnPetFeed, uiPocketPetFeedButtonEventCb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* petFeedLabel = lv_label_create(btnPetFeed);
  lv_label_set_text(petFeedLabel, "Feed");
  lv_obj_center(petFeedLabel);

  lv_obj_t* btnPetPlay = lv_btn_create(petPanel);
  lv_obj_set_size(btnPetPlay, 66, 28);
  lv_obj_align(btnPetPlay, LV_ALIGN_TOP_LEFT, 79, 142);
  lv_obj_set_style_radius(btnPetPlay, 8, LV_PART_MAIN);
  lv_obj_set_style_bg_color(btnPetPlay, lv_color_hex(0x1D4ED8), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(btnPetPlay, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_add_event_cb(btnPetPlay, uiPocketPetPlayButtonEventCb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* petPlayLabel = lv_label_create(btnPetPlay);
  lv_label_set_text(petPlayLabel, "Play");
  lv_obj_center(petPlayLabel);

  lv_obj_t* btnPetClean = lv_btn_create(petPanel);
  lv_obj_set_size(btnPetClean, 66, 28);
  lv_obj_align(btnPetClean, LV_ALIGN_TOP_LEFT, 154, 142);
  lv_obj_set_style_radius(btnPetClean, 8, LV_PART_MAIN);
  lv_obj_set_style_bg_color(btnPetClean, lv_color_hex(0x0E7490), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(btnPetClean, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_add_event_cb(btnPetClean, uiPocketPetCleanButtonEventCb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* petCleanLabel = lv_label_create(btnPetClean);
  lv_label_set_text(petCleanLabel, "Clean");
  lv_obj_center(petCleanLabel);

  lv_obj_t* btnPetSleep = lv_btn_create(petPanel);
  lv_obj_set_size(btnPetSleep, 66, 28);
  lv_obj_align(btnPetSleep, LV_ALIGN_TOP_LEFT, 4, 174);
  lv_obj_set_style_radius(btnPetSleep, 8, LV_PART_MAIN);
  lv_obj_set_style_bg_color(btnPetSleep, lv_color_hex(0x6D28D9), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(btnPetSleep, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_add_event_cb(btnPetSleep, uiPocketPetSleepButtonEventCb, LV_EVENT_CLICKED, nullptr);
  uiLabelPetSleepButton = lv_label_create(btnPetSleep);
  lv_label_set_text(uiLabelPetSleepButton, "Sleep");
  lv_obj_center(uiLabelPetSleepButton);

  lv_obj_t* btnPetHeal = lv_btn_create(petPanel);
  lv_obj_set_size(btnPetHeal, 66, 28);
  lv_obj_align(btnPetHeal, LV_ALIGN_TOP_LEFT, 79, 174);
  lv_obj_set_style_radius(btnPetHeal, 8, LV_PART_MAIN);
  lv_obj_set_style_bg_color(btnPetHeal, lv_color_hex(0xBE185D), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(btnPetHeal, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_add_event_cb(btnPetHeal, uiPocketPetHealButtonEventCb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* petHealLabel = lv_label_create(btnPetHeal);
  lv_label_set_text(petHealLabel, "Heal");
  lv_obj_center(petHealLabel);

  lv_obj_t* btnPetReset = lv_btn_create(petPanel);
  lv_obj_set_size(btnPetReset, 66, 28);
  lv_obj_align(btnPetReset, LV_ALIGN_TOP_LEFT, 154, 174);
  lv_obj_set_style_radius(btnPetReset, 8, LV_PART_MAIN);
  lv_obj_set_style_bg_color(btnPetReset, lv_color_hex(0x475569), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(btnPetReset, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_add_event_cb(btnPetReset, uiPocketPetResetButtonEventCb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* petResetLabel = lv_label_create(btnPetReset);
  lv_label_set_text(petResetLabel, "Reset");
  lv_obj_center(petResetLabel);

  lv_obj_t* calTitle = lv_label_create(tabCalendar);
  lv_obj_align(calTitle, LV_ALIGN_TOP_LEFT, 8, 38);
  lv_label_set_text(calTitle, "Daily Calendar");
  lv_obj_set_style_text_color(calTitle, lv_color_hex(0xFDE68A), LV_PART_MAIN);

  lv_obj_t* calSyncBtn = lv_btn_create(tabCalendar);
  lv_obj_set_size(calSyncBtn, 74, 30);
  lv_obj_align(calSyncBtn, LV_ALIGN_TOP_RIGHT, -8, 34);
  lv_obj_set_style_radius(calSyncBtn, 8, LV_PART_MAIN);
  lv_obj_set_style_bg_color(calSyncBtn, lv_color_hex(0x14532D), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(calSyncBtn, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_add_event_cb(calSyncBtn, uiCloudSyncButtonEventCb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* calSyncLabel = lv_label_create(calSyncBtn);
  lv_label_set_text(calSyncLabel, "Sync");
  lv_obj_center(calSyncLabel);

  uiListCalendar = lv_list_create(tabCalendar);
  lv_obj_set_size(uiListCalendar, SCREEN_WIDTH - 16, 180);
  lv_obj_align(uiListCalendar, LV_ALIGN_TOP_MID, 0, 68);
  lv_obj_set_style_radius(uiListCalendar, 10, LV_PART_MAIN);
  lv_obj_set_style_bg_color(uiListCalendar, lv_color_hex(0x101820), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(uiListCalendar, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_color(uiListCalendar, lv_color_hex(0x334155), LV_PART_MAIN);
  lv_obj_set_style_border_width(uiListCalendar, 1, LV_PART_MAIN);
  lv_obj_set_scrollbar_mode(uiListCalendar, LV_SCROLLBAR_MODE_ACTIVE);

  uiLabelCalendarStatus = lv_label_create(tabCalendar);
  lv_obj_set_width(uiLabelCalendarStatus, SCREEN_WIDTH - 16);
  lv_obj_align(uiLabelCalendarStatus, LV_ALIGN_BOTTOM_LEFT, 8, -(NAV_CLEARANCE - 6));
  lv_label_set_long_mode(uiLabelCalendarStatus, LV_LABEL_LONG_DOT);
  lv_label_set_text(uiLabelCalendarStatus, "Calendar idle");
  lv_obj_set_style_text_color(uiLabelCalendarStatus, lv_color_hex(0x9FB3CF), LV_PART_MAIN);

  lv_obj_t* syncTasksTitle = lv_label_create(tabSyncTasks);
  lv_obj_align(syncTasksTitle, LV_ALIGN_TOP_LEFT, 8, 38);
  lv_label_set_text(syncTasksTitle, "Interactive Tasks");
  lv_obj_set_style_text_color(syncTasksTitle, lv_color_hex(0x7DD3FC), LV_PART_MAIN);

  lv_obj_t* syncTasksSyncBtn = lv_btn_create(tabSyncTasks);
  lv_obj_set_size(syncTasksSyncBtn, 74, 30);
  lv_obj_align(syncTasksSyncBtn, LV_ALIGN_TOP_RIGHT, -8, 34);
  lv_obj_set_style_radius(syncTasksSyncBtn, 8, LV_PART_MAIN);
  lv_obj_set_style_bg_color(syncTasksSyncBtn, lv_color_hex(0x14532D), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(syncTasksSyncBtn, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_add_event_cb(syncTasksSyncBtn, uiCloudSyncButtonEventCb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* syncTasksSyncLabel = lv_label_create(syncTasksSyncBtn);
  lv_label_set_text(syncTasksSyncLabel, "Sync");
  lv_obj_center(syncTasksSyncLabel);

  lv_obj_t* todoListLabel = lv_label_create(tabSyncTasks);
  lv_obj_align(todoListLabel, LV_ALIGN_TOP_LEFT, 8, 74);
  lv_label_set_text(todoListLabel, "List");
  lv_obj_set_style_text_color(todoListLabel, lv_color_hex(0x93C5FD), LV_PART_MAIN);

  uiDdTodoEntity = lv_dropdown_create(tabSyncTasks);
  lv_obj_set_width(uiDdTodoEntity, 150);
  lv_obj_align(uiDdTodoEntity, LV_ALIGN_TOP_LEFT, 46, 68);
  lv_obj_set_style_radius(uiDdTodoEntity, 8, LV_PART_MAIN);
  lv_obj_set_style_bg_color(uiDdTodoEntity, lv_color_hex(0x10233C), LV_PART_MAIN);
  lv_obj_set_style_text_color(uiDdTodoEntity, lv_color_hex(0xEAF2FF), LV_PART_MAIN);
  lv_dropdown_set_options(uiDdTodoEntity, activeTodoEntity().c_str());
  lv_obj_add_event_cb(uiDdTodoEntity, uiTodoEntityDropdownEventCb, LV_EVENT_VALUE_CHANGED, nullptr);

  lv_obj_t* todoListsRefreshBtn = lv_btn_create(tabSyncTasks);
  lv_obj_set_size(todoListsRefreshBtn, 64, 30);
  lv_obj_align(todoListsRefreshBtn, LV_ALIGN_TOP_RIGHT, -8, 68);
  lv_obj_set_style_radius(todoListsRefreshBtn, 8, LV_PART_MAIN);
  lv_obj_set_style_bg_color(todoListsRefreshBtn, lv_color_hex(0x1E40AF), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(todoListsRefreshBtn, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_add_event_cb(todoListsRefreshBtn, uiTodoEntityRefreshButtonEventCb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* todoListsRefreshLabel = lv_label_create(todoListsRefreshBtn);
  lv_label_set_text(todoListsRefreshLabel, "Lists");
  lv_obj_center(todoListsRefreshLabel);

  uiListSyncTasks = lv_list_create(tabSyncTasks);
  lv_obj_set_size(uiListSyncTasks, SCREEN_WIDTH - 16, 138);
  lv_obj_align(uiListSyncTasks, LV_ALIGN_TOP_MID, 0, 104);
  lv_obj_set_style_radius(uiListSyncTasks, 10, LV_PART_MAIN);
  lv_obj_set_style_bg_color(uiListSyncTasks, lv_color_hex(0x0F172A), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(uiListSyncTasks, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_color(uiListSyncTasks, lv_color_hex(0x334155), LV_PART_MAIN);
  lv_obj_set_style_border_width(uiListSyncTasks, 1, LV_PART_MAIN);
  lv_obj_set_scrollbar_mode(uiListSyncTasks, LV_SCROLLBAR_MODE_ACTIVE);

  uiLabelSyncTasksStatus = lv_label_create(tabSyncTasks);
  lv_obj_set_width(uiLabelSyncTasksStatus, SCREEN_WIDTH - 16);
  lv_obj_align(uiLabelSyncTasksStatus, LV_ALIGN_BOTTOM_LEFT, 8, -(NAV_CLEARANCE - 6));
  lv_label_set_long_mode(uiLabelSyncTasksStatus, LV_LABEL_LONG_DOT);
  lv_label_set_text(uiLabelSyncTasksStatus, "Task sync idle");
  lv_obj_set_style_text_color(uiLabelSyncTasksStatus, lv_color_hex(0x9FB3CF), LV_PART_MAIN);

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
  uiUpdateDictateButtonLabel();
  uiUpdateMicMuteButton();
  uiSetWifiMenuStatus("Set Wi-Fi then Connect");
  uiSetNotesStatus("Ready for dictation");
  uiSetCalendarStatus("Calendar idle");
  uiSetSyncTasksStatus("Task sync idle");
  uiRefreshTodoEntityDropdown();
  uiRefreshCalendarList();
  uiRefreshSyncTasksList();
  uiRefreshPerformanceMetrics();
  uiUpdatePocketPet();
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
        const uint16_t next = (current + 1) % TAB_COUNT;
        lv_tabview_set_act(uiTabview, next, LV_ANIM_OFF);
      }
      uiSetWifiMenuStatus("Long press: toggled tab");
    } else {
      uiSetWifiMenuStatus("Short press: no action");
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
  if (!handsFreeEnabled || !localWakeReady || localWakePaused || micMuted) {
    vTaskDelay(pdMS_TO_TICKS(20));
    return ESP_FAIL;
  }
  if (!audioReady || !i2sMicReady) {
    vTaskDelay(pdMS_TO_TICKS(20));
    return ESP_FAIL;
  }
  if (state != AssistantState::Idle || captureRequested || assistantStopRequested) {
    vTaskDelay(pdMS_TO_TICKS(12));
    return ESP_FAIL;
  }

  const TickType_t ticks =
      timeoutMs == 0 || timeoutMs == UINT32_MAX ? pdMS_TO_TICKS(80) : pdMS_TO_TICKS(timeoutMs);
  return i2s_read(micCapture.port(), out, len, bytesRead, ticks);
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
  outError = "ESP_SR unavailable (select ESP SR partition)";
  return false;
#else
  if (!i2sMicReady) {
    outError = "local wake requires I2S mic";
    return false;
  }
  if (localWakeReady) {
    return true;
  }
  if (!micCapture.prepareCaptureClock()) {
    outError = "I2S capture clock setup failed";
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
  Serial.println("[WAKE] local command detector ready (ok bob / okay bob)");
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

void pollLocalWakeTrigger() {
  if (!handsFreeEnabled || !localWakeEnabled || !localWakeReady || micMuted) {
    return;
  }
  if (!localWakeTriggerPending || captureRequested || state != AssistantState::Idle) {
    return;
  }

  const uint32_t now = millis();
  if (now < handsFreeCooldownUntilMs) {
    return;
  }

  localWakeTriggerPending = false;
  handsFreeCooldownUntilMs = now + LOCAL_WAKE_COOLDOWN_MS;
  wakeBypassUntilMs = now + LOCAL_WAKE_BYPASS_MS;
  pendingCaptureMode = CaptureMode::Assistant;
  bypassWakeWordForNextCapture = false;
  captureRequested = true;
  Serial.println(String("[WAKE] local trigger phrase=") + localWakePhraseId);
  setState(AssistantState::Idle, "Wake heard locally");
}

void pollHandsFreeTrigger() {
#if ASSISTANT_ENABLE_HANDS_FREE
  if (!handsFreeEnabled || !audioReady || state != AssistantState::Idle || captureRequested || micMuted) {
    return;
  }

  if (localWakeEnabled && localWakeReady) {
#if ASSISTANT_LOCAL_WAKE_USE_VAD_FALLBACK
    // Keep VAD as fallback while local wake is enabled.
#else
    return;
#endif
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
  const float adaptiveThreshold = vadNoiseFloorNorm * 1.10f + 0.0010f;
  if (adaptiveThreshold > dynamicVadThreshold) {
    dynamicVadThreshold = adaptiveThreshold;
  }
  if (dynamicVadThreshold < 0.004f) {
    dynamicVadThreshold = 0.004f;
  }
  if (dynamicVadThreshold > 0.050f) {
    dynamicVadThreshold = 0.050f;
  }

  float peakTriggerThreshold = vadNoiseFloorPeakNorm * 1.55f + 0.014f;
  if (peakTriggerThreshold < 0.018f) {
    peakTriggerThreshold = 0.018f;
  }
  if (peakTriggerThreshold > 0.25f) {
    peakTriggerThreshold = 0.25f;
  }

  const float speechRmsGate = dynamicVadThreshold > 0.014f ? dynamicVadThreshold : 0.014f;
  const float floorDelta = rmsNorm - vadNoiseFloorNorm;
  const float peakDelta = peakNorm - vadNoiseFloorPeakNorm;
  const bool clearlyAboveFloor = floorDelta >= 0.0022f || peakDelta >= 0.028f;
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
    pendingCaptureMode = CaptureMode::Assistant;
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

#if ASSISTANT_ENABLE_HANDS_FREE
  handsFreeEnabled = true;
#else
  handsFreeEnabled = false;
#endif
  localWakeEnabled = handsFreeEnabled && (ASSISTANT_LOCAL_WAKE_ENABLED != 0);
  Serial.println(String("[WAKE] hands-free ") + (handsFreeEnabled ? "enabled" : "disabled"));
  Serial.println(String("[WAKE] phrases: ") + String(HA_WAKE_PHRASE));
  Serial.println(String("[WAKE] local detector ") + (localWakeEnabled ? "enabled" : "disabled"));
  Serial.println(String("[CORE] setup on core ") + xPortGetCoreID());

  pinMode(PIN_BOOT_BUTTON, INPUT_PULLUP);
  pinMode(PIN_TFT_BL, OUTPUT);
  digitalWrite(PIN_TFT_BL, HIGH);

  // Re-run TFT_eSPI constructor at runtime to ensure internal SPI state is valid
  // before begin(); some ESP32-S3 core/library combinations boot with stale defaults.
  new (&tft) TFT_eSPI(SCREEN_WIDTH, SCREEN_HEIGHT);
  tft.begin();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);

  lv_init();
  lv_disp_draw_buf_init(&drawBuf, drawBufPixels, drawBufPixelsAlt, SCREEN_WIDTH * 20);

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
  loadPocketPetState();
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
    if (localWakeEnabled) {
#if ASSISTANT_HAS_LOCAL_SR && __has_include(<ESP_SR.h>)
      ESP_SR.onEvent(nullptr);  // Force ESP_SR linkage so srmodels.bin is exported for flashing.
#endif
      String wakeErr;
      if (!startLocalWakeEngine(wakeErr)) {
        localWakeEnabled = false;
        localWakeReady = false;
        Serial.println(String("[WAKE] local disabled: ") + wakeErr);
      }
    }
  }

  sdReady = initSdCard();
  if (!sdReady) {
    Serial.println("[SD] init failed");
  } else {
    String cacheErr;
    if (loadCloudCache(cacheErr)) {
      uiSetCalendarStatus("Loaded cached agenda");
      uiSetSyncTasksStatus("Loaded cached tasks");
      uiRefreshCalendarList();
      uiRefreshSyncTasksList();
    } else {
      Serial.println(String("[CACHE] ") + cacheErr);
    }
  }

  if (configuredSsid.length() > 0) {
    setState(AssistantState::ConnectingWiFi, String("Connecting ") + configuredSsid);
    if (connectToWifi(configuredSsid, configuredPassword, 15000)) {
      setState(AssistantState::Idle, "Ready");
    } else {
      setState(AssistantState::Idle, "Wi-Fi offline");
    }
  } else {
    setState(AssistantState::Idle, "Set Wi-Fi in Settings > Wi-Fi");
  }

  if (WiFi.status() == WL_CONNECTED) {
    syncClockIfNeeded();
    String todoErr;
    if (fetchTodoEntityOptionsFromHomeAssistant(todoErr)) {
      uiRefreshTodoEntityDropdown();
    } else {
      Serial.println(String("[TODO] list refresh: ") + todoErr);
    }
    String syncErr;
    if (!syncCloudData(true, syncErr)) {
      Serial.println(String("[SYNC] initial sync skipped: ") + syncErr);
    }
  }

  uiRefreshNow();
}

void loop() {
  static uint32_t lastUiPollMs = 0;
  static uint32_t cpuWindowStartMs = 0;
  static uint64_t cpuBusyUsAccum = 0;
  static uint64_t cpuTotalUsAccum = 0;

  const uint64_t loopStartUs = micros();
  const uint32_t now = millis();
  if (cpuWindowStartMs == 0) {
    cpuWindowStartMs = now;
  }

  lv_timer_handler();
  pollBootButton();
  pollLocalWakeTrigger();
  pollHandsFreeTrigger();
  updatePocketPetLoop(now);
  maybeRunScheduledCloudSync(now);

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

  if (todoEntityRefreshRequested) {
    todoEntityRefreshRequested = false;
    String err;
    if (fetchTodoEntityOptionsFromHomeAssistant(err)) {
      uiRefreshTodoEntityDropdown();
      uiSetSyncTasksStatus("To-do lists refreshed");
    } else {
      uiSetSyncTasksStatus("List refresh failed: " + clipText(err, 38));
    }
  }

  if (captureRequested) {
    const bool bypassWakeWord = bypassWakeWordForNextCapture;
    const CaptureMode requestedMode = pendingCaptureMode;
    activeCaptureMode = requestedMode;
    pendingCaptureMode = CaptureMode::Assistant;
    bypassWakeWordForNextCapture = false;
    captureRequested = false;
    runCapturePipeline(bypassWakeWord, requestedMode);
    activeCaptureMode = CaptureMode::Assistant;
    uiUpdateTalkButtonLabel();
    uiUpdateDictateButtonLabel();
  }

  if (cloudSyncRequested) {
    cloudSyncRequested = false;
    String syncErr;
    if (!syncCloudData(true, syncErr)) {
      uiSetCalendarStatus("Sync failed: " + clipText(syncErr, 40));
      uiSetSyncTasksStatus("Sync failed: " + clipText(syncErr, 40));
    }
  }

  if ((now - lastUiPollMs) > 1000) {
    lastUiPollMs = now;
    uiUpdateStatus();
    uiRefreshPerformanceMetrics();
  }
  uiUpdateMicLevelLine();
  checkCalendarAlerts(now);

  const uint64_t loopPreDelayUs = micros();
  cpuBusyUsAccum += (loopPreDelayUs - loopStartUs);
  delay(5);
  const uint64_t loopEndUs = micros();
  cpuTotalUsAccum += (loopEndUs - loopStartUs);

  if ((now - cpuWindowStartMs) >= PERF_REFRESH_INTERVAL_MS && cpuTotalUsAccum > 0) {
    const float usage = (100.0f * static_cast<float>(cpuBusyUsAccum)) / static_cast<float>(cpuTotalUsAccum);
    if (usage < 0.0f) {
      loopCpuUsagePercent = 0.0f;
    } else if (usage > 100.0f) {
      loopCpuUsagePercent = 100.0f;
    } else {
      loopCpuUsagePercent = usage;
    }
    cpuBusyUsAccum = 0;
    cpuTotalUsAccum = 0;
    cpuWindowStartMs = now;
  }
}
