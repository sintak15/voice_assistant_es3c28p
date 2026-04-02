#pragma once

// Copy this file to secrets.h and set your values there.
// If secrets.h is missing, these defaults are used.

#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD ""
#endif

#ifndef OPENAI_API_KEY
#define OPENAI_API_KEY ""
#endif

// Home Assistant backend configuration
#ifndef HA_BASE_URL
#define HA_BASE_URL ""
#endif

#ifndef HA_ACCESS_TOKEN
#define HA_ACCESS_TOKEN ""
#endif

#ifndef HA_STT_PROVIDER
#define HA_STT_PROVIDER ""
#endif

#ifndef HA_LANGUAGE
#define HA_LANGUAGE "en-US"
#endif

#ifndef HA_AGENT_ID
#define HA_AGENT_ID ""
#endif

#ifndef HA_TTS_ENGINE_ID
#define HA_TTS_ENGINE_ID ""
#endif

#ifndef HA_WAKE_PHRASE
#define HA_WAKE_PHRASE "ok bob,okay bob"
#endif

#ifndef ASSISTANT_ENABLE_HANDS_FREE
#define ASSISTANT_ENABLE_HANDS_FREE 1
#endif

#ifndef ASSISTANT_LOCAL_WAKE_ENABLED
#define ASSISTANT_LOCAL_WAKE_ENABLED 1
#endif

#ifndef ASSISTANT_LOCAL_WAKE_USE_VAD_FALLBACK
#define ASSISTANT_LOCAL_WAKE_USE_VAD_FALLBACK 0
#endif

#ifndef ASSISTANT_VAD_TRIGGER_RMS
#define ASSISTANT_VAD_TRIGGER_RMS 0.015f
#endif

#ifndef ASSISTANT_VAD_TRIGGER_FRAMES
#define ASSISTANT_VAD_TRIGGER_FRAMES 3
#endif

// Cloud dashboard integration (Google Calendar/Tasks or any compatible JSON endpoint)
// Calendar endpoint should return an array or an object with `items` containing:
//   id, summary/title, start{dateTime|date}, end{dateTime|date}
#ifndef CLOUD_CALENDAR_EVENTS_URL
#define CLOUD_CALENDAR_EVENTS_URL ""
#endif

// Optional Home Assistant calendar entity fallback, e.g. "calendar.family"
// If set, firmware calls:
//   /api/calendars/<entity>?start=<utc>&end=<utc>
#ifndef HA_CALENDAR_ENTITY_ID
#define HA_CALENDAR_ENTITY_ID ""
#endif

#ifndef HA_TODO_ENTITY_ID
#define HA_TODO_ENTITY_ID ""
#endif

// Tasks endpoint should return an array or an object with `items` containing:
//   id, title/name/text, status/completed, due
#ifndef CLOUD_TASKS_LIST_URL
#define CLOUD_TASKS_LIST_URL ""
#endif

// Webhook/endpoint to mark task completion.
// POST body: {id, title, completed, status}
#ifndef CLOUD_TASK_COMPLETE_WEBHOOK_URL
#define CLOUD_TASK_COMPLETE_WEBHOOK_URL ""
#endif

// Bearer token for cloud calendar/task APIs (if needed).
// If empty, Home Assistant token is used for HA URLs.
#ifndef CLOUD_API_BEARER_TOKEN
#define CLOUD_API_BEARER_TOKEN ""
#endif

// Number of days ahead to request during each sync.
#ifndef CLOUD_SYNC_DAYS_AHEAD
#define CLOUD_SYNC_DAYS_AHEAD 3
#endif

// POSIX timezone string used by scheduled 6:00 AM sync and local-time rendering.
// Change this to your locale if needed.
#ifndef DEVICE_TZ
#define DEVICE_TZ "CST6CDT,M3.2.0/2,M11.1.0/2"
#endif
