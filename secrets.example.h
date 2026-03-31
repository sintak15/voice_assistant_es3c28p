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
#define HA_WAKE_PHRASE "ok bob,okay bob,hey bob"
#endif

#ifndef ASSISTANT_ENABLE_HANDS_FREE
#define ASSISTANT_ENABLE_HANDS_FREE 1
#endif

#ifndef ASSISTANT_VAD_TRIGGER_RMS
#define ASSISTANT_VAD_TRIGGER_RMS 0.015f
#endif

#ifndef ASSISTANT_VAD_TRIGGER_FRAMES
#define ASSISTANT_VAD_TRIGGER_FRAMES 3
#endif
