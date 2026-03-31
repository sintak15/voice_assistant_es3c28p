# ESP32-S3 Personal Voice Assistant Starter

This is a clean Arduino sketch starter for the LCDWiki 2.8-inch ESP32-S3 display board (`ES3C28P` / `ES3N28P`).

## What this starter does now

- Initializes the ILI9341 display using the board pin map from LCDWiki.
- Initializes FT6336 touch over I2C.
- Falls back to the BOOT button (`GPIO0`) if touch is unavailable.
- Initializes ES8311 codec and captures microphone audio over I2S (16kHz, PCM16, mono).
- Plays Home Assistant TTS WAV replies over I2S speaker output.
- Mounts microSD via SDIO and saves each capture as WAV in `/voice/`.
- Connects to WiFi (if configured).
- On touch or BOOT button press, records 3 seconds of audio.
- Hands-free mode: VAD-triggered capture with wake phrase filtering (STT-based).
- Debug screen with live health and latency stats.
- Optional Home Assistant backend flow:
  - Sends audio to `POST /api/stt/{provider}`
  - Sends transcript to `POST /api/conversation/process`
  - Requests TTS audio URL via `POST /api/tts_get_url`
  - Streams and plays WAV reply audio

## Hardware Pin Map (from LCDWiki)

- LCD SPI: `CS=10, DC=46, MOSI=11, SCK=12, MISO=13, BL=45`
- Touch I2C: `SDA=16, SCL=15, RST=18, INT=17`
- Audio (next step): `EN=1, MCLK=4, BCLK=5, DOUT=6, LRCLK=7, DIN=8`
- SDIO: `CLK=38, CMD=40, D0=39, D1=41, D2=48, D3=47`

## Arduino Setup

1. Install ESP32 boards package for Arduino IDE.
2. Select board: `ESP32S3 Dev Module`.
3. Install libraries:
   - `Adafruit GFX Library`
   - `Adafruit ILI9341`
   - `ArduinoJson`
4. Create `secrets.h` in this folder and set credentials:

```cpp
#pragma once
#define WIFI_SSID "your_wifi"
#define WIFI_PASSWORD "your_password"
#define OPENAI_API_KEY ""

// Optional Home Assistant backend
#define HA_BASE_URL "http://homeassistant.local:8123"
#define HA_ACCESS_TOKEN "long_lived_access_token"
#define HA_STT_PROVIDER "whisper"
#define HA_LANGUAGE "en-US"
#define HA_AGENT_ID ""   // Optional, leave empty for default
#define HA_TTS_ENGINE_ID "tts.piper"  // Example: tts.piper

// Hands-free wake/VAD
#define HA_WAKE_PHRASE "hey assistant"
#define ASSISTANT_ENABLE_HANDS_FREE 1
#define ASSISTANT_VAD_TRIGGER_RMS 0.015f
#define ASSISTANT_VAD_TRIGGER_FRAMES 3
```

5. Open `voice_assistant_es3c28p.ino` and upload.

## SD Capture Files

- Recorded files are saved to `/voice/` on the microSD card.
- Naming pattern: `cap_00000.wav`, `cap_00001.wav`, etc.
- Audio format: `16kHz`, `mono`, `16-bit PCM WAV`.

## Controls

- Short BOOT press: trigger one capture.
- Long BOOT press: toggle debug screen.
- Touch top-left corner: toggle hands-free mode on/off.
- Touch top-right corner: toggle debug screen.
- Touch center/anywhere else: trigger one capture.

## Hands-Free Notes

- Wake phrase detection is STT-based (cloud/back-end), not an on-device wake-word model.
- VAD runs continuously only while idle and connected to Home Assistant.
- If speech is detected without the wake phrase, the utterance is ignored.

## Touch Troubleshooting

- If your module is `ES3N28P` (non-touch), use the BOOT button to trigger recording.
- If touch does not respond on `ES3C28P`, open Serial Monitor at `115200` and confirm I2C detects `0x38` (FT6336) during boot.
- I2C scan also typically shows `0x18` for ES8311 audio codec.

## Next Milestones

1. Upgrade wake phrase from STT-based to true on-device wake-word model.
2. Add on-screen conversation history with scrolling.
3. Add microphone gain/settings page.
4. Add streamed STT (partial transcript updates).
5. Add offline fallback (local intent mode).
