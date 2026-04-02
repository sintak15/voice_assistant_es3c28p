#line 1 "C:\\Users\\Justin\\Documents\\Arduino\\voice_assistant_es3c28p\\README.md"
# CYD S3 Voice Assistant (ESP32-S3 + 2.8in Touch)

A personal voice assistant firmware for ESP32-S3 2.8-inch touchscreen boards (Hosyond/LILYGO/Freenove-style variants) using Arduino.

This project focuses on:
- Touch-first local UI (listen/stop control)
- Microphone capture and voice activity diagnostics
- Home Assistant STT -> conversation -> TTS loop
- On-device playback through ES8311 + I2S speaker path
- Calendar + task dashboard workflows with offline cache

## Status

This repository contains active in-progress firmware and board tuning. Core voice flow is implemented, and current tuning work is aimed at improving responsiveness and audio smoothness.

## Features

- LVGL + TFT_eSPI UI on 240x320 portrait ILI9341 display
- Touch controls (FT6336) with a `Listen`/`Stop` talk button
- Wi-Fi onboarding controls in UI (scan/save/connect)
- 16 kHz mono PCM microphone capture via ES8311/I2S
- RMS + peak diagnostics for speech detection visibility
- Home Assistant integration:
  - `POST /api/stt/{provider}`
  - `POST /api/conversation/process`
  - `POST /api/tts_get_url`
  - WAV TTS streaming playback with prebuffering
- Optional hands-free mode using VAD + phrase matching (STT-gated)
- SD card WAV capture support for debug and validation
- Daily Calendar dashboard tab with scrollable agenda list
- Interactive Sync Tasks tab (tap to toggle complete)
- Offline cache for calendar/tasks on SD (`/voice/dashboard_cache.json`)
- Scheduled 6:00 AM sync trigger (local timezone via `DEVICE_TZ`)
- 10-minute pre-event chime alerts for upcoming appointments

## Repository Layout

- `voice_assistant_es3c28p.ino`: Main app/UI/state machine
- `board_pins.h`: Board pin map used by display/touch/audio/SD
- `i2s_mic_capture.h`: I2S capture/playback and audio-level helpers
- `es8311_codec.h`: ES8311 control over I2C
- `ft6336_touch.h`: FT6336 touch driver wrapper
- `tft_espi_local_setup.h`: Local TFT_eSPI setup (avoids global edits)
- `secrets.example.h`: Config template (copy to `secrets.h`)
- `THIRD_PARTY_NOTICES.md`: Upstream attribution and license references
- `LICENSE`: Project license

## Hardware Pin Map (Current Firmware)

From `board_pins.h`:

| Function | Pins |
|---|---|
| TFT SPI | `CS=10`, `DC=46`, `MOSI=11`, `SCK=12`, `MISO=13`, `BL=45` |
| Touch I2C | `SDA=16`, `SCL=15`, `RST=18`, `INT=17`, `ADDR=0x38` |
| Audio (ES8311/I2S) | `EN=1` (active low), `MCLK=4`, `BCLK=5`, `DOUT=8`, `LRCLK=7`, `DIN=6` |
| SDIO | `CLK=38`, `CMD=40`, `D0=39`, `D1=41`, `D2=48`, `D3=47` |
| Misc | `BOOT=0`, `RGB=42`, `BAT_ADC=9` |

Note: these boards are often sold under multiple names with small BOM/pin differences. If touch/audio is inconsistent, re-verify against your exact PCB silkscreen and vendor demo source.

## Prerequisites

- Arduino IDE 2.x or `arduino-cli`
- ESP32 boards platform for Arduino
- Libraries used by this sketch:
  - `TFT_eSPI`
  - `lvgl`
  - `ArduinoJson`
  - ESP32 core libs (`WiFi`, `HTTPClient`, `Preferences`, `SD_MMC`, etc.)

## Configuration

1. Copy `secrets.example.h` to `secrets.h`.
2. Fill in Wi-Fi and Home Assistant values.

Minimal required Home Assistant settings:
- `HA_BASE_URL`
- `HA_ACCESS_TOKEN`
- `HA_STT_PROVIDER`

Useful voice settings:
- `HA_LANGUAGE` (default `en-US`)
- `HA_TTS_ENGINE_ID` (for example `tts.piper`)
- `HA_WAKE_PHRASE` (comma-separated phrases, e.g. `ok bob,hey bob`)

Calendar/task dashboard settings:
- `CLOUD_CALENDAR_EVENTS_URL` (Google Calendar proxy/custom endpoint JSON)
- `HA_CALENDAR_ENTITY_ID` (optional Home Assistant calendar fallback)
- `CLOUD_TASKS_LIST_URL` (Google Tasks proxy/custom endpoint JSON)
- `CLOUD_TASK_COMPLETE_WEBHOOK_URL` (for completion updates)
- `CLOUD_API_BEARER_TOKEN` (optional bearer token for cloud endpoints)
- `CLOUD_SYNC_DAYS_AHEAD` (default `3`)
- `DEVICE_TZ` (POSIX timezone string for local rendering + 6:00 sync)

## Build and Flash (Recommended CLI Flow)

Use sequential compile then upload (do not skip compile):

```powershell
arduino-cli compile --fqbn esp32:esp32:esp32s3 \
  --build-path .\\build .

arduino-cli upload --fqbn esp32:esp32:esp32s3 \
  --port COM10 --input-dir .\\build .
```

If your board requires custom menu options (flash size, PSRAM mode, USB CDC), append them to `--fqbn` as needed.

## Runtime Controls

- UI `Listen` button: starts capture/assistant request
- UI `Stop` button (same control while active): requests cancellation
- Touch + status labels expose current assistant state and diagnostics
- Launcher tiles open `Calendar`, `To-Dos`, `Pocket Pet`, `Notes`, `Queue`, and `Wi-Fi`
- `Sync` tile/manual buttons fetch cloud calendar/tasks and refresh local cache
- Tap items in `Sync Tasks` to check/uncheck and send webhook updates

## Troubleshooting

### No speech detected
- Confirm mic RMS/peak values are changing during speech.
- Validate ES8311 I2C detect and I2S pin mapping.
- Confirm audio-enable line polarity (`EN` active-low on this board profile).

### STT HTTP failures (`415`, `-3`, `connection refused`)
- Verify Home Assistant URL and token.
- Confirm STT provider is installed and selected.
- Check payload format/provider expectations in Home Assistant logs.

### Wake phrase does not trigger
- Current hands-free mode is STT/VAD-gated phrase matching, not a dedicated on-device wake-word engine.
- Validate phrase list formatting in `HA_WAKE_PHRASE`.

### Calendar or tasks do not load
- Confirm endpoint URLs in `secrets.h` return JSON arrays (or `{ "items": [...] }`).
- If using private Google APIs, use a secure proxy/webhook that adds OAuth server-side.
- If using Home Assistant calendar fallback, set `HA_CALENDAR_ENTITY_ID`.
- Check `DEVICE_TZ` for correct local times and 6:00 AM scheduled sync behavior.

### TTS plays but sounds delayed/choppy
- Check Wi-Fi quality and Home Assistant host load.
- Adjust TTS prebuffer constants in `voice_assistant_es3c28p.ino`.
- Verify speaker codec volume path and sample-rate conversion path.

## Repository Hygiene

To keep pushes GitHub-safe, large generated/reference artifacts are excluded:
- `build/`
- `diagnostics/`
- firmware outputs (`*.bin`, `*.elf`, `*.map`, ...)
- local secrets (`secrets.h`)

If you need to keep vendor packs, store them outside this repo and reference links in docs instead of committing archives.

## Citations / Sources

Hardware and integration decisions were informed by these sources (accessed during project investigation):

1. Freenove FNK0104 product page: https://store.freenove.com/products/fnk0104?srsltid=AfmBOoo9tLaAnFIvsDp7I6YYGcHCNwwpFuMkk1zbFsszZ0aVWed3Zb_m
2. Freenove ESP32-S3 display docs: https://docs.freenove.com/projects/fnk0104/en/latest/fnk0104/codes/MAIN/Freenove_ESP32S3_Display.html
3. Freenove Media Kit / voice examples: https://docs.freenove.com/projects/fnk0102/en/latest/fnk0102/codes/Main/Preface.html
4. Freenove board repository: https://github.com/Freenove/Freenove_ESP32_S3_WROOM_Board
5. Waveshare ESP32-S3 Touch LCD 2.8 wiki: https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-2.8
6. Waveshare ESP32-S3 Touch LCD 2.8-C wiki: https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-2.8-C
7. CYD projects reference collection: https://github.com/bitbank2/CYD_Projects
8. ESP32 SmartDisplay reference code: https://github.com/rzeldent/esp32-smartdisplay
9. Home Assistant community troubleshooting thread: https://community.home-assistant.io/t/mostly-working-config-for-cheap-18-2-8-esp32-s3-touchscreen-with-speaker-and-mic/965950
10. Home Assistant wake-word troubleshooting thread: https://community.home-assistant.io/t/solved-micro-wake-word-not-working-on-esp32-s3-e-g-seed-studio-xiao-sense-esp32-s3-sense/873638
11. ESP32-2432S028 spec mirror PDF: https://github.com/witnessmenow/ESP32-Cheap-Yellow-Display/blob/main/OriginalDocumentation/2-Specification/ESP32-2432S028%20Specifications-EN.pdf

## Licensing

- This project is licensed under the MIT License. See [LICENSE](LICENSE).
- Third-party components, libraries, and documentation remain under their respective licenses/terms. See [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
