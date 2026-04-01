#line 1 "C:\\Users\\Justin\\Documents\\Arduino\\voice_assistant_es3c28p\\THIRD_PARTY_NOTICES.md"
# Third-Party Notices

This project depends on third-party libraries/services and references vendor documentation.

## Software Dependencies

1. ESP32 Arduino Core
   - Upstream: https://github.com/espressif/arduino-esp32
   - License: See upstream `LICENSE` (LGPL-2.1 in upstream repository).

2. TFT_eSPI (Bodmer)
   - Upstream: https://github.com/Bodmer/TFT_eSPI
   - License: See upstream `LICENSE`.

3. LVGL
   - Upstream: https://github.com/lvgl/lvgl
   - License: MIT.

4. ArduinoJson
   - Upstream: https://github.com/bblanchon/ArduinoJson
   - License: MIT.

5. Home Assistant APIs (STT/Conversation/TTS endpoints)
   - Upstream: https://github.com/home-assistant/core
   - License: See upstream license and terms.

## Hardware Documentation and Community References

The following external materials were used for board bring-up, pin mapping, and troubleshooting context. They are not redistributed by this repository and remain property of their respective owners:

- https://store.freenove.com/products/fnk0104?srsltid=AfmBOoo9tLaAnFIvsDp7I6YYGcHCNwwpFuMkk1zbFsszZ0aVWed3Zb_m
- https://docs.freenove.com/projects/fnk0104/en/latest/fnk0104/codes/MAIN/Freenove_ESP32S3_Display.html
- https://docs.freenove.com/projects/fnk0102/en/latest/fnk0102/codes/Main/Preface.html
- https://github.com/Freenove/Freenove_ESP32_S3_WROOM_Board
- https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-2.8
- https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-2.8-C
- https://github.com/bitbank2/CYD_Projects
- https://github.com/rzeldent/esp32-smartdisplay
- https://community.home-assistant.io/t/mostly-working-config-for-cheap-18-2-8-esp32-s3-touchscreen-with-speaker-and-mic/965950
- https://community.home-assistant.io/t/solved-micro-wake-word-not-working-on-esp32-s3-e-g-seed-studio-xiao-sense-esp32-s3-sense/873638
- https://github.com/witnessmenow/ESP32-Cheap-Yellow-Display/blob/main/OriginalDocumentation/2-Specification/ESP32-2432S028%20Specifications-EN.pdf

## Notes

- If you add bundled third-party code to this repository later, include the original license text in-tree and update this notice.
- If you redistribute vendor binaries/datasheets, verify you are permitted to do so under their terms.