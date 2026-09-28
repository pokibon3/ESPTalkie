# ESP32-C6 co-processor firmware for M5Stack Tab5

`network_adapter_esp32c6.bin` is the ESP-Hosted slave firmware **v2.12.13** with the
ESP-NOW overlay, built by ESPHome:

- Source: https://github.com/esphome/esp-hosted-firmware (release `v2.12.13`, ESP-IDF v5.5.5)
- License: Apache-2.0 (ESP-Hosted: Espressif Systems, overlay: ESPHome)
- SHA256: see `network_adapter_esp32c6.bin.sha256`

The Tab5 build embeds this file. At boot, if the C6 does not answer
ESP-NOW-over-CustomRpc requests, the P4 writes it to the C6 via esp-hosted OTA
and restarts. The host side (arduino-esp32 3.3.12) uses esp-hosted 2.12.13, so the
versions match.
