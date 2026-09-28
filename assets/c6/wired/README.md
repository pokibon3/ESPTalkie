# Tab5 C6: one-time wired flash

The Tab5 ships with ESP-Hosted **1.4.1** on the ESP32-C6. That firmware has no
OTA partitions, so it cannot be updated from the P4 (ESPTalkie reports
"C6 fw too old: wired flash needed"). Flash it once over UART; afterwards the
P4 can update it over SDIO.

## Files

| Offset | File | Origin |
|---|---|---|
| 0x0 | `bootloader.bin` | esp-hosted-mcu v2.12.13 `slave` example, ESP-IDF v5.5.4, esp32c6, 4MB DIO 80MHz |
| 0x8000 | `partition-table.bin` | same build (`partitions.esp32c6.csv`: nvs, otadata, phy_init, ota_0, ota_1) |
| 0xd000 | `ota_data_initial.bin` | same build |
| 0x10000 | `../network_adapter_esp32c6.bin` | ESPHome esp-hosted-firmware v2.12.13 (ESP-NOW overlay) |

License: Apache-2.0 (ESP-IDF / ESP-Hosted: Espressif Systems; overlay: ESPHome).

## Steps

1. Connect an ESP32 Downloader (USB-UART, 3.3 V) to the C6 download header on
   the Tab5 PCB and enter download mode, as in M5Stack's guide:
   https://docs.m5stack.com/en/guide/restore_factory/m5tab5_c6_wifi
2. `pip install esptool` (or use `~/.platformio/penv/bin/esptool.py` via `ESPTOOL=`)
3. `./flash_c6.sh /dev/cu.usbserial-XXXX`
4. Power-cycle the Tab5. ESPTalkie's serial log should show
   `esp-hosted host 2.12.13, C6 2.12.13` and `C6: ESP-NOW overlay present`.

To go back to the factory firmware, use M5Burner as described in the guide above.
