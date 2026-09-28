#!/bin/sh
# Wired (UART) flash of the Tab5's ESP32-C6: ESP-Hosted 2.12.13 + ESP-NOW overlay.
# Needed once: the factory C6 firmware (ESP-Hosted 1.4.1) has no OTA partitions,
# so it cannot be updated from the P4 over SDIO.
#
# Usage: ./flash_c6.sh /dev/cu.usbserial-XXXX
#   Connect an ESP32 Downloader (USB-UART) to the Tab5's C6 download header
#   and put the C6 into download mode first (see README.md).
set -e
PORT="${1:?usage: $0 <serial-port>}"
DIR="$(cd "$(dirname "$0")" && pwd)"
ESPTOOL="${ESPTOOL:-esptool.py}"
command -v "$ESPTOOL" >/dev/null 2>&1 || ESPTOOL="python3 -m esptool"

$ESPTOOL --chip esp32c6 -p "$PORT" -b 460800 erase_flash
$ESPTOOL --chip esp32c6 -p "$PORT" -b 460800 --before default_reset --after hard_reset \
  write_flash --flash_mode dio --flash_freq 80m --flash_size 4MB \
  0x0     "$DIR/bootloader.bin" \
  0x8000  "$DIR/partition-table.bin" \
  0xd000  "$DIR/ota_data_initial.bin" \
  0x10000 "$DIR/../network_adapter_esp32c6.bin"
