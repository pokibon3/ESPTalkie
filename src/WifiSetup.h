#pragma once

// Wi-Fi credential entry from a phone (M5Stack StopWatch, SETUP > WIFI).
//   1. The watch starts its own access point and shows a QR code; scanning it
//      joins the phone to that AP.
//   2. The phone opens the setup page (captive portal, or the second QR code /
//      http://192.168.4.1) and sends the home AP name and password.
//   3. The credentials are saved in NVS (used for NTP time sync).
// Blocks until saved, cancelled (yellow button) or timed out. ESP-NOW channel
// and LR mode are restored afterwards. Returns true if saved.
bool wifi_setup_run(int restore_channel);
