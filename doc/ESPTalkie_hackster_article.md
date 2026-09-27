# ESP32Talkie: A License-Free Wi-Fi Walkie-Talkie Built on M5StickS3

**Hackster.io metadata (fill these in on the submission form)**

- **Title:** ESP32Talkie: A License-Free Wi-Fi Walkie-Talkie on M5StickS3  
- **Summary (max 140 chars):** A pocket-size, license-free voice transceiver using ESP-NOW Long Range on M5StickS3 — 13 channels, \~1 km range, PTT and shake control.  
- **Difficulty:** Intermediate  
- **Type:** Protip / Showcase  
- **Tags:** m5stack, m5sticks3, esp32-s3, esp-now, walkie-talkie, audio, wireless, platformio  
- **Cover image:** ESP32Talkie.JPG from the repo (two units side by side works great)  
- **Things — Hardware:** M5StickS3 × 2 (or M5AtomS3 \+ Atomic Echo Base × 2\)  
- **Things — Software:** PlatformIO, Arduino framework (espressif32), M5Unified  
- **Code:** [https://github.com/pokibon3/ESPTalkie](https://github.com/pokibon3/ESPTalkie) (MIT License)

---

## Story

### Why build a walkie-talkie in 2026?

Walkie-talkies never really went away. On a hiking trail with no cell coverage, at a maker event inside a noisy hall, or just playing around the neighborhood with kids, a push-to-talk radio is still the fastest way to say "where are you?" The problem is that proper handheld transceivers need a license or registration in many countries, and toy-grade ones sound terrible.

Meanwhile, every ESP32 board already contains a capable 2.4 GHz radio — one that is certified as a Wi-Fi device and therefore license-free to operate. So I asked a simple question: **can a stock M5StickS3, with no extra RF hardware at all, become a real, usable voice transceiver?**

The answer is ESP32Talkie: a pocket-size walkie-talkie that streams live voice over Espressif's ESP-NOW protocol in Long Range mode. No pairing, no router, no internet, no license. Turn on two units, press the button, and talk — up to about 1 km line-of-sight.

### What it does

- **Real-time voice communication** between two or more units over 2.4 GHz Wi-Fi frequencies  
- **13 selectable channels** (2412–2472 MHz, 5 MHz spacing) so multiple groups can operate side by side  
- **Push-to-talk** with the front button, just like a classic transceiver  
- **Live status display**: TX/RX state, channel, volume, RSSI, and a signal/power level bar  
- **Shake-to-adjust UI**: flick the unit sideways to increase a value, vertically to decrease it — handy when your other hand is busy  
- **"Kero-Kero" (frog) voice effect modes** that pitch-shift your voice up for fun (M1 normal / M2 double speed / M3 triple speed)  
- **Settings persistence**: channel, volume, and voice mode survive power cycles  
- Runs entirely on the M5StickS3's built-in microphone, speaker, display, IMU, and battery — **zero external components**

### The key idea: ESP-NOW Long Range as a voice channel

ESP-NOW is Espressif's connectionless 2.4 GHz protocol. Unlike normal Wi-Fi it needs no access point and no association handshake — a device can broadcast frames to any listener on the same channel, with latency in the low milliseconds. That is exactly the semantics of a walkie-talkie: whoever is on your channel hears you.

Two ESP-NOW features make this practical:

1. **Broadcast addressing.** ESP32Talkie transmits to the broadcast MAC, so any number of receivers on the same channel hear the same audio. There is no pairing step at all.  
2. **Long Range (LR) mode.** The ESP32-S3 supports a proprietary PHY rate below 802.11b that trades bandwidth for sensitivity. With LR enabled, receive sensitivity reaches roughly **−98 dBm**, which is what stretches the range to about **1 km line-of-sight** from a \~5 mW/MHz transmitter — all still within license-free Wi-Fi rules. (The M5StickS3's radio operates under Japanese technical conformity certification 219-259730; check your local regulations for equivalents.)

The trade-off of LR mode is throughput, which drives the audio design below.

### Audio pipeline

Voice is captured from the built-in PDM microphone at **16 kHz** and transmitted as **8-bit samples**, keeping the stream at 16 kB/s — comfortably inside the LR-mode budget while still sounding clearly intelligible for speech.

The pipeline looks like this:

\[MIC\] → I2S DMA → gain → 16→8-bit conversion → ESP-NOW packets (magic header "ESPT1")

                                                        ↓  2.4 GHz broadcast

\[ESP-NOW RX\] → packet filter → OutputBuffer (jitter buffer) → 8→16-bit → I2S → \[SPEAKER\]

A few details that mattered in practice:

- **Jitter buffering.** ESP-NOW frames arrive in bursts and occasionally drop. Received PCM goes into a ring `OutputBuffer` that absorbs timing jitter before the I2S speaker task drains it in fixed-size chunks (320 samples per wakeup — large enough to cut task-switching overhead, small enough to keep latency conversational). Version 1.4 focused on stabilizing this playback path and noticeably improved audio quality.  
- **Packet filtering.** Every payload starts with a magic header (`"ESPT1"`). Since ESP-NOW broadcast will happily deliver frames from *any* nearby ESP-NOW device, the receiver drops anything that isn't an ESP32Talkie packet. This eliminated random noise bursts when other ESP-NOW gadgets were around.  
- **Power saving.** The Wi-Fi modem sleeps between activity (added in v1.3), which matters a lot on the M5StickS3's small LiPo battery. Peak consumption is about 1 W while transmitting.

The transport layer builds on the excellent `transport` and `OutputBuffer` classes from [atomic14's esp32-walkie-talkie](https://github.com/atomic14/esp32-walkie-talkie) project, adapted and extended for ESP-NOW LR broadcast, channel switching, and packet filtering.

### The "Kero-Kero" voice modes

Because a walkie-talkie should also be fun: BtnB lets you cycle through three transmit voice modes. M1 is your normal voice; M2 and M3 resample the outgoing audio at 2× and 3× speed, producing a chirpy "frog voice" (kero-kero is the Japanese onomatopoeia for a frog's croak). Kids love it, and it doubles as an instantly recognizable "who's talking" marker between units.

### User interface on a 0.85-inch screen

Fitting a full transceiver UI on the M5StickS3's tiny display took some iteration. The final layout has three zones:

- **Top:** Receive / Transmit status  
- **Middle:** two-digit channel number, volume, and RSSI in rounded panels  
- **Bottom:** a live level bar — incoming SIGNAL strength while receiving, mic POWER while transmitting

Controls follow a two-button-plus-motion scheme:

- **BtnA** — push to talk  
- **BtnB long-press** — cycle the edit target: VOL → CH → MODE (the active target highlights in green, and deselects automatically after 5 s of inactivity)  
- **BtnB click, or a shake** — change the selected value

The shake control uses the built-in IMU. The firmware reads the accelerometer, maps the axes to the current display orientation (so "sideways" is always sideways regardless of how you hold it), and applies a dominance test plus re-arm hysteresis and a 450 ms cooldown so that walking or PTT button presses never trigger false adjustments. A horizontal flick increases the value; a vertical flick decreases it. Adjusting the volume without looking away from what you're doing feels surprisingly natural.

All three settings — channel (1–13), volume (1–5), and voice mode — are written to NVS flash via `Preferences`, so the unit comes back exactly as you left it.

### Firmware structure

The project is a standard PlatformIO project (Arduino framework, espressif32, M5Unified):

- `src/main.cpp` — UI: display layout, button/shake handling, settings persistence  
- `src/Application.cpp` — the core: I2S mic capture, ESP-NOW TX/RX, jitter-buffered playback, pitch-effect processing, status display  
- `lib/transport/` — ESP-NOW transport (LR mode, channel control, magic-header filtering)  
- `lib/audio_output/` — the `OutputBuffer` jitter buffer

Two build environments are provided out of the box:

- `m5stack-sticks3` (default) — M5StickS3, fully self-contained  
- `m5stack-atoms3-echo-base` — M5AtomS3 \+ Atomic Echo Base speaker unit

Board-specific differences (mic gain, speaker driver, volume tables, display layout, which shake axes are active) are cleanly separated behind `TARGET_*` build flags in `config.h`, so porting to another M5Stack controller mostly means adding one more environment.

### Build it yourself

You need two of either supported device and nothing else.

1. Clone the repo:  
     
   git clone https://github.com/pokibon3/ESPTalkie.git  
     
2. Open it in VS Code with the PlatformIO extension.  
3. Pick your environment in `platformio.ini` (`m5stack-sticks3` is the default; switch the `default_envs` comment for AtomS3 \+ Echo Base).  
4. Build and upload to both units.  
5. Power both on, make sure they show the same channel, press BtnA, and talk.

That's the whole setup — no configuration files, no MAC addresses to exchange, no app.

### Specifications

- **Frequency:** 2412–2472 MHz, 13 channels at 5 MHz spacing  
- **Protocol:** ESP-NOW Long Range mode (broadcast)  
- **Emission:** G1D, D1D  
- **TX power:** approx. 5 mW/MHz  
- **RX sensitivity:** approx. −98 dBm  
- **Range:** up to \~1 km line-of-sight  
- **Audio:** 8-bit / 16 kHz sampling, \~1.0 W speaker output  
- **Controller:** M5StickS3 (or M5AtomS3 \+ Atomic Echo Base)  
- **Power:** 3.7 V LiPo, \~1 W max during TX  
- **Licensing:** License-free (operates as certified Wi-Fi equipment)

### Version history

- **v1.0** — initial release  
- **v1.1** — M5AtomS3 \+ Atomic Echo Base support, Kero-Kero voice  
- **v1.2** — M1/M2/M3 voice modes, shake-gesture control  
- **v1.3** — Wi-Fi sleep for lower power consumption  
- **v1.4** — audio quality improvements (stabilized RX playback), packet filtering

### What's next

- Audio codec upgrade (e.g. μ-law or ADPCM) for better quality in the same bandwidth  
- More M5Stack targets — the config layer is already structured for it  
- Group features such as a channel-busy indicator and unit IDs

### Closing thoughts

ESP32Talkie started as a "can it even work?" experiment and ended up as a device my family actually uses. The M5StickS3 turned out to be a remarkable platform for this: microphone, speaker, display, IMU, button, battery, and a long-range-capable radio in one tiny certified package. Everything in this project — the UI, the DSP, the transport — is just software on top of hardware you can buy off the shelf.

The full source is MIT-licensed at [**https://github.com/pokibon3/ESPTalkie**](https://github.com/pokibon3/ESPTalkie). Build two, pick a channel, and go talk to someone — no license required.

### Acknowledgments

The ESP-NOW transport and output-buffer design builds on [atomic14's esp32-walkie-talkie](https://github.com/atomic14/esp32-walkie-talkie) project — thank you for the great foundation.  
