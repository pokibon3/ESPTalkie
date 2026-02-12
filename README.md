# ESP32Talkie：WiFi音声トランシーバ
ESP32系ボードで動作する、WiFi音声トランシーバのサンプルコードです。
2.4GHz WiFiで音声通信ができるライセンスフリーのトランシーバで、Espressif社のESP-NOWプロトコルを使用します。

![AlgyanTalkie](AlgyanTalkie.jpg)

## 諸元
- 送受信周波数：2412～2472MHz（5MHz間隔13波）
- 電波形式：G1D, D1D
- 送信出力：4.7mW/MHz
- 受信感度：約-90dBm
- 音声出力：約0.2W
- プロトコル：ESP-NOW Long Range Mode（Espressif）
- 通信距離：最大1km（見通し距離）
- 制御マイコン：XIAO ESP32S3
- 電源：リチウムポリマー電池 3.7V, 500mA
- 消費電流：最大120mA（送信時）
- その他：ライセンスフリー
- 工事設計認証番号：217-230892

## その他
- PlatformIO用のプロジェクトです。
- atomic14氏の [ESP32-walkie-talkie](https://github.com/atomic14/esp32-walkie-talkie) プロジェクトから、`transport` クラスおよび `OutputBuffer` クラスを改造して利用しています。

※ XIAO ESP32「S3」用です。「C3」ではうまく動作しません。

## 現行実装との差分（追記）
- 現在の対象ボードは `M5StickS3 (m5stack-stamps3)` です。
- 送信音声は `M5.Mic.record(uint8_t*)` の8bitデータを送信しています。
- 画面は縦向きレイアウトです。
  - 上段: `Receive / Transmit` ステータス
  - 中段: チャンネル2桁表示、`VOL` と `RSSI`
  - 下段: レベルバー（受信時 `SIGNAL` / 送信時 `POWER`）
- 操作:
  - `BtnA`: 押下中に送信
  - `BtnB` クリック: `VOL/CH` の現在モード値を変更
  - `BtnB` 長押し: `VOL/CH` モードを切替
- 設定は `src/config.h` を参照してください。
