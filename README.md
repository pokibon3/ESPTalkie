# ESP32Talkie：WiFi音声トランシーバ
M5StickS3、M5AtomS3 + Atomic Echo Base、M5Paper Color、M5Stack StopWatch、M5Stack Tab5で動作する、WiFi音声トランシーバのサンプルコードです。
2.4GHz WiFiで音声通信ができるライセンスフリーのトランシーバで、Espressif社のESP-NOWプロトコルを使用します。

![ESP32Talkie](ESP32Talkie.JPG)

## 仕様
- 送受信周波数：2412～2472MHz（5MHz間隔13波）
- 電波形式：G1D, D1D
- 送信出力：約5mW/MHz
- 受信感度：約-98dBm
- 音声出力：約1.0W
- プロトコル：ESP-NOW Long Range Mode（Espressif）
- 通信距離：最大1km（見通し距離）
- 制御マイコン：M5StickS3 / M5AtomS3 + Atomic Echo Base / M5Paper Color / M5Stack StopWatch（いずれもESP32-S3）、M5Stack Tab5（ESP32-P4 + 無線用ESP32-C6）
- 電源：リチウムポリマー電池 3.7V（M5StickS3: 250mAh、M5Paper Color: 1250mAh、M5Stack StopWatch: 450mAh）
- 消費電力：最大約１W（送信時）
- その他：ライセンスフリー
- 工事設計認証番号：219-259730

## ビルド
- PlatformIO用のプロジェクトです。
- `platformio.ini` の `default_envs` でビルド対象を選択します。

| 環境名 | 対象機種 |
|---|---|
| `m5stack-sticks3`（デフォルト） | M5StickS3 |
| `m5stack-atoms3-echo-base` | M5AtomS3 + Atomic Echo Base |
| `m5stack-papercolor` | M5Paper Color |
| `m5stack-stopwatch` | M5Stack StopWatch（466x466 丸型AMOLED） |
| `m5stack-tab5` | M5Stack Tab5（縦持ち、pioarduino / Arduino 3.x） |

- atomic14氏の [ESP32-walkie-talkie](https://github.com/atomic14/esp32-walkie-talkie) プロジェクトから、`transport` クラスおよび `OutputBuffer` クラスを流用・改造して利用しています。

## 使用方法
- 送信音声は8bit 16kHzサンプリングで送受信しています。
- チャンネル・音量・MODEはNVSに保存され、次回起動時に復元されます。

### M5StickS3 / M5AtomS3 + Atomic Echo Base
- 画面表示
  - 上段: `Receive / Transmit` ステータス
  - 中段: チャンネル2桁表示、`VOL` と `RSSI`
  - 下段: レベルバー（受信時 `SIGNAL` / 送信時 `POWER`）
- 操作:
  - `BtnA`: Push to Talk
  - `BtnA` ダブルクリック: 連続送信の開始/停止（ステータスは `CONT TX`）
  - `BtnB` クリック: `VOL/CH/MODE` の現在モード値を変更
  - `BtnB` 長押し: `VOL/CH/MODE` モードを切替
  - 起動直後は `VOL/CH/MODE` のどれも未選択
  - モード選択は「最後のモード切替または値変更」から5秒後に自動でOFF
  - ふりふり操作（IMU）:
    - 横/縦判定は表示向きに合わせて自動補正
    - 横(X)に振る: 値を上げる
    - 縦(Y)に振る: 値を下げる
    - 奥行き(Z)に振る: `VOL/CH/MODE` モードを切替（M5StickS3では無効）
  - M5AtomS3はボタンが1つ（`BtnA`）のため、設定操作はふりふり操作で行います
  - `MODE`:
    - `M1`: 通常音声
    - `M2`: ケロケロボイス（倍速）
    - `M3`: ケロケロボイス（3倍速）

### 送受信の優先制御（全機種共通）

- 送信中に相手の送信を受けると受信に切替（受信優先）
- 相手の送信中にPTTを押すと割り込み送信（相手側は受信に切替わり、PTT押下中/連続送信中なら割り込み終了後に再開）
- PTT押下中または連続送信中は、相手の受信終了後（300ms + 0〜300msのランダム待ち）に送信を自動再開
- 連続送信: PTTを素早く2回押すと開始、もう一度2回押すと停止（タイムアウトなし）

### M5Paper Color

- `platformio.ini`の対象環境は`m5stack-papercolor`です。
- 通常時は400x600の名札画像を表示し、E-Ink画面は送受信中に更新しません。
- LED表示:
  - 左（電源）: 通電中は緑、バッテリー残量20%以下で点滅
  - 右（送受信）: 受信中は青、送信中は赤、連続送信中は赤点滅、エラー時は橙、待受中は消灯
- 操作:
  - 左上 + 左中ボタン同時押し: 名札画面と設定画面を切替
  - 名札画面
    - 上ボタン: Push to Talk
    - 上ボタン ダブルクリック: 連続送信の開始/停止
    - 左上/左中ボタンの単独操作は無効
  - 設定画面（送信不可）
    - 上ボタン: 設定項目を`CHANNEL → VOLUME → VOICE`の順に切替（選択中の項目を赤枠でハイライト）
    - 左上ボタン: 選択中の値を+1
    - 左中ボタン: 選択中の値を-1
    - 値は範囲端で折り返し（CH 1-13、VOL 1-5、VOICE M1-M3）
    - 画面描画中もキー入力を受け付け、最後の操作から1秒後に保存・再描画
- 設定画面には最終受信時のRSSI（`LAST RX`）も表示します。
- 名札画像は起動時にmicroSDカードのルートの`pokibon-transfer.png`（400x600推奨のPNG）を読み込みます。
  - SDカードが無い、ファイルが無い、読み込み/デコードに失敗した場合は、ファームウェアに埋め込んだ`assets/pokibon-transfer.png`を表示します。
  - 画像を差し替えたら再起動してください。

#### ネックストラップケース（3Dプリント）

![PaperColor NekoCase](doc/3d/PaperColor_NekoCase_preview.png)

- M5Paper Colorを縦置きで首から下げるための猫耳ケースです（`doc/3d/`）。
  - `PaperColor_NekoCase.step` / `PaperColor_NekoCase.stl`: ケース本体
  - `PaperColor_NekoCase_build.py`: CadQueryによる生成スクリプト（寸法パラメータ変更可）
  - `PaperColor.stl`: M5Stack公式の本体モデル（[M5_Hardware](https://github.com/m5stack/M5_Hardware/tree/master/Products/C151_PaperColor/Structures)、MIT License, Copyright (c) 2021 M5Stack。ライセンス全文は`doc/3d/LICENSE_M5Stack_M5_Hardware.txt`）
- 本体は上から差し込み、左右のラッチで固定します。猫耳のφ5mm穴にストラップを通します。
- 上面（上ボタン・マイク・LED）は塞がず、側面ボタン・USB-C・Groveポートは開口しています。
- 背面を下にしてサポートなしで印刷できます。

### M5Stack StopWatch
- 丸型AMOLED（466x466）で使用します。
- ボタン
  - 青ボタン（BtnB）: PTT。押している間送信、ダブルタップで連続送信のON/OFF
  - 黄ボタン（BtnA）: 1秒長押しでSETUP画面の開閉（誤操作防止のため、メイン画面では短押しは無効）
- メイン画面
  - 中央に画像（`assets/stopwatch-dog.jpg`、240x240 JPEGをファームに埋め込み）を円形に切り抜いて表示
  - 画像まわりのリングで状態表示: 青=待受、緑=受信中、赤=送信、橙=連続送信（上部に RECEIVE / RECEIVING / TRANSMIT / CONT TX）
  - 左: RSSI（送信中は送信出力 dBm）と8段メーター、右: 電池残量（充電中は CHG）、下: CH / VOL / VOICE
- SETUP画面
  - CHANNEL / VOLUME / VOICE（M1/M2/M3）
  - 黄ボタン短押し: 項目切替、青ボタン短押し: 値＋1、タッチ: [−]/[＋]ボタン・項目行の選択
  - 15秒操作がないと自動でメイン画面に戻る。SETUP中は送信しない（連続送信も停止）
- 着信通知: 無音（1.5秒以上）の後に受信が始まると、振動モーターを「ブブブ」と3回動作（自局送信中は動作しない）
  - 強さ・パターンは `config.h` の `STOPWATCH_VIBRATION_*` で調整、SETUP長押し時間・自動復帰時間は `STOPWATCH_SETUP_*`
- ビルド環境: `espressif32@6.12.0`（Arduino core 2.x）、M5Unified 0.2.23以降（StopWatch対応版）。16MB Flash / 8MB OPI PSRAM
- 画像を差し替える場合は `assets/stopwatch-dog.jpg`（240x240 のJPEG）を置き換えてビルドします

### M5Stack Tab5
- 縦持ち（720x1280）で使用します。
- メイン画面
  - SDカードの `/images/pokibon.jpeg`（または `.jpg`、`config.h` の `TAB5_IMAGE_PATH`）を全画面表示
  - 下部中央: PTTボタン（設定中のチャンネルを表示）、右下: SETUPボタン
  - PTTは押している間送信。指がボタン外に出ると解除。ダブルタップで連続送信のON/OFF。送信中はボタンが TX / CONT TX 表示
- SETUP画面（SETUPで開き、同じ位置の CLOSE で戻る）
  - CHANNEL −/＋、VOLUME −/＋、VOICE M1/M2/M3
  - レベルメーター（受信RSSI／送信出力）。受信が止まって1秒で「---」に戻る
  - チャンネルスキャン（START/STOP）
    - 1周: Wi-Fiアクセスポイントのスキャン（約2秒）→ CH1〜13でESP-NOWを各0.25秒待ち受け（LR・通常モードとも受信）
    - グラフ: 各CHの左の棒がWi-Fi APの最大RSSI（−60dBm以上 赤／−75dBm以上 黄／それ未満 緑、上の数字は台数）、右の水色の棒がESP-NOWの最大RSSI（上の数字は受信フレーム数）。棒は測定ごとに順次更新
    - 見出しに現在の動作（`Wi-Fi AP scan` / `ESP-NOW RX CHn`）、待ち受け中のCHは黄色表示と▼、設定中のCHは枠付き、最も空いているCHは番号が緑
    - STOP、または CLOSE でSETUPを抜けると、実行中の1周を終えてから設定中のチャンネル・LRモードに戻る。スキャン中は送信しない
- 無線（ESP-NOW）
  - ESP32-P4は無線を持たないため、ESP-NOWは内蔵ESP32-C6がesp-hosted（SDIO）経由で送受信します。
  - P4側: `lib/esp_now_hosted` が `esp_now_*` をesp-hostedのCustomRpcへ中継（送信は応答待ちなし）。Tab5ではpromiscuousモードが使えないため、RSSIは受信フレームごとの値を使用
  - C6側: ESPHomeの [esp-hosted-firmware](https://github.com/esphome/esp-hosted-firmware) v2.12.13（ESP-NOWオーバーレイ入り、Apache-2.0）を `assets/c6/` に同梱。P4側（arduino-esp32 3.3.12）のesp-hostedと同じ版
  - 起動時にC6がESP-NOW要求に応答しなければ、同梱ファームをP4からC6へ自動で書き込み、再起動します。出荷時のC6ファーム（esp-hosted 1.4.1）からもこの方法で更新できることを確認済み
  - 自動更新できない場合の予備として、UART接続でC6へ書き込むためのファイル一式を `assets/c6/wired/` に置いています
- ビルド環境: pioarduino（Arduino core 3.x / ESP-IDF 5.5）。初回ビルド時に自動でダウンロードされます
  - 同じPCで従来の `espressif32@6.12.0` と併用して `No module named 'intelhex'` が出た場合は、`~/.platformio/penv/bin/pip install intelhex` で解消します

## バージョン来歴
- v1.0: 新規作成
- v1.1: M5AtomS3 + Atomic Echo Base対応、ケロケロボイス対応
- v1.2: ケロケロボイスモード(M1/M2/M3)追加、ふりふりによるモード/値設定を追加
- v1.3: Wi-Fiをスリープモードにし、消費電力を削減
- v1.4: 音質改善（受信再生の安定化）、パケットフィルタ機能追加
- v1.5: M5Paper Color対応（名札表示・設定画面・LED表示）、連続送信、受信優先/割り込み送信制御、PaperColor用ネックストラップケース
- v1.6: M5Stack Tab5対応（C6経由のESP-NOW、タッチ操作、全画面画像、SETUP画面、チャンネルスキャン）
- v1.7: M5Stack StopWatch対応（丸型AMOLED・円形画像表示、青ボタンPTT、黄ボタン長押しSETUP、着信バイブレーション）

## ライセンス
　MIT License
