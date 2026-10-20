# catchrobo2026-pc

Catchrobo 2026 PC側 ROS2 ワークスペース。

WiFi経由でNUC (`catchrobo2026-nuc`) と通信し、ブラウザベースのWebGUIでロボットを操作する。
FastAPI + WebSocket サーバーとして動作し、ブラウザ ↔ WebSocket ↔ ROS2 のブリッジを担う。

---

## システム構成

```
catchrobo2026-pc/
├── src/
│   └── catchrobo_pc/               # ROS2 パッケージ
│       └── catchrobo_pc/
│           ├── web_gui_node.py      # FastAPI + WebSocket + ROS2 + Gemini 対話ブリッジノード
│           └── static/             # ブラウザ向け静的ファイル
│               ├── index.html      # メインHTML (6タブGUI)
│               ├── css/
│               │   └── style.css   # グラスモーフィズム ダークテーマ (チャット・マイクUI対応)
│               └── js/
│                   └── app.js      # WebSocket クライアント + 音声認識 + UI ロジック
└── launch/
    └── pc.launch.py                # GUIノード起動ファイル
```

---

## GUI 機能一覧

ブラウザで `http://localhost:8080` を開くと6つのタブが使用可能。

| タブ                 | 機能                                                                                           |
| -------------------- | ---------------------------------------------------------------------------------------------- |
| **接続設定**         | NUCのUSBポートをドロップダウンで選択・適用。CAN/シリアル統計表示。外部コントローラモードON/OFF |
| **アームろう対話**   | Gemini 3.8 Flash によるロボット対話・マイク音声入力・通常/会話モード切替・タイムライン表示     |
| **モジュール制御**   | MDD1 (PID目標値・パラメータ) / SV_1・SV_2 (バルブON/OFF)                                       |
| **ロボマスモーター** | 制御モード選択・6軸スライダー・フィードバック表示                                              |
| **システム監視**     | TX/RX/ERRカウンタ・制御モード状態                                                              |
| **ログ**             | WebSocket通信ログのリアルタイム表示                                                            |

### アームろう音声対話システム (Gemini 3.8 Flash)

PCのWeb GUIからマイクやテキストで話しかけることで、Google Gemini API (`gemini-3.8-flash`) と連携したロボットキャラクター「**アームろう**」とリアルタイムに対話できるシステムです。

#### 1. システムの流れ
1. **マイク/テキスト入力**: ブラウザの Web Speech API（マイクボタン 🎤）またはテキスト入力欄から日本語で入力。
2. **Gemini API 呼び出し (PC側)**: `web_gui_node` が `google.genai` の **Interactions API** を呼び出し、関西弁の返答テキストを生成。
3. **文脈維持**: 前回対話の `interaction_id` を引き継ぐことで、前後の文脈を理解した自然な会話を継続。
4. **音声合成・発話 (NUC側)**: 返答テキストを `/catchrobo/speech_cmd` トピックでNUCへ送信し、Open JTalk で即座に発話。

#### 2. 会話モード切替
画面上部のボタングループまたはサイドバーからモードを切り替え可能:
- **通常モード (`NORMAL`)**:
  - 従来通り、CAN/シリアル接続状態や制御モードの変更アナウンス、および約1分ごとのランダム雑談アナウンスを行います。
- **会話モード (`CHAT`)**:
  - ユーザーとの対話を優先するモード。NUC側のランダム雑談タイマーが自動抑制され、対話の邪魔になりません（通信途絶などの重要警告は通常通り発話されます）。

#### 3. APIキーの設定
以下のいずれかの方法で Gemini API キーを設定します（`web_gui_node` 起動時に自動検出されます）:
```bash
# ~/.bashrc に記載されている場合は自動読み込みされます
export GOOGLE_API_KEY="AIzaSy..."
# または
export GEMINI_API_KEY="AIzaSy..."
```

---

### スマホ用簡易UI (`/mobile`)

スマートフォンでの操作に特化した1画面の軽量UI。
ブラウザで `http://<PCのIPアドレス>:8080/mobile` を開くか、PC版GUIサイドバーの「スマホ接続 QR」からQRコードをスキャンして直接開くことができます。

- **大型クイック操作ボタン**:
  - **外部コントローラー ON/OFF**: ワンタップで外部シリアル制御の有効/無効を切り替え
  - **ロボマス 停止 (STOP - Mode 0)**: ワンタップで即座にロボマスモーターを停止
  - **ロボマス PID制御 (RUN - Mode 1)**: ワンタップでPID制御を開始
  - **全軸 0° リセット**: 6軸すべての目標値を0度に一括リセット
- **1画面デバッグモニタ**:
  - 接続状態インジケーター (WS / CAN / Serial / External)
  - ロボマス6軸 (RM1, RM2, LM1, LM2, SM1, LM3) の現在角度リアルタイム表示
  - 外部コントローラー (MDD1, MDD2) 各エンコーダー角度 (M1〜M4) & スイッチ状態 (S1〜S4)

### 外部コントローラモード

接続設定タブの **「外部コントローラ: ON/OFF」** で有効化できる。
有効時はPC側で `/dev/ttyACM*` を自動探索し、シリアル受信したMDD1およびMDD2データを以下にマッピングして送信する。

**MDD1:**
- `M1(deg)` → `RM2` 目標値 (`/catchrobo/motor_cmd[1]`)
- `M2(deg)` → `RM1` 目標値 (`/catchrobo/motor_cmd[0]`)
- `M3(deg)` > 45.0 のとき `SV_1` の `CH4` (bit3) を ON、45.0 以下で OFF
- `SW1` ON のとき `SV_1` の `CH3` (bit2) を ON
- `SW2` ON のとき `SM1` 電流指令値 `-2000.0` (-2.0A)
- `SW3` ON のとき `SM1` 電流指令値 `+2000.0` (+2.0A)

**MDD2:**
- `M1(deg)` → `LM2` 目標値 (反転, `/catchrobo/motor_cmd[3]`)
- `M2(deg)` → `LM1` 目標値 (反転, `/catchrobo/motor_cmd[2]`)
- `M3(deg)` → `Servo1` ch4 (85〜155度)
- `M4(deg)` → `Servo1` ch3 (0〜180度, 反転)
- `SW1` ON のとき `SV_1` ch1, ch2 ON
- `SW2` ON のとき `Servo1` ch2 が 40度 (OFF時 70度)
- `SW3`, `SW4` で `Servo1` ch5 の角度を連続増減 (0〜180度)

受信パケットは `sample/sample_serial.py` と同じフォーマットを想定:

```text
[0xAA][0x55][DEV_ID][12][deg0L][deg0H]...[deg3L][deg3H][lsw0][lsw1][lsw2][lsw3][XOR]
DEV_ID: 0x01 = MDD1
```

### ポート選択フロー

```
PCブラウザ (ドロップダウン選択)
  ↓ WebSocket
FastAPI (web_gui_node)
  ↓ /catchrobo/set_ports
NUC can_node     → USB-CANに接続
NUC serial_motor_node → シリアルポートに接続
```

NUCが起動していれば `/catchrobo/available_ports` トピックが自動更新され、ドロップダウンに反映される。

---

## ROS2 トピック一覧

### Publish (NUCへ送信)

| トピック                         | 型                  | 内容                                                      |
| -------------------------------- | ------------------- | --------------------------------------------------------- |
| `/catchrobo/motor_cmd`           | `Float32MultiArray` | 6軸目標値 [RM1, RM2, LM1, LM2, SM1, LM3] (degree)         |
| `/catchrobo/motor_mode`          | `String` (JSON)     | 制御モード `{"mode": 0\|1\|2}`                            |
| `/catchrobo/module_cmd`          | `String` (JSON)     | MDD/Solenoid 操作コマンド (下記参照)                      |
| `/catchrobo/set_ports`           | `String` (JSON)     | `{"can_port":"...", "serial_port":"..."}`                 |
| `/catchrobo/external_mdd_status` | `String` (JSON)     | 外部コントローラ生データ (MDD1/MDD2 deg, SW, port, stamp) |
| `/catchrobo/speech_cmd`          | `String`            | Gemini対話返答/直接発話テキスト (NUCで音声再生)           |
| `/catchrobo/sound_mode`          | `String`            | 音声モード切替 (`"NORMAL"` または `"CHAT"`)               |

### Subscribe (NUCから受信)

| トピック                     | 型                  | 内容                                           |
| ---------------------------- | ------------------- | ---------------------------------------------- |
| `/catchrobo/motor_fb`        | `Float32MultiArray` | 角度×6 + RPM×2                                 |
| `/catchrobo/can_status`      | `String` (JSON)     | CAN接続状態・モジュール状態・統計              |
| `/catchrobo/serial_status`   | `String` (JSON)     | シリアル接続状態・統計                         |
| `/catchrobo/available_ports` | `String` (JSON)     | 利用可能ポート一覧                             |
| `/catchrobo/sound_status`    | `String` (JSON)     | サウンドノード状態・現在のモード (`mode` など) |

### `module_cmd` JSON フォーマット

**MDD 目標値送信:**
```json
{"type":"mdd","name":"MDD1","action":"set_target","targets":[0,0,0,0]}
```

**MDD PIDパラメータ更新:**
```json
{"type":"mdd","name":"MDD1","action":"set_params","motor_idx":0,"p":10,"i":0,"d":0,"wheel":65,"mode":0,"dir":1}
```

**MDD パラメータ送信トリガー:**
```json
{"type":"mdd","name":"MDD1","action":"send_params"}
```

**Solenoid バルブ制御:**
```json
{"type":"solenoid","name":"SV_1","action":"set_valves","valves":3}
```

**Servo 目標角度送信:**
```json
{"type":"servo","name":"Servo1","action":"set_target","targets":[90,90,90,90,90,90]}
```

---

## WebSocket メッセージ仕様

### ブラウザ → サーバー

| `cmd` フィールド      | 内容                                                                     |
| --------------------- | ------------------------------------------------------------------------ |
| `motor_cmd`           | `{"cmd":"motor_cmd","targets":[0,0,0,0,0,0]}`                            |
| `motor_mode`          | `{"cmd":"motor_mode","mode":1}`                                          |
| `module_cmd`          | `{"cmd":"module_cmd","payload":{...}}`                                   |
| `set_ports`           | `{"cmd":"set_ports","can_port":"...","serial_port":"..."}`               |
| `ext_ctrl_mode`       | `{"cmd":"ext_ctrl_mode","enabled":true}`                                 |
| `ext_ctrl_get_status` | `{"cmd":"ext_ctrl_get_status"}`                                          |
| `set_sound_mode`      | `{"cmd":"set_sound_mode","mode":"NORMAL"\|"CHAT"}`                       |
| `chat_message`        | `{"cmd":"chat_message","text":"..."}` (Gemini呼び出し＆NUC発話)           |
| `reset_chat`          | `{"cmd":"reset_chat"}` (対話文脈IDのリセット)                            |
| `speak_test`          | `{"cmd":"speak_test","text":"..."}` (任意テキストの直接発話)             |

### 外部シリアル中継トピック (`/catchrobo/external_mdd_status`)

`web_gui_node` が `/dev/ttyACM*` から受信した sample 形式パケットを JSON として配信する。

```json
{
  "device_id": 1,
  "name": "MDD1",
  "port": "/dev/ttyACM0",
  "deg": [0.0, 0.0, 0.0, 0.0],
  "lsw": [0, 0, 0, 0],
  "packet_count": 123,
  "stamp": 1778844006.12
}
```

`debug_node` はこのトピックを購読して表示し、シリアルポートを直接開かない。

### サーバー → ブラウザ

| `type` フィールド     | 内容                                                         |
| --------------------- | ------------------------------------------------------------ |
| `can_status`          | CAN接続状態・モジュール状態                                  |
| `serial_status`       | シリアル接続状態・フィードバック                             |
| `motor_fb`            | モーターフィードバック値                                     |
| `available_ports`     | 利用可能なシリアルポート一覧                                 |
| `external_controller` | 外部コントローラ状態 (enabled/online/port/mdd1_deg/mdd1_lsw) |
| `sound_status`        | サウンドノード状態 (`{"mode":"NORMAL"\|"CHAT",...}`)          |
| `chat_thinking`       | Gemini思考中ステータス通知 (`{"user_text":"..."}`)           |
| `chat_reply`          | アームろう返答通知 (`{"reply_text":"...","interaction_id":"..."}`) |
| `chat_error`          | 対話エラー通知 (`{"message":"..."}`)                         |
| `chat_reset`          | 対話履歴クリア完了通知                                       |

---

## 環境要件

| 項目         | バージョン/内容                   |
| ------------ | --------------------------------- |
| OS           | Ubuntu 22.04 (またはWindows WSL2) |
| ROS2         | Humble Hawksbill                  |
| Python       | 3.10                              |
| fastapi      | `pip3 install fastapi`            |
| uvicorn      | `pip3 install uvicorn`            |
| pyserial     | `pip3 install pyserial`           |
| google-genai | `pip3 install google-genai`       |

**まとめてインストール:**
```bash
pip3 install fastapi uvicorn[standard] pyserial google-genai
```

---

## ビルド & 起動

### 初回ビルド

```bash
cd ~/catchrobo2026-pc
source /opt/ros/humble/setup.bash
colcon build --packages-select catchrobo_pc
source install/setup.bash
```

### 通常起動 (WebGUI + デバッグモニター)

> WezTerm が起動している状態で実行すること。デバッグモニターが自動的に新タブで開く。

```bash
cd ~/catchrobo2026-pc
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=0 ros2 launch catchrobo_pc pc.launch.py
```

起動後、ブラウザで以下にアクセス:

```
http://localhost:8080
```

### ノードを個別に起動する場合

**WebGUI のみ:**
```bash
ROS_DOMAIN_ID=0 ros2 run catchrobo_pc web_gui_node
```

**デバッグモニターのみ (別タブ/別ターミナルで):**
```bash
ROS_DOMAIN_ID=0 ros2 run catchrobo_pc debug_node
```

> `q` キーで終了。

### ソース更新後の再ビルド

```bash
cd ~/catchrobo2026-pc
colcon build --packages-select catchrobo_pc && source install/setup.bash
```

---

## 運用手順

### 基本的な操作フロー

1. **NUCを起動** (`catchrobo2026-nuc` の README 参照)
2. **このPCでlaunchファイルを実行**
3. **ブラウザで `http://localhost:8080` を開く**
4. **「接続設定」タブを開く**
   - `CAN ポート` のドロップダウンからUSB-CANのポートを選択（例: `/dev/ttyUSB0`）
   - `シリアルモーターポート` からロボマスモーターのポートを選択（例: `/dev/ttyACM0`）
   - 「**ポートを適用**」ボタンを押す
   - サイドバーのバッジが `● CAN: Online` に変わることを確認
  - 外部コントローラを使う場合は「**外部コントローラ: ON**」に切り替える
5. **「モジュール制御」タブ** → MDDのパラメータを設定し「パラメータ送信」→ 目標値を操作
6. **「ロボマスモーター」タブ** → モードを `PID制御` に切り替え → スライダーで操作

### 全画面表示

右上の `⛶` ボタンでブラウザを全画面にできます (F11 でも可)。

---

## トラブルシューティング

### NUCからポートリストが届かない

```bash
# NUCのノードが起動しているか確認
ros2 node list
# → /can_node, /serial_motor_node が表示されるはず

# トピックが届いているか確認
ros2 topic echo /catchrobo/available_ports
```

### WebSocketに接続できない

- ブラウザのアドレスが `http://localhost:8080` になっているか確認
- ノードが起動しているか確認: `ros2 node list`
- ポート競合確認: `ss -tlnp | grep 8080`

### NUCとPCでトピックが見えない

```bash
# 両端末で ROS_DOMAIN_ID を統一
export ROS_DOMAIN_ID=0

# 疎通確認
ros2 topic list              # PCで実行してNUCのトピックが見えるか
ping <NUCのIPアドレス>       # WiFi疎通確認
```

### ロボマスモーターが動かない

1. 「ロボマスモーター」タブでモードが `■ 停止` になっていないか確認
2. `⟳ PID制御` を選択してから目標値を動かす
3. シリアルバッジが `● Ser: Online` になっていることを確認

---

## 関連リポジトリ

- [catchrobo2026-nuc](../catchrobo2026-nuc) — NUC側 USB-CAN/シリアル 通信ノード群
- [Altair_module_system_control](../Documents/Altair_module_system_control) — モジュール単体確認ツール (Windows/Chrome向け)
