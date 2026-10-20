/**
 * app.js
 * Catchrobo 2026 Web GUI - メインアプリケーションロジック
 *
 * WebSocket経由でweb_gui_node.py (ROS2) と通信し、
 * モジュール制御・ロボマスモーター制御・状態表示を行う
 */

// ─────────────────────────────────────────────────────────
// グローバル状態
// ─────────────────────────────────────────────────────────
const state = {
  ws: null,
  wsConnected: false,

  // モーター目標値 [RM1, RM2, LM1, LM2, SM1, LM3] (degree)
  motorTargets: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  motorMode: 0,           // 0=停止 1=PID 2=開ループ
  motorFb: Array(8).fill(0.0),  // [ang×6, rpm×2]

  // モーター設定
  motorConfig: [
    { name: 'RM1', min: -20.0, max: 85.0 },
    { name: 'RM2', min: -20.0, max: 110.0 },
    { name: 'LM1', min: -20.0, max: 70.0 },
    { name: 'LM2', min: -15.0, max: 90.0 },
    { name: 'SM1', min: -90.0, max: 90.0 },
    { name: 'LM3', min: -10.0, max: 20.0 },
  ],

  // MDD1 状態
  mdd1: {
    motors: [
      { target: 0, mode: 1, p: 10, i: 0, d: 0, wheel: 65, dir: 1 },
      { target: 0, mode: 1, p: 10, i: 0, d: 0, wheel: 65, dir: 1 },
      { target: 0, mode: 1, p: 10, i: 0, d: 0, wheel: 65, dir: 1 },
      { target: 0, mode: 1, p: 10, i: 0, d: 0, wheel: 65, dir: 1 },
    ],
    appMode: 0,
    sw: [0, 0, 0, 0],
    err: 0,
    enc_deg: [0, 0, 0, 0],
    enc_rps: [0, 0, 0, 0],
  },

  // MDD2 状態
  mdd2: {
    motors: [
      { target: 0, mode: 0, p: 10, i: 0, d: 0, wheel: 65, dir: 1 },
      { target: 0, mode: 0, p: 10, i: 0, d: 0, wheel: 65, dir: 1 },
      { target: 0, mode: 0, p: 10, i: 0, d: 0, wheel: 65, dir: 1 },
      { target: 0, mode: 0, p: 10, i: 0, d: 0, wheel: 65, dir: 1 },
    ],
    appMode: 0,
    sw: [0, 0, 0, 0],
    err: 0,
    enc_deg: [0, 0, 0, 0],
    enc_rps: [0, 0, 0, 0],
  },

  // SV_1 / SV_2 バルブ状態
  sv1Valves: 0,
  sv2Valves: 0,

  // Servo1 状態
  servo1Ch: [90, 90, 90, 90, 90, 90],

  // ポート情報
  availablePorts: [],
  canPort: '',
  serialPort: '',

  // CAN/Serial ステータス
  canStatus: {},
  serialStatus: {},

  // 外部コントローラ状態
  externalCtrl: {
    enabled: false,
    online: false,
    port: '',
    last_rx_age: 999,
    packet_count: 0,
    mdd1_deg: [0, 0, 0, 0],
    mdd1_lsw: [0, 0, 0, 0],
    mdd2_deg: [0, 0, 0, 0],
    mdd2_lsw: [0, 0, 0, 0],
  },

  // ネットワーク情報
  networkInfo: null,

  // 音声会話状態
  soundMode: 'NORMAL',
  isRecognizing: false,
};

// ─────────────────────────────────────────────────────────
// WebSocket 接続管理
// ─────────────────────────────────────────────────────────
function connectWs() {
  const wsUrl = `ws://${location.host}/ws`;
  state.ws = new WebSocket(wsUrl);

  state.ws.onopen = () => {
    state.wsConnected = true;
    updateWsIndicator(true);
    addLog('WebSocket 接続', 'success');
    sendWs({ cmd: 'ext_ctrl_get_status' });
  };

  state.ws.onclose = () => {
    state.wsConnected = false;
    updateWsIndicator(false);
    addLog('WebSocket 切断 - 5秒後に再接続...', 'warn');
    setTimeout(connectWs, 5000);
  };

  state.ws.onerror = (e) => {
    addLog('WebSocket エラー', 'error');
  };

  state.ws.onmessage = (e) => {
    try {
      const msg = JSON.parse(e.data);
      handleServerMessage(msg);
    } catch (err) {
      console.error('WS メッセージ解析エラー:', err);
    }
  };
}

function sendWs(payload) {
  if (state.ws && state.wsConnected) {
    state.ws.send(JSON.stringify(payload));
  }
}

// ─────────────────────────────────────────────────────────
// サーバーメッセージ処理
// ─────────────────────────────────────────────────────────
function handleServerMessage(msg) {
  switch (msg.type) {
    case 'can_status':
      state.canStatus = msg.data;
      updateCanStatusUI(msg.data);
      break;
    case 'serial_status':
      state.serialStatus = msg.data;
      updateSerialStatusUI(msg.data);
      break;
    case 'motor_fb':
      state.motorFb = msg.data;
      updateMotorFbUI(msg.data);
      break;
    case 'available_ports':
      state.availablePorts = msg.data.ports || [];
      updatePortSelectors();
      break;
    case 'external_controller':
      state.externalCtrl = msg.data || state.externalCtrl;
      renderExternalControllerUI();
      break;
    case 'network_info':
      updateNetworkInfoUI(msg.data);
      break;
    case 'mdd1_target_sync':
      if (msg.targets) {
        for (let i = 0; i < 4; i++) {
          const el = document.getElementById(`mdd1-target-${i}`);
          if (el) el.value = msg.targets[i];
          if (state.mdd1.motors[i]) state.mdd1.motors[i].target = msg.targets[i];
        }
      }
      break;
    case 'sound_status':
      if (msg.data && msg.data.mode) {
        updateSoundModeUI(msg.data.mode);
      }
      break;
    case 'chat_thinking':
      renderChatThinking(msg.user_text);
      break;
    case 'chat_reply':
      renderChatReply(msg.user_text, msg.reply_text);
      break;
    case 'chat_error':
      renderChatError(msg.message);
      break;
    case 'chat_reset':
      resetChatUI();
      break;
    case 'log':
      if (msg.message) {
        addLog(msg.message, msg.level || 'info');
      }
      break;
  }
}

function setExternalControllerMode(enabled) {
  sendWs({ cmd: 'ext_ctrl_mode', enabled: !!enabled });
}

function toggleExternalControllerMode() {
  const enabled = !state.externalCtrl.enabled;
  setExternalControllerMode(enabled);
  addLog(`外部コントローラモード → ${enabled ? 'ON' : 'OFF'}`, enabled ? 'success' : 'warn');
}

function renderExternalControllerUI() {
  const s = state.externalCtrl;
  const status = document.getElementById('ext-ctrl-status');
  const detail = document.getElementById('ext-ctrl-detail');
  const btn = document.getElementById('btn-ext-ctrl-mode');

  if (status) {
    const mode = s.enabled ? 'MODE: ON' : 'MODE: OFF';
    const link = s.online ? 'ONLINE' : 'OFFLINE';
    status.textContent = `${mode} / ${link}`;
    status.style.color = s.online ? 'var(--success)' : 'var(--danger)';
  }

  if (detail) {
    const p = s.port || '-';
    const age = Number.isFinite(s.last_rx_age) ? s.last_rx_age.toFixed(2) : '999.00';
    detail.textContent = `PORT: ${p}  age:${age}s  pkt:${s.packet_count ?? 0}`;
  }

  // MDD1のデバッグ値をテーブルにセット
  const d1 = s.mdd1_deg || [0, 0, 0, 0];
  const sw1 = s.mdd1_lsw || [0, 0, 0, 0];
  for (let i = 0; i < 4; i++) {
    let val = d1[i];
    if (i === 3) val /= 10.0; // M4はギヤ比 10:1 を考慮
    setEl(`ext-mdd1-m${i+1}`, `${val >= 0 ? '+' : ''}${val.toFixed(1)}°`);
    const swBadge = document.getElementById(`ext-mdd1-sw${i+1}`);
    if (swBadge) {
      const isOn = (sw1[i] === 0);
      swBadge.className = `sw-badge ${isOn ? 'on' : 'off'}`;
    }
  }

  // MDD2のデバッグ値をテーブルにセット
  const d2 = s.mdd2_deg || [0, 0, 0, 0];
  const sw2 = s.mdd2_lsw || [0, 0, 0, 0];
  for (let i = 0; i < 4; i++) {
    // 現在の状態（M1〜M3そのまま、M4反転）を基準にすべて符号を反転する。
    // つまり M1〜M3 は符号を反転し、M4 は符号を反転せずギヤ比 10:1 を考慮して10で割る。
    let val = d2[i];
    if (i === 3) {
      val = val / 10.0;
    } else {
      val = -val;
    }
    setEl(`ext-mdd2-m${i+1}`, `${val >= 0 ? '+' : ''}${val.toFixed(1)}°`);
    const swBadge = document.getElementById(`ext-mdd2-sw${i+1}`);
    if (swBadge) {
      const isOn = (sw2[i] === 0);
      swBadge.className = `sw-badge ${isOn ? 'on' : 'off'}`;
    }
  }

  if (btn) {
    btn.textContent = s.enabled ? '外部コントローラ: ON' : '外部コントローラ: OFF';
    btn.classList.toggle('btn-success', !!s.enabled);
    btn.classList.toggle('btn-ghost', !s.enabled);
  }
}

// ─────────────────────────────────────────────────────────
// CAN ステータス UI 更新
// ─────────────────────────────────────────────────────────
function updateCanStatusUI(data) {
  // サイドバーバッジ
  const badge = document.getElementById('can-badge');
  if (badge) {
    badge.className = 'conn-badge ' + (data.connected ? 'online' : 'offline');
    badge.textContent = data.connected ? `CAN: ${data.port || 'Online'}` : 'CAN: Offline';
  }

  const modules = data.modules || {};
  
  // MDD1 状態
  const mdd1 = modules.MDD1 || {};
  if (Object.keys(mdd1).length) {
    state.mdd1.appMode = mdd1.app_mode || 0;
    state.mdd1.sw = mdd1.sw || [0, 0, 0, 0];
    state.mdd1.err = mdd1.err || 0;
    state.mdd1.enc_deg = mdd1.enc_deg || [0, 0, 0, 0];
    state.mdd1.enc_rps = mdd1.enc_rps || [0, 0, 0, 0];
    renderMddStatus('MDD1');
  }

  // MDD2 状態
  const mdd2 = modules.MDD2 || {};
  if (Object.keys(mdd2).length) {
    state.mdd2.appMode = mdd2.app_mode || 0;
    state.mdd2.sw = mdd2.sw || [0, 0, 0, 0];
    state.mdd2.err = mdd2.err || 0;
    state.mdd2.enc_deg = mdd2.enc_deg || [0, 0, 0, 0];
    state.mdd2.enc_rps = mdd2.enc_rps || [0, 0, 0, 0];
    renderMddStatus('MDD2');
  }

  // 統計
  setEl('stat-can-tx', data.tx_count ?? '-');
  setEl('stat-can-rx', data.rx_count ?? '-');
  setEl('stat-can-err', data.error_count ?? '-');
}

function updateSerialStatusUI(data) {
  const badge = document.getElementById('serial-badge');
  if (badge) {
    badge.className = 'conn-badge ' + (data.connected ? 'online' : 'offline');
    badge.textContent = data.connected ? `Ser: ${data.port || 'Online'}` : 'Ser: Offline';
  }
  setEl('stat-serial-tx', data.tx_count ?? '-');
  setEl('stat-serial-rx', data.rx_count ?? '-');
  setEl('stat-serial-err', data.error_count ?? '-');
  setEl('stat-ctrl-mode', ['STOP', 'PID', 'OpenLoop'][data.control_mode ?? 0]);
}

// ─────────────────────────────────────────────────────────
// モーターフィードバック UI 更新
// ─────────────────────────────────────────────────────────
function updateMotorFbUI(fb) {
  const names = ['RM1', 'RM2', 'LM1', 'LM2', 'SM1', 'LM3'];
  names.forEach((name, i) => {
    const ang = fb[i] ?? 0;
    const rpm = i < 2 ? (fb[6 + i] ?? 0) : null;
    setEl(`fb-ang-${name}`, `${ang >= 0 ? '+' : ''}${ang.toFixed(1)}°`);
    if (rpm !== null) setEl(`fb-rpm-${name}`, `${rpm > 0 ? '+' : ''}${rpm} rpm`);
  });
}

// ─────────────────────────────────────────────────────────
// ポートセレクタ更新
// ─────────────────────────────────────────────────────────
function updatePortSelectors() {
  const ports = state.availablePorts;
  ['sel-can-port', 'sel-serial-port'].forEach(id => {
    const sel = document.getElementById(id);
    if (!sel) return;
    const cur = sel.value;
    sel.innerHTML = '<option value="">-- ポートを選択 --</option>';
    ports.forEach(p => {
      const opt = document.createElement('option');
      opt.value = opt.textContent = p;
      if (p === cur) opt.selected = true;
      sel.appendChild(opt);
    });
  });
}

// ─────────────────────────────────────────────────────────
// ロボマスモーター制御
// ─────────────────────────────────────────────────────────
function setMotorTarget(idx, val) {
  state.motorTargets[idx] = parseFloat(val);
  setEl(`motor-val-${idx}`, val);
  // スライダーと数値入力を同期
  const sl = document.getElementById(`motor-slider-${idx}`);
  const num = document.getElementById(`motor-num-${idx}`);
  if (sl) sl.value = val;
  if (num) num.value = val;
  sendWs({ cmd: 'motor_cmd', targets: [...state.motorTargets] });
}

function resetMotor(idx) {
  setMotorTarget(idx, 0);
}

function resetAllMotors() {
  state.motorTargets.fill(0);
  for (let i = 0; i < 6; i++) {
    const sl = document.getElementById(`motor-slider-${i}`);
    const num = document.getElementById(`motor-num-${i}`);
    if (sl) sl.value = 0;
    if (num) num.value = 0;
    setEl(`motor-val-${i}`, '0.0');
  }
  sendWs({ cmd: 'motor_cmd', targets: [0, 0, 0, 0, 0, 0] });
}

function setMotorMode(mode) {
  state.motorMode = mode;
  document.querySelectorAll('.mode-pill').forEach((el, i) => {
    el.classList.toggle('active', i === mode);
  });
  sendWs({ cmd: 'motor_mode', mode });
  addLog(`モーターモード → ${['停止', 'PID', '開ループ'][mode]}`, 'info');
}

// ─────────────────────────────────────────────────────────
// モジュール制御 (MDD1 / MDD2)
// ─────────────────────────────────────────────────────────
function sendMddTarget(name) {
  const isMdd1 = (name === 'MDD1');
  const prefix = isMdd1 ? 'mdd1' : 'mdd2';
  const mdd = isMdd1 ? state.mdd1 : state.mdd2;

  const targets = [];
  for (let i = 0; i < 4; i++) {
    const el = document.getElementById(`${prefix}-target-${i}`);
    targets.push(el ? parseInt(el.value) || 0 : 0);
    mdd.motors[i].target = targets[i];
  }
  sendWs({
    cmd: 'module_cmd',
    payload: { type: 'mdd', name: name, action: 'set_target', targets },
  });
}

function sendMddParams(name) {
  sendWs({
    cmd: 'module_cmd',
    payload: { type: 'mdd', name: name, action: 'send_params' },
  });
  addLog(`${name} パラメータ送信要求`, 'info');
}

function updateMddParam(name, idx, key, val) {
  const isMdd1 = (name === 'MDD1');
  const mdd = isMdd1 ? state.mdd1 : state.mdd2;
  mdd.motors[idx][key] = parseFloat(val);
  sendWs({
    cmd: 'module_cmd',
    payload: {
      type: 'mdd', name: name, action: 'set_params',
      motor_idx: idx, ...mdd.motors[idx]
    },
  });
}

function renderMddStatus(name) {
  const isMdd1 = (name === 'MDD1');
  const mdd = isMdd1 ? state.mdd1 : state.mdd2;
  const prefix = isMdd1 ? 'mdd1' : 'mdd2';

  setEl(`${prefix}-mode-str`, mdd.appMode === 1 ? 'CTRL MODE' : 'PARAM MODE');
  const modeEl = document.getElementById(`${prefix}-mode-str`);
  if (modeEl) {
    modeEl.style.color = mdd.appMode === 1 ? 'var(--success)' : 'var(--text-dim)';
  }
  setEl(`${prefix}-sw`, `SW:[${mdd.sw.join(',')}]  Err:${mdd.err}`);
  mdd.enc_deg.forEach((v, i) => setEl(`${prefix}-enc-${i}`, `${v >= 0 ? '+' : ''}${v.toFixed(1)}°`));
}

// ─────────────────────────────────────────────────────────
// Solenoid 制御
// ─────────────────────────────────────────────────────────
function toggleValve(svName, idx) {
  if (svName === 'SV_1') {
    state.sv1Valves ^= (1 << idx);
    renderValves('SV_1', state.sv1Valves);
    sendWs({ cmd: 'module_cmd', payload: { type: 'solenoid', name: 'SV_1', action: 'set_valves', valves: state.sv1Valves } });
  } else {
    state.sv2Valves ^= (1 << idx);
    renderValves('SV_2', state.sv2Valves);
    sendWs({ cmd: 'module_cmd', payload: { type: 'solenoid', name: 'SV_2', action: 'set_valves', valves: state.sv2Valves } });
  }
}

function renderValves(svName, bits) {
  for (let i = 0; i < 12; i++) {
    const el = document.getElementById(`valve-${svName}-${i}`);
    if (!el) continue;
    const on = (bits >> i) & 1;
    el.classList.toggle('active', !!on);
    const st = el.querySelector('.v-st');
    if (st) st.textContent = on ? 'ON' : 'OFF';
  }
}

// ─────────────────────────────────────────────────────────
// Servo 制御
// ─────────────────────────────────────────────────────────
function setServoTarget(idx, val) {
  val = parseInt(val);
  state.servo1Ch[idx] = val;
  // スライダーと数値入力を同期
  const sl = document.getElementById(`servo-slider-${idx}`);
  const num = document.getElementById(`servo-num-${idx}`);
  if (sl) sl.value = val;
  if (num) num.value = val;
  sendWs({
    cmd: 'module_cmd',
    payload: { type: 'servo', name: 'Servo1', action: 'set_target', targets: [...state.servo1Ch] }
  });
}

function resetServoSingle(idx) {
  setServoTarget(idx, 90);
}

function resetServo1() {
  state.servo1Ch.fill(90);
  for (let i = 0; i < 6; i++) {
    const sl = document.getElementById(`servo-slider-${i}`);
    const num = document.getElementById(`servo-num-${i}`);
    if (sl) sl.value = 90;
    if (num) num.value = 90;
  }
  sendWs({
    cmd: 'module_cmd',
    payload: { type: 'servo', name: 'Servo1', action: 'set_target', targets: [90, 90, 90, 90, 90, 90] }
  });
}

function renderServo1Status() {
  // 高頻度な同期による引き戻しバグを防ぐため、受信データによる自動同期は無効化しています。
}

// ─────────────────────────────────────────────────────────
// ポート設定送信
// ─────────────────────────────────────────────────────────
function applyPorts() {
  const canPort = document.getElementById('sel-can-port')?.value || '';
  const serialPort = document.getElementById('sel-serial-port')?.value || '';
  if (!canPort && !serialPort) {
    addLog('ポートが選択されていません', 'warn');
    return;
  }
  state.canPort = canPort;
  state.serialPort = serialPort;
  sendWs({ cmd: 'set_ports', can_port: canPort, serial_port: serialPort });
  addLog(`ポート設定送信: CAN=${canPort}  Serial=${serialPort}`, 'info');
}

// ─────────────────────────────────────────────────────────
// ユーティリティ
// ─────────────────────────────────────────────────────────
function setEl(id, text) {
  const el = document.getElementById(id);
  if (el) el.textContent = text;
}

function addLog(msg, type = 'info') {
  const box = document.getElementById('log-box');
  if (!box) return;
  const now = new Date();
  const ts = now.toTimeString().slice(0, 12);
  const line = document.createElement('div');
  line.className = `log-line ${type}`;
  line.textContent = `[${ts}] ${msg}`;
  box.appendChild(line);
  box.scrollTop = box.scrollHeight;
  // 最大500行
  while (box.children.length > 500) box.removeChild(box.firstChild);
}

function updateWsIndicator(connected) {
  const el = document.getElementById('ws-indicator');
  if (!el) return;
  el.className = connected ? 'connected' : 'disconnected';
  el.querySelector('span').textContent = connected ? 'WebSocket 接続中' : 'WebSocket 切断';
}

// ─────────────────────────────────────────────────────────
// ナビゲーション
// ─────────────────────────────────────────────────────────
function switchView(viewId) {
  document.querySelectorAll('.view').forEach(v => v.classList.remove('active'));
  document.querySelectorAll('.nav-item').forEach(b => b.classList.remove('active'));
  const v = document.getElementById(viewId);
  if (v) v.classList.add('active');
  const btn = document.querySelector(`.nav-item[data-view="${viewId}"]`);
  if (btn) btn.classList.add('active');
}

// ─────────────────────────────────────────────────────────
// QRコード & スマホ接続モーダル制御
// ─────────────────────────────────────────────────────────
let qrCodeInstance = null;

async function fetchNetworkInfo() {
  try {
    const res = await fetch('/api/network_info');
    if (res.ok) {
      const data = await res.json();
      state.networkInfo = data;
      updateNetworkInfoUI(data);
    }
  } catch (e) {
    sendWs({ cmd: 'get_network_info' });
  }
}

function getQrEffectiveUrl() {
  const netSelect = document.getElementById('qr-net-select');
  const modeSelect = document.getElementById('qr-ui-mode');
  let baseUrl = (netSelect && netSelect.value) ? netSelect.value : `http://${location.hostname}:8080`;
  const mode = modeSelect ? modeSelect.value : 'mobile';
  
  baseUrl = baseUrl.replace(/\/+$/, '');
  return (mode === 'mobile') ? `${baseUrl}/mobile` : baseUrl;
}

function updateNetworkInfoUI(data) {
  state.networkInfo = data;
  const select = document.getElementById('qr-net-select');
  if (!select) return;

  const interfaces = data.interfaces || [];
  select.innerHTML = '';

  if (interfaces.length === 0) {
    select.innerHTML = '<option value="http://localhost:8080">ローカルホスト (http://localhost:8080)</option>';
  } else {
    interfaces.forEach(iface => {
      const opt = document.createElement('option');
      opt.value = iface.url;
      opt.textContent = `${iface.name} - ${iface.url}`;
      select.appendChild(opt);
    });
  }

  onQrNetworkChange();
}

function onQrUiModeChange() {
  onQrNetworkChange();
}

function onQrNetworkChange() {
  const targetUrl = getQrEffectiveUrl();
  const urlInput = document.getElementById('qr-url-input');
  if (urlInput) urlInput.value = targetUrl;
  renderQrCode(targetUrl);
}

function renderQrCode(url) {
  const box = document.getElementById('qrcode-box');
  if (!box) return;
  box.innerHTML = '';
  try {
    if (typeof QRCode !== 'undefined') {
      qrCodeInstance = new QRCode(box, {
        text: url,
        width: 200,
        height: 200,
        colorDark: '#0f172a',
        colorLight: '#ffffff',
        correctLevel: QRCode.CorrectLevel.H
      });
    }
  } catch (e) {
    console.error('QRコード描画エラー:', e);
  }
}

function openQrModal() {
  const modal = document.getElementById('qr-modal');
  if (modal) {
    modal.style.display = 'flex';
    fetchNetworkInfo();
  }
}

function closeQrModal() {
  const modal = document.getElementById('qr-modal');
  if (modal) {
    modal.style.display = 'none';
  }
}

function copyQrUrl() {
  const input = document.getElementById('qr-url-input');
  if (!input) return;
  input.select();
  navigator.clipboard.writeText(input.value).then(() => {
    addLog(`コピー完了: ${input.value}`, 'success');
  }).catch(() => {
    document.execCommand('copy');
    addLog(`コピー完了: ${input.value}`, 'success');
  });
}

// 全画面切り替え
function toggleFullscreen() {
  if (!document.fullscreenElement) {
    document.documentElement.requestFullscreen();
  } else {
    document.exitFullscreen();
  }
}

// ─────────────────────────────────────────────────────────
// 初期化
// ─────────────────────────────────────────────────────────
document.addEventListener('DOMContentLoaded', () => {
  // WebSocket 接続
  connectWs();

  // ネットワーク情報の初回読み込み
  fetchNetworkInfo();

  // ナビゲーションボタン
  document.querySelectorAll('.nav-item').forEach(btn => {
    btn.addEventListener('click', () => switchView(btn.dataset.view));
  });

  // 全画面ボタン
  document.getElementById('btn-fullscreen')?.addEventListener('click', toggleFullscreen);

  // 初期ビュー
  switchView('view-connection');

  addLog('Catchrobo 2026 WebGUI 起動', 'success');
});

// ─────────────────────────────────────────────────────────
// アームろう 音声対話 & マイク認識ロジック
// ─────────────────────────────────────────────────────────
let speechRecognitionInstance = null;

function setSoundMode(mode) {
  mode = (mode || 'NORMAL').toUpperCase();
  state.soundMode = mode;
  updateSoundModeUI(mode);
  sendWs({ cmd: 'set_sound_mode', mode: mode });
  addLog(`音声モード切り替え → ${mode === 'CHAT' ? '会話モード' : '通常モード'}`, 'info');
}

function updateSoundModeUI(mode) {
  state.soundMode = mode;
  const isChat = (mode === 'CHAT');

  // サイドバーのバッジ
  const badge = document.getElementById('sound-badge');
  if (badge) {
    if (isChat) {
      badge.textContent = '音声: 会話中 (アームろう)';
      badge.style.background = 'rgba(236,72,153,0.18)';
      badge.style.color = '#f472b6';
      badge.style.borderColor = 'rgba(236,72,153,0.4)';
    } else {
      badge.textContent = '音声: 通常モード';
      badge.style.background = 'rgba(59,130,246,0.15)';
      badge.style.color = '#60a5fa';
      badge.style.borderColor = 'rgba(59,130,246,0.3)';
    }
  }

  // view-chat のボタン
  const btnNormal = document.getElementById('btn-sound-normal');
  const btnChat = document.getElementById('btn-sound-chat');
  if (btnNormal && btnChat) {
    if (isChat) {
      btnNormal.classList.remove('active');
      btnChat.classList.add('active');
    } else {
      btnNormal.classList.add('active');
      btnChat.classList.remove('active');
    }
  }

  // アームろうカードのタグ
  const tag = document.getElementById('chat-mode-status-tag');
  if (tag) {
    if (isChat) {
      tag.textContent = '会話モード動作中';
      tag.className = 'armro-mode-tag tag-chat';
    } else {
      tag.textContent = '通常モード動作中';
      tag.className = 'armro-mode-tag tag-normal';
    }
  }
}

function sendChatMessage(text) {
  const input = document.getElementById('chat-input-text');
  const message = (text !== undefined ? text : (input ? input.value : '')).trim();
  if (!message) return;

  if (input) input.value = '';

  // 会話モードでなければ自動的に会話モードに切り替え
  if (state.soundMode !== 'CHAT') {
    setSoundMode('CHAT');
  }

  // ユーザーのメッセージを画面に追加
  appendUserMessage(message);

  // 思考中プレースホルダーを表示
  showThinkingIndicator();

  // WebSocket経由でPCノードへ送信 (Gemini呼び出し＆NUC発話へ)
  sendWs({ cmd: 'chat_message', text: message });
}

function sendDirectSpeech() {
  const input = document.getElementById('chat-input-text');
  const text = (input ? input.value : '').trim();
  if (!text) {
    addLog('発話するテキストを入力してください', 'warn');
    return;
  }
  sendWs({ cmd: 'speak_test', text: text });
  addLog(`直接発話送信: "${text}"`, 'info');
}

function clearChatHistory() {
  sendWs({ cmd: 'reset_chat' });
}

function resetChatUI() {
  const container = document.getElementById('chat-messages-container');
  if (container) {
    container.innerHTML = `
      <div class="chat-msg msg-armro">
        <div class="msg-avatar"><i class="fa-solid fa-robot"></i></div>
        <div class="msg-body">
          <div class="msg-sender">アームろう</div>
          <div class="msg-text">会話の記憶をリセットしたで！新しいお話、いつでもどうぞやで！</div>
        </div>
      </div>
    `;
  }
  addLog('会話履歴をリセットしました', 'info');
}

function appendUserMessage(text) {
  const container = document.getElementById('chat-messages-container');
  if (!container) return;

  // 思考中メッセージがあれば除去
  removeThinkingIndicator();

  const msgDiv = document.createElement('div');
  msgDiv.className = 'chat-msg msg-user';
  msgDiv.innerHTML = `
    <div class="msg-avatar"><i class="fa-solid fa-user"></i></div>
    <div class="msg-body">
      <div class="msg-sender">あなた</div>
      <div class="msg-text">${escapeHtml(text)}</div>
    </div>
  `;
  container.appendChild(msgDiv);
  container.scrollTop = container.scrollHeight;
}

function showThinkingIndicator() {
  removeThinkingIndicator();
  const container = document.getElementById('chat-messages-container');
  if (!container) return;

  const thinkingDiv = document.createElement('div');
  thinkingDiv.id = 'chat-thinking-msg';
  thinkingDiv.className = 'chat-msg msg-armro msg-thinking';
  thinkingDiv.innerHTML = `
    <div class="msg-avatar"><i class="fa-solid fa-robot"></i></div>
    <div class="msg-body">
      <div class="msg-sender">アームろう</div>
      <div class="msg-text">
        考え中やで...
        <span class="dot-pulse"><span></span><span></span><span></span></span>
      </div>
    </div>
  `;
  container.appendChild(thinkingDiv);
  container.scrollTop = container.scrollHeight;
}

function removeThinkingIndicator() {
  const el = document.getElementById('chat-thinking-msg');
  if (el) el.remove();
}

function renderChatThinking(userText) {
  showThinkingIndicator();
}

function renderChatReply(userText, replyText) {
  removeThinkingIndicator();
  const container = document.getElementById('chat-messages-container');
  if (!container) return;

  const msgDiv = document.createElement('div');
  msgDiv.className = 'chat-msg msg-armro';
  msgDiv.innerHTML = `
    <div class="msg-avatar"><i class="fa-solid fa-robot"></i></div>
    <div class="msg-body">
      <div class="msg-sender">アームろう</div>
      <div class="msg-text">${escapeHtml(replyText)}</div>
    </div>
  `;
  container.appendChild(msgDiv);
  container.scrollTop = container.scrollHeight;
}

function renderChatError(errMsg) {
  removeThinkingIndicator();
  const container = document.getElementById('chat-messages-container');
  if (!container) return;

  const msgDiv = document.createElement('div');
  msgDiv.className = 'chat-msg msg-armro';
  msgDiv.innerHTML = `
    <div class="msg-avatar" style="background:var(--danger)"><i class="fa-solid fa-triangle-exclamation"></i></div>
    <div class="msg-body">
      <div class="msg-sender" style="color:var(--danger)">システムエラー</div>
      <div class="msg-text" style="border-color:var(--danger);color:#fca5a5;">${escapeHtml(errMsg)}</div>
    </div>
  `;
  container.appendChild(msgDiv);
  container.scrollTop = container.scrollHeight;
}

function escapeHtml(str) {
  if (!str) return '';
  return str.replace(/[&<>"']/g, function(m) {
    return {
      '&': '&amp;',
      '<': '&lt;',
      '>': '&gt;',
      '"': '&quot;',
      "'": '&#39;'
    }[m];
  });
}

// ─────────────────────────────────────────────────────────
// Web Speech API マイク音声認識
// ─────────────────────────────────────────────────────────
function toggleSpeechRecognition() {
  if (state.isRecognizing) {
    stopSpeechRecognition();
  } else {
    startSpeechRecognition();
  }
}

function startSpeechRecognition() {
  const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
  if (!SpeechRecognition) {
    addLog('お使いのブラウザはWeb Speech API（マイク音声認識）に対応していません。Google ChromeやEdgeを推奨します。', 'danger');
    alert('お使いのブラウザは音声認識に対応していません。テキスト入力欄から話しかけてください。');
    return;
  }

  try {
    if (speechRecognitionInstance) {
      speechRecognitionInstance.abort();
    }

    const recognition = new SpeechRecognition();
    speechRecognitionInstance = recognition;
    recognition.lang = 'ja-JP';
    recognition.continuous = false;
    recognition.interimResults = true;
    recognition.maxAlternatives = 1;

    let finalTranscript = '';

    recognition.onstart = () => {
      state.isRecognizing = true;
      const btnMic = document.getElementById('btn-mic');
      if (btnMic) btnMic.classList.add('recording');

      const statusBar = document.getElementById('speech-recognition-bar');
      const statusText = document.getElementById('speech-recognition-status-text');
      if (statusBar) statusBar.style.display = 'flex';
      if (statusText) statusText.textContent = '聞いてるで！話しかけてな...';
    };

    recognition.onresult = (event) => {
      let interimTranscript = '';
      for (let i = event.resultIndex; i < event.results.length; ++i) {
        if (event.results[i].isFinal) {
          finalTranscript += event.results[i][0].transcript;
        } else {
          interimTranscript += event.results[i][0].transcript;
        }
      }

      const input = document.getElementById('chat-input-text');
      const statusText = document.getElementById('speech-recognition-status-text');
      const currentText = finalTranscript || interimTranscript;

      if (input && currentText) {
        input.value = currentText;
      }
      if (statusText && currentText) {
        statusText.textContent = `認識中: "${currentText}"`;
      }
    };

    recognition.onerror = (event) => {
      console.warn('音声認識エラー:', event.error);
      if (event.error === 'not-allowed') {
        addLog('マイクへのアクセスが許可されていません。ブラウザのアドレスバーからマイク許可を確認してください。', 'danger');
      } else if (event.error !== 'no-speech') {
        addLog(`音声認識エラー: ${event.error}`, 'warn');
      }
      stopSpeechRecognition();
    };

    recognition.onend = () => {
      stopSpeechRecognition();
      if (finalTranscript.trim()) {
        sendChatMessage(finalTranscript.trim());
      }
    };

    recognition.start();
  } catch (err) {
    console.error('マイク起動例外:', err);
    addLog(`マイク起動に失敗しました: ${err.message}`, 'danger');
    stopSpeechRecognition();
  }
}

function stopSpeechRecognition() {
  state.isRecognizing = false;
  if (speechRecognitionInstance) {
    try {
      speechRecognitionInstance.stop();
    } catch (e) {}
    speechRecognitionInstance = null;
  }

  const btnMic = document.getElementById('btn-mic');
  if (btnMic) btnMic.classList.remove('recording');

  const statusBar = document.getElementById('speech-recognition-bar');
  if (statusBar) statusBar.style.display = 'none';
}
