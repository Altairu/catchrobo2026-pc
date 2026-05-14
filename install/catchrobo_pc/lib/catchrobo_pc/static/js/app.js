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

  // モーター目標値 [RM1, RM2, LM1, LM2, SM1] (degree)
  motorTargets: [0.0, 0.0, 0.0, 0.0, 0.0],
  motorMode: 0,           // 0=停止 1=PID 2=開ループ
  motorFb: Array(7).fill(0.0),  // [ang×5, rpm×2]

  // モーター設定
  motorConfig: [
    { name: 'RM1', min: -20.0, max: 60.0 },
    { name: 'RM2', min: -15.0, max: 90.0 },
    { name: 'LM1', min: -20.0, max: 30.0 },
    { name: 'LM2', min: -10.0, max: 20.0 },
    { name: 'SM1', min: -90.0, max: 90.0 },
  ],

  // MDD1 状態
  mdd1: {
    motors: [
      { target: 0, mode: 0, p: 10, i: 0, d: 0, wheel: 65, dir: 1 },
      { target: 0, mode: 0, p: 10, i: 0, d: 0, wheel: 65, dir: 1 },
      { target: 0, mode: 0, p: 10, i: 0, d: 0, wheel: 65, dir: 1 },
      { target: 0, mode: 0, p: 10, i: 0, d: 0, wheel: 65, dir: 1 },
    ],
    appMode: 0,
    sw: [0,0,0,0],
    err: 0,
    enc_deg: [0,0,0,0],
    enc_rps: [0,0,0,0],
  },

  // SV_1 / SV_2 バルブ状態
  sv1Valves: 0,
  sv2Valves: 0,

  // ポート情報
  availablePorts: [],
  canPort: '',
  serialPort: '',

  // CAN/Serial ステータス
  canStatus: {},
  serialStatus: {},
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

  // MDD1 状態
  const modules = data.modules || {};
  const mdd = modules.MDD1 || {};
  if (Object.keys(mdd).length) {
    state.mdd1.appMode  = mdd.app_mode || 0;
    state.mdd1.sw       = mdd.sw || [0,0,0,0];
    state.mdd1.err      = mdd.err || 0;
    state.mdd1.enc_deg  = mdd.enc_deg || [0,0,0,0];
    state.mdd1.enc_rps  = mdd.enc_rps || [0,0,0,0];
    renderMdd1Status();
  }

  // 統計
  setEl('stat-can-tx',  data.tx_count ?? '-');
  setEl('stat-can-rx',  data.rx_count ?? '-');
  setEl('stat-can-err', data.error_count ?? '-');
}

function updateSerialStatusUI(data) {
  const badge = document.getElementById('serial-badge');
  if (badge) {
    badge.className = 'conn-badge ' + (data.connected ? 'online' : 'offline');
    badge.textContent = data.connected ? `Ser: ${data.port || 'Online'}` : 'Ser: Offline';
  }
  setEl('stat-serial-tx',  data.tx_count ?? '-');
  setEl('stat-serial-rx',  data.rx_count ?? '-');
  setEl('stat-serial-err', data.error_count ?? '-');
  setEl('stat-ctrl-mode',  ['STOP','PID','OpenLoop'][data.control_mode ?? 0]);
}

// ─────────────────────────────────────────────────────────
// モーターフィードバック UI 更新
// ─────────────────────────────────────────────────────────
function updateMotorFbUI(fb) {
  const names = ['RM1','RM2','LM1','LM2','SM1'];
  names.forEach((name, i) => {
    const ang = fb[i] ?? 0;
    const rpm = i < 2 ? (fb[5+i] ?? 0) : null;
    setEl(`fb-ang-${name}`, `${ang >= 0 ? '+' : ''}${ang.toFixed(1)}°`);
    if (rpm !== null) setEl(`fb-rpm-${name}`, `${rpm > 0 ? '+' : ''}${rpm} rpm`);
  });
}

// ─────────────────────────────────────────────────────────
// ポートセレクタ更新
// ─────────────────────────────────────────────────────────
function updatePortSelectors() {
  const ports = state.availablePorts;
  ['sel-can-port','sel-serial-port'].forEach(id => {
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
  if (sl)  sl.value  = val;
  if (num) num.value = val;
  sendWs({ cmd: 'motor_cmd', targets: [...state.motorTargets] });
}

function resetMotor(idx) {
  setMotorTarget(idx, 0);
}

function resetAllMotors() {
  state.motorTargets.fill(0);
  for (let i = 0; i < 5; i++) {
    const sl = document.getElementById(`motor-slider-${i}`);
    const num = document.getElementById(`motor-num-${i}`);
    if (sl)  sl.value  = 0;
    if (num) num.value = 0;
    setEl(`motor-val-${i}`, '0.0');
  }
  sendWs({ cmd: 'motor_cmd', targets: [0,0,0,0,0] });
}

function setMotorMode(mode) {
  state.motorMode = mode;
  document.querySelectorAll('.mode-pill').forEach((el, i) => {
    el.classList.toggle('active', i === mode);
  });
  sendWs({ cmd: 'motor_mode', mode });
  addLog(`モーターモード → ${['停止','PID','開ループ'][mode]}`, 'info');
}

// ─────────────────────────────────────────────────────────
// モジュール制御 (MDD1)
// ─────────────────────────────────────────────────────────
function sendMddTarget() {
  const targets = [];
  for (let i = 0; i < 4; i++) {
    const el = document.getElementById(`mdd-target-${i}`);
    targets.push(el ? parseInt(el.value) || 0 : 0);
    state.mdd1.motors[i].target = targets[i];
  }
  sendWs({
    cmd: 'module_cmd',
    payload: { type: 'mdd', name: 'MDD1', action: 'set_target', targets },
  });
}

function sendMddParams() {
  sendWs({
    cmd: 'module_cmd',
    payload: { type: 'mdd', name: 'MDD1', action: 'send_params' },
  });
  addLog('MDD1 パラメータ送信要求', 'info');
}

function updateMddParam(idx, key, val) {
  state.mdd1.motors[idx][key] = parseFloat(val);
  sendWs({
    cmd: 'module_cmd',
    payload: { type: 'mdd', name: 'MDD1', action: 'set_params',
               motor_idx: idx, ...state.mdd1.motors[idx] },
  });
}

function renderMdd1Status() {
  const mdd = state.mdd1;
  setEl('mdd1-mode-str', mdd.appMode === 1 ? 'CTRL MODE' : 'PARAM MODE');
  const modeEl = document.getElementById('mdd1-mode-str');
  if (modeEl) {
    modeEl.style.color = mdd.appMode === 1 ? 'var(--success)' : 'var(--text-dim)';
  }
  setEl('mdd1-sw', `SW:[${mdd.sw.join(',')}]  Err:${mdd.err}`);
  mdd.enc_deg.forEach((v, i) => setEl(`mdd-enc-${i}`, `${v >= 0 ? '+' : ''}${v.toFixed(1)}°`));
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
// ポート設定送信
// ─────────────────────────────────────────────────────────
function applyPorts() {
  const canPort    = document.getElementById('sel-can-port')?.value    || '';
  const serialPort = document.getElementById('sel-serial-port')?.value || '';
  if (!canPort && !serialPort) {
    addLog('ポートが選択されていません', 'warn');
    return;
  }
  state.canPort    = canPort;
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
  const ts  = now.toTimeString().slice(0, 12);
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
