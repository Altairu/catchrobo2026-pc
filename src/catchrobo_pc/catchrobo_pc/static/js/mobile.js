/**
 * mobile.js
 * Catchrobo 2026 スマホ用簡易操作・デバッグUI ロジック
 */

const mobileState = {
  ws: null,
  wsConnected: false,
  canConnected: false,
  serialConnected: false,

  // ロボマスモーター状態
  motorMode: 0, // 0=停止, 1=PID, 2=開ループ
  motorFb: [0, 0, 0, 0, 0, 0],

  // 外部コントローラー状態
  extCtrl: {
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
};

// ─────────────────────────────────────────────────────────
// WebSocket 接続管理
// ─────────────────────────────────────────────────────────
function connectWs() {
  const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
  const wsUrl = `${protocol}//${location.host}/ws`;
  mobileState.ws = new WebSocket(wsUrl);

  mobileState.ws.onopen = () => {
    mobileState.wsConnected = true;
    updateIndicator('stat-ws', true);
    showToast('サーバーに接続しました');
    sendWs({ cmd: 'ext_ctrl_get_status' });
  };

  mobileState.ws.onclose = () => {
    mobileState.wsConnected = false;
    updateIndicator('stat-ws', false);
    updateIndicator('stat-can', false);
    updateIndicator('stat-ser', false);
    updateIndicator('stat-ext', false);
    setTimeout(connectWs, 3000);
  };

  mobileState.ws.onerror = (e) => {
    console.error('WebSocket エラー:', e);
  };

  mobileState.ws.onmessage = (e) => {
    try {
      const msg = JSON.parse(e.data);
      handleWsMessage(msg);
    } catch (err) {
      console.error('WS 解析エラー:', err);
    }
  };
}

function sendWs(payload) {
  if (mobileState.ws && mobileState.wsConnected) {
    mobileState.ws.send(JSON.stringify(payload));
  }
}

function vibrate(ms = 25) {
  if (navigator.vibrate) {
    try {
      navigator.vibrate(ms);
    } catch (_) {}
  }
}

// ─────────────────────────────────────────────────────────
// メッセージハンドラ
// ─────────────────────────────────────────────────────────
function handleWsMessage(msg) {
  switch (msg.type) {
    case 'can_status':
      mobileState.canConnected = !!msg.data.connected;
      updateIndicator('stat-can', mobileState.canConnected);
      break;

    case 'serial_status':
      mobileState.serialConnected = !!msg.data.connected;
      updateIndicator('stat-ser', mobileState.serialConnected);
      if (typeof msg.data.control_mode === 'number') {
        updateModeUI(msg.data.control_mode);
      }
      break;

    case 'external_controller':
      mobileState.extCtrl = msg.data || mobileState.extCtrl;
      renderExtControllerUI();
      break;

    case 'motor_fb':
      mobileState.motorFb = msg.data || [];
      renderMotorFbUI();
      break;
  }
}

// ─────────────────────────────────────────────────────────
// UI更新関数
// ─────────────────────────────────────────────────────────
function updateIndicator(id, online) {
  const el = document.getElementById(id);
  if (!el) return;
  el.classList.toggle('online', !!online);
  el.classList.toggle('offline', !online);
}

// 外部コントローラー UI
function renderExtControllerUI() {
  const s = mobileState.extCtrl;
  updateIndicator('stat-ext', s.online);

  const btn = document.getElementById('btn-ext-ctrl');
  const btnLabel = document.getElementById('ext-btn-label');
  const portInfo = document.getElementById('ext-port-info');
  const onlineBadge = document.getElementById('ext-online-badge');
  const rxAge = document.getElementById('ext-rx-age');

  if (btn) {
    btn.classList.toggle('on', !!s.enabled);
    btn.classList.toggle('off', !s.enabled);
  }

  if (btnLabel) {
    btnLabel.textContent = s.enabled ? '外部コントローラ: ON' : '外部コントローラ: OFF';
  }

  if (portInfo) {
    const p = s.port || '未接続';
    const age = Number.isFinite(s.last_rx_age) ? `${s.last_rx_age.toFixed(1)}s` : '-';
    portInfo.textContent = `${p} | age: ${age} | pkt: ${s.packet_count ?? 0}`;
  }

  if (onlineBadge) {
    onlineBadge.textContent = s.online ? 'ONLINE' : 'OFFLINE';
    onlineBadge.classList.toggle('online', !!s.online);
    onlineBadge.classList.toggle('offline', !s.online);
  }

  if (rxAge) {
    const age = Number.isFinite(s.last_rx_age) ? s.last_rx_age.toFixed(1) : '999';
    rxAge.textContent = `age: ${age}s`;
  }

  // MDD1 デバッグ表示
  const d1 = s.mdd1_deg || [0, 0, 0, 0];
  const sw1 = s.mdd1_lsw || [0, 0, 0, 0];
  for (let i = 0; i < 4; i++) {
    const el = document.getElementById(`ext-d1-${i}`);
    if (el) {
      const v = d1[i];
      el.textContent = `${v >= 0 ? '+' : ''}${v.toFixed(1)}°`;
    }
    const swEl = document.getElementById(`ext-sw1-${i}`);
    if (swEl) {
      const isOn = (sw1[i] === 0);
      swEl.classList.toggle('on', isOn);
      swEl.classList.toggle('off', !isOn);
    }
  }

  // MDD2 デバッグ表示
  const d2 = s.mdd2_deg || [0, 0, 0, 0];
  const sw2 = s.mdd2_lsw || [0, 0, 0, 0];
  for (let i = 0; i < 4; i++) {
    const el = document.getElementById(`ext-d2-${i}`);
    if (el) {
      const v = d2[i];
      el.textContent = `${v >= 0 ? '+' : ''}${v.toFixed(1)}°`;
    }
    const swEl = document.getElementById(`ext-sw2-${i}`);
    if (swEl) {
      const isOn = (sw2[i] === 0);
      swEl.classList.toggle('on', isOn);
      swEl.classList.toggle('off', !isOn);
    }
  }
}

// 制御モード UI
function updateModeUI(mode) {
  mobileState.motorMode = mode;

  const btnStop = document.getElementById('btn-mode-stop');
  const btnPid = document.getElementById('btn-mode-pid');
  const btnOpen = document.getElementById('btn-mode-openloop');
  const badge = document.getElementById('current-mode-badge');

  if (btnStop) btnStop.classList.toggle('active', mode === 0);
  if (btnPid) btnPid.classList.toggle('active', mode === 1);
  if (btnOpen) btnOpen.classList.toggle('active', mode === 2);

  if (badge) {
    if (mode === 0) {
      badge.textContent = '停止中';
      badge.className = 'badge-mini mode-stop';
    } else if (mode === 1) {
      badge.textContent = 'PID 制御中';
      badge.className = 'badge-mini mode-pid';
    } else {
      badge.textContent = '開ループ';
      badge.className = 'badge-mini';
    }
  }
}

// ロボマス モーターFB UI
function renderMotorFbUI() {
  const names = ['RM1', 'RM2', 'LM1', 'LM2', 'SM1', 'LM3'];
  names.forEach((name, i) => {
    const el = document.getElementById(`ang-${name}`);
    if (el) {
      const val = mobileState.motorFb[i] ?? 0;
      el.textContent = `${val >= 0 ? '+' : ''}${val.toFixed(1)}°`;
    }
  });
}

// ─────────────────────────────────────────────────────────
// ユーザーアクション
// ─────────────────────────────────────────────────────────
function toggleExtController() {
  vibrate(30);
  const next = !mobileState.extCtrl.enabled;
  sendWs({ cmd: 'ext_ctrl_mode', enabled: next });
  showToast(`外部コントローラ → ${next ? 'ON' : 'OFF'}`);
}

function setMotorControlMode(mode) {
  vibrate(30);
  mobileState.motorMode = mode;
  updateModeUI(mode);
  sendWs({ cmd: 'motor_mode', mode: mode });
  const labels = ['🛑 停止', '⚡ PID 制御', '🔬 開ループ'];
  showToast(`モード → ${labels[mode] || mode}`);
}

function resetAllMotors() {
  vibrate(40);
  sendWs({ cmd: 'motor_cmd', targets: [0, 0, 0, 0, 0, 0] });
  showToast('全軸 0° リセット指令送信');
}

// トースト通知
let toastTimeout = null;
function showToast(text) {
  const toast = document.getElementById('toast');
  if (!toast) return;
  toast.textContent = text;
  toast.classList.add('show');
  if (toastTimeout) clearTimeout(toastTimeout);
  toastTimeout = setTimeout(() => {
    toast.classList.remove('show');
  }, 2000);
}

// ─────────────────────────────────────────────────────────
// 初期化
// ─────────────────────────────────────────────────────────
document.addEventListener('DOMContentLoaded', () => {
  connectWs();
});
