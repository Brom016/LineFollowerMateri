/*
 * ════════════════════════════════════════════════════════════════
 *  LINE FOLLOWER – ESP32 + TB6612FNG + 8ch TCRT5000
 *  WITH FULL WEB CONTROL PANEL (WiFi Access Point)
 *
 *  LIBRARY YANG DIBUTUHKAN (install via Library Manager / GitHub):
 *    1. ESPAsyncWebServer  → https://github.com/me-no-dev/ESPAsyncWebServer
 *    2. AsyncTCP           → https://github.com/me-no-dev/AsyncTCP
 *    3. ArduinoJson        → Library Manager: "ArduinoJson" by Benoit Blanchon
 *
 *  CARA AKSES WEB:
 *    1. Nyalakan ESP32
 *    2. Hubungkan HP/Laptop ke WiFi:  LineFollower  (pass: robot1234)
 *    3. Buka browser: http://192.168.4.1
 *
 *  FITUR WEB:
 *    <i class="fa-solid fa-chart-column"></i> Monitor   – Sensor bar real-time, grafik posisi, kecepatan motor
 *    <i class="fa-solid fa-gamepad"></i> Kontrol   – Start/Stop, Kalibrasi otomatis, Info sistem
 *    <i class="fa-solid fa-sliders"></i> Settings  – PID, semua parameter speed & timing
 *    <i class="fa-solid fa-floppy-disk"></i> Profil    – Simpan & load hingga 8 profil berbeda
 *    <i class="fa-solid fa-book"></i> Panduan   – Penjelasan PID, cara tuning, cara kalibrasi
 *
 *  MODE START:
 *    • Web Mode   – Robot menunggu tombol START dari web
 *    • Direct Mode – Robot auto-start saat menyala;
 *                    tombol BOOT (GPIO0) juga bisa toggle Start/Stop
 *
 *  PIN HARDWARE:
 *    Motor Kiri  : AIN1=21, AIN2=22, PWMA=23
 *    Motor Kanan : BIN1=18, BIN2=5,  PWMB=15
 *    Standby     : STBY=19
 *    Sensor (ADC1): S0=GPIO36,S1=GPIO39,S2=GPIO34,S3=GPIO35
 *                    S4=GPIO32,S5=GPIO33,S6=GPIO37,S7=GPIO38
 *    Start Btn   : GPIO0 (tombol BOOT bawaan)
 *
 *  ⚠️  CATATAN ADC: Pin 27 & 14 adalah ADC2, tidak bisa digunakan
 *      bersamaan dengan WiFi. Jika ada masalah pada sensor 6 & 7,
 *      pindah ke pin ADC1 (36,39,34,35,32,33,25,26) dan sesuaikan
 *      array SENSOR_PINS di bawah.
 *
 *  JIKA MOTOR BERPUTAR TERBALIK:
 *    Tukar AIN1↔AIN2 atau BIN1↔BIN2 untuk motor yang bermasalah.
 *  JIKA ROBOT FOLLOW KE ARAH SALAH:
 *    Tukar semua pin Motor A dengan Motor B.
 * ════════════════════════════════════════════════════════════════
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncWebSocket.h>
#include <Preferences.h>
#include <ArduinoJson.h>

// ──────────────────────────────────────────────────────────────
//  MOTOR PINS – TB6612FNG
// ──────────────────────────────────────────────────────────────
#define AIN1  21
#define AIN2  22
#define PWMA  23
#define BIN1  18
#define BIN2   5
#define PWMB  15
#define STBY  19

// ──────────────────────────────────────────────────────────────
//  TOMBOL START FISIK (Direct Mode) – GPIO0 = tombol BOOT bawaan
// ──────────────────────────────────────────────────────────────
#define START_BTN_PIN  0

// ──────────────────────────────────────────────────────────────
//  SENSOR PINS – 8ch TCRT5000
//  Index 0 = paling KIRI, index 7 = paling KANAN
//  ⚠️ Pin 27,14 = ADC2, konflik dengan WiFi → ganti jika perlu
// ──────────────────────────────────────────────────────────────
#define SENSOR_COUNT  8
// MAPPING BARU – semuanya ADC1, aman dengan WiFi:
// S0=GPIO36(ADC1_CH0)  S1=GPIO39(ADC1_CH3)  S2=GPIO34(ADC1_CH6)  S3=GPIO35(ADC1_CH7)
// S4=GPIO32(ADC1_CH4)  S5=GPIO33(ADC1_CH5)  S6=GPIO37(ADC1_CH1)  S7=GPIO38(ADC1_CH2)
// CATATAN: GPIO37/38 hanya di ESP32-WROVER.
// Untuk WROOM: ganti S6=GPIO4 dan S7=GPIO2 (gunakan DO pin sensor, bukan AO)
const uint8_t SENSOR_PINS[SENSOR_COUNT] = {36, 39, 34, 35, 32, 33, 37, 38};

// ──────────────────────────────────────────────────────────────
//  PWM – ESP32 Core 3.x API (ledcAttach / ledcWrite pakai pin)
// ──────────────────────────────────────────────────────────────
#define PWM_FREQ  10000
#define PWM_RES   8

// ──────────────────────────────────────────────────────────────
//  BOBOT SENSOR UNTUK WEIGHTED AVERAGE
//  Error = 0 → garis tepat di tengah
//  Error < 0 → garis di kiri  | Error > 0 → garis di kanan
// ──────────────────────────────────────────────────────────────
const int16_t SENSOR_WEIGHT[SENSOR_COUNT] = {
  -3500, -2500, -1500, -500, 500, 1500, 2500, 3500
};

// ──────────────────────────────────────────────────────────────
//  KONFIGURASI RUNTIME (dapat diubah dari web, tersimpan di flash)
// ──────────────────────────────────────────────────────────────
struct Config {
  float  kp          = 0.045f;
  float  ki          = 0.000f;
  float  kd          = 0.030f;
  int    baseSpeed   = 150;
  int    maxSpeed    = 200;
  int    spinSpeed   = 130;
  int    searchSpeed = 70;
  int    spinTimeMs  = 350;
  int    junctionMs  = 80;
  int    finishMs    = 500;
  int    threshold   = 2000;  // fallback threshold jika belum kalibrasi
} cfg;

// ──────────────────────────────────────────────────────────────
//  STATE MACHINE
// ──────────────────────────────────────────────────────────────
enum RobotState {
  ST_FOLLOW,
  ST_SPIN_L,
  ST_SPIN_R,
  ST_FINISH,
  ST_STOPPED,
  ST_CALIBRATE
};

enum StartMode {
  MODE_WEB,     // Tunggu perintah START dari web
  MODE_DIRECT   // Auto-start saat menyala + tombol fisik
};

RobotState  g_state     = ST_STOPPED;
StartMode   g_startMode = MODE_WEB;
bool        g_running   = false;

// ──────────────────────────────────────────────────────────────
//  DATA SENSOR
// ──────────────────────────────────────────────────────────────
bool     g_sensor[SENSOR_COUNT];
uint16_t g_adcRaw[SENSOR_COUNT];
uint16_t sensorMin[SENSOR_COUNT];
uint16_t sensorMax[SENSOR_COUNT];
bool     g_calibrated = false;

// ──────────────────────────────────────────────────────────────
//  PID STATE
// ──────────────────────────────────────────────────────────────
float   g_lastError  = 0.0f;
float   g_integral   = 0.0f;
float   g_correction = 0.0f;
int32_t g_lastPos    = 0;
int     g_speedA     = 0;
int     g_speedB     = 0;

// ──────────────────────────────────────────────────────────────
//  JUNCTION / FINISH DETECTION
// ──────────────────────────────────────────────────────────────
bool     g_allBlackActive = false;
uint32_t g_allBlackStart  = 0;

// ──────────────────────────────────────────────────────────────
//  CALIBRATION TIMING
// ──────────────────────────────────────────────────────────────
bool     g_calibrating    = false;
uint32_t g_calibStart     = 0;
#define  CALIB_DURATION_MS 5000

// ──────────────────────────────────────────────────────────────
//  WIFI & WEB SERVER
// ──────────────────────────────────────────────────────────────
const char* AP_SSID = "LineFollower";
const char* AP_PASS = "robot1234";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
Preferences    prefs;

// ──────────────────────────────────────────────────────────────
//  TIMING
// ──────────────────────────────────────────────────────────────
uint32_t lastWsUpdate = 0;
uint32_t lastBtnPress = 0;
#define WS_UPDATE_MS   50

// ──────────────────────────────────────────────────────────────
//  PROFIL TERSIMPAN
// ──────────────────────────────────────────────────────────────
#define MAX_PROFILES 8

// ══════════════════════════════════════════════════════════════
//  HTML WEB PAGE (disimpan di flash / PROGMEM)
// ══════════════════════════════════════════════════════════════
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="id">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0,maximum-scale=1.0">
<title>Line Follower Control</title>
<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.5.0/css/all.min.css">
<style>
:root{
  --g1:#1b5e20;--g2:#2e7d32;--g3:#388e3c;--g4:#43a047;
  --g5:#4caf50;--g6:#81c784;--g7:#c8e6c9;--g8:#e8f5e9;
  --white:#fff;--text:#1a2e1c;--text2:#4a6e4c;
  --red:#e53935;--orange:#ff9800;--shadow:0 2px 10px rgba(0,0,0,.12);
}
*{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent}
body{background:var(--g8);color:var(--text);font-family:'Segoe UI',system-ui,sans-serif;
  min-height:100vh;overflow-x:hidden}

/* Header */
.hdr{background:linear-gradient(135deg,var(--g1) 0%,var(--g3) 100%);color:#fff;
  padding:10px 14px;display:flex;align-items:center;gap:10px;
  box-shadow:0 2px 8px rgba(0,0,0,.3);position:sticky;top:0;z-index:50}
.hdr h1{font-size:1.05rem;font-weight:700;flex:1}
.hdr-right{display:flex;align-items:center;gap:8px}
.badge{padding:3px 9px;border-radius:12px;font-size:.7rem;font-weight:800;
  background:var(--g5);color:#fff;letter-spacing:.3px;white-space:nowrap}
.badge.stopped{background:#757575}
.badge.spin{background:var(--orange)}
.badge.finish{background:var(--red)}
.badge.calib{background:#7b1fa2}
.ws-dot{width:10px;height:10px;border-radius:50%;background:#e53935;
  box-shadow:0 0 5px #e53935;flex-shrink:0;transition:.3s}
.ws-dot.on{background:#69f0ae;box-shadow:0 0 6px #69f0ae}

/* Tabs */
.tabs{display:flex;background:var(--g2);overflow-x:auto;
  -webkit-overflow-scrolling:touch;scrollbar-width:none}
.tabs::-webkit-scrollbar{display:none}
.tab{flex:0 0 auto;padding:9px 15px;color:rgba(255,255,255,.65);font-size:.8rem;
  font-weight:700;cursor:pointer;border-bottom:3px solid transparent;
  transition:.2s;user-select:none;white-space:nowrap}
.tab.on{color:#fff;border-bottom-color:#69f0ae}

/* Pages */
.page{display:none;padding:12px;max-width:500px;margin:0 auto;padding-bottom:24px}
.page.on{display:block}

/* Card */
.card{background:var(--white);border-radius:14px;padding:13px 14px;
  box-shadow:var(--shadow);margin-bottom:12px}
.card-title{color:var(--g2);font-size:.88rem;font-weight:700;margin-bottom:10px;
  display:flex;align-items:center;gap:6px}

/* Sensor strip */
.sstrip{display:flex;gap:3px;margin-bottom:10px}
.sbox{flex:1;height:44px;border-radius:7px;background:#e0e0e0;
  display:flex;align-items:center;justify-content:center;
  font-size:.68rem;font-weight:800;color:#fff;transition:.12s;
  position:relative;overflow:hidden}
.sbox.on{background:var(--g1);box-shadow:0 0 6px rgba(27,94,32,.4)}
.sbox .snum{position:absolute;bottom:2px;font-size:.6rem;opacity:.7}

/* Position track */
.pos-wrap{background:#e8f5e9;border-radius:8px;height:22px;
  position:relative;overflow:hidden;margin:6px 0;border:1px solid var(--g7)}
.pos-center{position:absolute;top:0;left:50%;width:1px;height:100%;background:var(--g6)}
.pos-ind{position:absolute;top:1px;bottom:1px;width:14px;border-radius:4px;
  background:var(--g2);transition:left .08s;transform:translateX(-50%)}
.pos-label{display:flex;justify-content:space-between;font-size:.7rem;color:var(--text2);margin-top:2px}

/* Chart */
.chart-wrap{border-radius:10px;overflow:hidden;background:#f1f8f2;border:1px solid var(--g7)}
canvas{display:block;width:100%!important}

/* Motor */
.mrow{display:flex;gap:8px;margin-top:8px}
.mbox{flex:1;background:var(--g8);border-radius:10px;padding:9px;text-align:center;border:1px solid var(--g7)}
.mbox .mlabel{font-size:.68rem;color:var(--text2);display:block;margin-bottom:2px}
.mbox .mval{font-size:1.25rem;font-weight:800;color:var(--g2);line-height:1}
.mbox .mdir{font-size:.65rem;color:var(--text2);margin-top:1px}
.mbar-wrap{height:5px;background:#c8e6c9;border-radius:3px;margin-top:5px;overflow:hidden}
.mbar{height:100%;width:0%;background:var(--g4);border-radius:3px;transition:.08s}
.mbar.rev{background:var(--orange)}
.corr-row{display:flex;justify-content:space-between;margin-top:8px;
  padding-top:8px;border-top:1px solid var(--g7);font-size:.78rem;color:var(--text2)}
.corr-row strong{color:var(--g2)}

/* ADC raw grid */
.adc-grid{display:grid;grid-template-columns:repeat(4,1fr);gap:5px;margin-top:2px}
.adc-cell{background:var(--g8);border-radius:6px;padding:4px 2px;text-align:center;
  font-size:.7rem;border:1px solid var(--g7);transition:.12s}
.adc-cell.on{background:var(--g7);border-color:var(--g5);font-weight:700;color:var(--g1)}
.adc-cell .alabel{color:var(--text2);font-size:.6rem;display:block}

/* Buttons */
.btn{display:flex;align-items:center;justify-content:center;gap:7px;
  padding:13px 16px;border:none;border-radius:12px;font-size:.95rem;
  font-weight:800;cursor:pointer;transition:.15s;width:100%;
  margin-bottom:8px;letter-spacing:.2px;-webkit-appearance:none}
.btn:active{transform:scale(.96)}
.btn-start{background:linear-gradient(135deg,var(--g4),var(--g3));color:#fff;
  box-shadow:0 3px 10px rgba(67,160,71,.35)}
.btn-stop{background:linear-gradient(135deg,#ef5350,var(--red));color:#fff;
  box-shadow:0 3px 10px rgba(229,57,53,.3)}
.btn-calib{background:linear-gradient(135deg,#ce93d8,#9c27b0);color:#fff}
.btn-sec{background:var(--g7);color:var(--g2);border:2px solid var(--g6)}
.btn:disabled{opacity:.5;pointer-events:none}

/* Mode toggle */
.mtoggle{display:flex;background:var(--g7);border-radius:12px;padding:4px;gap:4px;margin-bottom:10px}
.mbtn{flex:1;padding:9px 6px;border:none;border-radius:10px;font-size:.8rem;
  font-weight:700;cursor:pointer;background:transparent;color:var(--g2);
  transition:.2s;-webkit-appearance:none}
.mbtn.on{background:linear-gradient(135deg,var(--g4),var(--g3));color:#fff;
  box-shadow:0 2px 8px rgba(67,160,71,.3)}

/* Calib progress */
.calib-prog{display:none;margin-top:10px}
.calib-prog.show{display:block}
.prog-bar{background:#e0e0e0;border-radius:8px;height:10px;overflow:hidden}
.prog-fill{height:100%;width:0%;background:linear-gradient(90deg,var(--g5),var(--g3));
  border-radius:8px;transition:width .1s linear}
.prog-text{font-size:.78rem;color:var(--text2);margin-bottom:4px;display:flex;
  justify-content:space-between}

/* System info table */
.sysinfo{width:100%;border-collapse:collapse;font-size:.8rem}
.sysinfo td{padding:5px 0}
.sysinfo td:first-child{color:var(--text2)}
.sysinfo td:last-child{text-align:right;font-weight:700;color:var(--g1)}
.sysinfo tr{border-bottom:1px solid var(--g8)}

/* Settings */
.srow{margin-bottom:14px}
.srow label{display:flex;justify-content:space-between;font-size:.8rem;
  color:var(--text2);font-weight:700;margin-bottom:5px}
.srow label .sv{color:var(--g2);font-size:.85rem}
input[type=range]{width:100%;accent-color:var(--g4);height:6px;cursor:pointer}
.pid-grid{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px;margin-bottom:4px}
.pid-cell label{display:block;text-align:center;font-size:.75rem;color:var(--text2);
  font-weight:800;margin-bottom:4px}
.pid-cell input{width:100%;padding:8px 6px;border:2px solid var(--g6);border-radius:8px;
  font-size:.9rem;text-align:center;outline:none;transition:.2s;
  -webkit-appearance:none;appearance:none}
.pid-cell input:focus{border-color:var(--g4)}
input[type=number]{-moz-appearance:textfield}
input[type=number]::-webkit-outer-spin-button,
input[type=number]::-webkit-inner-spin-button{-webkit-appearance:none;margin:0}
input[type=text]{width:100%;padding:9px 12px;border:2px solid var(--g6);border-radius:9px;
  font-size:.9rem;outline:none;transition:.2s}
input[type=text]:focus{border-color:var(--g4)}

/* Profiles */
.plist{display:flex;flex-direction:column;gap:7px;margin-bottom:10px}
.pitem{background:var(--g8);border:2px solid var(--g7);border-radius:11px;
  padding:9px 11px;display:flex;align-items:flex-start;gap:8px;transition:.15s}
.pitem:hover,.pitem.sel{border-color:var(--g5);background:var(--g7)}
.pname{font-weight:800;font-size:.88rem;color:var(--text)}
.pmeta{font-size:.68rem;color:var(--text2);margin-top:2px;line-height:1.5}
.pbtn-wrap{display:flex;gap:4px;flex-shrink:0;margin-left:auto}
.pbtn{padding:5px 10px;border:none;border-radius:7px;font-size:.72rem;
  cursor:pointer;font-weight:700;-webkit-appearance:none}
.pbtn-load{background:var(--g5);color:#fff}
.pbtn-del{background:#ffcdd2;color:#c62828}
.save-row{display:flex;gap:8px;align-items:center}
.save-row input{flex:1}
.save-row .btn{width:auto;margin:0;padding:9px 14px;font-size:.82rem}

/* Guide */
.gsec{margin-bottom:14px;padding-bottom:14px;border-bottom:1px solid var(--g7)}
.gsec:last-child{border-bottom:none;margin-bottom:0}
.gsec h4{color:var(--g2);font-size:.88rem;font-weight:800;margin-bottom:7px;
  padding-left:6px;border-left:3px solid var(--g5)}
.gsec p,.gsec li{font-size:.81rem;line-height:1.7;color:var(--text2)}
.gsec ul,.gsec ol{padding-left:18px;margin-top:4px}
.gsec li{margin-bottom:3px}
.formula{background:var(--g8);border-left:4px solid var(--g4);padding:8px 12px;
  border-radius:0 8px 8px 0;margin:7px 0;font-family:'Courier New',monospace;
  font-size:.82rem;color:var(--g1);white-space:pre-wrap;line-height:1.6}
.tip{background:#fff8e1;border:1px solid #ffd54f;border-radius:9px;
  padding:8px 11px;font-size:.79rem;color:#5d4037;margin-top:7px;line-height:1.5}
.warn{background:#fce4ec;border:1px solid #f48fb1;border-radius:9px;
  padding:8px 11px;font-size:.79rem;color:#880e4f;margin-top:7px;line-height:1.5}
.step-num{display:inline-block;width:20px;height:20px;border-radius:50%;
  background:var(--g4);color:#fff;font-size:.72rem;font-weight:800;
  text-align:center;line-height:20px;margin-right:4px;flex-shrink:0}

/* Toast */
.toast{position:fixed;bottom:20px;left:50%;transform:translateX(-50%) translateY(20px);
  background:rgba(27,94,32,.95);color:#fff;padding:9px 22px;border-radius:20px;
  font-size:.83rem;font-weight:700;opacity:0;transition:.25s;pointer-events:none;
  z-index:100;white-space:nowrap;backdrop-filter:blur(4px)}
.toast.show{opacity:1;transform:translateX(-50%) translateY(0)}

/* Divider */
.hdivider{height:1px;background:var(--g7);margin:10px 0}
</style>
</head>
<body>

<div class="hdr">
  <i class="fa-solid fa-robot" style="font-size:1.2rem;opacity:.9"></i>
  <h1>Line Follower Control</h1>
  <div class="hdr-right">
    <span id="stateBadge" class="badge stopped">STOPPED</span>
    <div class="ws-dot" id="wsDot"></div>
  </div>
</div>

<div class="tabs">
  <div class="tab on" onclick="gotoTab('monitor')"><i class="fa-solid fa-chart-column"></i> Monitor</div>
  <div class="tab"      onclick="gotoTab('control')"><i class="fa-solid fa-gamepad"></i> Kontrol</div>
  <div class="tab"      onclick="gotoTab('settings')"><i class="fa-solid fa-sliders"></i> Settings</div>
  <div class="tab"      onclick="gotoTab('profiles')"><i class="fa-solid fa-floppy-disk"></i> Profil</div>
  <div class="tab"      onclick="gotoTab('guide')"><i class="fa-solid fa-book"></i> Panduan</div>
</div>

<!-- ══════════════════ MONITOR PAGE ══════════════════ -->
<div id="pg-monitor" class="page on">

  <div class="card">
    <div class="card-title"><i class="fa-solid fa-microchip"></i> Sensor Array (Hitam = Aktif)</div>
    <div class="sstrip" id="sstrip">
      <div class="sbox" id="sb0"><span class="snum">0</span></div>
      <div class="sbox" id="sb1"><span class="snum">1</span></div>
      <div class="sbox" id="sb2"><span class="snum">2</span></div>
      <div class="sbox" id="sb3"><span class="snum">3</span></div>
      <div class="sbox" id="sb4"><span class="snum">4</span></div>
      <div class="sbox" id="sb5"><span class="snum">5</span></div>
      <div class="sbox" id="sb6"><span class="snum">6</span></div>
      <div class="sbox" id="sb7"><span class="snum">7</span></div>
    </div>
    <div class="pos-wrap">
      <div class="pos-center"></div>
      <div class="pos-ind" id="posInd" style="left:50%"></div>
    </div>
    <div class="pos-label">
      <span>← KIRI</span>
      <span>Posisi: <strong id="posVal">0</strong></span>
      <span>KANAN →</span>
    </div>
  </div>

  <div class="card">
    <div class="card-title"><i class="fa-solid fa-chart-line"></i> Grafik Posisi Real-time</div>
    <div class="chart-wrap">
      <canvas id="chart" height="110"></canvas>
    </div>
    <div style="display:flex;justify-content:space-between;font-size:.7rem;
         color:var(--text2);margin-top:4px">
      <span>+3500 (Kanan)</span>
      <span style="color:var(--g3);font-weight:700">Error: <span id="errVal">0</span></span>
      <span>-3500 (Kiri)</span>
    </div>
  </div>

  <div class="card">
    <div class="card-title"><i class="fa-solid fa-bolt"></i> Motor &amp; PID Output</div>
    <div class="mrow">
      <div class="mbox">
        <span class="mlabel">Motor KIRI (A)</span>
        <div class="mval" id="mvA">0</div>
        <div class="mdir" id="mdA">—</div>
        <div class="mbar-wrap"><div class="mbar" id="mbA"></div></div>
      </div>
      <div class="mbox">
        <span class="mlabel">Motor KANAN (B)</span>
        <div class="mval" id="mvB">0</div>
        <div class="mdir" id="mdB">—</div>
        <div class="mbar-wrap"><div class="mbar" id="mbB"></div></div>
      </div>
    </div>
    <div class="corr-row">
      <span>PID Correction: <strong id="corrVal">0.0</strong></span>
      <span>Aktif: <strong id="activeVal">0</strong>/8 sensor</span>
    </div>
  </div>

  <div class="card">
    <div class="card-title"><i class="fa-solid fa-satellite-dish"></i> ADC Raw per Sensor</div>
    <div class="adc-grid" id="adcGrid">
      <div class="adc-cell" id="adc0"><span class="alabel">S0</span>—</div>
      <div class="adc-cell" id="adc1"><span class="alabel">S1</span>—</div>
      <div class="adc-cell" id="adc2"><span class="alabel">S2</span>—</div>
      <div class="adc-cell" id="adc3"><span class="alabel">S3</span>—</div>
      <div class="adc-cell" id="adc4"><span class="alabel">S4</span>—</div>
      <div class="adc-cell" id="adc5"><span class="alabel">S5</span>—</div>
      <div class="adc-cell" id="adc6"><span class="alabel">S6</span>—</div>
      <div class="adc-cell" id="adc7"><span class="alabel">S7</span>—</div>
    </div>
  </div>
</div>

<!-- ══════════════════ CONTROL PAGE ══════════════════ -->
<div id="pg-control" class="page">

  <div class="card">
    <div class="card-title"><i class="fa-solid fa-rocket"></i> Mode Start</div>
    <div class="mtoggle">
      <button class="mbtn on" id="modeWebBtn" onclick="setMode('web')">
        <i class="fa-solid fa-globe"></i> Web Mode
      </button>
      <button class="mbtn" id="modeDirBtn" onclick="setMode('direct')">
        <i class="fa-solid fa-circle-dot"></i> Direct Mode
      </button>
    </div>
    <p id="modeDesc" style="font-size:.79rem;color:var(--text2);line-height:1.6">
      <strong>Web Mode:</strong> Robot diam menunggu perintah START dari halaman ini.
      Cocok untuk kompetisi agar bisa start tepat waktu.
    </p>
  </div>

  <div class="card">
    <div class="card-title"><i class="fa-solid fa-gamepad"></i> Kontrol Robot</div>
    <button class="btn btn-start" id="btnStart" onclick="doStart()">
      <i class="fa-solid fa-play"></i>&nbsp; START
    </button>
    <button class="btn btn-stop" id="btnStop" onclick="doStop()">
      <i class="fa-solid fa-stop"></i>&nbsp; STOP
    </button>
  </div>

  <div class="card">
    <div class="card-title"><i class="fa-solid fa-microscope"></i> Kalibrasi Sensor Otomatis</div>
    <p style="font-size:.8rem;color:var(--text2);line-height:1.6;margin-bottom:10px">
      Gerakkan robot perlahan di atas area <strong>hitam & putih</strong>
      selama <strong>5 detik</strong>. Threshold tiap sensor akan dihitung otomatis.
    </p>
    <button class="btn btn-calib" id="btnCalib" onclick="doCalib()">
      <i class="fa-solid fa-wand-magic-sparkles"></i>&nbsp; Mulai Kalibrasi (5 detik)
    </button>
    <div class="calib-prog" id="calibProg">
      <div class="prog-text">
        <span id="calibMsg">Kalibrasi berjalan...</span>
        <span id="calibSec">5s</span>
      </div>
      <div class="prog-bar"><div class="prog-fill" id="progFill"></div></div>
    </div>
    <div id="calibDone" style="display:none;margin-top:8px;font-size:.79rem;
         color:var(--g2);font-weight:700;background:var(--g7);padding:7px 10px;
         border-radius:8px"></div>
  </div>

  <div class="card">
    <div class="card-title"><i class="fa-solid fa-circle-info"></i> Status Sistem</div>
    <table class="sysinfo">
      <tr><td>Status Robot</td><td id="si-state">—</td></tr>
      <tr><td>Mode Start</td><td id="si-mode">—</td></tr>
      <tr><td>Kalibrasi</td><td id="si-calib">—</td></tr>
      <tr><td>Kp / Ki / Kd</td><td id="si-pid">—</td></tr>
      <tr><td>Base / Max Speed</td><td id="si-speed">—</td></tr>
      <tr><td>Spin Speed / Time</td><td id="si-spin">—</td></tr>
    </table>
  </div>
</div>

<!-- ══════════════════ SETTINGS PAGE ══════════════════ -->
<div id="pg-settings" class="page">

  <div class="card">
    <div class="card-title"><i class="fa-solid fa-sliders"></i> PID Controller</div>
    <div class="pid-grid">
      <div class="pid-cell">
        <label>Kp</label>
        <input type="number" id="kp" step="0.001" min="0" max="2" value="0.045">
      </div>
      <div class="pid-cell">
        <label>Ki</label>
        <input type="number" id="ki" step="0.0001" min="0" max="0.5" value="0.000">
      </div>
      <div class="pid-cell">
        <label>Kd</label>
        <input type="number" id="kd" step="0.001" min="0" max="2" value="0.030">
      </div>
    </div>
    <p style="font-size:.72rem;color:var(--text2);margin-top:6px;text-align:center">
      output = Kp×err + Ki×Σerr + Kd×Δerr
    </p>
  </div>

  <div class="card">
    <div class="card-title"><i class="fa-solid fa-bolt"></i> Kecepatan Motor (0-255)</div>
    <div class="srow">
      <label>Base Speed <span class="sv" id="sv-base">150</span></label>
      <input type="range" id="baseSpeed" min="50" max="255" value="150"
        oninput="sv('base',this.value)">
    </div>
    <div class="srow">
      <label>Max Speed <span class="sv" id="sv-max">200</span></label>
      <input type="range" id="maxSpeed" min="50" max="255" value="200"
        oninput="sv('max',this.value)">
    </div>
    <div class="srow">
      <label>Spin Speed – belok 90° <span class="sv" id="sv-spin">130</span></label>
      <input type="range" id="spinSpeed" min="50" max="255" value="130"
        oninput="sv('spin',this.value)">
    </div>
    <div class="srow">
      <label>Search Speed – cari garis <span class="sv" id="sv-srch">70</span></label>
      <input type="range" id="searchSpeed" min="20" max="180" value="70"
        oninput="sv('srch',this.value)">
    </div>
  </div>

  <div class="card">
    <div class="card-title"><i class="fa-solid fa-clock"></i> Timing</div>
    <div class="srow">
      <label>Spin Time ms – durasi putar 90° <span class="sv" id="sv-spint">350</span></label>
      <input type="range" id="spinTime" min="100" max="1200" step="10" value="350"
        oninput="sv('spint',this.value)">
    </div>
    <div class="srow">
      <label>Junction Hold ms – crossing lurus <span class="sv" id="sv-junc">80</span></label>
      <input type="range" id="junction" min="20" max="400" step="5" value="80"
        oninput="sv('junc',this.value)">
    </div>
    <div class="srow">
      <label>Finish Hold ms – kotak finish <span class="sv" id="sv-fin">500</span></label>
      <input type="range" id="finish" min="150" max="3000" step="50" value="500"
        oninput="sv('fin',this.value)">
    </div>
  </div>

  <button class="btn btn-start" onclick="applySettings()">
    <i class="fa-solid fa-check"></i>&nbsp; Terapkan &amp; Simpan Settings
  </button>
  <button class="btn btn-sec" onclick="refreshSettings()">
    <i class="fa-solid fa-rotate"></i>&nbsp; Refresh dari Robot
  </button>
</div>

<!-- ══════════════════ PROFILES PAGE ══════════════════ -->
<div id="pg-profiles" class="page">

  <div class="card">
    <div class="card-title"><i class="fa-solid fa-floppy-disk"></i> Simpan Settings Saat Ini</div>
    <div class="save-row">
      <input type="text" id="pName" placeholder="Nama profil (cth: Lurus Cepat)" maxlength="18">
      <button class="btn btn-start" onclick="saveProfile()"><i class="fa-solid fa-save"></i> Simpan</button>
    </div>
    <p style="font-size:.73rem;color:var(--text2);margin-top:6px">
      Profil menyimpan semua nilai PID, speed, dan timing saat ini.
    </p>
  </div>

  <div class="card">
    <div class="card-title"><i class="fa-solid fa-folder-open"></i> Profil Tersimpan
      <span style="margin-left:auto;font-size:.7rem;color:var(--text2)" id="pCountLbl"></span>
    </div>
    <div class="plist" id="pList">
      <div style="text-align:center;color:var(--text2);font-size:.84rem;
           padding:18px"><i class="fa-solid fa-inbox" style="font-size:1.4rem;display:block;margin-bottom:5px;color:var(--g6)"></i>Belum ada profil tersimpan</div>
    </div>
    <button class="btn btn-sec" onclick="loadProfiles()" style="margin-top:4px">
      <i class="fa-solid fa-rotate"></i>&nbsp; Refresh Daftar Profil
    </button>
  </div>
</div>

<!-- ══════════════════ GUIDE PAGE ══════════════════ -->
<div id="pg-guide" class="page">
  <div class="card">

    <div class="gsec">
      <h4><i class="fa-solid fa-thumbtack"></i> Apa itu PID Controller?</h4>
      <p>PID (Proportional–Integral–Derivative) adalah algoritma kontrol otomatis
      yang membuat robot selalu berusaha menempatkan garis tepat di tengah
      array sensor. Semakin jauh garis dari tengah, semakin besar koreksi
      yang diberikan ke motor.</p>
    </div>

    <div class="gsec">
      <h4><i class="fa-solid fa-calculator"></i> Rumus &amp; Cara Kerja PID</h4>
      <div class="formula">correction = Kp×error + Ki×Σerror + Kd×(error − error_lama)

SpeedMotorKiri  = BaseSpeed + correction
SpeedMotorKanan = BaseSpeed − correction</div>
      <ul>
        <li><strong>error</strong> = posisi garis (weighted average 8 sensor)</li>
        <li><strong>Kp × error</strong> → koreksi proporsional terhadap seberapa jauh garis dari tengah</li>
        <li><strong>Ki × Σerror</strong> → koreksi akumulasi; atasi error steady-state (jarang dipakai)</li>
        <li><strong>Kd × Δerror</strong> → redaman; merespons kecepatan perubahan error (atasi osilasi)</li>
      </ul>
    </div>

    <div class="gsec">
      <h4><i class="fa-solid fa-ruler"></i> Sistem Bobot Sensor</h4>
      <div class="formula">S0=-3500 S1=-2500 S2=-1500 S3=-500
S4=+500  S5=+1500 S6=+2500 S7=+3500

error = weighted_average(sensor aktif)
Contoh: S3 & S4 aktif → error = (-500+500)/2 = 0 ✅</div>
      <ul>
        <li>Error = 0 → garis tepat di tengah → lurus sempurna</li>
        <li>Error negatif → garis di kiri → motor kiri diperlambat, kanan dipercepat</li>
        <li>Error positif → garis di kanan → motor kanan diperlambat, kiri dipercepat</li>
      </ul>
    </div>

    <div class="gsec">
      <h4><i class="fa-solid fa-bullseye"></i> Langkah-langkah Tuning PID</h4>
      <ol>
        <li>Set <strong>Ki=0, Kd=0</strong>, Base Speed 130, lalu naikkan <strong>Kp</strong> perlahan (0.01 per langkah)</li>
        <li>Hentikan saat robot mulai zig-zag / berosilasi → itu nilai Kp terlalu besar</li>
        <li>Turunkan Kp sedikit (−20%) agar di bawah titik osilasi</li>
        <li>Naikkan <strong>Kd</strong> perlahan sampai osilasi teredam dan robot mulus</li>
        <li>Naikkan Base Speed bertahap setelah lintasan lurus stabil</li>
        <li>Tambah Kp +10–20% jika tikungan 90° tajam tidak terkejar</li>
        <li><strong>Ki</strong> biasanya tidak diperlukan → biarkan 0</li>
      </ol>
      <div class="tip"><i class="fa-solid fa-lightbulb" style="color:#f9a825"></i> Tuning di lantai kompetisi aktual! Hasil berbeda di permukaan & pencahayaan berbeda.</div>
    </div>

    <div class="gsec">
      <h4><i class="fa-solid fa-gauge-high"></i> Penjelasan Parameter Speed</h4>
      <ul>
        <li><strong>Base Speed</strong> – kecepatan jelajah normal. Mulai 130–150.</li>
        <li><strong>Max Speed</strong> – batas atas per motor saat PID boost. Hindari terlalu tinggi agar motor tidak stall.</li>
        <li><strong>Spin Speed</strong> – kecepatan saat pivot turn 90°. Terlalu cepat → lewat garis; terlalu lambat → tidak sampai 90°.</li>
        <li><strong>Search Speed</strong> – kecepatan creep maju saat garis hilang (semua sensor putih).</li>
      </ul>
    </div>

    <div class="gsec">
      <h4><i class="fa-solid fa-clock"></i> Penjelasan Parameter Timing</h4>
      <ul>
        <li><strong>Spin Time ms</strong> – durasi pivot turn. Kalibrasi: ukur berapa ms robot berputar tepat 90° di lantai aktual.</li>
        <li><strong>Junction Hold ms</strong> – threshold all-black dianggap "garis horizontal / crossing" → robot lanjut lurus. Default 80ms.</li>
        <li><strong>Finish Hold ms</strong> – threshold all-black dianggap "kotak finish" → robot berhenti. Default 500ms.</li>
      </ul>
      <div class="tip"><i class="fa-solid fa-lightbulb" style="color:#f9a825"></i> Logika: all-black &lt; JunctionHold → lewat lurus | all-black &gt; FinishHold → STOP</div>
    </div>

    <div class="gsec">
      <h4><i class="fa-solid fa-microscope"></i> Kalibrasi Sensor - Kenapa Penting?</h4>
      <p>Setiap sensor TCRT5000 memiliki karakteristik berbeda. Kalibrasi otomatis
      mengukur nilai ADC minimum (putih) dan maksimum (hitam) lalu menetapkan
      threshold di titik tengah untuk setiap sensor secara individual.</p>
      <ol style="margin-top:6px">
        <li>Buka tab <strong>Kontrol</strong></li>
        <li>Tekan <strong>Mulai Kalibrasi</strong></li>
        <li>Gerakkan robot perlahan melewati area hitam dan putih selama 5 detik</li>
        <li>Threshold otomatis tersimpan di flash ESP32</li>
      </ol>
      <div class="tip"><i class="fa-solid fa-lightbulb" style="color:#f9a825"></i> Kalibrasi ulang setiap berpindah lokasi atau pencahayaan berubah!</div>
    </div>

    <div class="gsec">
      <h4><i class="fa-solid fa-rocket"></i> Mode Start – Perbedaan Web vs Direct</h4>
      <ul>
        <li><strong>Web Mode</strong> – Robot diam saat menyala. START hanya dari tombol web.
          Ideal untuk kompetisi: posisikan robot → tekan START tepat waktunya.</li>
        <li><strong>Direct Mode</strong> – Robot langsung jalan begitu ESP32 menyala.
          Tombol BOOT (GPIO0) bisa toggle Start/Stop. Web tetap bisa STOP kapan saja.</li>
      </ul>
    </div>

    <div class="gsec">
      <h4><i class="fa-solid fa-plug"></i> Referensi Pin Hardware</h4>
      <div class="formula">Motor Kiri  : AIN1=21  AIN2=22  PWMA=23
Motor Kanan : BIN1=18  BIN2=5   PWMB=15
Standby     : STBY=19  (harus HIGH agar motor jalan)
Sensor      : S0=34 S1=35 S2=32 S3=33
              S4=25 S5=26 S6=27 S7=14
Start Btn   : GPIO0 (tombol BOOT bawaan ESP32)</div>
      <div class="warn"><i class="fa-solid fa-triangle-exclamation"></i> GPIO37 &amp; GPIO38 hanya ada di ESP32-WROVER — tidak bisa bersamaan dengan WiFi.
Jika sensor 6 atau 7 bermasalah, gunakan pin ADC1 (misal 36, 39) dan
ubah array SENSOR_PINS di kode.</div>
      <div class="tip">💡 ESP32 butuh regulator 3.3V/5V tersendiri dari baterai 7.4V LiPo.
Gunakan buck converter + LDO sebelum VCC ESP32 & TB6612FNG.</div>
    </div>

  </div><!-- end card -->
</div><!-- end guide page -->

<div class="toast" id="toast"></div>

<script>
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
//  WebSocket
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
let wsock, reconnTimer;
const chartData = new Array(120).fill(0);
let chartCtx, chartW=0, chartH=0;

function connectWS() {
  clearTimeout(reconnTimer);
  wsock = new WebSocket('ws://' + location.hostname + '/ws');
  wsock.onopen  = wsOpen;
  wsock.onclose = wsClose;
  wsock.onerror = () => wsock.close();
  wsock.onmessage = e => { try { dispatch(JSON.parse(e.data)); } catch(x){} };
}

function wsOpen() {
  setDot(true);
  showToast('Terhubung ke robot');
  send('getConfig');
  send('getProfiles');
}

function wsClose() {
  setDot(false);
  reconnTimer = setTimeout(connectWS, 2000);
}

function send(cmd, extra={}) {
  if (!wsock || wsock.readyState !== 1) { showToast('Belum terhubung'); return; }
  wsock.send(JSON.stringify({ cmd, ...extra }));
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
//  Message dispatcher
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
function dispatch(d) {
  if      (d.type === 'sensor')      onSensor(d);
  else if (d.type === 'config')      onConfig(d);
  else if (d.type === 'profiles')    onProfiles(d.profiles);
  else if (d.type === 'calibResult') onCalibResult(d);
  else if (d.type === 'toast')       showToast(d.msg);
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
//  Sensor data handler
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
let lastActiveCt = 0;
function onSensor(d) {
  // Sensor boxes
  let active = 0;
  for (let i = 0; i < 8; i++) {
    const el = document.getElementById('sb' + i);
    if (d.s[i]) { el.classList.add('on'); active++; }
    else           el.classList.remove('on');
    // ADC
    const ac = document.getElementById('adc' + i);
    if (ac && d.adc && d.adc[i] !== undefined) {
      ac.innerHTML = '<span class="alabel">S'+i+'</span>' + d.adc[i];
      if (d.s[i]) ac.classList.add('on'); else ac.classList.remove('on');
    }
  }
  lastActiveCt = active;

  // Position indicator
  const pos = d.pos || 0;
  const pct = ((pos + 3500) / 7000 * 100);
  document.getElementById('posInd').style.left = Math.min(Math.max(pct, 1), 99) + '%';
  document.getElementById('posVal').textContent = pos;
  document.getElementById('errVal').textContent = (d.err || 0).toFixed(1);
  document.getElementById('corrVal').textContent = (d.corr || 0).toFixed(2);
  document.getElementById('activeVal').textContent = active;

  // Motors
  const sA = d.sA || 0, sB = d.sB || 0;
  document.getElementById('mvA').textContent = Math.abs(sA);
  document.getElementById('mvB').textContent = Math.abs(sB);
  document.getElementById('mdA').innerHTML = sA > 0 ? '<i class="fa-solid fa-arrow-up fa-xs"></i> MAJU' : sA < 0 ? '<i class="fa-solid fa-arrow-down fa-xs"></i> MUNDUR' : '<i class="fa-solid fa-minus fa-xs"></i> BERHENTI';
  document.getElementById('mdB').innerHTML = sB > 0 ? '<i class="fa-solid fa-arrow-up fa-xs"></i> MAJU' : sB < 0 ? '<i class="fa-solid fa-arrow-down fa-xs"></i> MUNDUR' : '<i class="fa-solid fa-minus fa-xs"></i> BERHENTI';
  const bA = document.getElementById('mbA'), bB = document.getElementById('mbB');
  bA.style.width = (Math.abs(sA)/255*100) + '%';
  bB.style.width = (Math.abs(sB)/255*100) + '%';
  sA < 0 ? bA.classList.add('rev') : bA.classList.remove('rev');
  sB < 0 ? bB.classList.add('rev') : bB.classList.remove('rev');

  // State badge
  const STATES = { FOLLOW:'FOLLOW',STOPPED:'STOPPED',SPIN_L:'SPIN KIRI',
                   SPIN_R:'SPIN KANAN',FINISH:'SELESAI!',CALIBRATE:'KALIBRASI' };
  const SCLS   = { FOLLOW:'',STOPPED:'stopped',SPIN_L:'spin',SPIN_R:'spin',
                   FINISH:'finish',CALIBRATE:'calib' };
  const badge  = document.getElementById('stateBadge');
  badge.textContent = STATES[d.state] || d.state;
  badge.className   = 'badge ' + (SCLS[d.state] || '');

  // System info
  document.getElementById('si-state').textContent  = STATES[d.state] || d.state;
  document.getElementById('si-mode').textContent   = d.mode === 'web' ? 'Web Mode' : 'Direct Mode';
  document.getElementById('si-calib').textContent  = d.calibrated ? 'Sudah dikalibrasi' : 'Belum kalibrasi';

  // Mode buttons sync
  document.getElementById('modeWebBtn').classList.toggle('on', d.mode === 'web');
  document.getElementById('modeDirBtn').classList.toggle('on', d.mode === 'direct');

  // Calib progress
  if (d.calibProgress !== undefined) {
    const pct2 = d.calibProgress;
    document.getElementById('calibProg').classList.add('show');
    document.getElementById('progFill').style.width = pct2 + '%';
    const sec = Math.ceil((100 - pct2) / 20);
    document.getElementById('calibSec').textContent = sec + 's';
    if (pct2 >= 99) {
      document.getElementById('calibProg').classList.remove('show');
      document.getElementById('btnCalib').disabled = false;
    }
  }

  // Chart
  chartData.push(pos);
  if (chartData.length > 120) chartData.shift();
  drawChart();
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
//  Config handler
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
function onConfig(d) {
  function set(id, val) { const el=document.getElementById(id); if(el&&val!==undefined) el.value=val; }
  function setsv(key, val) { const el=document.getElementById('sv-'+key); if(el&&val!==undefined) el.textContent=val; }

  if (d.kp !== undefined) set('kp', parseFloat(d.kp).toFixed(3));
  if (d.ki !== undefined) set('ki', parseFloat(d.ki).toFixed(4));
  if (d.kd !== undefined) set('kd', parseFloat(d.kd).toFixed(3));
  if (d.baseSpeed   !== undefined) { set('baseSpeed',   d.baseSpeed);   setsv('base', d.baseSpeed);  }
  if (d.maxSpeed    !== undefined) { set('maxSpeed',    d.maxSpeed);    setsv('max',  d.maxSpeed);   }
  if (d.spinSpeed   !== undefined) { set('spinSpeed',   d.spinSpeed);   setsv('spin', d.spinSpeed);  }
  if (d.searchSpeed !== undefined) { set('searchSpeed', d.searchSpeed); setsv('srch', d.searchSpeed);}
  if (d.spinTime    !== undefined) { set('spinTime',    d.spinTime);    setsv('spint',d.spinTime);   }
  if (d.junction    !== undefined) { set('junction',    d.junction);    setsv('junc', d.junction);   }
  if (d.finish      !== undefined) { set('finish',      d.finish);      setsv('fin',  d.finish);     }

  // system info panel
  if (d.kp !== undefined)
    document.getElementById('si-pid').textContent =
      parseFloat(d.kp).toFixed(3)+' / '+parseFloat(d.ki).toFixed(4)+' / '+parseFloat(d.kd).toFixed(3);
  if (d.baseSpeed !== undefined)
    document.getElementById('si-speed').textContent = d.baseSpeed + ' / ' + d.maxSpeed;
  if (d.spinSpeed !== undefined)
    document.getElementById('si-spin').textContent = d.spinSpeed + ' / ' + d.spinTime + 'ms';
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
//  Profiles
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
function onProfiles(profiles) {
  const list = document.getElementById('pList');
  const lbl  = document.getElementById('pCountLbl');
  lbl.textContent = (profiles && profiles.length > 0) ? profiles.length + '/' + 8 + ' tersimpan' : '';

  if (!profiles || profiles.length === 0) {
    list.innerHTML = '<div style="text-align:center;color:var(--text2);font-size:.84rem;padding:18px"><i class="fa-solid fa-inbox" style="font-size:1.4rem;display:block;margin-bottom:5px;color:var(--g6)"></i>Belum ada profil tersimpan</div>';
    return;
  }
  list.innerHTML = profiles.map((p, i) => `
    <div class="pitem">
      <div style="flex:1;min-width:0">
        <div class="pname">${escHtml(p.name)}</div>
        <div class="pmeta">
          Kp=${p.kp} Ki=${p.ki} Kd=${p.kd}<br>
          Base=${p.baseSpeed} Max=${p.maxSpeed} Spin=${p.spinSpeed} SpinT=${p.spinTime}ms
        </div>
      </div>
      <div class="pbtn-wrap">
        <button class="pbtn pbtn-load" onclick="doLoadProfile(${i})"><i class='fa-solid fa-folder-open'></i> Load</button>
        <button class="pbtn pbtn-del" onclick="doDelProfile(${i})"><i class='fa-solid fa-trash'></i></button>
      </div>
    </div>
  `).join('');
}

function saveProfile() {
  const name = document.getElementById('pName').value.trim();
  if (!name) { showToast('<i class="fa-solid fa-triangle-exclamation"></i> Masukkan nama profil'); return; }
  send('saveProfile', { name });
  document.getElementById('pName').value = '';
  setTimeout(() => send('getProfiles'), 300);
  showToast('<i class="fa-solid fa-floppy-disk"></i> Profil "' + name + '" disimpan!');
}

function loadProfiles() { send('getProfiles'); }

function doLoadProfile(idx) {
  send('loadProfile', { idx });
  showToast('<i class="fa-solid fa-folder-open"></i> Profil dimuat!');
  setTimeout(() => { send('getConfig'); gotoTab('settings'); }, 300);
}

function doDelProfile(idx) {
  send('deleteProfile', { idx });
  setTimeout(() => send('getProfiles'), 300);
  showToast('<i class="fa-solid fa-trash"></i> Profil dihapus');
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
//  Calibration
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
function doCalib() {
  document.getElementById('btnCalib').disabled = true;
  document.getElementById('calibProg').classList.add('show');
  document.getElementById('progFill').style.width = '0%';
  document.getElementById('calibSec').textContent = '5s';
  document.getElementById('calibDone').style.display = 'none';
  document.getElementById('calibMsg').textContent = 'Gerakkan robot di atas hitam & putih...';
  send('calibrate');
}

function onCalibResult(d) {
  document.getElementById('calibProg').classList.remove('show');
  document.getElementById('btnCalib').disabled = false;
  const el = document.getElementById('calibDone');
  el.style.display = 'block';
  el.innerHTML = '<i class="fa-solid fa-circle-check" style="color:var(--g3)"></i> Kalibrasi selesai! Threshold: ' + (d.thresholds || []).join(', ');
  showToast('<i class="fa-solid fa-check"></i> Kalibrasi berhasil!');
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
//  Controls
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
function doStart() {
  send('start');
  showToast('<i class="fa-solid fa-play"></i> Robot START!');
}
function doStop() {
  send('stop');
  showToast('<i class="fa-solid fa-stop"></i> Robot STOP!');
}

function setMode(mode) {
  send('setMode', { mode });
  const desc = {
    web:    '<strong>Web Mode:</strong> Robot menunggu perintah START dari halaman ini. Cocok untuk kompetisi.',
    direct: '<strong>Direct Mode:</strong> Robot langsung jalan saat menyala. Tombol BOOT (GPIO0) toggle Start/Stop. Web tetap bisa STOP.'
  };
  document.getElementById('modeDesc').innerHTML = desc[mode] || '';
  showToast('Mode: ' + (mode === 'web' ? 'Web' : 'Direct'));
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
//  Settings
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
function sv(key, val) {
  document.getElementById('sv-' + key).textContent = val;
}

function applySettings() {
  const g = id => document.getElementById(id).value;
  const data = {
    kp:          parseFloat(g('kp'))      || 0,
    ki:          parseFloat(g('ki'))      || 0,
    kd:          parseFloat(g('kd'))      || 0,
    baseSpeed:   parseInt(g('baseSpeed')) || 150,
    maxSpeed:    parseInt(g('maxSpeed'))  || 200,
    spinSpeed:   parseInt(g('spinSpeed')) || 130,
    searchSpeed: parseInt(g('searchSpeed')) || 70,
    spinTime:    parseInt(g('spinTime'))  || 350,
    junction:    parseInt(g('junction'))  || 80,
    finish:      parseInt(g('finish'))    || 500,
  };
  send('setConfig', data);
  showToast('<i class="fa-solid fa-check"></i> Settings disimpan!');
}

function refreshSettings() {
  send('getConfig');
  showToast('<i class="fa-solid fa-rotate"></i> Refresh...');
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
//  Chart (canvas)
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
function initChart() {
  const c = document.getElementById('chart');
  if (!c) return;
  chartW = c.offsetWidth;
  chartH = 110;
  const dpr = window.devicePixelRatio || 1;
  c.width  = chartW * dpr;
  c.height = chartH * dpr;
  chartCtx = c.getContext('2d');
  chartCtx.scale(dpr, dpr);
}

function drawChart() {
  if (!chartCtx || !chartW) return;
  const w = chartW, h = chartH;
  chartCtx.clearRect(0, 0, w, h);

  // bg
  chartCtx.fillStyle = '#f1f8f2';
  chartCtx.fillRect(0, 0, w, h);

  // grid
  chartCtx.strokeStyle = '#c8e6c9';
  chartCtx.lineWidth = 0.5;
  for (let i = 1; i <= 3; i++) {
    const y = (i / 4) * h;
    chartCtx.beginPath(); chartCtx.moveTo(0,y); chartCtx.lineTo(w,y); chartCtx.stroke();
  }

  // zero line
  chartCtx.strokeStyle = '#81c784';
  chartCtx.lineWidth = 1;
  chartCtx.setLineDash([5,5]);
  chartCtx.beginPath(); chartCtx.moveTo(0,h/2); chartCtx.lineTo(w,h/2); chartCtx.stroke();
  chartCtx.setLineDash([]);

  // data
  const n = chartData.length;
  if (n < 2) return;
  chartCtx.strokeStyle = '#1b5e20';
  chartCtx.lineWidth = 1.8;
  chartCtx.beginPath();
  for (let i = 0; i < n; i++) {
    const x = (i / (n-1)) * w;
    const y = h/2 - (chartData[i] / 3500) * (h/2 - 5);
    i === 0 ? chartCtx.moveTo(x,y) : chartCtx.lineTo(x,y);
  }
  chartCtx.stroke();

  // labels
  chartCtx.fillStyle = '#388e3c';
  chartCtx.font = '10px sans-serif';
  chartCtx.fillText('+3500', 3, 12);
  chartCtx.fillText('  0',   3, h/2 + 4);
  chartCtx.fillText('-3500', 3, h - 3);
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
//  Tab navigation
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
const TAB_IDS = ['monitor','control','settings','profiles','guide'];
function gotoTab(name) {
  TAB_IDS.forEach((t,i) => {
    document.getElementById('pg-'+t).classList.toggle('on', t===name);
    document.querySelectorAll('.tab')[i].classList.toggle('on', t===name);
  });
  if (name === 'monitor') setTimeout(() => { initChart(); drawChart(); }, 30);
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
//  Helpers
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
function setDot(on) {
  document.getElementById('wsDot').classList.toggle('on', on);
}

let toastTimer;
function showToast(msg) {
  const el = document.getElementById('toast');
  el.innerHTML = msg;
  el.classList.add('show');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => el.classList.remove('show'), 2500);
}

function escHtml(s) {
  return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;');
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
//  Init
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
window.addEventListener('load', () => {
  initChart();
  connectWS();
});
window.addEventListener('resize', () => { initChart(); drawChart(); });
</script>
</body>
</html>
)rawliteral";

// ══════════════════════════════════════════════════════════════
//  FORWARD DECLARATIONS
// ══════════════════════════════════════════════════════════════
void readSensors();
int  countActive();
int32_t weightedPosition();
void driveMotors(int sA, int sB);
void stopMotors();
void spinTurn(bool left);
void handleAllBlack();
void runStateMachine();
void broadcastSensorData();
void broadcastConfig(AsyncWebSocketClient* client);
void broadcastProfiles();
void loadConfigFromPrefs();
void saveConfigToPrefs();
void onWsEvent(AsyncWebSocket* srv, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len);
void handleWsMessage(AsyncWebSocketClient* client, String msg);

// ══════════════════════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Serial.println("\n[BOOT] Line Follower Web Control v1.0");

  // ── Motor & standby ───────────────────────────────────────────
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  stopMotors();

  // ── ESP32 Core 3.x PWM ───────────────────────────────────────
  ledcAttach(PWMA, PWM_FREQ, PWM_RES);
  ledcAttach(PWMB, PWM_FREQ, PWM_RES);
  ledcWrite(PWMA, 0);
  ledcWrite(PWMB, 0);

  // ── Sensor & button pins ──────────────────────────────────────
  for (int i = 0; i < SENSOR_COUNT; i++) {
    pinMode(SENSOR_PINS[i], INPUT);
    sensorMin[i] = 4095;
    sensorMax[i] = 0;
    g_adcRaw[i]  = 0;
  }
  pinMode(START_BTN_PIN, INPUT_PULLUP);

  // ── Load saved settings from flash ───────────────────────────
  prefs.begin("lf", false);   // namespace "lf" (max 15 char)
  loadConfigFromPrefs();

  // ── WiFi Access Point ─────────────────────────────────────────
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.printf("[WiFi] AP: %-16s | IP: %s\n",
                AP_SSID, WiFi.softAPIP().toString().c_str());

  // ── WebSocket ─────────────────────────────────────────────────
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  // ── HTTP server ───────────────────────────────────────────────
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send_P(200, "text/html", INDEX_HTML);
  });
  server.begin();
  Serial.println("[HTTP] Server started → http://192.168.4.1");

  // ── Direct mode: auto-start ───────────────────────────────────
  if (g_startMode == MODE_DIRECT) {
    g_running = true;
    g_state   = ST_FOLLOW;
    Serial.println("[AUTO] Direct mode: robot started");
  } else {
    Serial.println("[WAIT] Web mode: waiting for web START command");
  }
}

// ══════════════════════════════════════════════════════════════
//  MAIN LOOP
// ══════════════════════════════════════════════════════════════
void loop() {
  ws.cleanupClients();

  // ── Calibration mode (non-blocking sweep) ────────────────────
  if (g_calibrating) {
    uint32_t elapsed = millis() - g_calibStart;

    if (elapsed < CALIB_DURATION_MS) {
      // Sample sensors continuously
      for (int i = 0; i < SENSOR_COUNT; i++) {
        int v = analogRead(SENSOR_PINS[i]);
        if (v < sensorMin[i]) sensorMin[i] = v;
        if (v > sensorMax[i]) sensorMax[i] = v;
        g_adcRaw[i] = v;
        // derive digital during calib too
        int thr = (sensorMin[i] + sensorMax[i]) / 2;
        if (thr < 100) thr = cfg.threshold;  // guard early in calib
        g_sensor[i] = (v >= thr);
      }

      // Broadcast progress every 100ms
      if (millis() - lastWsUpdate >= 100) {
        int prog = (int)(elapsed * 100 / CALIB_DURATION_MS);
        StaticJsonDocument<320> doc;
        doc["type"]          = "sensor";
        doc["state"]         = "CALIBRATE";
        doc["mode"]          = (g_startMode == MODE_WEB) ? "web" : "direct";
        doc["calibrated"]    = g_calibrated;
        doc["calibProgress"] = prog;
        doc["pos"]  = 0; doc["err"] = 0; doc["corr"] = 0;
        doc["sA"]   = 0; doc["sB"]  = 0;
        JsonArray sa = doc.createNestedArray("s");
        JsonArray ad = doc.createNestedArray("adc");
        for (int i = 0; i < SENSOR_COUNT; i++) {
          sa.add(g_sensor[i] ? 1 : 0);
          ad.add(g_adcRaw[i]);
        }
        String out; serializeJson(doc, out);
        ws.textAll(out);
        lastWsUpdate = millis();
      }
    } else {
      // ── Calibration DONE ──────────────────────────────────────
      g_calibrating = false;
      g_calibrated  = true;
      g_state       = ST_STOPPED;
      g_running     = false;

      // Save calibration data to flash
      prefs.putBytes("calMin", sensorMin, sizeof(sensorMin));
      prefs.putBytes("calMax", sensorMax, sizeof(sensorMax));
      prefs.putBool("calibd", true);

      // Report results
      Serial.println("[CALIB] Done! Thresholds:");
      StaticJsonDocument<256> doc;
      doc["type"] = "calibResult";
      JsonArray th = doc.createNestedArray("thresholds");
      for (int i = 0; i < SENSOR_COUNT; i++) {
        int mid = (sensorMin[i] + sensorMax[i]) / 2;
        th.add(mid);
        Serial.printf("  S%d: min=%4d max=%4d mid=%4d\n",
                      i, sensorMin[i], sensorMax[i], mid);
      }
      String out; serializeJson(doc, out);
      ws.textAll(out);
    }
    return;  // Skip robot logic during calibration
  }

  // ── Physical button (Direct Mode toggle) ─────────────────────
  if (g_startMode == MODE_DIRECT) {
    bool pressed = (digitalRead(START_BTN_PIN) == LOW);
    if (pressed && (millis() - lastBtnPress > 500)) {
      lastBtnPress = millis();
      if (!g_running) {
        g_running   = true;
        g_state     = ST_FOLLOW;
        g_integral  = 0;
        g_lastError = 0;
        Serial.println("[BTN] START");
      } else {
        g_running = false;
        g_state   = ST_STOPPED;
        stopMotors();
        Serial.println("[BTN] STOP");
      }
    }
  }

  // ── Main robot logic ──────────────────────────────────────────
  if (g_running && g_state != ST_FINISH) {
    readSensors();
    runStateMachine();
  } else if (!g_running) {
    stopMotors();
  }

  // ── Broadcast sensor data ─────────────────────────────────────
  if (millis() - lastWsUpdate >= WS_UPDATE_MS) {
    broadcastSensorData();
    lastWsUpdate = millis();
  }
}

// ══════════════════════════════════════════════════════════════
//  STATE MACHINE
// ══════════════════════════════════════════════════════════════
void runStateMachine() {
  switch (g_state) {

    // ── PID Line Follow ────────────────────────────────────────
    case ST_FOLLOW: {
      int active = countActive();

      if (active == SENSOR_COUNT) {
        handleAllBlack();
        break;
      }
      if (g_allBlackActive) {
        g_allBlackActive = false;
        g_integral = 0;
      }
      if (active == 0) {
        driveMotors(cfg.searchSpeed, cfg.searchSpeed);
        break;
      }

      // Dead-end / T-junction detection
      bool leftA   = g_sensor[0] || g_sensor[1] || g_sensor[2];
      bool centerA = g_sensor[3] || g_sensor[4];
      bool rightA  = g_sensor[5] || g_sensor[6] || g_sensor[7];

      if (leftA && !centerA && !rightA && active <= 3) {
        g_state = ST_SPIN_L;
        break;
      }
      if (rightA && !centerA && !leftA && active <= 3) {
        g_state = ST_SPIN_R;
        break;
      }

      // PID
      int32_t pos  = weightedPosition();
      float   err  = (float)pos;
      g_integral  += err;
      g_integral   = constrain(g_integral, -6000.0f, 6000.0f);
      float deriv  = err - g_lastError;
      g_correction = (cfg.kp * err) + (cfg.ki * g_integral) + (cfg.kd * deriv);
      g_lastError  = err;

      g_speedA = constrain((int)(cfg.baseSpeed + g_correction), -cfg.maxSpeed, cfg.maxSpeed);
      g_speedB = constrain((int)(cfg.baseSpeed - g_correction), -cfg.maxSpeed, cfg.maxSpeed);
      driveMotors(g_speedA, g_speedB);
      break;
    }

    case ST_SPIN_L:
      stopMotors(); delay(60);
      spinTurn(true);
      g_integral = 0; g_lastError = 0;
      g_state = ST_FOLLOW;
      Serial.println("[SPIN] Left 90°");
      break;

    case ST_SPIN_R:
      stopMotors(); delay(60);
      spinTurn(false);
      g_integral = 0; g_lastError = 0;
      g_state = ST_FOLLOW;
      Serial.println("[SPIN] Right 90°");
      break;

    case ST_FINISH:
      stopMotors();
      g_running = false;
      Serial.println("[FINISH] Robot stopped.");
      break;

    default:
      break;
  }
}

// ══════════════════════════════════════════════════════════════
//  SENSOR FUNCTIONS
// ══════════════════════════════════════════════════════════════
void readSensors() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    int v        = analogRead(SENSOR_PINS[i]);
    g_adcRaw[i]  = (uint16_t)v;
    int thr      = g_calibrated
                   ? (sensorMin[i] + sensorMax[i]) / 2
                   : cfg.threshold;
    g_sensor[i]  = (v >= thr);
  }
}

int countActive() {
  int n = 0;
  for (int i = 0; i < SENSOR_COUNT; i++) n += (int)g_sensor[i];
  return n;
}

int32_t weightedPosition() {
  int32_t wsum = 0; int ac = 0;
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (g_sensor[i]) { wsum += SENSOR_WEIGHT[i]; ac++; }
  }
  if (!ac) return g_lastPos;
  g_lastPos = wsum / ac;
  return g_lastPos;
}

// ══════════════════════════════════════════════════════════════
//  MOTOR FUNCTIONS
// ══════════════════════════════════════════════════════════════
void driveMotors(int sA, int sB) {
  // Motor A (KIRI)
  if (sA >= 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);  }
  else          { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); sA = -sA; }
  ledcWrite(PWMA, (uint8_t)min(sA, 255));

  // Motor B (KANAN)
  if (sB >= 0) { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);  }
  else          { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); sB = -sB; }
  ledcWrite(PWMB, (uint8_t)min(sB, 255));
}

void stopMotors() {
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
  ledcWrite(PWMA, 0);
  ledcWrite(PWMB, 0);
  g_speedA = 0;
  g_speedB = 0;
}

// Pivot turn: satu motor maju, satu mundur
// left=true  → motor kiri mundur, kanan maju  → berputar kiri
// left=false → motor kiri maju,  kanan mundur → berputar kanan
void spinTurn(bool left) {
  if (left) {
    digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH);
  }
  ledcWrite(PWMA, (uint8_t)cfg.spinSpeed);
  ledcWrite(PWMB, (uint8_t)cfg.spinSpeed);
  delay(cfg.spinTimeMs);  // ← dikalibrasi untuk tepat 90°
  stopMotors();
  delay(80);   // settle time
}

// ══════════════════════════════════════════════════════════════
//  JUNCTION / FINISH DETECTOR
// ══════════════════════════════════════════════════════════════
void handleAllBlack() {
  uint32_t now = millis();
  if (!g_allBlackActive) {
    g_allBlackActive = true;
    g_allBlackStart  = now;
    driveMotors(cfg.baseSpeed, cfg.baseSpeed);
    return;
  }
  uint32_t held = now - g_allBlackStart;
  if (held >= (uint32_t)cfg.finishMs) {
    Serial.printf("[FINISH] All-black %u ms\n", held);
    g_state = ST_FINISH;
    return;
  }
  int sp = (held < (uint32_t)cfg.junctionMs) ? cfg.baseSpeed : cfg.baseSpeed - 20;
  driveMotors(sp, sp);
}

// ══════════════════════════════════════════════════════════════
//  BROADCAST SENSOR DATA VIA WEBSOCKET
// ══════════════════════════════════════════════════════════════
void broadcastSensorData() {
  if (ws.count() == 0) return;

  const char* stateStr = "STOPPED";
  switch (g_state) {
    case ST_FOLLOW:    stateStr = "FOLLOW";    break;
    case ST_SPIN_L:    stateStr = "SPIN_L";    break;
    case ST_SPIN_R:    stateStr = "SPIN_R";    break;
    case ST_FINISH:    stateStr = "FINISH";    break;
    case ST_CALIBRATE: stateStr = "CALIBRATE"; break;
    default: break;
  }

  StaticJsonDocument<512> doc;
  doc["type"]      = "sensor";
  doc["state"]     = stateStr;
  doc["mode"]      = (g_startMode == MODE_WEB) ? "web" : "direct";
  doc["calibrated"]= g_calibrated;
  doc["running"]   = g_running;
  doc["pos"]       = g_lastPos;
  doc["err"]       = g_lastError;
  doc["corr"]      = g_correction;
  doc["sA"]        = g_speedA;
  doc["sB"]        = g_speedB;

  JsonArray sa = doc.createNestedArray("s");
  JsonArray ad = doc.createNestedArray("adc");
  for (int i = 0; i < SENSOR_COUNT; i++) {
    sa.add(g_sensor[i] ? 1 : 0);
    ad.add((int)g_adcRaw[i]);
  }

  String out;
  serializeJson(doc, out);
  ws.textAll(out);
}

// ══════════════════════════════════════════════════════════════
//  CONFIG PERSISTENCE
// ══════════════════════════════════════════════════════════════
void loadConfigFromPrefs() {
  cfg.kp          = prefs.getFloat("kp",    0.045f);
  cfg.ki          = prefs.getFloat("ki",    0.000f);
  cfg.kd          = prefs.getFloat("kd",    0.030f);
  cfg.baseSpeed   = prefs.getInt("base",    150);
  cfg.maxSpeed    = prefs.getInt("maxS",    200);
  cfg.spinSpeed   = prefs.getInt("spnS",    130);
  cfg.searchSpeed = prefs.getInt("srch",    70);
  cfg.spinTimeMs  = prefs.getInt("spnT",    350);
  cfg.junctionMs  = prefs.getInt("junc",    80);
  cfg.finishMs    = prefs.getInt("fin",     500);
  g_startMode     = prefs.getBool("dMode",  false) ? MODE_DIRECT : MODE_WEB;

  // Restore calibration data if available
  if (prefs.getBool("calibd", false)) {
    prefs.getBytes("calMin", sensorMin, sizeof(sensorMin));
    prefs.getBytes("calMax", sensorMax, sizeof(sensorMax));
    g_calibrated = true;
    Serial.println("[PREFS] Calibration restored");
  }
  Serial.printf("[PREFS] Loaded: Kp=%.3f Ki=%.4f Kd=%.3f Base=%d\n",
                cfg.kp, cfg.ki, cfg.kd, cfg.baseSpeed);
}

void saveConfigToPrefs() {
  prefs.putFloat("kp",   cfg.kp);
  prefs.putFloat("ki",   cfg.ki);
  prefs.putFloat("kd",   cfg.kd);
  prefs.putInt("base",   cfg.baseSpeed);
  prefs.putInt("maxS",   cfg.maxSpeed);
  prefs.putInt("spnS",   cfg.spinSpeed);
  prefs.putInt("srch",   cfg.searchSpeed);
  prefs.putInt("spnT",   cfg.spinTimeMs);
  prefs.putInt("junc",   cfg.junctionMs);
  prefs.putInt("fin",    cfg.finishMs);
  prefs.putBool("dMode", g_startMode == MODE_DIRECT);
}

// ══════════════════════════════════════════════════════════════
//  BROADCAST CONFIG TO WEBSOCKET CLIENT
// ══════════════════════════════════════════════════════════════
void broadcastConfig(AsyncWebSocketClient* client) {
  StaticJsonDocument<320> doc;
  doc["type"]        = "config";
  doc["kp"]          = cfg.kp;
  doc["ki"]          = cfg.ki;
  doc["kd"]          = cfg.kd;
  doc["baseSpeed"]   = cfg.baseSpeed;
  doc["maxSpeed"]    = cfg.maxSpeed;
  doc["spinSpeed"]   = cfg.spinSpeed;
  doc["searchSpeed"] = cfg.searchSpeed;
  doc["spinTime"]    = cfg.spinTimeMs;
  doc["junction"]    = cfg.junctionMs;
  doc["finish"]      = cfg.finishMs;
  String out;
  serializeJson(doc, out);
  if (client) client->text(out);
  else        ws.textAll(out);
}

// ══════════════════════════════════════════════════════════════
//  BROADCAST PROFILES LIST
// ══════════════════════════════════════════════════════════════
void broadcastProfiles() {
  int count = prefs.getInt("pCnt", 0);
  StaticJsonDocument<1536> doc;
  doc["type"] = "profiles";
  JsonArray arr = doc.createNestedArray("profiles");
  char key[14];

  for (int i = 0; i < count; i++) {
    JsonObject p = arr.createNestedObject();
    snprintf(key, sizeof(key), "p%d_n",  i); p["name"]       = prefs.getString(key, "");
    snprintf(key, sizeof(key), "p%d_kp", i); p["kp"]         = prefs.getFloat(key,  0.045f);
    snprintf(key, sizeof(key), "p%d_ki", i); p["ki"]         = prefs.getFloat(key,  0.0f);
    snprintf(key, sizeof(key), "p%d_kd", i); p["kd"]         = prefs.getFloat(key,  0.03f);
    snprintf(key, sizeof(key), "p%d_bs", i); p["baseSpeed"]  = prefs.getInt(key,    150);
    snprintf(key, sizeof(key), "p%d_ms", i); p["maxSpeed"]   = prefs.getInt(key,    200);
    snprintf(key, sizeof(key), "p%d_ss", i); p["spinSpeed"]  = prefs.getInt(key,    130);
    snprintf(key, sizeof(key), "p%d_rs", i); p["searchSpeed"]= prefs.getInt(key,    70);
    snprintf(key, sizeof(key), "p%d_st", i); p["spinTime"]   = prefs.getInt(key,    350);
  }
  String out; serializeJson(doc, out);
  ws.textAll(out);
}

// ══════════════════════════════════════════════════════════════
//  WEBSOCKET EVENT HANDLER
// ══════════════════════════════════════════════════════════════
void onWsEvent(AsyncWebSocket* srv, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len) {

  if (type == WS_EVT_CONNECT) {
    Serial.printf("[WS] Client #%u connected from %s\n",
                  client->id(), client->remoteIP().toString().c_str());
    broadcastConfig(client);
    broadcastProfiles();
  }
  else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("[WS] Client #%u disconnected\n", client->id());
  }
  else if (type == WS_EVT_DATA) {
    AwsFrameInfo* info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len
        && info->opcode == WS_TEXT) {
      // Null-terminate & parse
      String msg = "";
      msg.reserve(len + 1);
      for (size_t i = 0; i < len; i++) msg += (char)data[i];
      handleWsMessage(client, msg);
    }
  }
}

// ══════════════════════════════════════════════════════════════
//  WEBSOCKET MESSAGE HANDLER
// ══════════════════════════════════════════════════════════════
void handleWsMessage(AsyncWebSocketClient* client, String msg) {
  StaticJsonDocument<512> doc;
  if (deserializeJson(doc, msg) != DeserializationError::Ok) return;

  const char* cmd = doc["cmd"] | "";

  // ── START ────────────────────────────────────────────────────
  if (strcmp(cmd, "start") == 0) {
    if (!g_calibrating) {
      g_running   = true;
      g_state     = ST_FOLLOW;
      g_integral  = 0;
      g_lastError = 0;
      g_correction= 0;
      Serial.println("[CMD] START");
    }
  }

  // ── STOP ─────────────────────────────────────────────────────
  else if (strcmp(cmd, "stop") == 0) {
    g_running = false;
    g_state   = ST_STOPPED;
    stopMotors();
    Serial.println("[CMD] STOP");
  }

  // ── CALIBRATE ────────────────────────────────────────────────
  else if (strcmp(cmd, "calibrate") == 0) {
    if (g_running) { g_running = false; stopMotors(); }
    for (int i = 0; i < SENSOR_COUNT; i++) {
      sensorMin[i] = 4095;
      sensorMax[i] = 0;
    }
    g_calibrating = true;
    g_calibStart  = millis();
    g_state       = ST_CALIBRATE;
    Serial.println("[CMD] CALIBRATE started");
  }

  // ── SET CONFIG ───────────────────────────────────────────────
  else if (strcmp(cmd, "setConfig") == 0) {
    if (doc.containsKey("kp"))          cfg.kp          = doc["kp"].as<float>();
    if (doc.containsKey("ki"))          cfg.ki          = doc["ki"].as<float>();
    if (doc.containsKey("kd"))          cfg.kd          = doc["kd"].as<float>();
    if (doc.containsKey("baseSpeed"))   cfg.baseSpeed   = doc["baseSpeed"].as<int>();
    if (doc.containsKey("maxSpeed"))    cfg.maxSpeed    = doc["maxSpeed"].as<int>();
    if (doc.containsKey("spinSpeed"))   cfg.spinSpeed   = doc["spinSpeed"].as<int>();
    if (doc.containsKey("searchSpeed")) cfg.searchSpeed = doc["searchSpeed"].as<int>();
    if (doc.containsKey("spinTime"))    cfg.spinTimeMs  = doc["spinTime"].as<int>();
    if (doc.containsKey("junction"))    cfg.junctionMs  = doc["junction"].as<int>();
    if (doc.containsKey("finish"))      cfg.finishMs    = doc["finish"].as<int>();
    saveConfigToPrefs();
    broadcastConfig(nullptr);
    Serial.printf("[CFG] Updated: Kp=%.3f Kd=%.3f Base=%d\n",
                  cfg.kp, cfg.kd, cfg.baseSpeed);
  }

  // ── GET CONFIG ───────────────────────────────────────────────
  else if (strcmp(cmd, "getConfig") == 0) {
    broadcastConfig(client);
  }

  // ── SET MODE ─────────────────────────────────────────────────
  else if (strcmp(cmd, "setMode") == 0) {
    const char* mode = doc["mode"] | "web";
    bool isDirect    = (strcmp(mode, "direct") == 0);
    g_startMode      = isDirect ? MODE_DIRECT : MODE_WEB;
    saveConfigToPrefs();
    // Direct mode: auto-start
    if (isDirect && !g_running) {
      g_running   = true;
      g_state     = ST_FOLLOW;
      g_integral  = 0;
      g_lastError = 0;
    }
    Serial.printf("[MODE] %s\n", mode);
  }

  // ── SAVE PROFILE ─────────────────────────────────────────────
  else if (strcmp(cmd, "saveProfile") == 0) {
    const char* name = doc["name"] | "";
    if (strlen(name) > 0) {
      int count = prefs.getInt("pCnt", 0);
      if (count < MAX_PROFILES) {
        char key[14];
        snprintf(key, sizeof(key), "p%d_n",  count); prefs.putString(key, name);
        snprintf(key, sizeof(key), "p%d_kp", count); prefs.putFloat(key,  cfg.kp);
        snprintf(key, sizeof(key), "p%d_ki", count); prefs.putFloat(key,  cfg.ki);
        snprintf(key, sizeof(key), "p%d_kd", count); prefs.putFloat(key,  cfg.kd);
        snprintf(key, sizeof(key), "p%d_bs", count); prefs.putInt(key,    cfg.baseSpeed);
        snprintf(key, sizeof(key), "p%d_ms", count); prefs.putInt(key,    cfg.maxSpeed);
        snprintf(key, sizeof(key), "p%d_ss", count); prefs.putInt(key,    cfg.spinSpeed);
        snprintf(key, sizeof(key), "p%d_rs", count); prefs.putInt(key,    cfg.searchSpeed);
        snprintf(key, sizeof(key), "p%d_st", count); prefs.putInt(key,    cfg.spinTimeMs);
        prefs.putInt("pCnt", count + 1);
        broadcastProfiles();
        Serial.printf("[PROF] Saved '%s' at slot %d\n", name, count);
      } else {
        Serial.println("[PROF] Max profiles reached!");
      }
    }
  }

  // ── GET PROFILES ─────────────────────────────────────────────
  else if (strcmp(cmd, "getProfiles") == 0) {
    broadcastProfiles();
  }

  // ── LOAD PROFILE ─────────────────────────────────────────────
  else if (strcmp(cmd, "loadProfile") == 0) {
    int idx   = doc["idx"].as<int>();
    int count = prefs.getInt("pCnt", 0);
    if (idx >= 0 && idx < count) {
      char key[14];
      snprintf(key, sizeof(key), "p%d_kp", idx); cfg.kp          = prefs.getFloat(key, cfg.kp);
      snprintf(key, sizeof(key), "p%d_ki", idx); cfg.ki          = prefs.getFloat(key, cfg.ki);
      snprintf(key, sizeof(key), "p%d_kd", idx); cfg.kd          = prefs.getFloat(key, cfg.kd);
      snprintf(key, sizeof(key), "p%d_bs", idx); cfg.baseSpeed   = prefs.getInt(key,   cfg.baseSpeed);
      snprintf(key, sizeof(key), "p%d_ms", idx); cfg.maxSpeed    = prefs.getInt(key,   cfg.maxSpeed);
      snprintf(key, sizeof(key), "p%d_ss", idx); cfg.spinSpeed   = prefs.getInt(key,   cfg.spinSpeed);
      snprintf(key, sizeof(key), "p%d_rs", idx); cfg.searchSpeed = prefs.getInt(key,   cfg.searchSpeed);
      snprintf(key, sizeof(key), "p%d_st", idx); cfg.spinTimeMs  = prefs.getInt(key,   cfg.spinTimeMs);
      saveConfigToPrefs();
      broadcastConfig(nullptr);
      Serial.printf("[PROF] Loaded slot %d\n", idx);
    }
  }

  // ── DELETE PROFILE ───────────────────────────────────────────
  else if (strcmp(cmd, "deleteProfile") == 0) {
    int idx   = doc["idx"].as<int>();
    int count = prefs.getInt("pCnt", 0);
    if (idx >= 0 && idx < count) {
      // Shift all slots above idx down by one
      char src[14], dst[14];
      for (int i = idx; i < count - 1; i++) {
        snprintf(src,sizeof(src),"p%d_n", i+1); snprintf(dst,sizeof(dst),"p%d_n", i);
        prefs.putString(dst, prefs.getString(src, ""));
        snprintf(src,sizeof(src),"p%d_kp",i+1); snprintf(dst,sizeof(dst),"p%d_kp",i);
        prefs.putFloat(dst, prefs.getFloat(src, 0));
        snprintf(src,sizeof(src),"p%d_ki",i+1); snprintf(dst,sizeof(dst),"p%d_ki",i);
        prefs.putFloat(dst, prefs.getFloat(src, 0));
        snprintf(src,sizeof(src),"p%d_kd",i+1); snprintf(dst,sizeof(dst),"p%d_kd",i);
        prefs.putFloat(dst, prefs.getFloat(src, 0));
        snprintf(src,sizeof(src),"p%d_bs",i+1); snprintf(dst,sizeof(dst),"p%d_bs",i);
        prefs.putInt(dst, prefs.getInt(src, 0));
        snprintf(src,sizeof(src),"p%d_ms",i+1); snprintf(dst,sizeof(dst),"p%d_ms",i);
        prefs.putInt(dst, prefs.getInt(src, 0));
        snprintf(src,sizeof(src),"p%d_ss",i+1); snprintf(dst,sizeof(dst),"p%d_ss",i);
        prefs.putInt(dst, prefs.getInt(src, 0));
        snprintf(src,sizeof(src),"p%d_rs",i+1); snprintf(dst,sizeof(dst),"p%d_rs",i);
        prefs.putInt(dst, prefs.getInt(src, 0));
        snprintf(src,sizeof(src),"p%d_st",i+1); snprintf(dst,sizeof(dst),"p%d_st",i);
        prefs.putInt(dst, prefs.getInt(src, 0));
      }
      prefs.putInt("pCnt", count - 1);
      broadcastProfiles();
      Serial.printf("[PROF] Deleted slot %d\n", idx);
    }
  }
}

/*
 * ════════════════════════════════════════════════════════════════
 *  RINGKASAN LIBRARY & INSTALASI
 * ════════════════════════════════════════════════════════════════
 *
 *  1. Install Arduino IDE + ESP32 Board Support
 *     Tools → Board → ESP32 Arduino (Espressif Systems)
 *
 *  2. Install library via Library Manager:
 *     • ArduinoJson (by Benoit Blanchon) — versi 6.x
 *
 *  3. Install library manual (download ZIP, import via Sketch → Include Library):
 *     • ESPAsyncWebServer → https://github.com/me-no-dev/ESPAsyncWebServer
 *     • AsyncTCP         → https://github.com/me-no-dev/AsyncTCP
 *
 *  4. Board settings:
 *     • Board     : ESP32 Dev Module (atau sesuai hardware)
 *     • Flash Size: 4MB
 *     • Upload Speed: 115200 atau 921600
 *     • Partition : Default 4MB with spiffs  (atau Default)
 *
 *  5. Compile & upload. Setelah upload:
 *     • Hubungkan ke WiFi "LineFollower" (pass: robot1234)
 *     • Buka browser: http://192.168.4.1
 *
 * ════════════════════════════════════════════════════════════════
 *  TROUBLESHOOTING
 * ════════════════════════════════════════════════════════════════
 *
 *  Error "ledcAttach not declared":
 *    → Update ESP32 board support ke versi 3.x
 *
 *  Sensor 6/7 tidak baca dengan benar:
 *    → ADC2 (pin 27,14) konflik WiFi
 *    → Ganti ke pin ADC1: 36,39 di array SENSOR_PINS
 *
 *  Motor tidak berputar / berputar terbalik:
 *    → Cek STBY=HIGH, lalu tukar AIN1↔AIN2 atau BIN1↔BIN2
 *
 *  Web tidak bisa diakses:
 *    → Pastikan terhubung ke WiFi "LineFollower"
 *    → Coba: http://192.168.4.1 (bukan https)
 *
 *  Robot terlalu zig-zag:
 *    → Kp terlalu besar, turunkan 20%
 *    → Naikkan Kd untuk meredam osilasi
 *
 *  Robot tidak bisa belok 90°:
 *    → Kalibrasi Spin Time Ms di halaman Settings
 *    → Mulai dari 200ms, naikkan bertahap
 * ════════════════════════════════════════════════════════════════
 */
