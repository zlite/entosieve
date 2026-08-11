#include <Arduino.h>
#include <M5Stack.h>
#include <Module_Stepmotor.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>

namespace {
constexpr char WIFI_SSID[] = "Archive_01";
constexpr char WIFI_PASS[] = "Barcodes4@ll";

constexpr uint8_t STEP_X_PIN = 16;
constexpr uint8_t DIR_X_PIN = 17;
constexpr uint8_t STEP_DRIVER_I2C_ADDRESS = 0x27;
constexpr uint8_t TOP_LIMIT_INDEX = 0;     // L0
constexpr uint8_t BOTTOM_LIMIT_INDEX = 1;  // L1
constexpr bool LIMIT_ACTIVE_LOW = true;

// Calibrate this for the lead screw/pulley attached to the stepper.
constexpr float STEPS_PER_CM = 100.0f;
constexpr float DEFAULT_TOP_CM = 2.0f;
constexpr float DEFAULT_BOTTOM_CM = 0.0f;
constexpr float DEFAULT_SPEED_CM_S = 0.5f;
constexpr uint16_t DEFAULT_RUN_DURATION_SECONDS = 60;
constexpr long HOMING_TRAVEL_CM = 100;

enum class MotionState {
  Idle,
  HomingToTop,
  HomingToBottom,
  CyclingUp,
  CyclingDown,
  Jogging,
  Paused,
  Stopped,
};

Module_Stepmotor driver;
WebServer server(80);
Preferences preferences;

MotionState motionState = MotionState::Idle;
MotionState pausedState = MotionState::Idle;
float topPositionCm = DEFAULT_TOP_CM;
float bottomPositionCm = DEFAULT_BOTTOM_CM;
float speedCmS = DEFAULT_SPEED_CM_S;
float activeSpeedCmS = DEFAULT_SPEED_CM_S;
long currentPositionSteps = 0;
long targetPositionSteps = 0;
unsigned long lastStepUs = 0;
unsigned long lastLimitPollMs = 0;
bool wifiReady = false;
bool driverReady = false;
bool motorEnabled = false;
bool sequenceActive = false;
uint16_t completedCycles = 0;
uint16_t runDurationSeconds = DEFAULT_RUN_DURATION_SECONDS;
unsigned long sequenceStartedMs = 0;
unsigned long pauseStartedMs = 0;
unsigned long accumulatedPauseMs = 0;
unsigned long lastDisplayUpdateMs = 0;
bool displayLayoutDrawn = false;

long cmToSteps(float cm) {
  return lroundf(cm * STEPS_PER_CM);
}

float stepsToCm(long steps) {
  return static_cast<float>(steps) / STEPS_PER_CM;
}

bool topLimitTriggered() {
  const bool level = driver.ext_io_status[TOP_LIMIT_INDEX] != 0;
  return LIMIT_ACTIVE_LOW ? !level : level;
}

bool bottomLimitTriggered() {
  const bool level = driver.ext_io_status[BOTTOM_LIMIT_INDEX] != 0;
  return LIMIT_ACTIVE_LOW ? !level : level;
}

const char *stateName() {
  switch (motionState) {
    case MotionState::Idle:
      return "Idle";
    case MotionState::HomingToTop:
      return "Homing top";
    case MotionState::HomingToBottom:
      return "Homing bottom";
    case MotionState::CyclingUp:
      return "Moving up";
    case MotionState::CyclingDown:
      return "Moving down";
    case MotionState::Jogging:
      return "Jogging";
    case MotionState::Paused:
      return "Paused";
    case MotionState::Stopped:
      return "Stopped";
  }
  return "Unknown";
}

void applySpeed() {
  speedCmS = max(0.1f, speedCmS);
}

void loadSettings() {
  preferences.begin("entosieve", true);
  topPositionCm = preferences.getFloat("top_cm", DEFAULT_TOP_CM);
  bottomPositionCm = preferences.getFloat("bottom_cm", DEFAULT_BOTTOM_CM);
  speedCmS = preferences.getFloat("speed", DEFAULT_SPEED_CM_S);
  runDurationSeconds =
      preferences.getUShort("duration", DEFAULT_RUN_DURATION_SECONDS);
  preferences.end();

  topPositionCm = constrain(topPositionCm, 0.0f, 50.0f);
  bottomPositionCm = constrain(bottomPositionCm, 0.0f, 50.0f);
  speedCmS = constrain(speedCmS, 0.1f, 5.0f);
  runDurationSeconds = constrain(runDurationSeconds, 15, 30 * 60);
  applySpeed();
}

void saveSettings() {
  preferences.begin("entosieve", false);
  preferences.putFloat("top_cm", topPositionCm);
  preferences.putFloat("bottom_cm", bottomPositionCm);
  preferences.putFloat("speed", speedCmS);
  preferences.putUShort("duration", runDurationSeconds);
  preferences.end();
}

void setMotorEnabled(bool enabled) {
  if (!driverReady || motorEnabled == enabled) {
    return;
  }
  driver.enableMotor(enabled ? 1 : 0);
  motorEnabled = enabled;
}

void stopMotion(MotionState state = MotionState::Stopped) {
  sequenceActive = false;
  motionState = state;
  pausedState = MotionState::Idle;
  targetPositionSteps = currentPositionSteps;
  sequenceStartedMs = 0;
  pauseStartedMs = 0;
  accumulatedPauseMs = 0;
  setMotorEnabled(false);
}

void moveToSteps(long target) {
  targetPositionSteps = target;
  lastStepUs = micros();
  if (targetPositionSteps != currentPositionSteps) {
    setMotorEnabled(true);
  }
}

long distanceToGo() {
  return targetPositionSteps - currentPositionSteps;
}

void runMotor() {
  const long distance = distanceToGo();
  if (distance == 0) {
    return;
  }

  setMotorEnabled(true);

  const float stepsPerSecond = max(1.0f, activeSpeedCmS * STEPS_PER_CM);
  const unsigned long intervalUs =
      static_cast<unsigned long>(1000000.0f / stepsPerSecond);
  const unsigned long now = micros();
  if (now - lastStepUs < intervalUs) {
    return;
  }

  lastStepUs = now;
  const bool forward = distance > 0;
  digitalWrite(DIR_X_PIN, forward ? HIGH : LOW);
  digitalWrite(STEP_X_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(STEP_X_PIN, LOW);
  currentPositionSteps += forward ? 1 : -1;
}

void startHome() {
  applySpeed();
  activeSpeedCmS = speedCmS;
  sequenceActive = false;
  pausedState = MotionState::Idle;
  moveToSteps(currentPositionSteps + cmToSteps(HOMING_TRAVEL_CM));
  motionState = MotionState::HomingToTop;
}

void startSequence() {
  if (topPositionCm < bottomPositionCm) {
    const float oldTop = topPositionCm;
    topPositionCm = bottomPositionCm;
    bottomPositionCm = oldTop;
  }

  applySpeed();
  activeSpeedCmS = speedCmS;
  completedCycles = 0;
  sequenceActive = true;
  pausedState = MotionState::Idle;
  sequenceStartedMs = millis();
  pauseStartedMs = 0;
  accumulatedPauseMs = 0;
  moveToSteps(cmToSteps(topPositionCm));
  motionState = MotionState::CyclingUp;
}

void toggleSequence() {
  if (sequenceActive || motionState == MotionState::Paused) {
    stopMotion();
    return;
  }
  startSequence();
}

void togglePause() {
  if (motionState == MotionState::Paused) {
    if (sequenceActive && pauseStartedMs != 0) {
      accumulatedPauseMs += millis() - pauseStartedMs;
      pauseStartedMs = 0;
    }
    setMotorEnabled(true);
    motionState = pausedState;
    pausedState = MotionState::Idle;
    lastStepUs = micros();
    return;
  }

  if (motionState == MotionState::HomingToTop ||
      motionState == MotionState::HomingToBottom ||
      motionState == MotionState::CyclingUp ||
      motionState == MotionState::CyclingDown ||
      motionState == MotionState::Jogging) {
    pausedState = motionState;
    motionState = MotionState::Paused;
    if (sequenceActive) {
      pauseStartedMs = millis();
    }
    setMotorEnabled(false);
  }
}

void nudge(float cm) {
  applySpeed();
  activeSpeedCmS = speedCmS;
  sequenceActive = false;
  pausedState = MotionState::Idle;
  moveToSteps(currentPositionSteps + cmToSteps(cm));
  motionState = MotionState::Jogging;
}

void finishCycleLeg() {
  completedCycles++;
  if (sequenceActive) {
    moveToSteps(cmToSteps(topPositionCm));
    motionState = MotionState::CyclingUp;
  } else {
    stopMotion(MotionState::Idle);
  }
}

unsigned long activeRunElapsedMs() {
  if (!sequenceActive || sequenceStartedMs == 0) {
    return 0;
  }
  unsigned long pausedMs = accumulatedPauseMs;
  if (motionState == MotionState::Paused && pauseStartedMs != 0) {
    pausedMs += millis() - pauseStartedMs;
  }
  return millis() - sequenceStartedMs - pausedMs;
}

bool sequenceTimeExpired() {
  return sequenceActive &&
         activeRunElapsedMs() >= static_cast<unsigned long>(runDurationSeconds) * 1000UL;
}

void updateMotion() {
  if (!driverReady) {
    return;
  }

  if (sequenceTimeExpired()) {
    stopMotion(MotionState::Idle);
    return;
  }

  switch (motionState) {
    case MotionState::HomingToTop:
      if (topLimitTriggered()) {
        currentPositionSteps = cmToSteps(topPositionCm);
        moveToSteps(currentPositionSteps - cmToSteps(HOMING_TRAVEL_CM));
        motionState = MotionState::HomingToBottom;
      } else {
        runMotor();
      }
      break;
    case MotionState::HomingToBottom:
      if (bottomLimitTriggered()) {
        currentPositionSteps = cmToSteps(bottomPositionCm);
        targetPositionSteps = currentPositionSteps;
        stopMotion(MotionState::Idle);
      } else {
        runMotor();
      }
      break;
    case MotionState::CyclingUp:
      if (topLimitTriggered()) {
        currentPositionSteps = cmToSteps(topPositionCm);
        moveToSteps(cmToSteps(bottomPositionCm));
        motionState = MotionState::CyclingDown;
      } else if (distanceToGo() == 0) {
        moveToSteps(cmToSteps(bottomPositionCm));
        motionState = MotionState::CyclingDown;
      } else {
        runMotor();
      }
      break;
    case MotionState::CyclingDown:
      if (bottomLimitTriggered()) {
        currentPositionSteps = cmToSteps(bottomPositionCm);
        finishCycleLeg();
      } else if (distanceToGo() == 0) {
        finishCycleLeg();
      } else {
        runMotor();
      }
      break;
    case MotionState::Jogging:
      if (distanceToGo() == 0) {
        stopMotion(MotionState::Idle);
      } else {
        runMotor();
      }
      break;
    case MotionState::Idle:
    case MotionState::Paused:
    case MotionState::Stopped:
      break;
  }
}

String jsonStatus() {
  String json = "{";
  json += "\"state\":\"" + String(stateName()) + "\",";
  json += "\"topLimit\":" + String(topLimitTriggered() ? "true" : "false") + ",";
  json += "\"bottomLimit\":" + String(bottomLimitTriggered() ? "true" : "false") + ",";
  json += "\"positionCm\":" + String(stepsToCm(currentPositionSteps), 2) + ",";
  json += "\"targetCm\":" + String(stepsToCm(targetPositionSteps), 2) + ",";
  json += "\"topPositionCm\":" + String(topPositionCm, 2) + ",";
  json += "\"bottomPositionCm\":" + String(bottomPositionCm, 2) + ",";
  json += "\"speedCmS\":" + String(speedCmS, 2) + ",";
  json += "\"sequenceActive\":" + String(sequenceActive ? "true" : "false") + ",";
  json += "\"paused\":" + String(motionState == MotionState::Paused ? "true" : "false") + ",";
  json += "\"completedCycles\":" + String(completedCycles) + ",";
  json += "\"elapsedRunSec\":" + String(activeRunElapsedMs() / 1000UL) + ",";
  json += "\"runDurationSec\":" + String(runDurationSeconds) + ",";
  json += "\"driverReady\":" + String(driverReady ? "true" : "false") + ",";
  json += "\"l0Raw\":" + String(driver.ext_io_status[0]) + ",";
  json += "\"l1Raw\":" + String(driver.ext_io_status[1]) + ",";
  json += "\"ip\":\"" + WiFi.localIP().toString() + "\"";
  json += "}";
  return json;
}

void handleRoot() {
  server.send(200, "text/html", R"HTML(
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Entosieve Digital</title>
  <style>
    :root { color-scheme: dark; font-family: system-ui, sans-serif; }
    body { margin: 0; background: #111827; color: #f9fafb; }
    main { max-width: 720px; margin: 0 auto; padding: 24px; }
    h1 { margin: 0 0 16px; font-size: 24px; }
    section { border: 1px solid #374151; border-radius: 8px; padding: 16px; margin: 14px 0; background: #1f2937; }
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(160px, 1fr)); gap: 12px; }
    .metric { padding: 12px; border-radius: 6px; background: #111827; }
    .label { color: #9ca3af; font-size: 12px; text-transform: uppercase; }
    .value { margin-top: 4px; font-size: 22px; font-weight: 700; }
    label { display: block; margin: 14px 0; }
    input[type=range] { width: 100%; }
    button { min-height: 44px; padding: 0 18px; border: 0; border-radius: 6px; background: #2563eb; color: white; font-weight: 700; transition: background .15s ease, color .15s ease, transform .08s ease; }
    button:active, button.clicked { background: #f9fafb; color: #111827; transform: translateY(1px); }
    button.stop { background: #dc2626; }
    button.stop:active, button.stop.clicked { background: #f9fafb; color: #dc2626; }
    .buttons { display: flex; gap: 10px; flex-wrap: wrap; }
    .ok { color: #34d399; }
    .hit { color: #f87171; }
    .notice { min-height: 22px; color: #bfdbfe; font-size: 14px; }
  </style>
</head>
<body>
<main>
  <h1>Entosieve Digital</h1>
  <section class="grid">
    <div class="metric"><div class="label">State</div><div class="value" id="state">-</div></div>
    <div class="metric"><div class="label">Top limit</div><div class="value" id="top">-</div></div>
    <div class="metric"><div class="label">Bottom limit</div><div class="value" id="bottom">-</div></div>
    <div class="metric"><div class="label">Position</div><div class="value"><span id="pos">-</span> cm</div></div>
    <div class="metric"><div class="label">Cycles</div><div class="value"><span id="cycles">-</span></div></div>
    <div class="metric"><div class="label">Run time</div><div class="value"><span id="runTime">-</span></div></div>
    <div class="metric"><div class="label">Driver</div><div class="value" id="driver">-</div></div>
  </section>
  <section>
    <label>Top position: <strong><span id="topVal">2.0</span> cm</strong>
      <input id="topSlider" type="range" min="0" max="50" step="0.1">
    </label>
    <label>Bottom position: <strong><span id="bottomVal">0.0</span> cm</strong>
      <input id="bottomSlider" type="range" min="0" max="50" step="0.1">
    </label>
    <label>Speed: <strong><span id="speedVal">0.5</span> cm/s</strong>
      <input id="speedSlider" type="range" min="0.1" max="5" step="0.1">
    </label>
    <label>Run time: <strong><span id="durationVal">1.0</span> min</strong>
      <input id="durationSlider" type="range" min="0.25" max="30" step="0.25">
    </label>
  </section>
  <section class="buttons">
    <button data-command="home">Home</button>
    <button data-command="start">Start</button>
    <button data-command="pause">Pause</button>
    <button data-command="saveSettings">Save Settings</button>
    <button data-command="nudgeUp">Nudge Up</button>
    <button data-command="nudgeDown">Nudge Down</button>
    <button class="stop" data-command="stop">Stop</button>
  </section>
  <div class="notice" id="notice"></div>
</main>
<script>
const topSlider = document.getElementById('topSlider');
const bottomSlider = document.getElementById('bottomSlider');
const speedSlider = document.getElementById('speedSlider');
const durationSlider = document.getElementById('durationSlider');
const notice = document.getElementById('notice');
let loading = true;

function setText(id, value) { document.getElementById(id).textContent = value; }
function limitText(id, active) {
  const el = document.getElementById(id);
  el.textContent = active ? 'Triggered' : 'Open';
  el.className = active ? 'value hit' : 'value ok';
}
function flashButton(button) {
  button.classList.add('clicked');
  setTimeout(() => button.classList.remove('clicked'), 220);
}
async function command(name, button) {
  flashButton(button);
  notice.textContent = 'Sending ' + button.textContent + '...';
  try {
    if (name === 'saveSettings') await saveConfig();
    const res = await fetch('/api/' + name, { method: 'POST' });
    if (!res.ok) throw new Error('HTTP ' + res.status);
    notice.textContent = button.textContent + ' accepted';
    await refresh();
  } catch (err) {
    notice.textContent = button.textContent + ' failed: ' + err.message;
  }
}
async function saveConfig() {
  if (loading) return;
  const params = new URLSearchParams({
    top: topSlider.value,
    bottom: bottomSlider.value,
    speed: speedSlider.value,
    duration: Math.round(Number(durationSlider.value) * 60)
  });
  await fetch('/api/config?' + params.toString(), { method: 'POST' });
}
function showSliderValues() {
  setText('topVal', Number(topSlider.value).toFixed(1));
  setText('bottomVal', Number(bottomSlider.value).toFixed(1));
  setText('speedVal', Number(speedSlider.value).toFixed(1));
  setText('durationVal', Number(durationSlider.value).toFixed(2).replace(/\.00$/, '.0'));
}
function formatDuration(seconds) {
  const m = Math.floor(seconds / 60);
  const s = seconds % 60;
  return m + ':' + String(s).padStart(2, '0');
}
async function refresh() {
  const res = await fetch('/api/status');
  const data = await res.json();
  setText('state', data.state);
  limitText('top', data.topLimit);
  limitText('bottom', data.bottomLimit);
  setText('pos', Number(data.positionCm).toFixed(2));
  setText('cycles', data.completedCycles);
  setText('runTime', formatDuration(data.elapsedRunSec) + '/' + formatDuration(data.runDurationSec));
  setText('driver', data.driverReady ? 'Ready' : 'Missing');
  if (loading) {
    topSlider.value = data.topPositionCm;
    bottomSlider.value = data.bottomPositionCm;
    speedSlider.value = data.speedCmS;
    durationSlider.value = data.runDurationSec / 60;
    showSliderValues();
    loading = false;
  }
}
[topSlider, bottomSlider, speedSlider, durationSlider].forEach(slider => {
  slider.addEventListener('input', showSliderValues);
  slider.addEventListener('change', saveConfig);
});
document.querySelectorAll('button[data-command]').forEach(button => {
  button.addEventListener('click', () => command(button.dataset.command, button));
});
setInterval(refresh, 500);
refresh();
</script>
</body>
</html>
)HTML");
}

void handleStatus() {
  server.send(200, "application/json", jsonStatus());
}

void handleConfig() {
  if (server.hasArg("top")) {
    topPositionCm = server.arg("top").toFloat();
  }
  if (server.hasArg("bottom")) {
    bottomPositionCm = server.arg("bottom").toFloat();
  }
  if (server.hasArg("speed")) {
    speedCmS = max(0.1f, server.arg("speed").toFloat());
    applySpeed();
  }
  if (server.hasArg("duration")) {
    runDurationSeconds =
        constrain(server.arg("duration").toInt(), 15, 30 * 60);
  }
  server.send(200, "application/json", jsonStatus());
}

void setupRoutes() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/config", HTTP_POST, handleConfig);
  server.on("/api/home", HTTP_POST, []() {
    startHome();
    server.send(200, "application/json", jsonStatus());
  });
  server.on("/api/start", HTTP_POST, []() {
    startSequence();
    server.send(200, "application/json", jsonStatus());
  });
  server.on("/api/pause", HTTP_POST, []() {
    togglePause();
    server.send(200, "application/json", jsonStatus());
  });
  server.on("/api/stop", HTTP_POST, []() {
    stopMotion();
    server.send(200, "application/json", jsonStatus());
  });
  server.on("/api/saveSettings", HTTP_POST, []() {
    saveSettings();
    server.send(200, "application/json", jsonStatus());
  });
  server.on("/api/nudgeUp", HTTP_POST, []() {
    nudge(2.0f);
    server.send(200, "application/json", jsonStatus());
  });
  server.on("/api/nudgeDown", HTTP_POST, []() {
    nudge(-2.0f);
    server.send(200, "application/json", jsonStatus());
  });
}

void pollLimits() {
  if (!driverReady || millis() - lastLimitPollMs < 25) {
    return;
  }
  lastLimitPollMs = millis();
  driver.getExtIOStatus();
}

void drawDisplayLayout() {
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(10, 10);
  M5.Lcd.print("Entosieve Digital");
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(10, 45);
  M5.Lcd.print("IP: ");
  M5.Lcd.setCursor(10, 65);
  M5.Lcd.print("Driver: ");
  M5.Lcd.setCursor(10, 85);
  M5.Lcd.print("State: ");
  M5.Lcd.setCursor(10, 105);
  M5.Lcd.print("L0 top: ");
  M5.Lcd.setCursor(10, 125);
  M5.Lcd.print("L1 bottom: ");
  M5.Lcd.setCursor(10, 145);
  M5.Lcd.print("Pos: ");
  M5.Lcd.setCursor(10, 165);
  M5.Lcd.print("Speed: ");
  M5.Lcd.setCursor(10, 185);
  M5.Lcd.print("Run: ");
  M5.Lcd.setCursor(10, 205);
  M5.Lcd.print("Cycles: ");
  M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK);
  M5.Lcd.setCursor(28, 220);
  M5.Lcd.print("Start/Stop");
  M5.Lcd.setCursor(136, 220);
  M5.Lcd.print("Pause");
  displayLayoutDrawn = true;
}

void printValue(int x, int y, int w, const String& value,
                uint16_t color = TFT_WHITE) {
  M5.Lcd.fillRect(x, y, w, 12, TFT_BLACK);
  M5.Lcd.setTextColor(color, TFT_BLACK);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(x, y);
  M5.Lcd.print(value);
}

void updateDisplay() {
  if (!displayLayoutDrawn) {
    drawDisplayLayout();
  }
  if (millis() - lastDisplayUpdateMs < 300) {
    return;
  }
  lastDisplayUpdateMs = millis();

  printValue(34, 45, 190, wifiReady ? WiFi.localIP().toString() : "connecting");
  printValue(58, 65, 90, driverReady ? "ready" : "missing",
             driverReady ? TFT_GREEN : TFT_RED);
  printValue(52, 85, 130, stateName());
  printValue(58, 105, 120,
             String(topLimitTriggered() ? "HIT" : "open") + " raw=" +
                 String(driver.ext_io_status[0]),
             topLimitTriggered() ? TFT_RED : TFT_GREEN);
  printValue(76, 125, 120,
             String(bottomLimitTriggered() ? "HIT" : "open") + " raw=" +
                 String(driver.ext_io_status[1]),
             bottomLimitTriggered() ? TFT_RED : TFT_GREEN);
  printValue(40, 145, 120, String(stepsToCm(currentPositionSteps), 2) + " cm");
  printValue(52, 165, 120, String(speedCmS, 2) + " cm/s");
  printValue(34, 185, 140,
             String(activeRunElapsedMs() / 1000UL) + "/" +
                 String(runDurationSeconds) + " s");
  printValue(58, 205, 120, String(completedCycles));
}

void handlePhysicalButtons() {
  if (M5.BtnA.wasPressed()) {
    toggleSequence();
  }
  if (M5.BtnB.wasPressed()) {
    togglePause();
  }
}

void connectWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("Connecting to %s", WIFI_SSID);
  const unsigned long startMs = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startMs < 20000) {
    delay(250);
    Serial.print(".");
    updateDisplay();
  }
  wifiReady = WiFi.status() == WL_CONNECTED;
  Serial.println();
  if (wifiReady) {
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi connection failed; web app unavailable.");
  }
}
}  // namespace

void setup() {
  M5.begin(true, false, true, false);
  Serial.begin(115200);
  Wire.begin(21, 22, 400000UL);

  M5.Lcd.setBrightness(120);
  M5.Lcd.fillScreen(TFT_BLACK);

  pinMode(STEP_X_PIN, OUTPUT);
  pinMode(DIR_X_PIN, OUTPUT);
  digitalWrite(STEP_X_PIN, LOW);
  digitalWrite(DIR_X_PIN, HIGH);

  loadSettings();

  driverReady = driver.init(Wire, STEP_DRIVER_I2C_ADDRESS);
  if (driverReady) {
    driver.resetMotor(0, 0);
    driver.enableMotor(0);
    motorEnabled = false;
    driver.getExtIOStatus();
  }

  currentPositionSteps = cmToSteps(bottomPositionCm);
  targetPositionSteps = currentPositionSteps;

  connectWifi();
  setupRoutes();
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  M5.update();
  handlePhysicalButtons();
  pollLimits();
  server.handleClient();
  updateMotion();
  updateDisplay();
}
