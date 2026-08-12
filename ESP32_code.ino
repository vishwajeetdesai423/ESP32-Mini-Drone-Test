/*
  ================================================================
  ESP32 MINI DRONE TEST SYSTEM
  ================================================================
  Board:        ESP32 DOIT DevKit V1
  Sensor:       MPU6050 (I2C, auto-detect 0x68 / 0x69)
  Arduino Core: ESP32 Arduino Core 3.3.7

  Wiring:
    MPU6050 VCC -> 3.3V
    MPU6050 GND -> GND
    MPU6050 SDA -> GPIO 21
    MPU6050 SCL -> GPIO 22

  Behaviour:
    - I2C scan on boot.
    - MPU6050 auto-detected at 0x68 or 0x69.
    - WHO_AM_I register is checked to identify the exact chip:
        0x68 -> genuine MPU6050, uses the Adafruit_MPU6050 library
        0x70 -> MPU6500-compatible clone (Adafruit's library rejects
                this ID), handled instead via direct raw registers
    - If MPU6050 is missing, Wi-Fi connection + web server still start.
    - Wi-Fi STATION mode: connects to an existing network
      SSID "VISHU", Password "IOT123456" (this must be a real
      router/hotspot network with this name, since the ESP32 no
      longer creates its own network).
    - IP address is assigned by that router/hotspot (printed in
      the Serial Monitor after connecting).
    - Web dashboard with live sensor data, Roll/Pitch, and graphs.

  No motor / ESC / PID control is included in this test sketch.
  ================================================================
*/

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ---------------------------------------------------------------
// I2C PINS
// ---------------------------------------------------------------
#define SDA_PIN 21
#define SCL_PIN 22

// ---------------------------------------------------------------
// WIFI CONFIG (Station mode - connects to an existing network)
// ---------------------------------------------------------------
const char* ssid     = "VISHU";
const char* password = "IOT123456";

// ---------------------------------------------------------------
// GLOBAL OBJECTS
// ---------------------------------------------------------------
Adafruit_MPU6050 mpu;
WebServer server(80);

bool  mpuConnected = false;
uint8_t mpuAddress = 0;   // filled in by the I2C scanner
bool  rawMode = false;    // true when the chip is an MPU6500-compatible
                           // clone (WHO_AM_I = 0x70) that the Adafruit
                           // MPU6050 library rejects - we then talk to
                           // it directly via raw registers instead

bool  wifiConnected = false; // set after the STA connection attempt

// Gyro calibration offsets (rad/s, as returned by the library)
float gyroOffsetX = 0.0;
float gyroOffsetY = 0.0;
float gyroOffsetZ = 0.0;

// Latest sensor readings (converted to the display units)
float accelX = 0, accelY = 0, accelZ = 0;      // m/s^2
float gyroX_dps = 0, gyroY_dps = 0, gyroZ_dps = 0; // deg/s
float temperatureC = 0;                         // deg C

// Complementary filter output
float roll  = 0.0;
float pitch = 0.0;

// Non-blocking sensor update timing
unsigned long lastSensorUpdate = 0;
const unsigned long SENSOR_INTERVAL_MS = 10; // ~100 Hz

// Complementary filter constant (closer to 1 = trust gyro more)
const float FILTER_ALPHA = 0.98;

// ---------------------------------------------------------------
// I2C SCANNER
// ---------------------------------------------------------------
void scanI2C() {
  Serial.println("Starting I2C scanner...");
  Serial.println();

  byte deviceCount = 0;

  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at: 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      deviceCount++;

      // Remember an MPU6050-compatible address if we see one
      if (address == 0x68 || address == 0x69) {
        mpuAddress = address;
      }
    }
  }

  if (deviceCount == 0) {
    Serial.println("No I2C devices found!");
  }
  Serial.println();
}

// ---------------------------------------------------------------
// LOW-LEVEL I2C REGISTER HELPERS
// (raw register access, independent of the Adafruit driver -
//  used for the WHO_AM_I diagnostic check and manual wake-up)
// ---------------------------------------------------------------
byte readRegister(uint8_t i2cAddr, byte reg) {
  Wire.beginTransmission(i2cAddr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)i2cAddr, 1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0xFF; // no response
}

void writeRegister(uint8_t i2cAddr, byte reg, byte value) {
  Wire.beginTransmission(i2cAddr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// ---------------------------------------------------------------
// MPU6050 INITIALIZATION
// ---------------------------------------------------------------
bool initMPU6050() {
  if (mpuAddress == 0) {
    Serial.println("MPU6050 NOT FOUND (no device at 0x68 or 0x69).");
    return false;
  }

  // --- Raw WHO_AM_I check (register 0x75) ---
  // NOTE: MPU6050 always reports WHO_AM_I = 0x68, even when its
  // actual I2C address is 0x69 - that's normal for this chip.
  byte whoAmI = readRegister(mpuAddress, 0x75);
  Serial.print("WHO_AM_I = 0x");
  Serial.println(whoAmI, HEX);

  if (whoAmI == 0xFF) {
    Serial.println("MPU6050 NOT FOUND (no response to WHO_AM_I read).");
    return false;
  }
  if (whoAmI == 0x68) {
    Serial.println("MPU6050 identification PASSED.");
  } else if (whoAmI == 0x70) {
    Serial.println("MPU6500-compatible sensor detected (WHO_AM_I = 0x70).");
    Serial.println("Adafruit_MPU6050 checks for ID 0x68 and will reject");
    Serial.println("this chip, so raw register mode will be used instead.");
    rawMode = true;
  } else {
    Serial.println("Warning: WHO_AM_I value is unexpected for MPU6050.");
    Serial.println("Continuing anyway, but double-check the sensor module.");
  }

  // Manually clear the sleep bit (PWR_MGMT_1 register) - needed in
  // both modes so the chip is actually sampling data.
  writeRegister(mpuAddress, 0x6B, 0x00);
  delay(10);

  if (rawMode) {
    // Configure ranges directly via registers instead of the library:
    writeRegister(mpuAddress, 0x1C, 0x00); // ACCEL_CONFIG: +-2g
    writeRegister(mpuAddress, 0x1B, 0x00); // GYRO_CONFIG:  +-250 deg/s
    Serial.print("MPU6500-compatible sensor configured at address: 0x");
    Serial.println(mpuAddress, HEX);
    return true;
  }

  if (!mpu.begin(mpuAddress)) {
    Serial.println("MPU6050 NOT FOUND (sensor did not respond to begin()).");
    return false;
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.print("MPU6050 detected at address: 0x");
  Serial.println(mpuAddress, HEX);
  return true;
}

// ---------------------------------------------------------------
// GYRO CALIBRATION
// (runs once during setup(), before Wi-Fi starts - a short
//  blocking pause here is expected/required so the sensor
//  is not moved; it does NOT affect the main loop() timing)
// ---------------------------------------------------------------
void calibrateGyro() {
  if (!mpuConnected) {
    Serial.println("Skipping gyro calibration - MPU6050 not connected.");
    return;
  }

  Serial.println("Starting gyro calibration...");
  Serial.println("Keep MPU6050 completely STILL and LEVEL.");
  delay(1000); // give the user a moment to let go of the board

  const int samples = 1000;
  double sumX = 0, sumY = 0, sumZ = 0;

  if (rawMode) {
    // Raw register calibration - offsets end up in deg/s directly
    for (int i = 0; i < samples; i++) {
      Wire.beginTransmission(mpuAddress);
      Wire.write(0x43); // GYRO_XOUT_H
      Wire.endTransmission(false);
      Wire.requestFrom((int)mpuAddress, 6);
      if (Wire.available() >= 6) {
        int16_t gx = (Wire.read() << 8) | Wire.read();
        int16_t gy = (Wire.read() << 8) | Wire.read();
        int16_t gz = (Wire.read() << 8) | Wire.read();
        sumX += gx / 131.0; // +-250 dps range -> 131 LSB/(deg/s)
        sumY += gy / 131.0;
        sumZ += gz / 131.0;
      }
      delay(2);
    }
  } else {
    // Adafruit library calibration - offsets end up in rad/s
    sensors_event_t a, g, temp;
    for (int i = 0; i < samples; i++) {
      mpu.getEvent(&a, &g, &temp);
      sumX += g.gyro.x;
      sumY += g.gyro.y;
      sumZ += g.gyro.z;
      delay(2); // small pacing delay, one-time setup cost only
    }
  }

  gyroOffsetX = sumX / samples;
  gyroOffsetY = sumY / samples;
  gyroOffsetZ = sumZ / samples;

  Serial.println("Calibration complete.");
  Serial.print("GX offset ("); Serial.print(rawMode ? "deg/s" : "rad/s"); Serial.print("): "); Serial.println(gyroOffsetX, 6);
  Serial.print("GY offset ("); Serial.print(rawMode ? "deg/s" : "rad/s"); Serial.print("): "); Serial.println(gyroOffsetY, 6);
  Serial.print("GZ offset ("); Serial.print(rawMode ? "deg/s" : "rad/s"); Serial.print("): "); Serial.println(gyroOffsetZ, 6);
  Serial.println();
}

// ---------------------------------------------------------------
// SENSOR READ + COMPLEMENTARY FILTER
// (dispatches to the Adafruit-library path or the raw-register
//  path depending on which one initMPU6050() selected)
// ---------------------------------------------------------------
void applyComplementaryFilter(float dt) {
  // Angles from accelerometer alone (noisy but drift-free)
  float rollAcc  = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / PI;
  float pitchAcc = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;

  // Complementary filter: mostly gyro (integrated), corrected by accel
  roll  = FILTER_ALPHA * (roll  + gyroX_dps * dt) + (1.0 - FILTER_ALPHA) * rollAcc;
  pitch = FILTER_ALPHA * (pitch + gyroY_dps * dt) + (1.0 - FILTER_ALPHA) * pitchAcc;
}

void updateSensorDataLibrary(float dt) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  accelX = a.acceleration.x; // m/s^2
  accelY = a.acceleration.y;
  accelZ = a.acceleration.z;

  float gx_rad = g.gyro.x - gyroOffsetX; // rad/s, offset-corrected
  float gy_rad = g.gyro.y - gyroOffsetY;
  float gz_rad = g.gyro.z - gyroOffsetZ;

  gyroX_dps = gx_rad * 180.0 / PI; // convert to deg/s for display
  gyroY_dps = gy_rad * 180.0 / PI;
  gyroZ_dps = gz_rad * 180.0 / PI;

  temperatureC = temp.temperature;

  applyComplementaryFilter(dt);
}

void updateSensorDataRaw(float dt) {
  Wire.beginTransmission(mpuAddress);
  Wire.write(0x3B); // ACCEL_XOUT_H - 14 bytes: accel, temp, gyro
  Wire.endTransmission(false);
  Wire.requestFrom((int)mpuAddress, 14);

  if (Wire.available() < 14) {
    return; // bus glitch this cycle - just skip, try again next time
  }

  int16_t rawAX = (Wire.read() << 8) | Wire.read();
  int16_t rawAY = (Wire.read() << 8) | Wire.read();
  int16_t rawAZ = (Wire.read() << 8) | Wire.read();
  int16_t rawTemp = (Wire.read() << 8) | Wire.read();
  int16_t rawGX = (Wire.read() << 8) | Wire.read();
  int16_t rawGY = (Wire.read() << 8) | Wire.read();
  int16_t rawGZ = (Wire.read() << 8) | Wire.read();

  // Matches the ranges configured in initMPU6050():
  // Accel +-2g -> 16384 LSB/g ; Gyro +-250 dps -> 131 LSB/(deg/s)
  const float ACCEL_SCALE = 16384.0;
  const float GYRO_SCALE  = 131.0;
  const float G_TO_MS2    = 9.80665;

  accelX = (rawAX / ACCEL_SCALE) * G_TO_MS2;
  accelY = (rawAY / ACCEL_SCALE) * G_TO_MS2;
  accelZ = (rawAZ / ACCEL_SCALE) * G_TO_MS2;

  gyroX_dps = (rawGX / GYRO_SCALE) - gyroOffsetX; // offsets already in deg/s
  gyroY_dps = (rawGY / GYRO_SCALE) - gyroOffsetY;
  gyroZ_dps = (rawGZ / GYRO_SCALE) - gyroOffsetZ;

  temperatureC = (rawTemp / 340.0) + 36.53; // standard MPU temp formula

  applyComplementaryFilter(dt);
}

void updateSensorData(float dt) {
  if (rawMode) {
    updateSensorDataRaw(dt);
  } else {
    updateSensorDataLibrary(dt);
  }
}

// ---------------------------------------------------------------
// WIFI STATION CONNECT
// (bounded blocking wait, same pattern as gyro calibration -
//  this only runs once in setup(), never inside loop())
// ---------------------------------------------------------------
void connectToWiFi() {
  Serial.println("Connecting to WiFi...");

  WiFi.mode(WIFI_STA); // Station mode
  WiFi.begin(ssid, password);

  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 20) {
    delay(500);
    Serial.print(".");
    timeout++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.println("WiFi Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Signal Strength: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
  } else {
    wifiConnected = false;
    Serial.println("WiFi Connection Failed!");
    Serial.println("Web server will start anyway, but the dashboard");
    Serial.println("will not be reachable until WiFi connects.");
  }
  Serial.println();
}

// ---------------------------------------------------------------
// WEB DASHBOARD (HTML + CSS + JS, served from PROGMEM string)
// ---------------------------------------------------------------
const char DASHBOARD_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>ESP32 Mini Drone Test</title>
<style>
  * { box-sizing: border-box; }
  body {
    font-family: Arial, Helvetica, sans-serif;
    background: #0f1115;
    color: #e6e6e6;
    margin: 0;
    padding: 16px;
  }
  h1 {
    font-size: 20px;
    text-align: center;
    margin-bottom: 16px;
    color: #4fd1c5;
  }
  .card {
    background: #1a1d24;
    border-radius: 10px;
    padding: 14px 16px;
    margin-bottom: 14px;
    box-shadow: 0 2px 6px rgba(0,0,0,0.4);
  }
  .card h2 {
    font-size: 15px;
    margin: 0 0 10px 0;
    color: #9ae6b4;
    border-bottom: 1px solid #2d323c;
    padding-bottom: 6px;
  }
  .row {
    display: flex;
    justify-content: space-between;
    padding: 4px 0;
    font-size: 14px;
  }
  .label { color: #a0aec0; }
  .value { font-weight: bold; }
  .ok    { color: #48bb78; }
  .bad   { color: #f56565; }
  .grid3 {
    display: grid;
    grid-template-columns: 1fr 1fr 1fr;
    gap: 8px;
    text-align: center;
  }
  .grid3 div {
    background: #12151b;
    border-radius: 6px;
    padding: 8px 4px;
    font-size: 13px;
  }
  canvas {
    width: 100%;
    height: 140px;
    background: #12151b;
    border-radius: 6px;
  }
</style>
</head>
<body>

<h1>ESP32 MINI DRONE TEST DASHBOARD</h1>

<div class="card">
  <h2>SYSTEM STATUS</h2>
  <div class="row"><span class="label">ESP32:</span><span class="value ok">ONLINE</span></div>
  <div class="row"><span class="label">Wi-Fi:</span><span class="value" id="wifiStatus">--</span></div>
  <div class="row"><span class="label">MPU6050:</span><span class="value" id="mpuStatus">--</span></div>
</div>

<div class="card">
  <h2>ACCELEROMETER (m/s&sup2;)</h2>
  <div class="grid3">
    <div>X<br><span id="ax">--</span></div>
    <div>Y<br><span id="ay">--</span></div>
    <div>Z<br><span id="az">--</span></div>
  </div>
</div>

<div class="card">
  <h2>GYROSCOPE (&deg;/s)</h2>
  <div class="grid3">
    <div>X<br><span id="gx">--</span></div>
    <div>Y<br><span id="gy">--</span></div>
    <div>Z<br><span id="gz">--</span></div>
  </div>
</div>

<div class="card">
  <h2>TEMPERATURE</h2>
  <div class="row"><span class="label">Sensor Temp:</span><span class="value" id="temp">--</span></div>
</div>

<div class="card">
  <h2>ORIENTATION</h2>
  <div class="row"><span class="label">Roll:</span><span class="value" id="rollVal">--</span></div>
  <div class="row"><span class="label">Pitch:</span><span class="value" id="pitchVal">--</span></div>
</div>

<div class="card">
  <h2>ROLL GRAPH</h2>
  <canvas id="rollChart"></canvas>
</div>

<div class="card">
  <h2>PITCH GRAPH</h2>
  <canvas id="pitchChart"></canvas>
</div>

<script>
const MAX_POINTS = 60;
let rollHistory = [];
let pitchHistory = [];

function drawChart(canvas, history, colorLine) {
  const ctx = canvas.getContext('2d');
  const w = canvas.width  = canvas.clientWidth;
  const h = canvas.height = canvas.clientHeight;
  ctx.clearRect(0, 0, w, h);

  if (history.length < 2) return;

  const minVal = -90, maxVal = 90; // fixed range for angle graphs
  const stepX = w / (MAX_POINTS - 1);

  // zero line
  const zeroY = h - ((0 - minVal) / (maxVal - minVal)) * h;
  ctx.strokeStyle = "#2d323c";
  ctx.beginPath();
  ctx.moveTo(0, zeroY);
  ctx.lineTo(w, zeroY);
  ctx.stroke();

  ctx.strokeStyle = colorLine;
  ctx.lineWidth = 2;
  ctx.beginPath();
  for (let i = 0; i < history.length; i++) {
    const x = i * stepX;
    const clamped = Math.max(minVal, Math.min(maxVal, history[i]));
    const y = h - ((clamped - minVal) / (maxVal - minVal)) * h;
    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  }
  ctx.stroke();
}

function setText(id, value, unit) {
  document.getElementById(id).textContent =
    (typeof value === "number") ? value.toFixed(2) + (unit || "") : "N/A";
}

async function refreshData() {
  try {
    const res = await fetch('/data');
    const d = await res.json();

    const wifiEl = document.getElementById('wifiStatus');
    wifiEl.textContent = d.wifi;
    wifiEl.className = "value " + (d.wifi === "CONNECTED" ? "ok" : "bad");

    const mpuEl = document.getElementById('mpuStatus');
    mpuEl.textContent = d.mpu;
    mpuEl.className = "value " + (d.mpu === "CONNECTED" ? "ok" : "bad");

    setText('ax', d.ax, ' m/s2');
    setText('ay', d.ay, ' m/s2');
    setText('az', d.az, ' m/s2');
    setText('gx', d.gx, ' dps');
    setText('gy', d.gy, ' dps');
    setText('gz', d.gz, ' dps');
    setText('temp', d.temp, ' C');
    setText('rollVal', d.roll, ' deg');
    setText('pitchVal', d.pitch, ' deg');

    if (typeof d.roll === "number") {
      rollHistory.push(d.roll);
      if (rollHistory.length > MAX_POINTS) rollHistory.shift();
    }
    if (typeof d.pitch === "number") {
      pitchHistory.push(d.pitch);
      if (pitchHistory.length > MAX_POINTS) pitchHistory.shift();
    }

    drawChart(document.getElementById('rollChart'), rollHistory, "#4fd1c5");
    drawChart(document.getElementById('pitchChart'), pitchHistory, "#f6ad55");

  } catch (e) {
    document.getElementById('wifiStatus').textContent = "ERROR";
    document.getElementById('wifiStatus').className = "value bad";
  }
}

setInterval(refreshData, 150); // update every ~150 ms
refreshData();
</script>

</body>
</html>
)rawliteral";

// ---------------------------------------------------------------
// WEB SERVER HANDLERS
// ---------------------------------------------------------------
void handleRoot() {
  server.send_P(200, "text/html", DASHBOARD_HTML);
}

void handleData() {
  String json = "{";
  json += "\"wifi\":\"" + String(wifiConnected ? "CONNECTED" : "DISCONNECTED") + "\",";
  json += "\"mpu\":\"" + String(mpuConnected ? "CONNECTED" : "NOT FOUND") + "\",";

  if (mpuConnected) {
    json += "\"ax\":"   + String(accelX, 3)      + ",";
    json += "\"ay\":"   + String(accelY, 3)      + ",";
    json += "\"az\":"   + String(accelZ, 3)      + ",";
    json += "\"gx\":"   + String(gyroX_dps, 3)   + ",";
    json += "\"gy\":"   + String(gyroY_dps, 3)   + ",";
    json += "\"gz\":"   + String(gyroZ_dps, 3)   + ",";
    json += "\"temp\":" + String(temperatureC, 2) + ",";
    json += "\"roll\":" + String(roll, 2)        + ",";
    json += "\"pitch\":" + String(pitch, 2);
  } else {
    json += "\"ax\":\"N/A\",\"ay\":\"N/A\",\"az\":\"N/A\",";
    json += "\"gx\":\"N/A\",\"gy\":\"N/A\",\"gz\":\"N/A\",";
    json += "\"temp\":\"N/A\",\"roll\":\"N/A\",\"pitch\":\"N/A\"";
  }

  json += "}";
  server.send(200, "application/json", json);
}

// ---------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("================================");
  Serial.println("ESP32 MINI DRONE TEST SYSTEM");
  Serial.println("================================");
  Serial.println();

  // --- I2C init + scan ---
  Wire.begin(SDA_PIN, SCL_PIN);
  scanI2C();

  // --- MPU6050 init (never blocks forever) ---
  mpuConnected = initMPU6050();
  Serial.println();

  // --- Calibration (skipped automatically if not connected) ---
  calibrateGyro();

  // --- Wi-Fi connection (always attempted, regardless of MPU6050 state) ---
  connectToWiFi();

  // --- Web server routes (started even if WiFi connection failed) ---
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();

  Serial.println("Web server started.");
  Serial.println();
  if (wifiConnected) {
    Serial.println("Open:");
    Serial.print("http://");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi not connected - dashboard is not reachable yet.");
  }
  Serial.println();

  lastSensorUpdate = millis();
}

// ---------------------------------------------------------------
// LOOP (non-blocking)
// ---------------------------------------------------------------
void loop() {
  server.handleClient(); // always keep the web server responsive

  if (mpuConnected) {
    unsigned long now = millis();
    if (now - lastSensorUpdate >= SENSOR_INTERVAL_MS) {
      float dt = (now - lastSensorUpdate) / 1000.0;
      lastSensorUpdate = now;
      updateSensorData(dt);
    }
  }
  // If MPU6050 is not connected, we simply skip sensor updates.
  // No while(1) or blocking loop is ever used here, so Wi-Fi
  // and the web server keep working regardless of sensor state.
}
