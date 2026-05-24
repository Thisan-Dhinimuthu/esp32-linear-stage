#include <Arduino.h>
#include <AccelStepper.h>
#include <Keypad.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <WebServer.h>

// ---------------- WIFI ----------------
const char* ssid = "ESP32_LINEAR_STAGE";
const char* password = "12345678";
WebServer server(80);

// ---------------- OLED ----------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ---------------- TB6600 PINS ----------------
#define stepPin 21
#define dirPin 19
#define enablePin 18

// ---------------- LIMIT SWITCHES ----------------
#define leftLimit 34
#define rightLimit 35

// ---------------- ENCODER ----------------
#define encoderA 32
#define encoderB 33
volatile long encoderCount = 0;

// ---------------- CALIBRATION ----------------
const float countsPerMM = 90.0;
const float maxMM = 100.0;
const long maxCounts = maxMM * countsPerMM;
const long toleranceCounts = 7;

// ---------------- SPEED SETTINGS ----------------
const float maxSpeed = 8000.0;
const float acceleration = 3000.0;
const float minMoveSpeed = 80.0;

// ---------------- PID SETTINGS ----------------
float Kp = 2500.0;
float Ki = 0.0;
float Kd = 150.0;

float integral = 0;
float previousErrorMM = 0;
float currentSpeed = 0;

// ---------------- DIRECTION SETTINGS ----------------
#define HOME_DIR LOW
#define RIGHT_DIR HIGH
#define LEFT_DIR LOW

AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

// ---------------- KEYPAD ----------------
const byte ROWS = 4;
const byte COLS = 4;

char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

byte rowPins[ROWS] = {13, 12, 14, 27};
byte colPins[COLS] = {26, 25, 4, 23};

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

String inputString = "";
String statusText = "READY";
float lastTargetMM = 0;

// ---------------- OLED ----------------
void updateOLED(float targetMM) {
  noInterrupts();
  long countNow = encoderCount;
  interrupts();

  float actualMM = countNow / countsPerMM;
  float errorMM = targetMM - actualMM;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("Target: ");
  display.print(targetMM, 2);
  display.println(" mm");

  display.setCursor(0, 12);
  display.print("Actual: ");
  display.print(actualMM, 2);
  display.println(" mm");

  display.setCursor(0, 24);
  display.print("Count : ");
  display.println(countNow);

  display.setCursor(0, 36);
  display.print("Error : ");
  display.print(errorMM, 2);
  display.println(" mm");

  display.setCursor(0, 52);
  display.print("Status: ");
  display.println(statusText);

  display.display();
}

void showMessage(String line1, String line2) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(line1);
  display.setCursor(0, 14);
  display.println(line2);
  display.display();
}

// ---------------- ENCODER ISR ----------------
void IRAM_ATTR encoderISR() {
  if (digitalRead(encoderA) == digitalRead(encoderB)) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

// ---------------- MANUAL STEP FOR HOMING ----------------
void manualStep(bool moveRight, int delayUs) {
  digitalWrite(dirPin, moveRight ? RIGHT_DIR : LEFT_DIR);
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(delayUs);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(delayUs);
}

// ---------------- HOMING ----------------
void homeMotor() {
  digitalWrite(enablePin, LOW);

  statusText = "HOMING";
  showMessage("Homing...", "Find LEFT limit");

  unsigned long t = millis();

  while (digitalRead(leftLimit) == HIGH) {
    server.handleClient();

    if (millis() - t > 50000UL) {
      showMessage("ERROR", "Homing timeout!");
      digitalWrite(enablePin, HIGH);
      return;
    }
    manualStep(false, 600);
  }

  statusText = "LEFT LIMIT";
  showMessage("LEFT LIMIT", "Back off...");
  delay(200);

  for (int i = 0; i < 800; i++) {
    server.handleClient();
    manualStep(true, 600);
  }

  delay(200);

  showMessage("Homing...", "Slow creep...");
  t = millis();

  while (digitalRead(leftLimit) == HIGH) {
    server.handleClient();

    if (millis() - t > 10000UL) {
      showMessage("ERROR", "Creep timeout!");
      digitalWrite(enablePin, HIGH);
      return;
    }
    manualStep(false, 1500);
  }

  delay(200);

  for (int i = 0; i < 800; i++) {
    server.handleClient();
    manualStep(true, 1500);
  }

  noInterrupts();
  encoderCount = 0;
  interrupts();

  stepper.setCurrentPosition(0);
  integral = 0;
  previousErrorMM = 0;
  currentSpeed = 0;
  lastTargetMM = 0;

  statusText = "READY";
  updateOLED(0);
}

// ---------------- PID CLOSED LOOP MOVE ----------------
void moveToMM_PID(float targetMM) {
  if (targetMM < 0) targetMM = 0;
  if (targetMM > maxMM) targetMM = maxMM;

  lastTargetMM = targetMM;

  long targetCounts = targetMM * countsPerMM;

  digitalWrite(enablePin, LOW);

  integral = 0;
  previousErrorMM = 0;
  currentSpeed = 0;

  statusText = "PID MOVE";

  unsigned long lastTime = millis();
  unsigned long lastOLED = 0;

  while (true) {
    server.handleClient();

    noInterrupts();
    long countNow = encoderCount;
    interrupts();

    long errorCounts = targetCounts - countNow;
    float errorMM = errorCounts / countsPerMM;

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    if (dt <= 0) dt = 0.001;
    lastTime = now;

    if (abs(errorCounts) <= toleranceCounts && abs(currentSpeed) < 120) {
      currentSpeed = 0;
      stepper.setSpeed(0);
      statusText = "READY";
      updateOLED(targetMM);
      break;
    }

    if (digitalRead(leftLimit) == LOW) {
      noInterrupts();
      encoderCount = 0;
      interrupts();

      currentSpeed = 0;
      stepper.setSpeed(0);
      statusText = "LEFT LIMIT";
      updateOLED(targetMM);
      break;
    }

    if (digitalRead(rightLimit) == LOW) {
      noInterrupts();
      encoderCount = maxCounts;
      interrupts();

      currentSpeed = 0;
      stepper.setSpeed(0);
      statusText = "RIGHT LIMIT";
      updateOLED(targetMM);
      break;
    }

    integral += errorMM * dt;

    if (integral > 20) integral = 20;
    if (integral < -20) integral = -20;

    float derivative = (errorMM - previousErrorMM) / dt;
    previousErrorMM = errorMM;

    float pidSpeed = (Kp * errorMM) + (Ki * integral) + (Kd * derivative);

    if (pidSpeed > maxSpeed) pidSpeed = maxSpeed;
    if (pidSpeed < -maxSpeed) pidSpeed = -maxSpeed;

    if (abs(errorMM) > 0.2 && abs(pidSpeed) < minMoveSpeed) {
      pidSpeed = (pidSpeed > 0) ? minMoveSpeed : -minMoveSpeed;
    }

    float maxSpeedChange = acceleration * dt;

    if (pidSpeed > currentSpeed + maxSpeedChange) {
      currentSpeed += maxSpeedChange;
    } else if (pidSpeed < currentSpeed - maxSpeedChange) {
      currentSpeed -= maxSpeedChange;
    } else {
      currentSpeed = pidSpeed;
    }

    stepper.setSpeed(currentSpeed);
    stepper.runSpeed();

    if (millis() - lastOLED > 150) {
      updateOLED(targetMM);
      lastOLED = millis();
    }
  }
}

// ---------------- WEB PAGE ----------------
void handleRoot() {
  String page = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<title>ESP32 Linear Stage</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
body { font-family: Arial; background:#0f172a; color:white; text-align:center; }
.card { background:#1e293b; padding:15px; margin:15px; border-radius:12px; }
input, button { font-size:18px; padding:10px; margin:5px; border-radius:8px; width:80%; }
button { background:#14b8a6; color:white; border:none; }
.stop { background:#ef4444; }
.pid { background:#6366f1; }
</style>
</head>
<body>

<h2>ESP32 Linear Stage</h2>

<div class="card">
<h3>Live OLED Details</h3>
<p>Target: <span id="target">0</span> mm</p>
<p>Actual: <span id="actual">0</span> mm</p>
<p>Encoder Count: <span id="count">0</span></p>
<p>Error: <span id="error">0</span> mm</p>
<p>Status: <span id="status">READY</span></p>
</div>

<div class="card">
<h3>Distance Control</h3>
<input type="number" id="distance" placeholder="Enter distance 0-100 mm" step="0.01">
<button onclick="moveMotor()">Move</button>
<button onclick="homeMotor()">Home</button>
<button class="stop" onclick="disableMotor()">Disable Motor</button>
</div>

<div class="card">
<h3>PID Tuning</h3>
<p>Kp: <span id="kpNow">0</span></p>
<p>Ki: <span id="kiNow">0</span></p>
<p>Kd: <span id="kdNow">0</span></p>

<input type="number" id="kp" placeholder="Kp value" step="0.1">
<input type="number" id="ki" placeholder="Ki value" step="0.01">
<input type="number" id="kd" placeholder="Kd value" step="0.1">

<button class="pid" onclick="updatePID()">Update PID</button>
</div>

<script>
function updateData() {
  fetch('/data')
  .then(res => res.json())
  .then(data => {
    document.getElementById('target').innerHTML = data.target;
    document.getElementById('actual').innerHTML = data.actual;
    document.getElementById('count').innerHTML = data.count;
    document.getElementById('error').innerHTML = data.error;
    document.getElementById('status').innerHTML = data.status;

    document.getElementById('kpNow').innerHTML = data.kp;
    document.getElementById('kiNow').innerHTML = data.ki;
    document.getElementById('kdNow').innerHTML = data.kd;
  });
}

function moveMotor() {
  let d = document.getElementById('distance').value;
  fetch('/move?target=' + d);
}

function homeMotor() {
  fetch('/home');
}

function disableMotor() {
  fetch('/disable');
}

function updatePID() {
  let kp = document.getElementById('kp').value;
  let ki = document.getElementById('ki').value;
  let kd = document.getElementById('kd').value;
  fetch('/pid?kp=' + kp + '&ki=' + ki + '&kd=' + kd);
}

setInterval(updateData, 500);
</script>

</body>
</html>
)rawliteral";

  server.send(200, "text/html", page);
}

// ---------------- WEB DATA ----------------
void handleData() {
  noInterrupts();
  long countNow = encoderCount;
  interrupts();

  float actualMM = countNow / countsPerMM;
  float errorMM = lastTargetMM - actualMM;

  String json = "{";
  json += "\"target\":\"" + String(lastTargetMM, 2) + "\",";
  json += "\"actual\":\"" + String(actualMM, 2) + "\",";
  json += "\"count\":\"" + String(countNow) + "\",";
  json += "\"error\":\"" + String(errorMM, 2) + "\",";
  json += "\"status\":\"" + statusText + "\",";
  json += "\"kp\":\"" + String(Kp, 2) + "\",";
  json += "\"ki\":\"" + String(Ki, 2) + "\",";
  json += "\"kd\":\"" + String(Kd, 2) + "\"";
  json += "}";

  server.send(200, "application/json", json);
}

void handleMove() {
  if (server.hasArg("target")) {
    float target = server.arg("target").toFloat();

    server.send(200, "text/plain", "Moving");
    moveToMM_PID(target);
  } else {
    server.send(400, "text/plain", "No target value");
  }
}

void handlePID() {
  if (server.hasArg("kp")) {
    float newKp = server.arg("kp").toFloat();
    if (newKp >= 0 && newKp <= 10000) Kp = newKp;
  }

  if (server.hasArg("ki")) {
    float newKi = server.arg("ki").toFloat();
    if (newKi >= 0 && newKi <= 1000) Ki = newKi;
  }

  if (server.hasArg("kd")) {
    float newKd = server.arg("kd").toFloat();
    if (newKd >= 0 && newKd <= 5000) Kd = newKd;
  }

  integral = 0;
  previousErrorMM = 0;
  statusText = "PID UPDATED";
  updateOLED(lastTargetMM);

  server.send(200, "text/plain", "PID updated");
}

void handleHome() {
  server.send(200, "text/plain", "Homing");
  homeMotor();
}

void handleDisable() {
  digitalWrite(enablePin, HIGH);
  currentSpeed = 0;
  stepper.setSpeed(0);
  statusText = "DISABLED";
  updateOLED(lastTargetMM);

  server.send(200, "text/plain", "Disabled");
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);

  pinMode(enablePin, OUTPUT);
  pinMode(leftLimit, INPUT);
  pinMode(rightLimit, INPUT);

  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);

  digitalWrite(enablePin, LOW);

  attachInterrupt(digitalPinToInterrupt(encoderA), encoderISR, CHANGE);

  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(acceleration);

  Wire.begin(16, 17);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.display();

  showMessage("SYSTEM START", "PID + IoT");

  WiFi.softAP(ssid, password);

  Serial.println("WiFi Started");
  Serial.print("WiFi Name: ");
  Serial.println(ssid);
  Serial.print("Password: ");
  Serial.println(password);
  Serial.print("Open Browser: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/move", handleMove);
  server.on("/pid", handlePID);
  server.on("/home", handleHome);
  server.on("/disable", handleDisable);
  server.begin();

  delay(1000);

  homeMotor();
}

// ---------------- LOOP ----------------
void loop() {
  server.handleClient();

  char key = keypad.getKey();

  if (key) {
    if (key >= '0' && key <= '9') {
      inputString += key;
      statusText = "INPUT";
      updateOLED(inputString.toFloat());
    }

    else if (key == '#') {
      if (inputString.length() > 0) {
        float targetMM = inputString.toFloat();
        inputString = "";
        moveToMM_PID(targetMM);
      }
    }

    else if (key == '*') {
      inputString = "";
      statusText = "CLEARED";
      updateOLED(0);
    }

    else if (key == 'A') {
      homeMotor();
    }

    else if (key == 'D') {
      digitalWrite(enablePin, HIGH);
      statusText = "DISABLED";
      updateOLED(0);
    }
  }
}