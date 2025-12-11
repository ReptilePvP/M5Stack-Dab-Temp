#include <Arduino.h>
#include <M5CoreS3.h>
#include <M5Unified.h>
#include <M5GFX.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <M5_UNIT_8SERVO.h>
#include "M5UnitScroll.h"

#define PA_HUB_ADDR     0x70
#define SERVO_ADDR      0x25

// 1. PaHub Config
#define PAHUB_CH_SERVO  0
#define PAHUB_CH_NCIR   5
#define PAHUB_CH_SCROLL 1

// 2. Servo Config
#define SERVO_PORT 0

// --- OBJECTS ---
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
M5_UNIT_8SERVO servoUnit;
M5UnitScroll      scroll;
M5Canvas          canvas(&M5.Display);

// Variables
unsigned long previousMillis = 0;
const long interval = 100; // Screen refresh rate
int     servoCmd   = 90;        // 90 = STOP, 0–89 = CCW, 91–180 = CW
int16_t lastEncoder = 0;
String  msg = "Ready";

bool servoOK     = false;
bool scrollOK    = false;
bool ncirOK      = false;
bool tempEnabled = true;

float   lastTemp = -99.0;

// Button timing
uint32_t btnPressStart = 0;
bool     btnWasPressed = false;

void selectChannel(uint8_t ch) {
  Wire.beginTransmission(PA_HUB_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
  delay(6);                         // Critical for reliable switching
}

// Send command to continuous servo: 90 = stop
void setServoSpeed(int value) {
  value = constrain(value, 0, 180);
  if (abs(value - servoCmd) < 1) return;

  selectChannel(PAHUB_CH_SERVO);
  servoUnit.setServoAngle(SERVO_PORT, value);
  servoCmd = value;

  if (value == 90)       msg = "STOPPED";
  else if (value < 90)   msg = "CCW " + String(90 - value);
  else if (value > 90)   msg = "CW " + String(value - 90);
}

// Reliable NCIR read (returns Celsius). Does NOT re-init the sensor.
float readTemperature() {
  // Select NCIR channel
  selectChannel(PAHUB_CH_NCIR);
  delay(10);  // <-- REQUIRED for MLX90614 to respond

  // Read temperature safely
  for (int i = 0; i < 3; i++) {
    float t = mlx.readObjectTempC();
    if (!isnan(t) && t > -30 && t < 120) {
      ncirOK = true;
      return t;
    }
    delay(5);
  }

  ncirOK = false;
  return -99.0;
}



void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);

  // Initialize Wire (SDA=2, SCL=1 on M5CoreS3 as you had before)
  Wire.begin(2, 1);
  Wire.setClock(100000);

  M5.Display.setBrightness(110);
  canvas.createSprite(320, 240);
  canvas.setTextSize(2);
  canvas.setTextDatum(MC_DATUM);

  Serial.begin(115200);
  Serial.println("\n=== 360° Continuous Servo + Scroll + Toggleable NCIR ===");
  
  // Init NCIR: select the PAHub channel first, then init sensor once
  selectChannel(PAHUB_CH_NCIR);
  delay(10);   // required or begin() will fail
  mlx.begin(0x5A, &Wire);
  
  // Servo
  selectChannel(PAHUB_CH_SERVO);
  if (servoUnit.begin(&Wire, 2, 1, SERVO_ADDR)) {
    servoUnit.setOnePinMode(SERVO_PORT, SERVO_CTL_MODE);
    servoUnit.setServoAngle(SERVO_PORT, 90);    // STOP
    servoOK = true;
  } else {
    servoOK = false;
    Serial.println("Servo unit missing or failed to start");
  }

  // Scroll
  selectChannel(PAHUB_CH_SCROLL);
  delay(15);
  if (scroll.begin(&Wire)) {
    scrollOK = true;
    lastEncoder = scroll.getEncoderValue();
  } else {
    scrollOK = false;
    Serial.println("Scroll unit missing or failed to start");
  }

  // Prime temperature once (safe: readTemperature uses the previously-initialized sensor)
  lastTemp = readTemperature();

  setServoSpeed(90);  // ensure stopped at boot
}

void loop() {
  M5.update();

  if (scrollOK) {
    selectChannel(PAHUB_CH_SCROLL);

    // Encoder → speed control
    int16_t enc = scroll.getEncoderValue();
    int16_t delta = enc - lastEncoder;
    if (delta != 0) {
      int newCmd = servoCmd + delta * 2;        // ×2 for good speed range
      newCmd = constrain(newCmd, 0, 180);
      setServoSpeed(newCmd);
      lastEncoder = enc;
    }

    // Button handling
    bool pressed = !scroll.getButtonStatus();   // false = pressed

    if (pressed && !btnWasPressed) {
      btnPressStart = millis();
      btnWasPressed = true;
    }
    if (!pressed && btnWasPressed) {
      uint32_t duration = millis() - btnPressStart;
      if (duration < 700) {
        // Short press → toggle temperature
        tempEnabled = !tempEnabled;
        msg = tempEnabled ? "Temp ON" : "Temp OFF";
      } else {
        // Long press → STOP motor
        setServoSpeed(90);
        msg = "STOPPED";
      }
      btnWasPressed = false;
    }
  }

  // Temperature (only when enabled)
  static uint32_t tempTimer = 0;
  if (tempEnabled && millis() - tempTimer > 1200) {
    lastTemp = readTemperature();
    tempTimer = millis();
  }

  // ───── Display ─────
  canvas.fillSprite(BLACK);

  canvas.setTextColor(scrollOK ? GREEN : RED);
  canvas.drawString(scrollOK ? "Scroll OK" : "Scroll ERR", 160, 15);

  canvas.setTextColor(servoOK ? GREEN : RED);
  canvas.drawString("360° Servo", 160, 40);

  canvas.setTextColor(tempEnabled ? (ncirOK ? GREEN : ORANGE) : TFT_DARKGREY);
  canvas.drawString(tempEnabled ? (ncirOK ? "NCIR ON" : "NCIR ERR") : "NCIR OFF", 160, 70);

  canvas.setTextColor(WHITE);
  canvas.drawString("Speed: " + String(abs(servoCmd - 90)), 160, 110);

  canvas.setTextColor(servoCmd < 90 ? CYAN : (servoCmd > 90 ? YELLOW : WHITE));
  canvas.drawString(servoCmd < 90 ? "CCW" : (servoCmd > 90 ? "CW" : "STOP"), 160, 140);

  canvas.setTextColor(YELLOW);
  canvas.drawString(msg, 160, 170);

  canvas.setTextColor(tempEnabled && ncirOK ? CYAN : TFT_DARKGREY);
  String tempStr = tempEnabled && lastTemp > -50 ? String(lastTemp, 1) + "°C" : "--°C";
  canvas.drawString("Temp: " + tempStr, 160, 200);

  canvas.setTextSize(1);
  canvas.setTextColor(TFT_LIGHTGREY);
  canvas.drawString("Short press = Temp ON/OFF", 160, 220);
  canvas.drawString("Long press  = STOP motor", 160, 232);

  canvas.pushSprite(0, 0);

  delay(10);
}
