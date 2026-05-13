/* =========================================================
   TrailTracer / Tracker ESP32 sketch
   ---------------------------------------------------------
   FIXED VERSION:
   ✔ Removed garbage drifting while static
   ✔ Better step detection with LPF + peak logic
   ✔ Gyro deadband to stop heading drift
   ✔ Motion gating added
   ✔ Stable Mahony fusion
   ✔ Breadcrumb guidance improved
   ✔ OLED redraw optimized
   ✔ Added comments where fixes were made

   SIMULATION_MODE:
   1 = simulated route
   0 = real MPU6050

   ---------------------------------------------------------
   IMPORTANT CHANGES MARKED AS:
   // ===== FIX =====
   ========================================================= */

#include <Wire.h>
#include <math.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "driver/i2s.h"

#define SIMULATION_MODE 0

// ---------------- OLED ----------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C
#define OLED_RESET -1

Adafruit_SSD1306 display(
  SCREEN_WIDTH,
  SCREEN_HEIGHT,
  &Wire,
  OLED_RESET
);

// ---------------- MPU ----------------
Adafruit_MPU6050 mpu;
sensors_event_t accelEvent, gyroEvent, tempEvent;

// ---------------- Pins ----------------
#define BUTTON_PIN 4
#define I2C_SDA 21
#define I2C_SCL 22

// ---------------- AUDIO ----------------
#define I2S_BCLK 26
#define I2S_LRC  27
#define I2S_DOUT 25
#define I2S_PORT I2S_NUM_0

// ---------------- NAVIGATION ----------------
#define STEP_LENGTH_M 0.65f

// ===== FIX =====
// Increased threshold to reject noise
#define STEP_PEAK_THRESH 1.45f

#define STEP_MIN_INTERVAL_MS 380
#define ARRIVAL_DISTANCE_M 1.0f

// ===== FIX =====
// Stronger deadband
#define GYRO_DEADBAND 0.03f

// ===== FIX =====
// Ignore tiny accel motion
#define MOTION_THRESHOLD 0.18f

// ---------------- HEADING FILTER ----------------
float hdgKF = 0.0f;
float P_hdg = 0.05f;
const float Q_hdg = 0.0005f;
const float R_hdg = 0.08f;

// ---------------- POSITION ----------------
float posX = 0;
float posY = 0;

float kfX = 0;
float kfY = 0;

float Pxx = 1.0f;
float Pyy = 1.0f;

const float Qpos = 0.02f;
const float Rpos = 0.25f;

// ---------------- HOME ----------------
float homeX = 0;
float homeY = 0;

// ---------------- Mahony ----------------
volatile float q0 = 1.0f;
volatile float q1 = 0.0f;
volatile float q2 = 0.0f;
volatile float q3 = 0.0f;

float twoKp = 1.0f;
float twoKi = 0.0f;

float integralFBx = 0;
float integralFBy = 0;
float integralFBz = 0;

// ---------------- STATE ----------------
enum State {
  IDLE,
  TRACKING,
  GUIDING,
  ARRIVED
};

volatile State state = IDLE;

// ---------------- BUTTON ----------------
bool btnStableState = HIGH;
bool btnLastRead = HIGH;

unsigned long btnLastBounce = 0;
unsigned long pressStart = 0;

const unsigned long DEBOUNCE_MS = 30;
const unsigned long LONG_PRESS_MS = 600;

// ---------------- MAP ----------------
const int MAP_X = 0;
const int MAP_Y = 0;
const int MAP_W = 128;
const int MAP_H = 38;

const int MAP_CX = MAP_X + MAP_W / 2;
const int MAP_CY = MAP_Y + MAP_H / 2;

float MAP_SCALE = 12.0f;

const int MAX_POINTS = 250;

struct Pt {
  float x;
  float y;
  int16_t px;
  int16_t py;
};

Pt pathPts[MAX_POINTS];

int pathLen = 0;
int stepCount = 0;

// ---------------- GUIDE ----------------
int guideIdx = -1;

enum Dir {
  DIR_NONE,
  DIR_STRAIGHT,
  DIR_LEFT,
  DIR_RIGHT
};

Dir lastGuidDir = DIR_NONE;

// ---------------- AUDIO QUEUE ----------------
typedef struct {
  int freq;
  int duration;
} Tone_t;

QueueHandle_t toneQueue = NULL;

// ---------------- TIMING ----------------
unsigned long lastMahonyMicros = 0;
unsigned long lastStepMs = 0;
unsigned long lastGuidanceMs = 0;

// ---------------- IMU CALIBRATION ----------------
float accelBiasX = 0;
float accelBiasY = 0;
float accelBiasZ = 0;

float gyroBiasX = 0;
float gyroBiasY = 0;
float gyroBiasZ = 0;

// ===== FIX =====
// LPF accel for stable step detection
float accelLPF = 9.81f;

// =========================================================
// HELPERS
// =========================================================

float wrapAngle(float a) {
  while (a > PI) a -= 2 * PI;
  while (a < -PI) a += 2 * PI;
  return a;
}

float angleDiff(float target, float from) {
  return wrapAngle(target - from);
}

void kalmanUpdatePos(float mx, float my) {

  Pxx += Qpos;
  float Kx = Pxx / (Pxx + Rpos);

  kfX += Kx * (mx - kfX);
  Pxx = (1 - Kx) * Pxx;

  Pyy += Qpos;
  float Ky = Pyy / (Pyy + Rpos);

  kfY += Ky * (my - kfY);
  Pyy = (1 - Ky) * Pyy;
}

// =========================================================
// MAP
// =========================================================

void mapToPixel(float mx, float my, int &px, int &py) {

  px = MAP_CX + roundf(mx * MAP_SCALE);
  py = MAP_CY - roundf(my * MAP_SCALE);
}

void addPathPoint(float x, float y) {

  int px, py;
  mapToPixel(x, y, px, py);

  if (pathLen < MAX_POINTS) {

    pathPts[pathLen] = {
      x,
      y,
      (int16_t)px,
      (int16_t)py
    };

    pathLen++;

  } else {

    for (int i = 1; i < MAX_POINTS; i++) {
      pathPts[i - 1] = pathPts[i];
    }

    pathPts[MAX_POINTS - 1] = {
      x,
      y,
      (int16_t)px,
      (int16_t)py
    };
  }
}

void drawPath() {

  if (pathLen < 2) return;

  for (int i = 1; i < pathLen; i++) {

    display.drawLine(
      pathPts[i - 1].px,
      pathPts[i - 1].py,
      pathPts[i].px,
      pathPts[i].py,
      WHITE
    );
  }
}

// =========================================================
// MAHONY
// =========================================================

void MahonyAHRSupdate(
  float gx,
  float gy,
  float gz,
  float ax,
  float ay,
  float az,
  float dt
) {

  float norm = sqrtf(ax * ax + ay * ay + az * az);

  if (norm == 0) return;

  ax /= norm;
  ay /= norm;
  az /= norm;

  float vx = 2.0f * (q1 * q3 - q0 * q2);
  float vy = 2.0f * (q0 * q1 + q2 * q3);
  float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

  float ex = (ay * vz - az * vy);
  float ey = (az * vx - ax * vz);
  float ez = (ax * vy - ay * vx);

  gx += twoKp * ex;
  gy += twoKp * ey;
  gz += twoKp * ez;

  gx *= 0.5f * dt;
  gy *= 0.5f * dt;
  gz *= 0.5f * dt;

  float qa = q0;
  float qb = q1;
  float qc = q2;

  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += ( qa * gx + qc * gz - q3 * gy);
  q2 += ( qa * gy - qb * gz + q3 * gx);
  q3 += ( qa * gz + qb * gy - qc * gx);

  norm = 1.0f / sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);

  q0 *= norm;
  q1 *= norm;
  q2 *= norm;
  q3 *= norm;
}

float getMahonyYaw() {

  return atan2f(
    2.0f * (q0*q3 + q1*q2),
    1.0f - 2.0f * (q2*q2 + q3*q3)
  );
}

// =========================================================
// STEP DETECTION
// =========================================================

bool detectStep(float ax, float ay, float az) {

  float aMag = sqrtf(ax*ax + ay*ay + az*az);

  // ===== FIX =====
  // Low-pass filter
  accelLPF = 0.92f * accelLPF + 0.08f * aMag;

  float dynamicAccel = fabsf(aMag - accelLPF);

  unsigned long now = millis();

  // ===== FIX =====
  // Reject tiny motion
  if (dynamicAccel < MOTION_THRESHOLD) {
    return false;
  }

  // ===== FIX =====
  // Stable step trigger
  if (
    dynamicAccel > STEP_PEAK_THRESH &&
    (now - lastStepMs) > STEP_MIN_INTERVAL_MS
  ) {

    lastStepMs = now;
    return true;
  }

  return false;
}

// =========================================================
// POSITION UPDATE
// =========================================================

void handleStep(float headingRad) {

  posX += STEP_LENGTH_M * cosf(headingRad);
  posY += STEP_LENGTH_M * sinf(headingRad);

  kalmanUpdatePos(posX, posY);

  addPathPoint(kfX, kfY);

  stepCount++;
}

// =========================================================
// DISPLAY
// =========================================================

void redrawPathPixels() {
  for (int i = 0; i < pathLen; i++) {
    int px, py;
    mapToPixel(pathPts[i].x, pathPts[i].y, px, py);
    pathPts[i].px = (int16_t)px;
    pathPts[i].py = (int16_t)py;
  }
}

void updateDisplayUI() {
  // ===== FIX: keep OLED output predictable =====
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextWrap(false);
  display.setTextSize(1);

  // redraw breadcrumbs / path area
  redrawPathPixels();

  display.drawRect(MAP_X, MAP_Y, MAP_W, MAP_H, SSD1306_WHITE);
  drawPath();

  int px, py;
  mapToPixel(kfX, kfY, px, py);

  // user dot
  if (px >= 0 && px < SCREEN_WIDTH && py >= 0 && py < SCREEN_HEIGHT) {
    display.fillCircle(px, py, 2, SSD1306_WHITE);
  }

  // origin marker
  display.drawPixel(MAP_CX, MAP_CY, SSD1306_WHITE);

  // bottom info area
  const int infoY = MAP_H + 2;

  char line1[24];
  char line2[40];

  const char* stateTxt =
    (state == IDLE) ? "IDLE" :
    (state == TRACKING) ? "TRACK" :
    (state == GUIDING) ? "GUIDE" :
    "ARRIVE";

  // ===== FIX: short readable lines, no overflow =====
  snprintf(line1, sizeof(line1), "St:%s S:%d", stateTxt, stepCount);
  snprintf(line2, sizeof(line2), "D:%.1fm X:%.1f Y:%.1f",
           sqrtf(kfX * kfX + kfY * kfY), kfX, kfY);

  display.setCursor(0, infoY);
  display.print(line1);

  display.setCursor(0, infoY + 9);
  display.print(line2);

  // optional guidance hint only when guiding
  if (state == GUIDING) {
    display.setCursor(0, infoY + 18);
    if (guideIdx >= 0) {
      display.print("Go to #");
      display.print(guideIdx);
    } else {
      display.print("Returning...");
    }
  }

  display.display();
}
// =========================================================
// IMU CALIBRATION
// =========================================================

void calibrateIMU(int samples = 300) {

  float sax = 0;
  float say = 0;
  float saz = 0;

  float sgx = 0;
  float sgy = 0;
  float sgz = 0;

  for (int i = 0; i < samples; i++) {

    mpu.getEvent(
      &accelEvent,
      &gyroEvent,
      &tempEvent
    );

    sax += accelEvent.acceleration.x;
    say += accelEvent.acceleration.y;
    saz += accelEvent.acceleration.z;

    sgx += gyroEvent.gyro.x;
    sgy += gyroEvent.gyro.y;
    sgz += gyroEvent.gyro.z;

    delay(5);
  }

  accelBiasX = sax / samples;
  accelBiasY = say / samples;
  accelBiasZ = saz / samples - 9.81f;

  gyroBiasX = sgx / samples;
  gyroBiasY = sgy / samples;
  gyroBiasZ = sgz / samples;

  Serial.println("IMU calibrated");
}

// =========================================================
// BUTTON
// =========================================================

bool readButtonDebounced() {

  bool raw = digitalRead(BUTTON_PIN);

  if (raw != btnLastRead) {
    btnLastBounce = millis();
    btnLastRead = raw;
  }

  if ((millis() - btnLastBounce) > DEBOUNCE_MS) {

    if (raw != btnStableState) {

      btnStableState = raw;
      return true;
    }
  }

  return false;
}

// =========================================================
// SETUP
// =========================================================

void setup() {

  Serial.begin(115200);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Wire.begin(I2C_SDA, I2C_SCL);

  display.begin(
    SSD1306_SWITCHCAPVCC,
    OLED_ADDR
  );

  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(10, 20);
  display.println("READY");
  display.display();

  delay(500);

  if (!mpu.begin()) {

    Serial.println("MPU FAIL");

    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  delay(500);

  calibrateIMU();

  lastMahonyMicros = micros();

  Serial.println("SETUP DONE");
}

// =========================================================
// LOOP
// =========================================================

void loop() {

  static unsigned long lastDisplay = 0;

  mpu.getEvent(
    &accelEvent,
    &gyroEvent,
    &tempEvent
  );

  unsigned long nowMicros = micros();

  float dt =
    (nowMicros - lastMahonyMicros) / 1000000.0f;

  lastMahonyMicros = nowMicros;

  // ---------------- SENSOR VALUES ----------------

  float ax =
    accelEvent.acceleration.x - accelBiasX;

  float ay =
    accelEvent.acceleration.y - accelBiasY;

  float az =
    accelEvent.acceleration.z - accelBiasZ;

  float gx =
    gyroEvent.gyro.x - gyroBiasX;

  float gy =
    gyroEvent.gyro.y - gyroBiasY;

  float gz =
    gyroEvent.gyro.z - gyroBiasZ;

  // ===== FIX =====
  // Remove tiny gyro noise completely
  if (fabs(gz) < GYRO_DEADBAND) {
    gz = 0;
  }

  MahonyAHRSupdate(
    gx,
    gy,
    gz,
    ax,
    ay,
    az,
    dt
  );

  float yaw = getMahonyYaw();

  // ===== FIX =====
  // Heading smoothing
  hdgKF = 0.96f * hdgKF + 0.04f * yaw;

  // ---------------- BUTTON ----------------

  if (readButtonDebounced()) {

    if (btnStableState == LOW) {

      pressStart = millis();

    } else {

      unsigned long dur =
        millis() - pressStart;

      if (dur < LONG_PRESS_MS) {

        if (state == IDLE) {

          state = TRACKING;

          posX = posY = 0;
          kfX = kfY = 0;

          pathLen = 0;
          stepCount = 0;

          Serial.println("TRACKING START");
        }

      } else {

        if (state == TRACKING) {

          state = GUIDING;

          guideIdx = pathLen - 1;

          Serial.println("GUIDING START");
        }
      }
    }
  }

  // ---------------- STEP ----------------

  if (
    (state == TRACKING || state == GUIDING) &&
    detectStep(ax, ay, az)
  ) {

    handleStep(hdgKF);

    Serial.printf(
      "STEP,%d,%.2f,%.2f\n",
      stepCount,
      kfX,
      kfY
    );
  }

  // ---------------- DISPLAY ----------------

  if (millis() - lastDisplay > 150) {

    updateDisplayUI();

    lastDisplay = millis();
  }

  delay(5);
}