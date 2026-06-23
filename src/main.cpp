#include <Arduino.h>
#include <AS5600.h>
#include <math.h>

// I2C0 pins (Encoder 1)
#define I2C0_SDA 18
#define I2C0_SCL 19
// I2C1 pins (Encoder 2)
#define I2C1_SDA 21
#define I2C1_SCL 22
// Bus speed
#define bus_speed 400000
// Two I2C buses
TwoWire I2C_Bus0 = TwoWire(0);
TwoWire I2C_Bus1 = TwoWire(1);
// AS5600 objects
AS5600 encoder1(&I2C_Bus0);
AS5600 encoder2(&I2C_Bus1);

static const uint32_t I2C_TIMEOUT_MS = 20;
static const uint32_t PRINT_PERIOD_MS = 1;

static const float GEAR_RATIO = 64.0f;
static const int32_t COUNTS_PER_REV = 4096;
static const float LOAD_DEG_PER_COUNT = 360.0f / (COUNTS_PER_REV * GEAR_RATIO);

bool zeroCaptured1 = false;
int32_t loadZeroCounts1 = 0;
uint32_t lastPrintMs = 0;

void setup() {
  // Begin serial communication
  Serial.begin(115200);
  while(!Serial);
  Serial.println("----------------------------");
  Serial.println("Serial communication established");

  // I2C interfaces initialization
  I2C_Bus0.begin(I2C0_SDA, I2C0_SCL, bus_speed);
  I2C_Bus0.setTimeOut(I2C_TIMEOUT_MS);
  delay(200);
  Serial.println("Bus 0 initialized");
  I2C_Bus1.begin(I2C1_SDA, I2C1_SCL, bus_speed);
  I2C_Bus1.setTimeOut(I2C_TIMEOUT_MS);
  delay(200);
  Serial.println("Bus 1 initialized");
  Serial.println("----------------------------");

  // AS5600 initialization
  encoder1.begin();
  encoder1.resetCumulativePosition();
  delay(200);
  encoder2.begin();
  encoder2.resetCumulativePosition();
  delay(200);

  if (encoder1.isConnected()) {
        Serial.println("Encoder 1: CONNECTED");
    } else {
        Serial.println("Encoder 1: ERROR");
    }

    if (encoder2.isConnected()) {
        Serial.println("Encoder 2: CONNECTED");
    } else {
        Serial.println("Encoder 2: ERROR");
    }
  Serial.println("----------------------------");

}

void loop() {
    int32_t positionCounts1 = encoder1.getCumulativePosition();
    float loadAngleAbs = positionCounts1 * LOAD_DEG_PER_COUNT;
    if (!zeroCaptured1) {
        loadZeroCounts1 = positionCounts1;
        zeroCaptured1 = true;
    }
    float loadAngleRel = (positionCounts1 - loadZeroCounts1) * LOAD_DEG_PER_COUNT;
    float loadAngle360 = fmodf(loadAngleRel, 360.0f);
    if (loadAngle360 < 0.0f) {
        loadAngle360 += 360.0f;
    }

    float nowMs = millis();
    if (nowMs - lastPrintMs >= PRINT_PERIOD_MS) {
        lastPrintMs = nowMs;
        Serial.println(loadAngle360, 2);
    }
}