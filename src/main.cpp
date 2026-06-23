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
static const uint32_t SAMPLE_PERIOD_MS = 5;
static const uint32_t SETTLE_TIME_MS = 3000;

static const float GEAR_RATIO = 64.0f;
static const int32_t COUNTS_PER_REV = 4096;
static const float LOAD_DEG_PER_COUNT = 360.0f / (COUNTS_PER_REV * GEAR_RATIO);

bool zeroCaptured1 = false;
int32_t loadZeroCounts1 = 0;
uint32_t lastPrintMs = 0;
static const uint16_t MAX_SAMPLES = 3000;
uint32_t sampleTimesMs[MAX_SAMPLES];
float sampleAnglesDeg[MAX_SAMPLES];
uint16_t sampleCount = 0;
uint32_t lastSampleMs = 0;

const int PWM_FREQUENCY = 5000;
const int PWM_RESOLUTION = 8;

//Motor 1 associated variables
#define MOTOR_1_PWM  27
#define MOTOR_1_FORWARD 26
#define MOTOR_1_BACKWARD 25
const int MOTOR_1_PWM_CHANNEL = 0;
int motor_1_duty = 0;
bool motor_1_forward_dir = true;

static const uint32_t START_TIME_MS = 5000;
bool started = false;
bool finished = false;
static const uint32_t STOP_TIME_MS = 5500;
uint32_t stopCommandMs = 0;
bool stopAngleCaptured = false;
bool samplesDumped = false;
float stopAngleDeg = 0.0f;

void setup() {
  // Begin serial communication
  Serial.begin(115200);
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

  ledcSetup(MOTOR_1_PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_1_PWM, MOTOR_1_PWM_CHANNEL);
  delay(200);
  ledcWrite(MOTOR_1_PWM_CHANNEL, 0);
  pinMode(MOTOR_1_FORWARD, OUTPUT);
  delay(200);
  pinMode(MOTOR_1_BACKWARD, OUTPUT);
  delay(200);
  if(motor_1_forward_dir){
    digitalWrite(MOTOR_1_FORWARD, HIGH);
    digitalWrite(MOTOR_1_BACKWARD, LOW);
  }else{
    digitalWrite(MOTOR_1_FORWARD, LOW);
    digitalWrite(MOTOR_1_BACKWARD, HIGH);
  }

}

void dumpSamples() {
  Serial.println("time_ms,angle_deg");
  for (uint16_t i = 0; i < sampleCount; i++) {
    Serial.print(sampleTimesMs[i]);
    Serial.print(',');
    Serial.println(sampleAnglesDeg[i], 4);
  }
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

    uint32_t nowMs = millis();
    if (nowMs >= START_TIME_MS && started == false){
      ledcWrite(MOTOR_1_PWM_CHANNEL, 30);
      started = true;
    }
    if (started && !finished && (nowMs - lastSampleMs >= SAMPLE_PERIOD_MS) && sampleCount < MAX_SAMPLES) {
      lastSampleMs = nowMs;
      sampleTimesMs[sampleCount] = nowMs - START_TIME_MS;
      sampleAnglesDeg[sampleCount] = loadAngle360;
      sampleCount++;
    }
    if (nowMs >= STOP_TIME_MS && started == true && finished == false){
      ledcWrite(MOTOR_1_PWM_CHANNEL, 0);
      finished = true;
      stopCommandMs = nowMs;
    }

    if (finished && !stopAngleCaptured && (nowMs - stopCommandMs >= SETTLE_TIME_MS)) {
      stopAngleDeg = loadAngle360;
      stopAngleCaptured = true;
      if (sampleCount < MAX_SAMPLES) {
        sampleTimesMs[sampleCount] = nowMs - START_TIME_MS;
        sampleAnglesDeg[sampleCount] = stopAngleDeg;
        sampleCount++;
      }
      Serial.print("Stopped angle: ");
      Serial.println(stopAngleDeg, 4);
      dumpSamples();
      samplesDumped = true;
    }

}