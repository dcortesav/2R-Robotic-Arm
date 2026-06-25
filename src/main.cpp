#include <Arduino.h>
#include "soc/gpio_struct.h"

// ── Encoder 1 ─────────────────────────────────────────────────────────────────
#define ENC_A 19
#define ENC_B 18

// ── Encoder 2 ── NUEVO ────────────────────────────────────────────────────────
#define ENC_A2 35
#define ENC_B2 34

volatile long encoderCount  = 0;
volatile long encoderCount2 = 0;                          // NUEVO
portMUX_TYPE encMux  = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE encMux2 = portMUX_INITIALIZER_UNLOCKED;     // NUEVO

static unsigned long lastPrint = 0;

const int PPR        = 11;
const int CPR_MOTOR  = PPR * 4;
const int GEAR_RATIO = 64;
const int CPR_OUTPUT = CPR_MOTOR * GEAR_RATIO;

// ── Motor 1 L298N ─────────────────────────────────────────────────────────────
#define MOTOR_PWM  25
#define MOTOR_IN1  26
#define MOTOR_IN2  27

// ── Motor 2 L298N ── NUEVO ────────────────────────────────────────────────────
#define MOTOR_PWM2 14                                      // NUEVO
#define MOTOR_IN3  13                                      // NUEVO
#define MOTOR_IN4   4                                      // NUEVO

#define PWM_CHANNEL  0
#define PWM_CHANNEL2 1                                     // NUEVO
#define PWM_FREQ       20000
#define PWM_RESOLUTION 8

// ── PID Motor 1 ───────────────────────────────────────────────────────────────
float targetAngle      = 0;
float integral         = 0;
float previousError    = 0;
unsigned long previousPIDTime = 0;

// ── PID Motor 2 ── NUEVO ──────────────────────────────────────────────────────
float targetAngle2     = 0;                               // NUEVO
float integral2        = 0;                               // NUEVO
float previousError2   = 0;                               // NUEVO
unsigned long previousPIDTime2 = 0;                       // NUEVO

// ── PID ───────────────────────────────────────────────────────────────────────
float Kp = 5;
float Ki = 0.0;
float Kd = 0.8;
float Kg = 70.0f;

// ── PID Motor 2 ── NUEVO ──────────────────────────────────────────────────────
float Kp2 = 10;      // NUEVO
float Ki2 = 0.0;      // NUEVO
float Kd2 = 0.8;   // NUEVO
float Kg2 = 7.0f; // NUEVO

const float PWM_MIN = 30.0f;
const float PWM_MAX = 255.0f;
const float DEG2RAD = PI / 180.0f;

const float PWM_MIN_2 = 0.0f;
const float PWM_MAX_2 = 150.0f;

/*{0,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,
170,160,150,140,130,120,110,100,90,80,70,60,50,40,30,20,10};*/

// ── Trayectoria ───────────────────────────────────────────────────────────────
const float SETPOINTS_MOTOR1[] = {   // renombrado desde SETPOINTS
    0.0,2.63,5.26,7.88,10.5,13.11,15.72,18.32,20.9,23.48,26.04,28.58,31.11,33.62,36.11,38.59,41.03,43.46,45.85,48.23,50.57,52.88,55.16,57.41,59.62,61.8,63.93,66.03,68.09,70.11,72.08,74.01,75.89,77.72,79.5,81.24,82.91,84.54,86.11,87.62,89.07,90.46,91.79,93.06,94.26,95.4,96.47,97.46,98.39,99.25,99.25,99.67,100.06,100.41,100.72,101.0,101.25,101.46,101.63,101.78,101.89,101.97,102.01,102.03,102.01,101.97,101.9,101.8,101.68,101.53,101.36,101.16,100.94,100.71,100.45,100.17,99.88,99.57,99.25,98.93,98.6,98.27,97.96,97.68,97.43,97.25,97.17,97.21,97.39,97.71,98.16,98.7,99.32,99.98,100.68,101.41,102.14,102.88,103.63,104.37,105.1,105.83,106.54,107.24,107.93,108.59,109.24,109.87,110.47,111.06,111.62,112.15,112.66,113.15,113.6,114.03,114.43,114.8,115.14,115.44,115.72,115.96,116.17,116.35,116.49,116.6,116.67,116.7,116.7,116.66,116.58,116.45,116.29,116.08,115.83,115.54,115.2,114.81,114.38,113.89,113.36,112.77,112.14,111.45,110.71,109.91,109.07,108.18,107.23,106.24,105.21,104.14,103.04,101.92,100.78,99.63,98.5,97.4,96.34,95.37,94.51,93.8,93.27,92.94,92.79,92.79,92.91,93.11,93.37,93.68,94.01,94.36,94.71,95.07,95.42,95.77,96.1,96.4,96.69,96.94,97.14,97.3,97.4,97.42,97.35,97.17,96.86,96.39,95.73,94.83,93.67,92.19,90.37,88.17,85.59,82.66,79.43,75.99,72.42,68.84,65.32,61.94,58.76,55.82,53.14,50.73,48.59,46.72,45.1,43.72,42.57,41.63,40.89,40.32,39.91,39.65,39.52,39.52,39.62,39.82,40.11,40.48,40.92,41.42,41.98,42.58,43.21,43.88,44.56,45.25,45.95,46.63,47.27,47.86,48.37,48.75,48.98,49.01,48.86,48.53,48.07,47.51,46.91,46.29,45.68,45.08,44.52,44.01,43.54,43.13,42.78,42.5,42.27,42.11,42.02,41.99,42.01,42.1,42.25,42.46,42.71,43.02,43.38,43.78,44.23,44.71,45.23,45.79,46.37,46.99,47.63,48.29,48.98,49.68,50.39,51.12,51.86,52.6,53.36,54.11,54.87,55.63,56.38,57.14,57.88,58.62,59.36,60.08,60.79,61.5,62.19,62.87,63.53,64.18,64.81,65.43,66.03,66.62,67.19,67.74,68.28,68.8,69.31,69.79,70.27,70.72,71.17,71.6,72.02,72.43,72.85,73.27,73.71,74.17,74.66,75.19,75.75,76.34,76.96,77.62,78.3,79.0,79.73,80.48,81.25,82.04,82.83,83.64,84.46,85.28,86.11,86.94,87.76,88.58,89.4,90.2,91.0,91.77,92.54,93.28,94.0,94.7,95.37,96.02,96.63,97.22,97.78,98.3,98.79,99.25
};
const int   N_SETPOINTS = sizeof(SETPOINTS_MOTOR1) / sizeof(SETPOINTS_MOTOR1[0]);

// NUEVO — reemplazar con la trayectoria real del motor 2 (debe tener N_SETPOINTS elementos)
const float SETPOINTS_MOTOR2[N_SETPOINTS] = {
    0.0,1.89,3.77,5.66,7.54,9.42,11.29,13.15,15.0,16.85,18.68,20.51,22.32,24.11,25.89,27.66,29.4,31.13,32.83,34.52,36.18,37.82,39.43,41.02,42.58,44.12,45.62,47.09,48.53,49.93,51.31,52.64,53.94,55.2,56.43,57.61,58.75,59.85,60.9,61.91,62.87,63.79,64.65,65.47,66.23,66.95,67.61,68.21,68.76,69.25,69.25,69.44,69.7,70.05,70.46,70.95,71.51,72.13,72.83,73.58,74.39,75.26,76.18,77.15,78.16,79.21,80.3,81.43,82.58,83.76,84.96,86.18,87.41,88.66,89.9,91.15,92.4,93.64,94.87,96.07,97.25,98.39,99.48,100.52,101.47,102.31,103.02,103.57,103.96,104.18,104.28,104.29,104.23,104.14,104.03,103.93,103.83,103.74,103.68,103.64,103.64,103.66,103.72,103.82,103.95,104.12,104.32,104.57,104.86,105.19,105.56,105.96,106.41,106.9,107.43,107.99,108.59,109.23,109.9,110.61,111.35,112.12,112.92,113.75,114.61,115.49,116.39,117.32,118.26,119.22,120.19,121.18,122.17,123.17,124.18,125.18,126.19,127.18,128.17,129.15,130.11,131.06,131.98,132.87,133.74,134.57,135.37,136.13,136.85,137.52,138.14,138.71,139.23,139.7,140.12,140.49,140.82,141.11,141.37,141.63,141.91,142.23,142.63,143.12,143.73,144.43,145.21,146.06,146.97,147.91,148.89,149.89,150.91,151.95,153.01,154.07,155.14,156.21,157.28,158.36,159.42,160.47,161.52,162.54,163.53,164.5,165.42,166.3,167.11,167.85,168.5,169.05,169.47,169.75,169.88,169.86,169.68,169.36,168.9,168.33,167.67,166.92,166.11,165.25,164.34,163.39,162.42,161.43,160.43,159.41,158.38,157.35,156.32,155.29,154.25,153.23,152.2,151.18,150.17,149.17,148.18,147.2,146.23,145.28,144.34,143.42,142.51,141.63,140.76,139.91,139.09,138.3,137.53,136.8,136.11,135.46,134.85,134.29,133.75,133.24,132.72,132.18,131.61,131.01,130.37,129.68,128.96,128.19,127.38,126.53,125.64,124.73,123.78,122.8,121.81,120.79,119.75,118.7,117.64,116.57,115.49,114.42,113.34,112.27,111.21,110.15,109.11,108.09,107.08,106.1,105.14,104.2,103.29,102.42,101.57,100.76,99.99,99.26,98.57,97.91,97.31,96.74,96.23,95.76,95.34,94.96,94.64,94.37,94.14,93.97,93.84,93.77,93.74,93.76,93.83,93.94,94.09,94.29,94.52,94.8,95.1,95.44,95.8,96.17,96.56,96.96,97.34,97.69,98.0,98.22,98.32,98.26,98.0,97.54,96.9,96.1,95.19,94.18,93.11,92.0,90.85,89.69,88.51,87.32,86.14,84.96,83.8,82.65,81.52,80.42,79.34,78.3,77.3,76.34,75.42,74.55,73.74,72.98,72.28,71.64,71.07,70.57,70.14,69.78,69.5,69.29,69.16,69.12,69.14,69.25
};

int      setpointIdx    = 0;
uint32_t stepIntervalMs = 20UL;
uint32_t lastStepMs     = 0;

// ── ISR Encoder 1 ─────────────────────────────────────────────────────────────
void IRAM_ATTR encoderISR() {
    static uint8_t prevState = 0;
    uint8_t A = (GPIO.in >> ENC_A) & 1;
    uint8_t B = (GPIO.in >> ENC_B) & 1;
    uint8_t currentState = (A << 1) | B;
    uint8_t transition   = (prevState << 2) | currentState;
    prevState = currentState;
    portENTER_CRITICAL_ISR(&encMux);
    switch (transition) {
        case 0b0001: case 0b0111: case 0b1110: case 0b1000: encoderCount++;  break;
        case 0b0010: case 0b0100: case 0b1101: case 0b1011: encoderCount--;  break;
    }
    portEXIT_CRITICAL_ISR(&encMux);
}

// ── ISR Encoder 2 ── NUEVO ────────────────────────────────────────────────────
// GPIO 34/35 están en el banco extendido: registro GPIO.in1.val (bits pin-32)
void IRAM_ATTR encoderISR2() {
    static uint8_t prevState = 0;
    uint8_t A = (GPIO.in1.val >> (ENC_A2 - 32)) & 1;     // NUEVO
    uint8_t B = (GPIO.in1.val >> (ENC_B2 - 32)) & 1;     // NUEVO
    uint8_t currentState = (A << 1) | B;
    uint8_t transition   = (prevState << 2) | currentState;
    prevState = currentState;
    portENTER_CRITICAL_ISR(&encMux2);
    switch (transition) {
        case 0b0001: case 0b0111: case 0b1110: case 0b1000: encoderCount2++; break;
        case 0b0010: case 0b0100: case 0b1101: case 0b1011: encoderCount2--; break;
    }
    portEXIT_CRITICAL_ISR(&encMux2);
}

// ── Posición angular Motor 1 ──────────────────────────────────────────────────
float getOutputAngle() {
    portENTER_CRITICAL(&encMux);
    long count = encoderCount;
    portEXIT_CRITICAL(&encMux);
    return (count * 360.0f) / CPR_OUTPUT;
}

// ── Posición angular Motor 2 ── NUEVO ─────────────────────────────────────────
float getOutputAngle2() {
    portENTER_CRITICAL(&encMux2);
    long count = encoderCount2;
    portEXIT_CRITICAL(&encMux2);
    return (count * 360.0f) / CPR_OUTPUT;
}

// ── Motor 1 ───────────────────────────────────────────────────────────────────
void setMotor(float pwm) {
    pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
    if (fabsf(pwm) > 0.5f && fabsf(pwm) < PWM_MIN) pwm = copysignf(PWM_MIN, pwm);
    if (pwm > 0.5f) {
        digitalWrite(MOTOR_IN1, HIGH); digitalWrite(MOTOR_IN2, LOW);
        ledcWrite(PWM_CHANNEL, (uint32_t) pwm);
    } else if (pwm < -0.5f) {
        digitalWrite(MOTOR_IN1, LOW);  digitalWrite(MOTOR_IN2, HIGH);
        ledcWrite(PWM_CHANNEL, (uint32_t)(-pwm));
    } else {
        digitalWrite(MOTOR_IN1, LOW);  digitalWrite(MOTOR_IN2, LOW);
        ledcWrite(PWM_CHANNEL, 0);
    }
}

// ── Motor 2 ── NUEVO ──────────────────────────────────────────────────────────
void setMotor2(float pwm) {
    pwm = constrain(pwm, -PWM_MAX_2, PWM_MAX_2);
    if (fabsf(pwm) > 0.5f && fabsf(pwm) < PWM_MIN_2) pwm = copysignf(PWM_MIN_2, pwm);
    if (pwm > 0.5f) {
        digitalWrite(MOTOR_IN3, HIGH); digitalWrite(MOTOR_IN4, LOW);
        ledcWrite(PWM_CHANNEL2, (uint32_t) pwm);
    } else if (pwm < -0.5f) {
        digitalWrite(MOTOR_IN3, LOW);  digitalWrite(MOTOR_IN4, HIGH);
        ledcWrite(PWM_CHANNEL2, (uint32_t)(-pwm));
    } else {
        digitalWrite(MOTOR_IN3, LOW);  digitalWrite(MOTOR_IN4, LOW);
        ledcWrite(PWM_CHANNEL2, 0);
    }
}

// ── PID Motor 1 ───────────────────────────────────────────────────────────────
float computePID(float setpoint, float position) {
    unsigned long now = millis();
    float dt = (now - previousPIDTime) / 1000.0f;
    if (dt <= 0) dt = 0.001f;
    previousPIDTime = now;
    float error = setpoint - position;
    if (fabsf(error) < 0.5f) { integral = 0; previousError = error; return 0.0f; }
    integral += error * dt;
    float derivative = (error - previousError) / dt;
    previousError = error;
    float output = Kp * error + Ki * integral + Kd * derivative
                 + Kg * sinf(position * DEG2RAD);
    return constrain(output, -PWM_MAX, PWM_MAX);
}

// ── PID Motor 2 ── NUEVO ──────────────────────────────────────────────────────
float computePID2(float setpoint, float position, float position_1) {
    unsigned long now = millis();
    float dt = (now - previousPIDTime2) / 1000.0f;
    if (dt <= 0) dt = 0.001f;
    previousPIDTime2 = now;
    float error = setpoint - position;
    if (fabsf(error) < 0.5f) { integral2 = 0; previousError2 = error; return 0.0f; }
    integral2 += error * dt;
    float derivative = (error - previousError2) / dt;
    previousError2 = error;
    float output = Kp2 * error + Ki2 * integral2 + Kd2 * derivative
                 + Kg2 * sinf((position + position_1) * DEG2RAD);   // ← ÚNICO CAMBIO: Kp→Kp2, Ki→Ki2, Kd→Kd2, Kg→Kg2
    return constrain(output, -PWM_MAX_2, PWM_MAX_2);
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Serial.setTimeout(50);

    // Encoder 1
    pinMode(ENC_A, INPUT_PULLUP);
    pinMode(ENC_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_A),  encoderISR,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_B),  encoderISR,  CHANGE);

    // Encoder 2 — GPIO 34/35 no soportan INPUT_PULLUP; el encoder tiene pullup integrado
    pinMode(ENC_A2, INPUT);                                               // NUEVO
    pinMode(ENC_B2, INPUT);                                               // NUEVO
    attachInterrupt(digitalPinToInterrupt(ENC_A2), encoderISR2, CHANGE); // NUEVO
    attachInterrupt(digitalPinToInterrupt(ENC_B2), encoderISR2, CHANGE); // NUEVO

    // Motor 1
    pinMode(MOTOR_IN1, OUTPUT); pinMode(MOTOR_IN2, OUTPUT);
    ledcSetup(PWM_CHANNEL,  PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(MOTOR_PWM,  PWM_CHANNEL);

    // Motor 2
    pinMode(MOTOR_IN3, OUTPUT); pinMode(MOTOR_IN4, OUTPUT);              // NUEVO
    ledcSetup(PWM_CHANNEL2, PWM_FREQ, PWM_RESOLUTION);                   // NUEVO
    ledcAttachPin(MOTOR_PWM2, PWM_CHANNEL2);                             // NUEVO

    previousPIDTime  = millis();
    previousPIDTime2 = millis();                                          // NUEVO
    targetAngle      = SETPOINTS_MOTOR1[0];
    targetAngle2     = SETPOINTS_MOTOR2[0];                               // NUEVO
    lastStepMs       = millis();
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
    uint32_t nowMs = millis();

    if (nowMs - lastStepMs >= stepIntervalMs) {
        setpointIdx  = (setpointIdx + 1) % N_SETPOINTS;
        targetAngle  = SETPOINTS_MOTOR1[setpointIdx];
        targetAngle2 = SETPOINTS_MOTOR2[setpointIdx];                     // NUEVO
        integral     = 0.0f;
        integral2    = 0.0f;                                              // NUEVO
        lastStepMs   = nowMs;
    }

    float angle    = getOutputAngle();
    float control  = computePID(targetAngle, angle);
    setMotor(control);

    float angle2   = getOutputAngle2();                                   // NUEVO
    float control2 = computePID2(targetAngle2, angle2,angle);                  // NUEVO
    setMotor2(control2);                                                  // NUEVO

    if (millis() - lastPrint > 100) {
        lastPrint = millis();
        Serial.printf("M1 th:%.2f des:%.2f e:%.2f pwm:%.1f | M2 th:%.2f des:%.2f e:%.2f pwm:%.1f\n",
                      angle,  targetAngle,  targetAngle  - angle,  control,
                      angle2, targetAngle2, targetAngle2 - angle2, control2);
    }
}