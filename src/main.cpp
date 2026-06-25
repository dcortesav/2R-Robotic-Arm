#include <Arduino.h>
#include "soc/gpio_struct.h"

// ── Encoder ───────────────────────────────────────────────────────────────────
#define ENC_A 18
#define ENC_B 19

volatile long encoderCount = 0;
portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;

const int PPR        = 11;
const int CPR_MOTOR  = PPR * 4;
const int GEAR_RATIO = 64;
const int CPR_OUTPUT = CPR_MOTOR * GEAR_RATIO;

// ── Motor L298N ───────────────────────────────────────────────────────────────
#define MOTOR_PWM 25
#define MOTOR_IN1 26
#define MOTOR_IN2 27

#define PWM_CHANNEL    0
#define PWM_FREQ       20000
#define PWM_RESOLUTION 8

// ── PID ───────────────────────────────────────────────────────────────────────
float targetAngle = 0;

float Kp = 8;
float Ki = 0.0;
float Kd = 0.5;

float Kg            = 70.0f;
const float PWM_MIN = 30.0f;
const float PWM_MAX = 255.0f;
const float DEG2RAD = PI / 180.0f;

float integral        = 0;
float previousError   = 0;
unsigned long previousPIDTime = 0;

// ── Trayectoria ───────────────────────────────────────────────────────────────
const float SETPOINTS[] = {
    0, 20, 40, 60, 80, 100, 120, 140, 160, 180,
    160, 140, 120, 100, 80, 60, 40, 20, 0
};
const int    N_SETPOINTS    = sizeof(SETPOINTS) / sizeof(SETPOINTS[0]);
int          setpointIdx    = 0;
uint32_t     stepIntervalMs = 500UL;  // ms entre puntos — ajustar por Serial (ej: "500")
uint32_t     lastStepMs     = 0;

// ── ISR ───────────────────────────────────────────────────────────────────────
void IRAM_ATTR encoderISR() {
    static uint8_t prevState = 0;

    uint8_t A = (GPIO.in >> ENC_A) & 1;
    uint8_t B = (GPIO.in >> ENC_B) & 1;

    uint8_t currentState = (A << 1) | B;
    uint8_t transition   = (prevState << 2) | currentState;
    prevState = currentState;

    portENTER_CRITICAL_ISR(&encMux);
    switch (transition) {
        case 0b0001: case 0b0111: case 0b1110: case 0b1000: encoderCount++; break;
        case 0b0010: case 0b0100: case 0b1101: case 0b1011: encoderCount--; break;
    }
    portEXIT_CRITICAL_ISR(&encMux);
}

// ── Posición angular ──────────────────────────────────────────────────────────
float getOutputAngle() {
    portENTER_CRITICAL(&encMux);
    long count = encoderCount;
    portEXIT_CRITICAL(&encMux);
    return (count * 360.0f) / CPR_OUTPUT;
}

// ── Motor ─────────────────────────────────────────────────────────────────────
void setMotor(float pwm) {
    pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
    if (fabsf(pwm) > 0.5f && fabsf(pwm) < PWM_MIN)
        pwm = copysignf(PWM_MIN, pwm);

    if (pwm > 0.5f) {
        digitalWrite(MOTOR_IN1, HIGH);
        digitalWrite(MOTOR_IN2, LOW);
        ledcWrite(PWM_CHANNEL, (uint32_t) pwm);
    } else if (pwm < -0.5f) {
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, HIGH);
        ledcWrite(PWM_CHANNEL, (uint32_t)(-pwm));
    } else {
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, LOW);
        ledcWrite(PWM_CHANNEL, 0);
    }
}

// ── PID ───────────────────────────────────────────────────────────────────────
float computePID(float setpoint, float position) {
    unsigned long now = millis();
    float dt = (now - previousPIDTime) / 1000.0f;
    if (dt <= 0) dt = 0.001f;
    previousPIDTime = now;

    float error = setpoint - position;

    if (fabsf(error) < 0.5f) {
        integral      = 0;
        previousError = error;
        return 0.0f;
    }

    integral += error * dt;
    float derivative = (error - previousError) / dt;
    previousError = error;

    float output = Kp * error + Ki * integral + Kd * derivative
                 + Kg * sinf(position * DEG2RAD);

    return constrain(output, -PWM_MAX, PWM_MAX);
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Serial.setTimeout(50);

    pinMode(ENC_A, INPUT_PULLUP);
    pinMode(ENC_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_B), encoderISR, CHANGE);

    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(MOTOR_PWM, PWM_CHANNEL);

    previousPIDTime = millis();
    targetAngle     = SETPOINTS[0];
    lastStepMs      = millis();
    Serial.printf("Intervalo actual: %lu ms — cambiar enviando un número (ej: \"500\")\n",
                  stepIntervalMs);
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
    // Intervalo ajustable por Serial
    if (Serial.available()) {
        String s = Serial.readStringUntil('\n');
        s.trim();
        if (s.length() > 0) {
            stepIntervalMs = (uint32_t)s.toInt();
            Serial.printf("Intervalo: %lu ms\n", stepIntervalMs);
        }
    }

    // Avance cíclico de la trayectoria
    uint32_t nowMs = millis();
    if (nowMs - lastStepMs >= stepIntervalMs) {
        setpointIdx = (setpointIdx + 1) % N_SETPOINTS;
        targetAngle = SETPOINTS[setpointIdx];
        integral    = 0.0f;
        lastStepMs  = nowMs;
    }

    float angle   = getOutputAngle();
    float control = computePID(targetAngle, angle);
    setMotor(control);

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 100) {
        lastPrint = millis();
        Serial.printf("th:%.2f th_des:%.2f e:%.2f pwm:%.1f\n",
                      angle, targetAngle, targetAngle - angle, control);
    }
}