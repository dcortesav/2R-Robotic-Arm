#include <Arduino.h>
#include "soc/gpio_struct.h"   // ← NUEVO: GPIO.in para ISR rápida

// ── Encoder ───────────────────────────────────────────────────────────────────
#define ENC_A 18
#define ENC_B 19

volatile long encoderCount = 0;
portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;  // ← NUEVO

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

float Kp = 2;
float Ki = 0.01;
float Kd = 0.000;

const float Kg      = 10.0f;    // ← NUEVO: compensación gravitacional [PWM/sin(θ)]
const float PWM_MIN = 30.0f;   // ← NUEVO: umbral mínimo para vencer fricción estática
const float PWM_MAX = 150.0f;  // ← NUEVO: límite de movimiento brusco
const float DEG2RAD = PI / 180.0f;  // ← NUEVO

float integral      = 0;
float previousError = 0;
unsigned long previousPIDTime = 0;

// ── ISR ───────────────────────────────────────────────────────────────────────
void IRAM_ATTR encoderISR() {
    static uint8_t prevState = 0;

    uint8_t A = (GPIO.in >> ENC_A) & 1;   // ← GPIO.in en lugar de digitalRead
    uint8_t B = (GPIO.in >> ENC_B) & 1;   // ← GPIO.in en lugar de digitalRead

    uint8_t currentState = (A << 1) | B;
    uint8_t transition   = (prevState << 2) | currentState;
    prevState = currentState;

    portENTER_CRITICAL_ISR(&encMux);       // ← portMUX en lugar de nada
    switch (transition) {
        case 0b0001: case 0b0111: case 0b1110: case 0b1000: encoderCount++; break;
        case 0b0010: case 0b0100: case 0b1101: case 0b1011: encoderCount--; break;
    }
    portEXIT_CRITICAL_ISR(&encMux);        // ← portMUX
}

// ── Posición angular ──────────────────────────────────────────────────────────
float getOutputAngle() {
    portENTER_CRITICAL(&encMux);           // ← portMUX en lugar de noInterrupts
    long count = encoderCount;
    portEXIT_CRITICAL(&encMux);            // ← portMUX en lugar de interrupts
    return (count * 360.0f) / CPR_OUTPUT;
}

// ── Motor ─────────────────────────────────────────────────────────────────────
void setMotor(float pwm) {                 // ← float en lugar de int
    pwm = constrain(pwm, -PWM_MAX, PWM_MAX);  // ← PWM_MAX en lugar de 255
    if (fabsf(pwm) > 0.5f && fabsf(pwm) < PWM_MIN)  // ← NUEVO: PWM mínimo
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
float computePID(float setpoint, float position) {  // ← float return
    unsigned long now = millis();
    float dt = (now - previousPIDTime) / 1000.0f;
    if (dt <= 0) dt = 0.001f;
    previousPIDTime = now;

    float error = setpoint - position;

    if (fabsf(error) < 0.5f) {            // ← fabsf en lugar de abs
        integral      = 0;
        previousError = error;
        return 0.0f;
    }

    integral += error * dt;
    float derivative = (error - previousError) / dt;
    previousError = error;

    float output = Kp * error + Ki * integral + Kd * derivative
                 + Kg * sinf(position * DEG2RAD);  // ← NUEVO: feedforward gravitacional

    return constrain(output, -PWM_MAX, PWM_MAX);   // ← PWM_MAX, float return
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Serial.setTimeout(50);                 // ← NUEVO: evita bloqueo en readStringUntil

    pinMode(ENC_A, INPUT_PULLUP);
    pinMode(ENC_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_B), encoderISR, CHANGE);

    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(MOTOR_PWM, PWM_CHANNEL);

    previousPIDTime = millis();
    Serial.println("Listo. Enviar setpoint en grados (ej: 135)");
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
    // ← NUEVO: setpoint por Serial
    if (Serial.available()) {
        String s = Serial.readStringUntil('\n');
        s.trim();
        if (s.length() > 0) {
            targetAngle = s.toFloat();
            integral    = 0.0f;
            Serial.printf("Setpoint: %.2f°\n", targetAngle);
        }
    }

    float angle   = getOutputAngle();
    float control = computePID(targetAngle, angle);  // ← float en lugar de int
    setMotor(control);

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 100) {
        lastPrint = millis();
        // ← telemetría: ángulo, error, pwm (en lugar de target + angle + pwm)
        Serial.printf("th:%.2f th_des:%.2f e:%.2f  pwm:%.1f\n",
                      angle,targetAngle, targetAngle - angle, control);
    }
}