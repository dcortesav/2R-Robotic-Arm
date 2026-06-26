#include <Arduino.h>
#include "soc/gpio_struct.h"
#include <vector>               // trayectoria dinámica

// ── Encoder 1 ─────────────────────────────────────────────────────────────────
#define ENC_A 19
#define ENC_B 18

// ── Encoder 2 ─────────────────────────────────────────────────────────────────
#define ENC_A2 35
#define ENC_B2 34

volatile long encoderCount  = 0;
volatile long encoderCount2 = 0;
portMUX_TYPE encMux  = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE encMux2 = portMUX_INITIALIZER_UNLOCKED;

const int PPR        = 11;
const int CPR_MOTOR  = PPR * 4;
const int GEAR_RATIO = 64;
const int CPR_OUTPUT = CPR_MOTOR * GEAR_RATIO;

// ── Motor 1 L298N ─────────────────────────────────────────────────────────────
#define MOTOR_PWM  25
#define MOTOR_IN1  26
#define MOTOR_IN2  27

// ── Motor 2 L298N ─────────────────────────────────────────────────────────────
#define MOTOR_PWM2 14
#define MOTOR_IN3  13
#define MOTOR_IN4   4

#define PWM_CHANNEL    0
#define PWM_CHANNEL2   1
#define PWM_FREQ       20000
#define PWM_RESOLUTION 8

// ── PID Motor 1 ───────────────────────────────────────────────────────────────
float targetAngle      = 0;
float integral         = 0;
float previousError    = 0;
unsigned long previousPIDTime = 0;

// ── PID Motor 2 ───────────────────────────────────────────────────────────────
float targetAngle2     = 0;
float integral2        = 0;
float previousError2   = 0;
unsigned long previousPIDTime2 = 0;

float Kp = 5.0f,  Ki = 0.0f,  Kd = 0.8f,  Kg = 70.0f;
float Kp2 = 10.0f, Ki2 = 0.0f, Kd2 = 0.8f, Kg2 = 7.0f;

const float PWM_MIN   = 30.0f,  PWM_MAX   = 255.0f;
const float PWM_MIN_2 = 0.0f,   PWM_MAX_2 = 255.0f;
const float DEG2RAD   = PI / 180.0f;

// ── Máquina de estados ────────────────────────────────────────────────────────
enum RobotState { WAIT_FOR_TRAJECTORY, RUN_APPROACH, RUN_CLOVER, FINISHED };
RobotState robotState = WAIT_FOR_TRAJECTORY;

// ── Trayectoria dinámica ──────────────────────────────────────────────────────
std::vector<float> approachM1, approachM2, cloverM1, cloverM2;

int      nApproach      = 0;
int      nClover        = 0;
int      nReps          = 1;
int      trajIdx        = 0;
int      cloverRep      = 0;
uint32_t stepIntervalMs = 20UL;
uint32_t lastStepMs     = 0;

// ── Telemetría ────────────────────────────────────────────────────────────────
// Prefijo "T:" identifica telemetría. Python filtra por este prefijo.
// Frecuencia: 25 Hz (40 ms). No mezclar con mensajes de protocolo.
const uint32_t TELEMETRY_MS = 40UL;
uint32_t       lastTelemMs  = 0;

// ── Buffer Serial ─────────────────────────────────────────────────────────────
String lineBuffer = "";

// ── ISR Encoder 1 ─────────────────────────────────────────────────────────────
void IRAM_ATTR encoderISR() {
    static uint8_t prevState = 0;
    uint8_t A = (GPIO.in >> ENC_A) & 1;
    uint8_t B = (GPIO.in >> ENC_B) & 1;
    uint8_t cur = (A << 1) | B;
    uint8_t tr  = (prevState << 2) | cur;
    prevState = cur;
    portENTER_CRITICAL_ISR(&encMux);
    switch (tr) {
        case 0b0001: case 0b0111: case 0b1110: case 0b1000: encoderCount++;  break;
        case 0b0010: case 0b0100: case 0b1101: case 0b1011: encoderCount--;  break;
    }
    portEXIT_CRITICAL_ISR(&encMux);
}

// ── ISR Encoder 2 (banco extendido GPIO.in1.val) ──────────────────────────────
void IRAM_ATTR encoderISR2() {
    static uint8_t prevState = 0;
    uint8_t A = (GPIO.in1.val >> (ENC_A2 - 32)) & 1;
    uint8_t B = (GPIO.in1.val >> (ENC_B2 - 32)) & 1;
    uint8_t cur = (A << 1) | B;
    uint8_t tr  = (prevState << 2) | cur;
    prevState = cur;
    portENTER_CRITICAL_ISR(&encMux2);
    switch (tr) {
        case 0b0001: case 0b0111: case 0b1110: case 0b1000: encoderCount2++; break;
        case 0b0010: case 0b0100: case 0b1101: case 0b1011: encoderCount2--; break;
    }
    portEXIT_CRITICAL_ISR(&encMux2);
}

// ── Posición angular Motor 1 ──────────────────────────────────────────────────
float getOutputAngle() {
    portENTER_CRITICAL(&encMux);
    long c = encoderCount;
    portEXIT_CRITICAL(&encMux);
    return (c * 360.0f) / CPR_OUTPUT;
}

// ── Posición angular Motor 2 ──────────────────────────────────────────────────
float getOutputAngle2() {
    portENTER_CRITICAL(&encMux2);
    long c = encoderCount2;
    portEXIT_CRITICAL(&encMux2);
    return (c * 360.0f) / CPR_OUTPUT;
}

// ── Motor 1 ───────────────────────────────────────────────────────────────────
void setMotor(float pwm) {
    pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
    if (fabsf(pwm) > 0.5f && fabsf(pwm) < PWM_MIN) pwm = copysignf(PWM_MIN, pwm);
    if (pwm > 0.5f) {
        digitalWrite(MOTOR_IN1, HIGH); digitalWrite(MOTOR_IN2, LOW);
        ledcWrite(PWM_CHANNEL, (uint32_t)pwm);
    } else if (pwm < -0.5f) {
        digitalWrite(MOTOR_IN1, LOW);  digitalWrite(MOTOR_IN2, HIGH);
        ledcWrite(PWM_CHANNEL, (uint32_t)(-pwm));
    } else {
        digitalWrite(MOTOR_IN1, LOW);  digitalWrite(MOTOR_IN2, LOW);
        ledcWrite(PWM_CHANNEL, 0);
    }
}

// ── Motor 2 ───────────────────────────────────────────────────────────────────
void setMotor2(float pwm) {
    pwm = constrain(pwm, -PWM_MAX_2, PWM_MAX_2);
    if (fabsf(pwm) > 0.5f && fabsf(pwm) < PWM_MIN_2) pwm = copysignf(PWM_MIN_2, pwm);
    if (pwm > 0.5f) {
        digitalWrite(MOTOR_IN3, HIGH); digitalWrite(MOTOR_IN4, LOW);
        ledcWrite(PWM_CHANNEL2, (uint32_t)pwm);
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

// ── PID Motor 2 ───────────────────────────────────────────────────────────────
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
                 + Kg2 * sinf((position + position_1) * DEG2RAD);
    return constrain(output, -PWM_MAX_2, PWM_MAX_2);
}

// ── Parser CSV ────────────────────────────────────────────────────────────────
// Usa strtof (puntero directo) para evitar crear substrings temporales.
// Necesario para líneas de ~3 KB (400 floats) sin fragmentar heap.
void parseCSV(const String& s, std::vector<float>& out) {
    const char* p = s.c_str();
    char* end;
    while (*p) {
        float v = strtof(p, &end);
        if (end == p) break;
        out.push_back(v);
        p = end;
        if (*p == ',') p++;
    }
}

// ── Procesador de líneas del protocolo Serial ─────────────────────────────────
// Protocolo:
//   Python → ESP32 : TRAJ_START <n_app> <n_clv> <n_reps> <step_ms>
//                    M1A:<csv> | M2A:<csv> | M1C:<csv> | M2C:<csv>
//                    TRAJ_END
//   ESP32  → Python: OK:HEADER | OK:M1A | OK:M2A | OK:M1C | OK:M2C
//                    READY | ERROR:<msg> | T:<ms>,<th1>,<th2>,<sp1>,<sp2> | DONE
void processLine(const String& line) {
    if (line.startsWith("TRAJ_START ")) {
        int na, nc, nr, sms;
        if (sscanf(line.c_str() + 11, "%d %d %d %d", &na, &nc, &nr, &sms) == 4) {
            nApproach = na; nClover = nc; nReps = nr;
            stepIntervalMs = (uint32_t)sms;
            approachM1.clear(); approachM1.reserve(na);
            approachM2.clear(); approachM2.reserve(na);
            cloverM1.clear();   cloverM1.reserve(nc);
            cloverM2.clear();   cloverM2.reserve(nc);
            Serial.println("OK:HEADER");
        } else {
            Serial.println("ERROR:TRAJ_START format invalid");
        }
    } else if (line.startsWith("M1A:")) {
        parseCSV(line.substring(4), approachM1);
        Serial.println("OK:M1A");
    } else if (line.startsWith("M2A:")) {
        parseCSV(line.substring(4), approachM2);
        Serial.println("OK:M2A");
    } else if (line.startsWith("M1C:")) {
        parseCSV(line.substring(4), cloverM1);
        Serial.println("OK:M1C");
    } else if (line.startsWith("M2C:")) {
        parseCSV(line.substring(4), cloverM2);
        Serial.println("OK:M2C");
    } else if (line == "TRAJ_END") {
        bool ok = (int)approachM1.size() == nApproach &&
                  (int)approachM2.size() == nApproach &&
                  (int)cloverM1.size()   == nClover   &&
                  (int)cloverM2.size()   == nClover;
        if (ok) {
            trajIdx      = 0;
            cloverRep    = 0;
            targetAngle  = approachM1[0];
            targetAngle2 = approachM2[0];
            robotState   = RUN_APPROACH;
            lastStepMs   = millis();
            Serial.println("READY");
        } else {
            Serial.printf("ERROR:size M1A=%d M2A=%d M1C=%d M2C=%d expected_app=%d expected_clv=%d\n",
                (int)approachM1.size(), (int)approachM2.size(),
                (int)cloverM1.size(),   (int)cloverM2.size(),
                nApproach, nClover);
        }
    } else if (line == "PING") {
    // Responde siempre con el estado actual — útil si Python conecta en caliente
    if (robotState == WAIT_FOR_TRAJECTORY) {
        Serial.println("AWAITING_TRAJECTORY");
    } else {
        Serial.println("BUSY");
    }
}
}

// ── Lector Serial no bloqueante ───────────────────────────────────────────────
// Acumula caracteres en lineBuffer. Cuando llega '\n', procesa la línea completa.
// Corre en cada iteración del loop sin afectar el tiempo del PID.
void readSerialLine() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n') {
            lineBuffer.trim();
            if (lineBuffer.length() > 0) processLine(lineBuffer);
            lineBuffer = "";
        } else if (c != '\r') {
            lineBuffer += c;
        }
    }
}

// ── Telemetría ────────────────────────────────────────────────────────────────
// Formato: T:<ms>,<theta1>,<theta2>,<setpoint1>,<setpoint2>
// El prefijo "T:" permite que Python filtre sin ambigüedad con mensajes de protocolo.
void sendTelemetry(uint32_t nowMs) {
    if (nowMs - lastTelemMs < TELEMETRY_MS) return;
    lastTelemMs = nowMs;
    Serial.printf("T:%lu,%.2f,%.2f,%.2f,%.2f\n",
                  nowMs,
                  getOutputAngle(), getOutputAngle2(),
                  targetAngle, targetAngle2);
}

// ── Estado: RUN_APPROACH ──────────────────────────────────────────────────────
void runApproach(uint32_t nowMs) {
    float a1 = getOutputAngle();
    float a2 = getOutputAngle2();
    setMotor(computePID(targetAngle, a1));
    setMotor2(computePID2(targetAngle2, a2, a1));

    if (nowMs - lastStepMs >= stepIntervalMs) {
        lastStepMs = nowMs;
        trajIdx++;
        if (trajIdx >= nApproach) {
            // Transición a trébol — reset integradores
            trajIdx   = 0;
            integral  = 0; integral2 = 0;
            targetAngle  = cloverM1[0];
            targetAngle2 = cloverM2[0];
            robotState   = RUN_CLOVER;
        } else {
            targetAngle  = approachM1[trajIdx];
            targetAngle2 = approachM2[trajIdx];
        }
    }
}

// ── Estado: RUN_CLOVER ────────────────────────────────────────────────────────
void runClover(uint32_t nowMs) {
    float a1 = getOutputAngle();
    float a2 = getOutputAngle2();
    setMotor(computePID(targetAngle, a1));
    setMotor2(computePID2(targetAngle2, a2, a1));

    if (nowMs - lastStepMs >= stepIntervalMs) {
        lastStepMs = nowMs;
        trajIdx++;
        if (trajIdx >= nClover) {
            trajIdx = 0;
            cloverRep++;
            integral = 0; integral2 = 0;
            if (cloverRep >= nReps) {
                // Trayectoria completa
                setMotor(0);
                setMotor2(0);
                robotState = FINISHED;
                Serial.println("DONE");
            } else {
                // Siguiente repetición del trébol
                targetAngle  = cloverM1[0];
                targetAngle2 = cloverM2[0];
            }
        } else {
            targetAngle  = cloverM1[trajIdx];
            targetAngle2 = cloverM2[trajIdx];
        }
    }
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    // setRxBufferSize DEBE llamarse antes de begin().
    // 8 KB cubre líneas de hasta ~6400 floats — más que suficiente.
    Serial.setRxBufferSize(8192);
    Serial.begin(115200);
    Serial.setTimeout(50);

    // Encoder 1
    pinMode(ENC_A, INPUT_PULLUP);
    pinMode(ENC_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_A),  encoderISR,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_B),  encoderISR,  CHANGE);

    // Encoder 2 — GPIO 34/35 input-only, sin pullup interno
    pinMode(ENC_A2, INPUT);
    pinMode(ENC_B2, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENC_A2), encoderISR2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_B2), encoderISR2, CHANGE);

    // Motor 1
    pinMode(MOTOR_IN1, OUTPUT); pinMode(MOTOR_IN2, OUTPUT);
    ledcSetup(PWM_CHANNEL,  PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(MOTOR_PWM,  PWM_CHANNEL);

    // Motor 2
    pinMode(MOTOR_IN3, OUTPUT); pinMode(MOTOR_IN4, OUTPUT);
    ledcSetup(PWM_CHANNEL2, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(MOTOR_PWM2, PWM_CHANNEL2);

    previousPIDTime  = millis();
    previousPIDTime2 = millis();

    // Señal de que la ESP32 está lista para recibir trayectoria
    Serial.println("AWAITING_TRAJECTORY");
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
    uint32_t nowMs = millis();

    // Recepción Serial no bloqueante — siempre activa independiente del estado
    readSerialLine();

    switch (robotState) {
        case WAIT_FOR_TRAJECTORY:
            // Motores apagados. Solo readSerialLine() corre.
            break;

        case RUN_APPROACH:
            runApproach(nowMs);
            sendTelemetry(nowMs);
            break;

        case RUN_CLOVER:
            runClover(nowMs);
            sendTelemetry(nowMs);
            break;

        case FINISHED:
            // Motores detenidos en runClover(). Nada que hacer.
            break;
    }
}