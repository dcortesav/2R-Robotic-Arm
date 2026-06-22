#include <Arduino.h>
#include <AS5600.h>


const int PWM_FREQUENCY = 5000;
const int PWM_RESOLUTION = 10;

//Motor 1 associated variables
#define MOTOR_1_PWM  27
#define MOTOR_1_FORWARD 26
#define MOTOR_1_BACKWARD 25
const int MOTOR_1_PWM_CHANNEL = 0;

int duty = 0;
bool forward_dir = true;

void setup(){
  Serial.begin(115200);
  while(!Serial);
  ledcSetup(MOTOR_1_PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_1_PWM, MOTOR_1_PWM_CHANNEL);
  ledcWrite(MOTOR_1_PWM_CHANNEL, duty);
  pinMode(MOTOR_1_FORWARD, OUTPUT);
  pinMode(MOTOR_1_BACKWARD, OUTPUT);
  if(forward_dir){
    digitalWrite(MOTOR_1_FORWARD, HIGH);
    digitalWrite(MOTOR_1_BACKWARD, LOW);
  }else{
    digitalWrite(MOTOR_1_FORWARD, LOW);
    digitalWrite(MOTOR_1_BACKWARD, HIGH);
  }
  Serial.println("Setup Completed");
  delay(100);
}

void loop() {
  if (Serial.available() > 0) {
    int value = Serial.parseInt();
    Serial.print("Value: ");
    Serial.println(value);
    if (value > 0) {
      if(!forward_dir){
        digitalWrite(MOTOR_1_FORWARD, HIGH);
        digitalWrite(MOTOR_1_BACKWARD, LOW);
        forward_dir = true;
      }
      duty = value;
      ledcWrite(MOTOR_1_PWM_CHANNEL, duty);
    } else if (value < 0) {
      if(forward_dir){
        digitalWrite(MOTOR_1_FORWARD, LOW);
        digitalWrite(MOTOR_1_BACKWARD, HIGH);
        forward_dir = false;
      }
      duty = -value;
      ledcWrite(MOTOR_1_PWM_CHANNEL, duty);
    }
  }
}