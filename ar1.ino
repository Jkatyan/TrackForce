#include <Servo.h>
#include "MPU9250.h"
#include <Arduino.h>
#include "kalman.h"

char val; // Data received from the serial port

#define LEDPIN 13
#define WHEEL_MOTOR 9
#define SWERVE_MOTOR 11
#define POT_PIN A0
// IMU 4,5
#define TRIG_PIN 6
#define ECHO_PIN 7
#define STOP_DISTANCE 20
//int status;
float angle = 0; // for the IMU
float duration_us, distance_cm;
int pot;

Servo wheelMotor;
Servo swerveMotor;

Kalman::Kalman() {
    Q_angle = 0.001f;
    Q_bias = 0.003f;
    R_measure = 0.03f;

    angle = 0.0f; // Reset the angle
    bias = 0.0f; // Reset bias

    P[0][0] = 0.0f; 
    P[0][1] = 0.0f;
    P[1][0] = 0.0f;
    P[1][1] = 0.0f;
};

void setup() {
  pinMode(LEDPIN, OUTPUT); // Set pin as OUTPUT
  wheelMotor.attach(WHEEL_MOTOR);
  swerveMotor.attach(SWERVE_MOTOR);
  Serial.begin(9600); // Start serial communication at 9600 bits per second
  // configure the trigger pin to output mode
  pinMode(TRIG_PIN, OUTPUT);
  // configure the echo pin to input mode
  pinMode(ECHO_PIN, INPUT);
}

float Kalman::getAngle(float newAngle, float newRate, float dt) {
    rate = newRate - bias;
    angle += dt * rate;
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;
    
    float S = P[0][0] + R_measure; // Estimate error
 
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    float y = newAngle - angle; // Angle difference
    angle += K[0] * y;
    bias += K[1] * y;

    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
};

void Kalman::setAngle(float angle) { this->angle = angle; }; 
float Kalman::getRate() { return this->rate; };

void Kalman::setQangle(float Q_angle) { this->Q_angle = Q_angle; };
void Kalman::setQbias(float Q_bias) { this->Q_bias = Q_bias; };
void Kalman::setRmeasure(float R_measure) { this->R_measure = R_measure; };

float Kalman::getQangle() { return this->Q_angle; };
float Kalman::getQbias() { return this->Q_bias; };
float Kalman::getRmeasure() { return this->R_measure; };

void motorControl(int value, Servo motor) {
  motor.write(map(value, -100, 100, 1000, 2000));
}

void turnDegreesPID(int targetAngle, float kP, float kI, float kD) // PID Loop to get accurate positioning of turns
{
  float error;
  float range = 100;
  float integral = 0;
  float currentTime;
  float prevTime = 0;
  float prevError = 0;
  float derivative;;
  float power;
  float dt;
  while (abs(error) >= 1) {

    currentTime = millis();
    dt = currentTime - prevTime;
    prevTime = currentTime;
    error = targetAngle - Kalman::getAngle();
    integral += error * dt;
    if (error > 100) {
      integral = 0;
    }
    derivative = (error - prevError) / dt;
    prevError = error;
    power = error * kP + integral + kI + derivative * kD;
    power = fabs(power) > 100 ? 100 * power / fabs(power) : power;
    motorControl(power, swerveMotor);
    Serial.println(power);
    delay(20);
  }
}

float getAngle() {
  return 270 * analogRead(POT_PIN) / 1023;
}

void turnDegreesPot (int targetAngle, unsigned int power) {
  if (analogRead(POT_PIN) <= targetAngle - 10) {
    motorControl(-power, swerveMotor);
  }
  else if (analogRead(POT_PIN) >= targetAngle + 10) {
    motorControl(power, swerveMotor);
  }
  else {
    motorControl(0, swerveMotor);
    //targetAngle = 1000 for right taretAngle = 400 for left
  }
}

void loop() {

  Serial.println(getSonarDistance());
  digitalWrite(LEDPIN, HIGH);
  while(getSonarDistance()>10.0){
  pot = analogRead(0);
  if (Serial.available() > 0) { // Allow for Serial to connect before reading
    val = Serial.read();
  }
  if (getSonarDistance() <= 10){
    val = '0';
  }
  if (val == '0') {  //stop
    turnDegreesPID(700, 0.003, 0.000001, 0.02);
    motorControl(0, wheelMotor);
  }
  else if (val == '1') {  //forwards

    turnDegreesPID(700, 0.003, 0.000001, 0.02);
    motorControl(60, wheelMotor);
  }
  else if (val == '2') {

    turnDegreesPID(700, 0.003, 0.000001, 0.02);
    motorControl(-60, wheelMotor);

  }
  else if (val == '3') {
    turnDegreesPID(400, 0.003, 0.000001, 0.02);
    if(pot>500){
      motorControl(50, wheelMotor);
    }     

  }
  else if (val == '4') {
    turnDegreesPID(1000, 0.003, 0.000001, 0.02);
    if(pot>900){
      motorControl(50, wheelMotor);
    }     
  }
  else {
    turnDegreesPID(700, 0.003, 0.000001, 0.02);
    motorControl(0, wheelMotor);
  }
  }
  turnDegreesPID(700, 50);
  motorControl(0, wheelMotor);
  
}

float getSonarDistance() {
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // measure duration of pulse from ECHO pin
  duration_us = pulseIn(ECHO_PIN, HIGH);

  // calculate the distance
  distance_cm = 0.017 * duration_us;
  return distance_cm;
}
