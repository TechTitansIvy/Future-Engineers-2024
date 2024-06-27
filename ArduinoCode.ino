#include <Wire.h>
#include <VL53L0X.h>
#include <Servo.h>
#include "Wire.h"
#include <MPU6050_light.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH1106.h>

#define OLED_RESET 4
Adafruit_SH1106 display(OLED_RESET);

#if (SH1106_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SH1106.h!");
#endif

MPU6050 mpu(Wire);
unsigned long timer = 0;
float cumulativeRotation = 0; // Track cumulative rotation in degrees
float previousAngleZ = 0; // Track previous Z angle
int lapCount = 0; // Count the number of laps


// Define pin connections
int LEFT_MOTOR_PIN1 = 3;
int LEFT_MOTOR_PIN2 = 9;
int RIGHT_MOTOR_PIN1 = 10;
int RIGHT_MOTOR_PIN2 = 11;

int startbutton = 12;
#define SERVO_PIN 2

// Create objects for VL53L0X sensors
VL53L0X sensorFront;
VL53L0X sensorLeft;
VL53L0X sensorRight;

// Create servo object
Servo steeringServo;

// Define thresholds and initial servo angle
int middleDistance = 100; // Target distance from walls
int tolerance = 50; // Tolerance for being centered
int initialServoAngle = 90; // Servo angle for going straight
int maxLeftAngle = 60;  // Maximum left turn angle
int maxRightAngle = 120; // Maximum right turn angle

// PID control variables
float kp = 0.1;  // Proportional gain
float ki = 0.0;  // Integral gain
float kd = 0.1;  // Derivative gain

float previousError = 0;
float integral = 0;
String incomingData;

bool overtaking = false;
unsigned long overtakingStartTime = 0;
int overtakingDuration = 1500; // Time in milliseconds to overtake
int State = 0;
int StartState = 0;
int Speed = 150;
void setup() {

  Serial.begin(9600);
  Wire.begin();

  display.begin(SH1106_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(20, 20);
  display.println("Standby!");
  display.display();

  byte status = mpu.begin();
  while (status != 0) { }
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets();

  timer = millis();
  steeringServo.attach(SERVO_PIN);
  steeringServo.write(initialServoAngle);

  pinMode(startbutton, INPUT_PULLUP);

  pinMode(LEFT_MOTOR_PIN1, OUTPUT);
  pinMode(LEFT_MOTOR_PIN2, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN1, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN2, OUTPUT);


  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);

  digitalWrite(LEFT_MOTOR_PIN1, LOW);
  digitalWrite(LEFT_MOTOR_PIN2, LOW);
  digitalWrite(RIGHT_MOTOR_PIN1, LOW);
  digitalWrite(RIGHT_MOTOR_PIN2, LOW);


  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  delay(500);

  digitalWrite(5, HIGH);
  delay(150);
  sensorFront.init(true);
  delay(100);
  sensorFront.setAddress((uint8_t)01);
  digitalWrite(4, HIGH);
  delay(150);
  sensorLeft.init(true);
  delay(100);
  sensorLeft.setAddress((uint8_t)02);
  digitalWrite(6, HIGH);
  delay(150);
  sensorRight.init(true);
  delay(100);
  sensorRight.setAddress((uint8_t)03);

  sensorFront.startContinuous();
  sensorLeft.startContinuous();
  sensorRight.startContinuous();
}

void loop() {

  int buttonread = digitalRead(startbutton);

  if (buttonread == LOW and StartState == 0)
  {
    StartState = 1;
  }

  if (StartState == 1)
  {
    display.clearDisplay();
    int distanceLeft = sensorLeft.readRangeContinuousMillimeters();
    int distanceFront = sensorFront.readRangeContinuousMillimeters();
    int distanceRight = sensorRight.readRangeContinuousMillimeters();
    if (Serial.available() > 0) {
      incomingData = Serial.readStringUntil('\n');
    }

    if (incomingData == "left")
    {
      distanceRight = distanceRight - 200;
    }
    if (incomingData == "right")
    {
      distanceLeft = distanceLeft - 200;
    }

    mpu.update();

    int error = distanceLeft - distanceRight;
    integral += error;
    float derivative = error - previousError;
    float output = kp * error + ki * integral + kd * derivative;
    previousError = error;

    int newAngle = initialServoAngle + output;
    newAngle = constrain(newAngle, maxLeftAngle, maxRightAngle);

    if (lapCount == 3) {
      stopRobot();
    }
    else
    {
      steeringServo.write(newAngle);
      moveForward();
    }

    if ((millis() - timer) > 10) {
      timer = millis();

      float currentAngleZ = mpu.getAngleZ();
      float deltaAngleZ = currentAngleZ - previousAngleZ;

      if (deltaAngleZ > 180) {
        deltaAngleZ -= 360;
      } else if (deltaAngleZ < -180) {
        deltaAngleZ += 360;
      }

      cumulativeRotation += deltaAngleZ;
      previousAngleZ = currentAngleZ;

      if (cumulativeRotation >= 360 || cumulativeRotation <= -360) {
        lapCount++;

        cumulativeRotation = 0;
      }
    }
    display.setCursor(20, 25);
    display.print("Moving");
    display.setCursor(20, 45);
    display.print(incomingData);
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(20, 0);
    display.print("Laps : ");
    display.println(lapCount);
    display.display();
    delay(10);
  }
}

void moveForward() {
  analogWrite(LEFT_MOTOR_PIN1, Speed);
  digitalWrite(LEFT_MOTOR_PIN2, LOW);
  analogWrite(RIGHT_MOTOR_PIN2, Speed);
  digitalWrite(RIGHT_MOTOR_PIN1, LOW);
}

void stopRobot() {
  digitalWrite(LEFT_MOTOR_PIN1, LOW);
  digitalWrite(LEFT_MOTOR_PIN2, LOW);
  digitalWrite(RIGHT_MOTOR_PIN1, LOW);
  digitalWrite(RIGHT_MOTOR_PIN2, LOW);
}
