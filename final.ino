#include <AccelStepper.h>
#include <MPU6050.h>
#include <QMC5883LCompass.h>
#include <Wire.h>
#include <math.h>

MPU6050 mpu;
QMC5883LCompass compass;

#define MOTOR_INTERFACE_TYPE 4  // // Define motor interface type: 4-pin full-step mode

AccelStepper motor1_alt(MOTOR_INTERFACE_TYPE, 11, 9, 10, 8); // Altitude motor: IN1-IN4 = 11, 9, 10, 8
AccelStepper motor2_azi(MOTOR_INTERFACE_TYPE, 7, 5, 6, 4);   // Azimuth motor:  IN1-IN4 = 7, 5, 6, 4

// LED Pins
const int ledY = A0;
const int ledB = A1;  
const int ledR = A2;
const int ledG = A3;

// Serial buffer and timeout
#define MAX_INPUT 64
char input[MAX_INPUT];
byte index = 0;
unsigned long lastCharTime = 0;
const unsigned long serialTimeout = 100;  // ms

// Arduino-to-Pi feedback timing
unsigned long lastFeedback = 0;
unsigned long feedbackInterval = 500;

float targetAzimuth = 0.0;
float targetAltitude = 0.0;
bool targetReady = false;
bool stopRequested = false;
bool alignmentReported = false;

//float readInclinometer();
//float readMagnetometer();

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();
  compass.init();
  
  pinMode(ledB, OUTPUT);
  pinMode(ledR, OUTPUT);
  pinMode(ledY, OUTPUT);
  pinMode(ledG, OUTPUT);
  digitalWrite(ledB, LOW);
  digitalWrite(ledR, LOW);
  digitalWrite(ledY, LOW);
  digitalWrite(ledG, LOW);

  motor1_alt.setMaxSpeed(1000);
  motor1_alt.setAcceleration(200);
  motor2_azi.setMaxSpeed(1000);
  motor2_azi.setAcceleration(200);
  
  Serial.println("READY");
}

void readSerialNonBlocking() {
  while (Serial.available()) {
    char c = Serial.read();
    lastCharTime = millis();

    if (c == '\n') {
      input[index] = '\0';
      parseCommand(input);
      index = 0;
    } else if (index < MAX_INPUT - 1) {
      input[index++] = c;
    }
  }

  if (index > 0 && millis() - lastCharTime > serialTimeout) {
    index = 0;  // Flush partial message
  }
}

void parseCommand(char* msg) {
  if (strcmp(msg, "STOP") == 0) {
    stopRequested = true;
    targetReady = false;
    Serial.println("STOP received");
    return;
  }

  // Expecting "az alt"
  char* azStr = strtok(msg, " ");
  char* altStr = strtok(nullptr, " ");

  if (azStr && altStr) {
    targetAzimuth = atof(azStr);
    targetAltitude = atof(altStr);
    targetReady = true;
    alignmentReported = false;
    stopRequested = false;
    Serial.println("ACK");
  }
}

void loop() {
  // Always allow motors stepping
  motor1_alt.run();  
  motor2_azi.run();
  
  readSerialNonBlocking(); 
  
  if (stopRequested) {        // Gracefully decelerate to stop
      motor1_alt.stop();      // motor1_alt.setCurrentPosition(0);   // Optional reset
      motor2_azi.stop();      // motor2_azi.setCurrentPosition(0);   // Optional reset
      return;    
  }

  float currentAngle = readInclinometer();
  float currentHeading = readMagnetometer();
      
  if (targetReady) {

    // Calculate difference between current reading of inClinometer and target position
    float angleError = angularError(currentAngle, targetAltitude);
    if (fabs(angleError) > 20.0) {
      digitalWrite(ledG, HIGH);

      int angleDirection = (angleError > 0) ? 1 : -1;          
      motor1_alt.setSpeed(angleDirection * 300);   // speed in steps/sec
      motor1_alt.runSpeed();                       // continuous speed, non-blocking
    
    } else if (fabs(angleError) <= 5.0) {          // prevent jitter near the threshold
      motor1_alt.setSpeed(0);                      // stop motor
      digitalWrite(ledG, LOW); 
    }

    // Calculate difference between current reading if magnetometer and target position
    float headingError = angularError(currentHeading, targetAzimuth);
    if (fabs(headingError) > 20.0) {
      digitalWrite(ledY, HIGH);
    
      int headingDirection = (headingError > 0) ? 1 : -1;          
      motor2_azi.setSpeed(headingDirection * 300);  // speed in steps/sec
      motor2_azi.runSpeed();                        // continuous speed, non-blocking
    
    } else if (fabs(headingError) <= 5.0) {         // prevent jitter near the threshold
      motor2_azi.setSpeed(0);                       // stop motor
      digitalWrite(ledY, LOW);
    }

    // Send feedback to Pi at certain interval
    unsigned long now = millis();
    if(now - lastFeedback >= feedbackInterval){
      lastFeedback = now;
      char buffer[64];
      sprintf(buffer, "... angle delta: %.1f | heading delta: %.1f", angleError, headingError);
      Serial.println(buffer);
    }
    
    targetReady = false;

    if (!alignmentReported && fabs(angleError) <= 5.0 && fabs(headingError) <= 5.0) {
      Serial.println("Target aligned");
      alignmentReported = true;
    }
  }
    //sprintf(buffer, "Tracker Angle: %.2f | Heading: %.2f", currentAngle, currentHeading);
    //Serial.println(buffer);
    
}

float angularError(float current, float target) {
  float diff = fmod((target - current + 540.0), 360.0) - 180.0;
  return diff;
}

float readInclinometer() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  float Ax = ax/16384.0;
  float Ay = ay/16384.0;
  float Az = az/16384.0;
  return atan2(Ax, sqrt(Ay * Ay + Az * Az)) * 180 / PI;
}

float readMagnetometer() {
  compass.read();
  int heading = compass.getAzimuth();
  if (heading < 0)   heading += 360;
  if (heading > 360) heading -= 360;
  return heading;
}
