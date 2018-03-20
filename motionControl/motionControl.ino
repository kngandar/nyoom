#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// pin numbers
int brushL_pin = 7;
int brushR_pin = 8;
int servoL_pin = 6;
int servoR_pin = 5;

// constants
const int pulseUp = 675;
const int pulseCenter = 1553;
const int pulseDown = 2520;

int imuPos = 0; // global to store IMU "fwd" position
int servoPos = 2; // global for servo position. 1=up, 2=mid, 3=down

// init servos
Servo servoL, servoR, brushL, brushR;

//IMU setup
/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (400)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup() {
  // attach motors to pins
  brushL.attach(brushL_pin);
  brushR.attach(brushR_pin);
  servoL.attach(servoL_pin);
  servoR.attach(servoR_pin);
  
  // setup motors
  brushL.writeMicroseconds(2000); // max value
  brushR.writeMicroseconds(2000);
  delay(500);
  brushL.writeMicroseconds(700); // min value
  brushR.writeMicroseconds(700);
  delay(500);
  brushL.writeMicroseconds(1500); // middle value
  brushR.writeMicroseconds(1500);
  delay(500);
  
  // center servos
  setServoMid();
  delay(500);
  
  Serial.begin(115200);
  
  // Look for IMU
  if(!bno.begin()) {
    Serial.print("BNO055 not found.");
    while(!bno.begin()) {}
  }
  delay(1000);

  bno.setExtCrystalUse(true);
  // check IMU calibration
//  while (Serial.available() == 0) {
//    displayCalStatus();
//    delay(500);
//  }  

  Serial.println("Status: ready to go!!");
}


void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readString();
    
    if (input == "stop") {
      stopMotors();
    /** Image processing commands */
    } else if (input == "goDown") {
      goDown();
    } else if (input == "goUp") {
      goUp();
    /** XBOX control commands */
    } else if (input == "DU") { // face front
      // figure out when we can get the "front" measurement
      imuReorient();
    } else if (input == "DL") { // turn left
      setMotors(80,80);
    } else if (input == "DR") { // turn right
      setMotors(-70, -70);      
    } else if (input == "DD") { // stop (doesn't work)
      stopMotors();
    } else if (input == "A") { // forward
      setMotors(-80, 80);
    } else if (input == "B") { // backward      
      setMotors(80, -80);
    } else if (input == "Y") { // stop
      stopMotors();
    } else if (input == "X") { // panic button
      stopMotors();
      setServoMid(); // set servos to mid position
      delay(400); // dont run motors until servos done
      runStopMotors(-80, 80, 1000);
    } else if (input == "LB") { // servos up
      xboxServos("up");
    } else if (input == "RB") { // servos down
      xboxServos("down");
    } else {
      Serial.println("Error: invalid input");
    }
  }
}

/*
 * IMU SENSOR
 */

/** Display sensor calibration status */
void displayCalStatus(void) {
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system) {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}
 
/** 1st time: gets a reading for imuPos. After: reorient to initial value */
void imuReorient() {
  if (imuPos == 0) {
    sensors_event_t pos;
    bno.getEvent(&pos);
    imuPos = pos.orientation.x;
  } else {
    sensors_event_t curr;
    int tol = 15;
    int delta = 0;
    
    bno.getEvent(&curr);
    delta = checkDelta(curr.orientation.x - imuPos);
  
    if (delta < -tol && delta > -180) { 
      // LH-x: go CW
      if (servoPos != 3) {
        setServoMid();
        delay(400);
      }
      setMotors(-70, -70);
      while (delta < -tol && delta > -180) {}
      stopMotors();
    } else if (delta > tol && delta < 180) {
      // RH-x: go CCW
      if (servoPos != 3) {
        setServoMid();
        delay(400);
      }
      setMotors(80, 80);
      while (delta > tol && delta < 180) {}
      stopMotors();
    } 
  }
}

/** correct 0/360 boundary: put delta into the "expected" section */
int checkDelta(int delta) {
  if (delta > 180) {
    return delta-360;
  } else if (delta < -180) {
    return delta+360;
  } else {
    return delta;
  }
}


/*
 * MOTOR AND SERVO
 */

/** sets and stops the motors */
void runStopMotors(int left, int right, int runTime) {
  brushL.writeMicroseconds(motorValue(left));
  brushR.writeMicroseconds(motorValue(right));
  Serial.println("Status: running motors");
  delay(runTime);
  stopMotors();
}

void setMotors(int left, int right) {
  brushL.writeMicroseconds(motorValue(left));
  brushR.writeMicroseconds(motorValue(right));
  Serial.println("Status: running motors");
  delay(100);
}

/** translates value into the actual motor value. Returns '0' speed if invalid input. */
int motorValue(int value) {
  if ((value < -300) || (value > 300)) {
    return 1500;
  }
  return value + 1500;
}

/** stops the motors */
void stopMotors() {
  brushL.writeMicroseconds(motorValue(0));
  brushR.writeMicroseconds(motorValue(0));
  Serial.println("Status: motors stopped");
  delay(100);
}

/** sets servo to new position based on XBOX input */
void xboxServos(String upDown) {
  if (upDown == "up" && servoPos > 1 && servoPos <= 3) {
    // go up if current position is 2 to 3
    servoPos--;
  } else if (upDown == "down" && servoPos < 3 && servoPos >= 1) {
    // go down if current position is 1 to 2
    servoPos++;
  } else {
    // exit function. Cannot move up/down further
    return;
  }

  if (servoPos == 1) {
    setServoUp();
  } else if (servoPos == 2) {
    setServoMid();
  } else if (servoPos == 3) {
    setServoDown();
  } 
}

void setServoDown() {
  servoL.writeMicroseconds(pulseUp);
  servoR.writeMicroseconds(pulseDown);
  servoPos = 3;
}

void setServoMid() {
  servoL.writeMicroseconds(pulseCenter);
  servoR.writeMicroseconds(pulseCenter);
  servoPos = 2;
}

void setServoUp() {
  servoL.writeMicroseconds(pulseDown);
  servoR.writeMicroseconds(pulseUp);
  servoPos = 1;
}


/**
 * IMAGE PROCESSING COMMANDS
 */
 
void goDown() {
  // set servos to pos 1 (up)
  setServoUp();
  delay(100);
  // go forward for .5 seconds
  runStopMotors(80, -80, 500);
  // set servos back to initial position
  setServoMid();
}

void goUp() {
  // set servos to pos 5 (down)
  setServoDown();
  delay(100);
  // go forward for .5 seconds
  runStopMotors(80, -80, 500);
  // set servos back to initial position
  setServoMid();
}


