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
// for RC in case of sinking
const int pulseDownMid = 2036;
const int pulseUpMid = 1114;

const int servoDelay = 400;

int imuPos = 0; // global to store IMU "fwd" position
String imuStatus = "FWD";
int servoPos = 2; // global for servo position. 1=up, 2=mid, 3=down

// init servos
Servo servoL, servoR, brushL, brushR;

/** IMU setup */
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
  delay(servoDelay);
  
  Serial.begin(115200);
  
  // Look for IMU
  if(!bno.begin()) {
    Serial.print("BNO055 not found.");
    while(!bno.begin()) {}
  }
  delay(1000);

  bno.setExtCrystalUse(true);
  // check IMU calibration
  while (Serial.available() == 0) {
    displayCalStatus();
    delay(500);
  }  

  Serial.println("go");
}


void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readString();
    
    if (input == "stop") {
      stopMotors();
    } else if (input == "1") {
      doCourse();
      Serial.println("done");
    /** Image processing commands */
    } else if (input == "goDown") {
      goDown();
    } else if (input == "goUp") {
      goUp();
    } else if (input == "fwStop") {
      fwStop();
    } else if (input == "bwStop") {
      bwStop();
    } else if (input == "goFWD") {
      goFWD();
    } else if (input == "goBWD") {
      goBWD();
    } else if (input == "moveL") {
      moveL();  
    } else if (input == "moveR") {
      moveR();
    /** XBOX control commands */
    } else if (input == "DU") { // face front
      // figure out when we can get the "front" measurement
      //imuReorient();
      setServoDownMid();
    } else if (input == "DL") { // turn left
      setMotorsL();
    } else if (input == "DR") { // turn right
      setMotorsR();      
    } else if (input == "DD") { // stop (doesn't work)im
      setServoMid();
    } else if (input == "A") { // forward
      setMotorsFWD();
    } else if (input == "B") { // backward      
      setMotorsBWD();
    } else if (input == "Y") { // stop
      stopMotors();
    } else if (input == "X") { // panic button
      stopMotors();
      setServoMid();     // set servos to mid position
      delay(servoDelay); // dont run motors until servos done
      setMotorsBWD();    // go bwd to avoid bumping obstacle
      delay(1000);
      stopMotors();      // stop motors
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
 * FAKE AUTONOMOUS
 */
void doCourse() {
  // get "forward" IMU reading
  imuPos = 0;
  imuCheck();

  // obstacle 1: go forward
  setMotorsFWD();
  waitAndCheck(6);
  stopMotors();
  
  // obstacle 2
  setServoDown();
  delay(servoDelay);
  setMotorsFWD();
  delay(3000);
  stopMotors();
  setServoMid();
  delay(servoDelay);

  // obstacle 3
  setMotorsFWD();
  waitAndCheck(12);
  stopMotors();
}

/** Input n seconds you want to delay and IMU correct for */
void waitAndCheck(int n) {
  for (int i = 0; i < n; i++) {
    delay(1000);
    imuCheck();
  }
}


/*
 * IMU SENSOR
 */

/** slightly increases speed of L or R motor to correct IMU heading */
void imuCheck() {
  if (imuPos == 0) {
    sensors_event_t pos;
    bno.getEvent(&pos);
    imuPos = pos.orientation.x;
  } else {
    sensors_event_t curr;
    int tol = 5;
    int delta = 0;

    bno.getEvent(&curr);
    delta = checkDelta(curr.orientation.x - imuPos);
    
    Serial.print("imuPos = ");
    Serial.println(imuPos); 
    Serial.print("Current = ");
    Serial.println(curr.orientation.x);

    if (delta < -tol && delta > -180) { 
      // LH-x: correct right (Default L is -100)
      if (imuStatus != "R") {
        brushL.writeMicroseconds(motorValue(-130));
        imuStatus = "R";
      }      
    } else if (delta > tol && delta < 180) {
      // RH-x: correct left (default R is 80)
      if (imuStatus != "L") {
        brushR.writeMicroseconds(motorValue(110));
        imuStatus = "L";
      }
    }  else {
      if (imuStatus != "FWD") {
        setMotorsFWD();
        imuStatus = "FWD";
      }
    }
  }
}

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
  Serial.println(mag, DEC);
}
 
/** 1st time: gets a reading for imuPos. After: reorient to initial value */
void imuReorient() {
  if (imuPos == 0) {
    sensors_event_t pos;
    bno.getEvent(&pos);
    imuPos = pos.orientation.x;
  } else {
    sensors_event_t curr;
    int tol = 10;
    int delta = 0;

    bno.getEvent(&curr);
    delta = checkDelta(curr.orientation.x - imuPos);

    if (servoPos != 2) {
      setServoMid();
      delay(servoDelay);
    }
    
    Serial.print("imuPos =");
    Serial.println(imuPos);
    while (tol < abs(delta)) {
      bno.getEvent(&curr);
      delta = checkDelta(curr.orientation.x - imuPos);
      
      Serial.print("Current =");
      Serial.println(curr.orientation.x);

      if (delta < -tol && delta > -180) { 
        setMotorsR();
        delay(800);
        stopMotors();
        delay(200);
      } else if (delta > tol && delta < 180) {
        // RH-x: go CCW
        setMotorsL();
        delay(800);
        stopMotors();
        delay(200);
      } 
    }
    Serial.println("IMUdone");
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
void setMotors(int left, int right) {
  brushL.writeMicroseconds(motorValue(left));
  brushR.writeMicroseconds(motorValue(right));
  //Serial.println("Status: running motors");
  delay(100);
}

/** stops the motors */
void stopMotors() {
  brushL.writeMicroseconds(motorValue(0));
  brushR.writeMicroseconds(motorValue(0));
  //Serial.println("Status: motors stopped");
  delay(500);
}

void setMotorsFWD() {
  setMotors(-100, 80);
}

void setMotorsBWD() {
  setMotors(65, -80);
}

void setMotorsL() {
  setMotors(80, 80);
  delay(500);
  stopMotors();
}

void setMotorsR() {
  setMotors(-80, -80);
  delay(500);
  stopMotors();
}

/** translates value into the actual motor value. Returns '0' speed if invalid input. */
int motorValue(int value) {
  if ((value < -300) || (value > 300)) {
    return 1500;
  }
  return value + 1500;
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
  } else if (servoPos == 4) {
    // prevent shenanigans due to xbox down-mid command
    setServoMid();
  }
}

void setServoUp() {
  servoL.writeMicroseconds(pulseUp);
  servoR.writeMicroseconds(pulseDown);
  servoPos = 1;
}

void setServoMid() {
  servoL.writeMicroseconds(pulseCenter);
  servoR.writeMicroseconds(pulseCenter);
  servoPos = 2;
}

void setServoDown() {
  servoL.writeMicroseconds(pulseDown);
  servoR.writeMicroseconds(pulseUp);
  servoPos = 3;
}

void setServoDownMid() {
  servoL.writeMicroseconds(pulseDownMid);
  servoR.writeMicroseconds(pulseUpMid);
  // "invalid" position
  servoPos = 4;
}


/**
 * IMAGE PROCESSING COMMANDS
 */

/** Servos up, motors fwd for .5 s, motors stop, servos mid */ 
void goDown() {
  stopMotors();
  setServoUp();
  delay(servoDelay);
  setMotorsFWD();
  delay(1000);
  stopMotors();
  setServoMid();
  delay(servoDelay);
}

/** Servos down, motors fwd for .5 s, motors stop, servos mid */ 
void goUp() {
  stopMotors();
  setServoDown();
  delay(servoDelay);
  setMotorsFWD();
  delay(1000);
  stopMotors();
  setServoMid();
  delay(servoDelay);
}

/** stop and account for forward drift */
void fwStop() {
  stopMotors();
  setMotorsBWD();
  delay(500);
  stopMotors();
}

/** stop and account for backward drift */
void bwStop() {
  stopMotors();
  setMotorsFWD();
  delay(500);
  stopMotors();
}

/** set servos to mid, go fwd (without stopping) */
void goFWD() {
  stopMotors();
  setServoMid();
  delay(servoDelay);
  setMotorsFWD();
  delay(500); // prevent any shenanigans
}

/** set servos to mid, go bwd (without stopping) */
void goBWD() {
  stopMotors();
  setServoMid();
  delay(servoDelay);
  setMotorsBWD();
  delay(500); // prevent any shenanigans
}

/** shift sub to left */
void moveL() {
  stopMotors();
  imuReorient();
  setMotorsL();
  delay(500); // fix 90 deg timing
  stopMotors();
  setMotorsFWD();
  delay(500);
  stopMotors();
  imuReorient();
  Serial.println("imDone");
}

/** shift sub to right */
void moveR() {
  stopMotors();
  imuReorient();
  setMotorsR();
  delay(500); // fix 90 deg timing
  stopMotors();
  setMotorsFWD();
  delay(500);
  stopMotors();
  imuReorient();
  Serial.println("imDone");
}


