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
const int pulseUpMid = 1114;
const int pulseCenter = 1553;
const int pulseDownMid = 2036;
const int pulseDown = 2520;

// global variable for servo position T_T
// 1: up, 2: up-mid, 3: mid, 4: down-mid, 5: down
int servoPos = 3;
int imuPos = 0;

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
  setServoPos3();
  delay(500);
  
  Serial.begin(9600);
  
  // Look for IMU
  if(!bno.begin()) {
    Serial.print("BNO055 not found.");
    while(!bno.begin()) {}
  }
  delay(1000);

  bno.setExtCrystalUse(true);
  Serial.println("Status: ready to go!!");
}


void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readString();
    
    if (input == "runMotors") {
      runMotors();
    } else if (input == "100") {
      delay(250);
      foundGreen();
    } else if (input == "200") {
      delay(250);
      foundRed();
    } else if (input == "300") {
      delay(250);
      foundYellow();
    } else if (input == "turnCW") {
      turn90CW();
    } else if (input == "turnCCW") {
      turn90CCW();
    } else if (input == "stop") {
      stopMotors();
    /** Image processing commands */
    } else if (input == "goDown") {
      goDown();
    } else if (input == "goUp") {
      goUp();
    /** XBOX control commands */
    } else if (input == "DU_Front") { // face front
      // figure out when we can get the "front" measurement
      imuReorient();
    } else if (input == "DL_Left") {
      setMotors(-70, -70);
    } else if (input == "DL_Right") {
      setMotors(80,80);
    } else if (input == "DD_Motor_Stop") {
      stopMotors();
    } else if (input == "A_Motor_Forward") {
      setMotors(80, -80);
    } else if (input == "B_Motor_Backward") {
      setMotors(-80, 80);
    } else if (input == "Y_Servo_Middle") {
      setServoPos3();
    } else if (input == "X_Panic_Button") {
      stopMotors();
      setServoPos3(); // set servos to mid position
      delay(400); // dont run motors until servos done
      runStopMotors(-80, 80, 1000);
    } else if (input == "LB_Servo_Up") {
      xboxServos("up");
    } else if (input == "RB_Servo_Down") {
      xboxServos("down");
    } else {
      Serial.println("Error: invalid input");
    }
  }
}

/*
 * IMU SENSOR
 */
 
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
        setServoPos3();
        delay(400);
      }
      setMotors(-70, -70);
      while (delta < -tol && delta > -180) {}
      stopMotors();
    } else if (delta > tol && delta < 180) {
      // RH-x: go CCW
      if (servoPos != 3) {
        setServoPos3();
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
 
/** asks for inputs, runs the motors, stops motors */
void runMotors() {
  Serial.print("Input left motor speed: ");
  while(Serial.available() == 0) {}
  int speedL = Serial.parseInt();
  Serial.println(speedL, DEC);
  
  Serial.print("Input right motor speed: ");
  while(Serial.available() == 0) {}
  int speedR = Serial.parseInt();
  Serial.println(speedR, DEC);
  
  Serial.print("Input time [ms]: ");
  while(Serial.available() == 0) {}
  int runTime = Serial.parseInt();
  Serial.println(runTime, DEC);
  
  runStopMotors(speedL, speedR, runTime);
}

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
  if (upDown == "up" && servoPos > 1 && servoPos <= 5) {
    // go up if current position is 2 to 5
    servoPos--;
  } else if (upDown == "down" && servoPos < 5 && servoPos >= 1) {
    // go down if current position is 1 to 4
    servoPos++;
  } else {
    // exit function
    return;
  }

  // left servo is 'default' naming convention. Right servo is opposite
  if (servoPos == 1) {
    setServoPos1();
  } else if (servoPos == 2) {
    setServoPos2();
  } else if (servoPos == 3) {
    setServoPos3();
  } else if (servoPos == 4) {
    setServoPos4();
  } else if (servoPos == 5) {
    setServoPos5();
  } else {
    return;
  }
}

void setServoPos1() {
  servoL.writeMicroseconds(pulseUp);
  servoR.writeMicroseconds(pulseDown);
  servoPos = 1;
}

void setServoPos2() {
  servoL.writeMicroseconds(pulseUpMid);
  servoR.writeMicroseconds(pulseDownMid); 
  servoPos = 2; 
}

void setServoPos3() {
  servoL.writeMicroseconds(pulseCenter);
  servoR.writeMicroseconds(pulseCenter);
  servoPos = 3;
}

void setServoPos4() {
  servoL.writeMicroseconds(pulseDownMid);
  servoR.writeMicroseconds(pulseUpMid);
  servoPos = 4;
}

void setServoPos5() {
  servoL.writeMicroseconds(pulseDown);
  servoR.writeMicroseconds(pulseUp);
  servoPos = 5;
}


/**
 * IMAGE PROCESSING COMMANDS
 */
 
void goDown() {
  // set servos to pos 1 (up)
  setServoPos1();
  delay(100);
  // go forward for .5 seconds
  runStopMotors(80, -80, 500);
  // set servos back to initial position
  setServoPos3();
}

void goUp() {
  // set servos to pos 5 (down)
  setServoPos5();
  delay(100);
  // go forward for .5 seconds
  runStopMotors(80, -80, 500);
  // set servos back to initial position
  setServoPos3();
}


