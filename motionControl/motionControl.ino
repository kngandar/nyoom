#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// pin numbers
int brushL_pin = 7;//8;
int brushR_pin = 8;//7;
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
  servoL.writeMicroseconds(pulseCenter);
  servoR.writeMicroseconds(pulseCenter);
  delay(500);
  
  Serial.begin(9600);
  
  // Look for IMU
  if(!bno.begin()) {
    Serial.print("BNO055 not found.");
    while(!bno.begin()) {}
  }
  delay(1000);

  bno.setExtCrystalUse(true);
  //calibrate();
  Serial.println("Status: ready to go!!");
}


void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readString();
    
    if (input == "runMotors") {
      runMotors();
    } else if (input == "rotateServos") {
      rotateServos();
    } else if (input == "100") {
      delay(250);
      foundGreen();
    } else if (input == "200") {
      delay(250);
      foundRed();
    } else if (input == "300") {
      delay(250);
      foundYellow();
    } else if (input == "goForth") {
      stayOnCourse();
    } else if (input == "turnCW") {
      turn90CW();
    } else if (input == "turnCCW") {
      turn90CCW();
    /** XBOX control commands */
    } else if (input == "left") {
      setMotors(-70, -70);
    } else if (input == "right") {
      setMotors(80,80);
    } else if (input == "fwd") {
      setMotors(80, -80);
    } else if (input == "bwd") {
      setMotors(-80, 80);
    } else if (input == "stop") {
      stopMotors();
    } else if (input == "servoUp") {
      xboxServos("up");
    } else if (input == "servoDown") {
      xboxServos("down");
    } else if (input == "faceFront") {
      // figure out when we can get the "front" measurement
    } else if (input == "panic") {
      stopMotors();
      servoL.writeMicroseconds(pulseCenter);
      servoR.writeMicroseconds(pulseCenter);
      delay(500); // dont run motors until servos are mid
      runStopMotors(-80, 80, 1000);
    } else {
      Serial.println("Error: invalid input");
    }
  }
}


/*
 * COLOUR SENSOR
 */
void foundGreen() {
  // Set servos to middle position
  servoL.writeMicroseconds(pulseCenter);
  servoR.writeMicroseconds(pulseCenter);
  runStopMotors(80, 80, 2000);
  Serial.println("Rotated CW");
}

void foundRed() {
  // Set servos to middle position
  servoL.writeMicroseconds(pulseCenter);
  servoR.writeMicroseconds(pulseCenter);
  runStopMotors(-70, -70, 2000);
  Serial.println("Rotated CCW");
}

void foundYellow() {
  // Set servos to down position
  servoL.writeMicroseconds(pulseDown);
  servoR.writeMicroseconds(pulseUp);
  runStopMotors(-100, 100, 3000);
  Serial.println("Went up for 3 seconds");
}


/*
 * IMU SENSOR
 */

/** stays within +- tol degrees of initial angle. Enter anything in console to exit */
void stayOnCourse() {
  sensors_event_t initA;
  sensors_event_t curr;
  bno.getEvent(&initA);

  int angle = initA.orientation.x;
  int tol = 15;
  int delta = 0;

  Serial.print("Initial angle = ");
  Serial.println(angle);

  while(Serial.available() == 0) {
    bno.getEvent(&curr);
    delta = checkDelta(curr.orientation.x - angle);

    if (delta < -tol && delta > -180) { 
      // LH-x: go CW
      Serial.println("going CW");
      servoL.writeMicroseconds(pulseCenter);
      servoR.writeMicroseconds(pulseCenter);
      runStopMotors(-70, -70, 1000);
      delay(400);
    } else if (delta > tol && delta < 180) {
      // RH-x: go CCW
      Serial.println("going CCW");
      servoL.writeMicroseconds(pulseCenter);
      servoR.writeMicroseconds(pulseCenter);
      runStopMotors(80, 80, 1000);
      delay(400);
    } else {
      Serial.println("staying still");
      delay(400);
    }
  }
  Serial.readString();
  Serial.println("done.");
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

/** returns system calibration status as an int */
int getCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  return system;
}

/** force user to calibrate the sensors */
void calibrate() {
  Serial.println("Calibrate the sensor!");
  int isGood = 0;
  while (isGood < 3) {
    /* Get a new sensor event */
    sensors_event_t event;
    bno.getEvent(&event);

    /* Optional: Display calibration status */
    displayCalStatus();

    /* New line for the next sample */
    Serial.println("");

    /* Wait the specified delay before requesting nex data */
    delay(BNO055_SAMPLERATE_DELAY_MS);

    isGood = getCalStatus();
  }
  Serial.println("calibrated?");
  displayCalStatus();
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

/** asks for inputs, rotates servos. Options: "up", "mid", "down" */
void rotateServos() {
  Serial.print("Input servo position (up, mid, down): ");
  while(Serial.available() == 0) {}
  String pos = Serial.readString();
  Serial.println(pos);

  setServos(pos);
}

/** sets both servos to the input string (up, mid, down) */
void setServos(String pos) {
  int posNew = servoPosition(pos);

  if (posNew > 0) {
    servoL.writeMicroseconds(posNew);
    servoR.writeMicroseconds(posNew);
    Serial.println("Status: servos moved");
  } else {
    Serial.println("Error: servos invalid input");
  }
}

/** returns servo position constant for an input string, or 0 if invalid input */
int servoPosition(String input) {
  if (input == "up") {
    return pulseUp;
  } else if (input == "mid") {
    return pulseCenter;
  } else if (input == "down") {
    return pulseDown;
  } else {
    return 0;
  }
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

  int pulse = getServoPulse(servoPos);
  if (pulse > 0) {
    servoL.writeMicroseconds(pulse);
    servoR.writeMicroseconds(pulse);
  }
}

/** returns the pulse length for the input position */
int getServoPulse(int pos) {
  if (pos == 1) {
    return pulseUp;
  } else if (pos == 2) {
    return pulseUpMid;
  } else if (pos == 3) {
    return pulseCenter;
  } else if (pos == 4) {
    return pulseDownMid;
  } else if (pos == 5) {
    return pulseDown;
  } else {
    return 0;
  }
}


/** test to try turning 90 degrees clockwise (right) */
void turn90CW() {
  runStopMotors(80, 80, 1000);
  runStopMotors(-70,-70,100);
}

/** turn CCW (left) */
void turn90CCW() {
  runStopMotors(-70, -70, 1000);
  runStopMotors(80,80,100);
}


