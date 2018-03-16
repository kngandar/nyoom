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
const int pulseCenter = 1553;
const int pulseDown = 2520;

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
  if(Serial.available() > 0) {
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
    } else if (input == "attemptCourse") {
      attemptCourse();
    } else if (input == "turnCW") {
      turn90CW();
    } else if (input == "turnCCW") {
      turn90CCW();
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
  setMotors(80, 80, 2000);
  Serial.println("Rotated CW");
}

void foundRed() {
  // Set servos to middle position
  servoL.writeMicroseconds(pulseCenter);
  servoR.writeMicroseconds(pulseCenter);
  setMotors(-70, -70, 2000);
  Serial.println("Rotated CCW");
}

void foundYellow() {
  // Set servos to down position
  servoL.writeMicroseconds(pulseDown);
  servoR.writeMicroseconds(pulseUp);
  setMotors(-100, 100, 3000);
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
      setMotors(-70, -70, 1000);
      delay(400);
    } else if (delta > tol && delta < 180) {
      // RH-x: go CCW
      Serial.println("going CCW");
      servoL.writeMicroseconds(pulseCenter);
      servoR.writeMicroseconds(pulseCenter);
      setMotors(80, 80, 1000);
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
  
  setMotors(speedL, speedR, runTime);
}

/** sets and stops the motors */
void setMotors(int left, int right, int runTime) {
  brushL.writeMicroseconds(motorValue(left));
  brushR.writeMicroseconds(motorValue(right));
  Serial.println("Status: running motors");
  delay(runTime);
  stopMotors();
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
  delay(500);
}

/** asks for inputs, rotates servos. Options: "up", "mid", "down" */
void rotateServos() {
  Serial.print("Input left servo position (up, mid, down): ");
  while(Serial.available() == 0) {}
  String left = Serial.readString();
  Serial.println(left);
  
  Serial.print("Input right servo position (up, mid, down): ");
  while(Serial.available() == 0) {}
  String right = Serial.readString();
  Serial.println(right);
  
  setServos(left, right);
}

/** sets servo positions to inputs: "up", "mid", "down" */
// NOTE: doesn't account for differing up/down stuff (?)
void setServos(String left, String right) {
  int posL = servoPosition(left);
  int posR = servoPosition(right);

  if (posL > 0) {
    servoL.writeMicroseconds(posL);
    Serial.println("Status: left servo moved");
  } else {
    Serial.println("Error: left servo invalid input");
  }
  
  if (posR > 0) {
    servoR.writeMicroseconds(posR);
    Serial.println("Status: right servo moved");
  } else {
    Serial.println("Error: right servo invalid input");
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

/** test to try turning 90 degrees clockwise */
void turn90CW() {
  setMotors(80, 80, 1000);
  setMotors(-70,-70,100);
}

void turn90CCW() {
  setMotors(-70, -70, 1000);
  setMotors(80,80,100);
}

/** test to try straight, left, straight, right, straight */
void attemptCourse() {
  setMotors(80, -70, 2000);
  turn90CCW();
  setMotors(80, -70, 1500);
  turn90CW();
  setMotors(80, -70, 2000);
  Serial.println("Status: attempted autonomous stuff");
}


