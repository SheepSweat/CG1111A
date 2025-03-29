#include <MeMCore.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/*
 CONSTANTS with MeDCMotor and MeLine initialisation.
*/
MeDCMotor leftMotor(M1); //port M1
MeDCMotor rightMotor(M2); //port M2

// Our right motor is more powerful than the left, adding a negative bias to the right side.
const float rightBias = 0.98;

// Reduces the speed of rotation to prevent slipping.
const float rotateBias = 0.80;
uint8_t motorSpeed = 255;

// Ultrasonic values.
#define ultrasonic 10 // PIN value.
#define TIMEOUT 2000 //max microseconds to wait acc to max dist of wall
#define SPEED_OF_SOUND 340

// Wall detection values.
#define DIST_UPPER_BOUND 155 //160 was pretty good also
#define DIST_LOWER_BOUND 75
#define NO_WALL_BOUND 165

// Line Follower.
MeLineFollower lineFinder(PORT_1);

// RGB Constants.
#define LDR 0
#define LDRWait 10
float whiteArray[] = {941, 933, 878};
float blackArray[] = {584, 591, 490};
float greyDiff[] = {357, 342, 388};

#define RED 0
#define GREEN 1
#define BLUE 2

// RED.
#define RED_R 182
#define RED_G 57
#define RED_B 40

// ORANGE.
#define ORANGE_R 176
#define ORANGE_G 115
#define ORANGE_B 56

// GREEN.
#define GREEN_R 93
#define GREEN_G 158
#define GREEN_B 111

// PINK.
#define PINK_R 195
#define PINK_G 174
#define PINK_B 164

// LIGHT BLUE.
#define LBLUE_R 95
#define LBLUE_G 152
#define LBLUE_B 167

// WHITE.
#define WHITE_R 190
#define WHITE_G 199
#define WHITE_B 193

// For celebratory tune.
MeBuzzer buzzer;
#define F3  174
#define G3  196
#define A3n  220
#define C4  261
#define D4  293
#define E4  330
#define F4  349
#define G4  392
#define A4  440
#define B4  493
#define C5  523
#define D5  587
#define REST 0

// IR Values.
#define IR 1
#define ON 1
#define OFF 0

// PID Values.
float integral_sum = 0;

// PD constants.
const float Kp = 2.0;
const float Kd = 0.4;
const float Ki = 0.05;
 
// Global error variables.
float error = 0; 
float previousError = 0;

// Motor speed base constant.
const int baseSpeed = 255;

//No Wall detected base constants.
const long leftNoWallBound = 130;
const long rightNoWallBound = 75;
const long leftBaseDist = 117;
const long rightBaseDist = 60;

/*
 HELPER FUNCTIONS.
*/

// Code for playing celebratory tune.
void celebrate() {
  // Define the melody notes in frequency (in Hz).
  int melody[] = {
    F4, C4, F4, A4, F4, A4, D5, C5, C5, G4, E4, C4,
    C4, F4, G4, A4, A4, A4, G4, F4, E4, F4, F4, F4, E4, D4, C4, D4, C4, REST, F4, G4, F4, E4, REST,
    C4, E4, F4, G4, E4, F4, G4, A4, D5, C5, REST,
    A4, A4, C4, REST, C4, E4, G4, F4, F4, C4, A3n, F3
  };

  // Defines the duration of each note. (4 = quarter note, 8 = eighth note, etc.)
  int noteDurations[] = {
    4, 6, 16, 8, 8, 8, 8, 8, 8, 8, 8, 8,
    8, 8, 8, 4, 4, 8, 8, 8, 8, 4, 4, 8, 8, 8, 8, 4, 4, 8, 8, 8, 8, 2, 8,
    8, 8, 8, 4, 4, 4, 4, 6, 16, 2, 4,
    6, 16, 4, 8, 8, 8, 8, 4, 4, 4, 4, 2
  };

  // Base duration for a quarter note.
  int tempo = 1200;

  // Play each note in the melody array
  for (int i = 0; i < sizeof(melody) / sizeof(int); i++) {
    int note = melody[i];
    int duration = tempo / noteDurations[i];

    if (note == 0) {
      delay(duration); // Rest for the note duration if it's a pause.
    } else {
      buzzer.tone(note, duration); // Play the note.
    }

    delay(duration * 0.3); // Short delay between notes for spacing.
  }
}

// Code for moving forward for some short interval.
void moveForward(int duration) {
  leftMotor.run(-motorSpeed);
  rightMotor.run(motorSpeed * rightBias);
  delay(duration);
  STOP();
}

// Code for moving along wall.
void setMotorSpeeds(int leftMotorSpeed, int rightMotorSpeed, int duration) {
  leftMotor.run(-leftMotorSpeed);
  rightMotor.run(rightMotorSpeed * rightBias);
  delay(duration);
}

// Code for turning right 90 deg.
void rotateRight() {
  leftMotor.run(-motorSpeed);
  rightMotor.run(-motorSpeed);
  delay(335);
  STOP();
}

// Code for turning right 90 deg for second right.
void rotateRight2() {
  leftMotor.run(-motorSpeed);
  rightMotor.run(-motorSpeed);
  delay(355);
  STOP();
}

// Code for turning left 90 deg.
void rotateLeft() {
  leftMotor.run(motorSpeed);
  rightMotor.run(motorSpeed);
  delay(355);
  STOP();
}

// Code for a 180 deg turn.
void uTurn() {
  leftMotor.run(motorSpeed);
  rightMotor.run(motorSpeed);
  delay(700);
  STOP();
}

// Code for a 90 deg left, forward and 90 deg left.
void doubleLeftTurn() {
  rotateLeft();
  delay(100);
  moveForward(860);
  delay(100);
  rotateLeft();
}

// Code for a 90 deg right, forward and 90 deg right.
void doubleRightTurn() {
  rotateRight2();
  delay(100);
  moveForward(820);
  delay(100);
  rotateRight2();
}

// Sets both motors to 0.
void STOP() {
  leftMotor.run(0);
  rightMotor.run(0);
}

// Takes num readings from the LDR (pin A0) of the color sensor. Ignores the first 20 values.
int getAverageReadingFromLDR(int num, int COLOR) {
  int total = 0;
  for (int i = 0; i < num; i += 1) {
    int curr_reading = (analogRead(LDR) - blackArray[COLOR])/(greyDiff[COLOR])*255;
    if (i > 20) {
      total += curr_reading;
    }
    delay(LDRWait);
  }
  return (total / (num - 20));
}

// Sets the pins and reads from the LDR. Note: Red is 0, Green is 1 & Blue is 2.
int getRGBReading(int rgb_val) {
  if (rgb_val == RED) {
    analogWrite(A2, 255);
    analogWrite(A3, 255);

  } else if (rgb_val == GREEN) {
    analogWrite(A2, 0);
    analogWrite(A3, 255);

  } else if (rgb_val == BLUE) {
    analogWrite(A2, 255);
    analogWrite(A3, 0);

  }
  return getAverageReadingFromLDR(40, rgb_val);
}

// Reads the color index and turns the robot respectively.
void turnEUC(int colorIndex) {
  switch (colorIndex) {
    case 0:
        Serial.println("DETECTED RED!");
        rotateLeft();
        break;
    case 1:
        Serial.println("DETECTED ORANGE!");
        uTurn();
        break;
    case 2:
        Serial.println("DETECTED GREEN!");
        rotateRight();
        break;
    case 3:
        Serial.println("DETECTED PINK!");
        doubleLeftTurn();
        break;
    case 4:
        Serial.println("DETECTED LIGHT BLUE!");
        doubleRightTurn();
        break;
    case 5:
        Serial.println("DETECTED WHITE!");
        celebrate();
        break;
    default:
        Serial.println("NO COLOR!");
        break;
  }
}

/* -- EUCLIDEAN DISTANCE! --START */

// Function to calculate Euclidean distance between two colors.
float calculateDistance(int inputColor[3], int targetColor[3]) {
    float distance = 0;
    for (int i = 0; i < 3; i++) {
        distance += pow(inputColor[i] - targetColor[i], 2);
    }
    return sqrt(distance);
}

// Recursive function to find the closest color with threshold filtering and Euclidean fallback.
int readColorEUC(int rgbReading[3], int errorshift) {
    int colors[6][3] = {
        {RED_R, RED_G, RED_B},  // Red
        {ORANGE_R, ORANGE_G, ORANGE_B},  // Orange
        {GREEN_R, GREEN_G, GREEN_B},  // Green
        {PINK_R, PINK_G, PINK_B},  // Pink
        {LBLUE_R, LBLUE_G, LBLUE_B},  // Light Blue
        {WHITE_R, WHITE_G, WHITE_B},  // White
    };

    int remainingColors[6];
    int remainingCount = 0;

    // Step 1: Threshold filtering to find colors within errorshift tolerance.
    for (int i = 0; i < 6; i++) {
        if (abs(colors[i][0] - rgbReading[0]) <= errorshift &&
            abs(colors[i][1] - rgbReading[1]) <= errorshift &&
            abs(colors[i][2] - rgbReading[2]) <= errorshift) {
            remainingColors[remainingCount++] = i;
        }
    }

    // Step 2: If only one color remains within threshold, return it as the closest color.
    if (remainingCount == 1) {
        return remainingColors[0];
    }

    // Step 3: If errorshift is 5 and still multiple colors remain, use pure Euclidean distance.
    if (errorshift <= 5) {
        int closestIndex = -1;
        float minDistance = -1;

        for (int i = 0; i < 6; i++) {
            float distance = calculateDistance(rgbReading, colors[i]);

            if (minDistance == -1 || distance < minDistance) {
                minDistance = distance;
                closestIndex = i;
            }
        }
        return closestIndex;
    }

    // Step 4: Recursive call with reduced errorshift if more than one color remains.
    return readColorEUC(rgbReading, errorshift - 1);
}
/* -- EUCLIDEAN DISTANCE! --END */


/* --PID FUNCTIONS--START */

void setIREmitter(int input) {
  if (input == ON) {
    analogWrite(A2, 0);
    analogWrite(A3, 0);
  } else {
    analogWrite(A2, 255);
    analogWrite(A3, 255);
  }
}

// Reads ultrasonic sensor data, returns distance in millimetres.
long leftSideDist() {
  digitalWrite(ultrasonic, HIGH);
  //DELAY EDIT
  delayMicroseconds(10);
  digitalWrite(ultrasonic, LOW);

  long duration = pulseIn(ultrasonic, HIGH);
  long dist = (duration * SPEED_OF_SOUND / 2.0) / 1000;
  
  return dist;
}

// Reads IR sensor data, return distance in millimetres.
long rightSideDist() {
  const long sampleSize = 125;
  float totalReflectedIR = 0;

  for (long i = 0; i < sampleSize; i += 1) {
    setIREmitter(OFF);
    long ambientReading = analogRead(IR);
    setIREmitter(ON);
    long emitterReading = analogRead(IR);

    totalReflectedIR = totalReflectedIR + (emitterReading - ambientReading);
  }

  float avgReflectedIR = totalReflectedIR / sampleSize;

  float arir = avgReflectedIR;
  long actualDistance = 1044.7 * pow(arir, -0.707);

  return actualDistance;
}

void moveAlongWall() {
  // Variables for storing sensor readings and errors.
  float leftDistance, rightDistance;
  float proportional, derivative, integral;
  float controlSignal;

  // Read the distance sensors.
  leftDistance = leftSideDist();
  rightDistance = rightSideDist();

  bool leftNoWall = (leftDistance > leftNoWallBound);
  bool rightNoWall = (rightDistance > rightNoWallBound);

  if (leftNoWall) {
    leftDistance = leftBaseDist;
  }
  
  if (rightNoWall) {
    rightDistance = rightBaseDist;
  }

  // Calculate error (difference between left and right distances).
  const float centreBias = leftBaseDist - rightBaseDist;
  error = leftDistance - rightDistance - centreBias;

  // Proportional term.
  proportional = Kp * error;

  // Derivative term.
  derivative = Kd * (error - previousError);

  //Integral term.
  integral_sum = integral_sum + error;
  integral = Ki * integral_sum;

  // PD control signal.
  controlSignal = proportional + derivative + integral;

  // Adjust motor speeds based on control signal.
  int leftMotorSpeed = baseSpeed - controlSignal;
  int rightMotorSpeed = baseSpeed + controlSignal;  

  // Constrain speeds to ensure they remain within a safe range (255).
  leftMotorSpeed = constrain(leftMotorSpeed, 0, baseSpeed);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, baseSpeed);

  // Set motor speeds.
  setMotorSpeeds(leftMotorSpeed, rightMotorSpeed, 2);

  // Store current error as previous error for next iteration.
  previousError = error;
  delay(5);
}
/* --PID FUNCTIONS--END */

void setup() {
  delay(2000);
  Serial.begin(9600);
}

void loop() {
  int sensorState = lineFinder.readSensors();
  
  bool blackLineDetected = ((sensorState == S1_IN_S2_IN) || (sensorState == S1_IN_S2_OUT) ||  (sensorState == S1_OUT_S2_IN));
  if (blackLineDetected) { 
      integral_sum = 0;     
      STOP();
      int rgbReading[3] = {0, 0, 0};
      rgbReading[0] = getRGBReading(RED);
      rgbReading[1] = getRGBReading(GREEN);
      rgbReading[2] = getRGBReading(BLUE);
      
      Serial.println("RGB SEEN!");
      Serial.println(rgbReading[0]);
      Serial.println(rgbReading[1]);
      Serial.println(rgbReading[2]);

      int colorIndex = readColorEUC(rgbReading, 20);
      turnEUC(colorIndex);
  } else {
    moveAlongWall();
  }
}