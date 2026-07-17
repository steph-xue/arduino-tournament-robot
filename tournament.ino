#include <Servo.h>

// Define sensor input pins
const int analogInPin1 = A0;  // Left sensor
const int analogInPin2 = A1;  // Right sensor

// Define motor pins
const int motorAPin_A = 8;   // Motor A direction
const int motorAPin_B = 9;   // Motor A speed
const int motorBPin_A = 4;   // Motor B direction
const int motorBPin_B = 5;   // Motor B speed

// Ultrasonic Sensor Pins
const int trigPinFront = 10;
const int echoPinFront = 11;
const int trigPinLeftSide = 6;
const int echoPinLeftSide = 7;
const int trigPinRightSide = 3;
const int echoPinRightSide = 2;

// Thresholds for detection
const int whiteThreshold1 = 460;  // Sensor value indicating a white line
const int whiteThreshold2 = 350;  // Sensor value indicating a white line
const int whiteThreshold3 = 350;  // Sensor value indicating a white line
const int frontSetPoint = 20;  // Distance in cm before turning inside corner
const int sideSetPoint = 20;   // Ideal distance from the wall
const float K_p = 5.0;        // Proportional gain for PID correction
const float K_p_turn = 1;     // Proportional gain for wall-following adjustments

// Variables for distance sensor readings and filtering
float cmFront = 0;
float prevCmFront = 0;
float filteredCmFront = 0;

float cmRightSide = 0;
float prevCmRightSide = 0;
float filteredCmRightSide = 0;

float cmLeftSide = 0;
float prevCmLeftSide = 0;
float filteredCmLeftSide = 0;

bool lineFollowingMode = true; 
bool wallFollowingMode = false;  
bool objectFollowingMode = false;

// Global flag to indicate when the robot has detected an object and should stop permanently
bool objectDetected = false;

void setup() {
  Serial.begin(9600);
  pinMode(motorAPin_A, OUTPUT);
  pinMode(motorAPin_B, OUTPUT);
  pinMode(motorBPin_A, OUTPUT);
  pinMode(motorBPin_B, OUTPUT);
  pinMode(trigPinFront, OUTPUT);
  pinMode(echoPinFront, INPUT);
  pinMode(trigPinRightSide, OUTPUT);
  pinMode(echoPinRightSide, INPUT);
  pinMode(trigPinLeftSide, OUTPUT);
  pinMode(echoPinLeftSide, INPUT);
}

void loop() {

  if (objectDetected) {
    Serial.println("Object detected and stopped permanently.");
    stopRobot();
    while (true);
  }

  int sensorLeft = analogRead(analogInPin1);
  int sensorRight = analogRead(analogInPin2);
  cmFront = getDistance(trigPinFront, echoPinFront);
  cmRightSide = getDistance(trigPinRightSide, echoPinRightSide);
  cmLeftSide = getDistance(trigPinLeftSide, echoPinLeftSide);

  // Apply a simple running average filter (averaging with previous reading)
  filteredCmFront = (cmFront + prevCmFront) / 2;
  filteredCmRightSide = (cmRightSide + prevCmRightSide) / 2;
  filteredCmLeftSide = (cmLeftSide + prevCmLeftSide) / 2;

  Serial.print("Ls: "); 
  Serial.print(sensorLeft);
  Serial.print(" | Rs: "); 
  Serial.print(sensorRight);
  Serial.print(" | Fd: "); 
  Serial.print(filteredCmFront);
  Serial.print(" | Ld "); 
  Serial.print(filteredCmLeftSide);
  Serial.print(" | Rd: "); 
  Serial.println(filteredCmRightSide);

 // **Mode Switching Logic**
  if (lineFollowingMode) {
    // **Switch to wall following if a wall is detected**
    if ((filteredCmLeftSide < 20 || filteredCmRightSide < 20 || filteredCmFront < 20) && (filteredCmLeftSide != 0 && filteredCmRightSide != 0 && filteredCmFront != 0)) {
      Serial.println("Wall detected! Switching to Wall Following Mode...");
      moveForwardStartLine();
      wallFollowingMode = true;
      lineFollowingMode = false;
      objectFollowingMode = false;
    } else {
      followLine(sensorLeft, sensorRight);
    }
  } 
  else if (wallFollowingMode) {
    // **Switch to object following if white line is detected**
    if (sensorLeft > whiteThreshold2 && sensorRight > whiteThreshold2) {
      Serial.println("White line detected! Switching to Object Following Mode...");
      moveForwardStart1();
      moveCurveStart();
      moveForwardStart2();
      delay(300);
      objectFollowingMode = true;
      wallFollowingMode = false;
      lineFollowingMode = false;
    } else {
      followWall(filteredCmFront, filteredCmLeftSide, filteredCmRightSide);
    }
  } 
  else if (objectFollowingMode) {
    followObject(filteredCmFront, filteredCmLeftSide, filteredCmRightSide, sensorLeft, sensorRight);
  }


  // Save current sensor values for the next iteration (for filtering)
  prevCmFront = cmFront;
  prevCmRightSide = cmRightSide;
  prevCmLeftSide = cmLeftSide;
  
  delay(300);
}

void followLine(int sensorLeft, int sensorRight) {
  // Determine movement based on sensor readings
  if (sensorLeft > whiteThreshold1 && sensorRight > whiteThreshold1) {
    // Both sensors detect black: move forward
    Serial.println("Moving forward");
    driveForwardLine();
  } else if (sensorLeft > sensorRight + 10) {
    // Left sensor on white, right sensor on black: turn left 
    Serial.println("Turning left");
    turnLeftLine();
  } else if (sensorLeft + 10 < sensorRight) {
    // Right sensor on white, left sensor on black: turn right 
    Serial.println("Turning right");
    turnRightLine();
  } else {
    // Else: Search for line
    Serial.println("Searching for line");
    searchForLine(sensorLeft, sensorRight);
  }
}

void followWall(float cmFront, float cmLeftSide, float cmRightSide) {

  // **Wall Following (ePID-based wheel adjustments)**
    if ((cmLeftSide < 50 && cmFront > frontSetPoint) && cmFront < 800) { 
        Serial.println("Following wall...");
        moveForwardWall(255, 255);
    }
    // ** Corner Detection → Turn Right**
    else if ((cmFront < frontSetPoint || cmFront > 800)) {
      if (cmRightSide >= cmLeftSide) {
        stopRobot();
        Serial.println("Corner Detected! Turning Right...");
        forwardWall();
        backwardWall();
        turnRightWall();
      } else {
        stopRobot();
        Serial.println("Corner Detected! Turning Left...");
        backwardWall();
        turnLeftWall();
      }
    }
    else {
      // Last case if nothing in front or on the side
        Serial.println("Moving forward");
        moveForwardWall(255, 255);
    }
}

void followObject(float objectFrontDistance, float objectLeftSideDistance, float objectRightSideDistance, int sensorLeft, int sensorRight) {

  if (sensorLeft > whiteThreshold3 || sensorRight > whiteThreshold3) {
    Serial.println("White line detected! Preventing movement past the line.");
    backward();
    turnLeft();
    stopRobot();
    return;
  }

 // ** Object detected in front → STOP PERMANENTLY**
  if (objectFrontDistance > 0 && objectFrontDistance <= 5) {
    Serial.println("Object found. Stopping permanently...");
    stopRobot();
    objectDetected = true; // Set flag to prevent further movement
    return;
  }
  if (objectFrontDistance > 0 && objectFrontDistance <= 50) {
    Serial.println("Object detected in front! Moving towards it...");
    moveForwardObject(150, 150);
    return;
  } 

  // ** Object detected on the left side → Rotate until it is in front**
  if (objectLeftSideDistance > 0 && objectLeftSideDistance <= 50) {
    Serial.println("Object detected on the left side! Rotating left to face it...");
    turnLeft();
    delay(300);
    return;
  }

  // ** Object detected on the left side → Rotate until it is in front**
  if (objectRightSideDistance > 0 && objectRightSideDistance <= 50) {
    Serial.println("Object detected on the right side! Rotating right to face it...");
    turnRight();
    delay(300);
    return;
  }

  // **3No object detected → Begin 360° search sweep**
  moveForwardSearch();
  Serial.println("No object detected. Starting 360° sweep...");

  for (int i = 0; i < 13; i++) { // 18 steps of 20° each = 360°
    turnLeftSmall();
    delay(200);
    stopRobot();

    // Re-measure distances
    objectFrontDistance = getDistance(trigPinFront, echoPinFront);
    objectLeftSideDistance = getDistance(trigPinLeftSide, echoPinLeftSide);
    objectRightSideDistance = getDistance(trigPinRightSide, echoPinRightSide);

    Serial.print("Sweeping... ");
    Serial.print("| Front: ");
    Serial.print(objectFrontDistance);
    Serial.print(" cm | Left: ");
    Serial.println(objectLeftSideDistance);
    Serial.print(" cm | Right: ");
    Serial.println(objectRightSideDistance);

    // If white line detected, move back and turn left
    if (sensorLeft > whiteThreshold3 || sensorRight > whiteThreshold3) {
      Serial.println("White line detected! Preventing movement past the line.");
      backward();
      turnLeft();
      stopRobot();
      return;
    }
    // If an object is detected at any point, stop searching and move toward it
    if (objectFrontDistance > 0 && objectFrontDistance <= 50) {
      Serial.println("Object found during sweep! Moving towards it...");
      moveForwardObject(150, 150);
      return;
    }
    if (objectLeftSideDistance > 0 && objectLeftSideDistance <= 50) {
      Serial.println("Object detected on the left side during sweep. Aligning left...");
      turnLeftSearch();
      delay(300);
      return;
    }
    if (objectRightSideDistance > 0 && objectRightSideDistance <= 50) {
      Serial.println("Object detected on the right side during sweep. Aligning right...");
      turnRightSearch();
      delay(300);
      return;
    }
  }
  Serial.println("360° Sweep completed. No object found.");
  return;
}


float getDistance(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(10);
  digitalWrite(trig, HIGH);
  delayMicroseconds(50);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH, 60000);
  return (duration / 2.0) / 29.1;
}

// Function to drive forward
void driveForwardLine() {
  analogWrite(motorAPin_A, 0);
  analogWrite(motorAPin_B, 150);
  analogWrite(motorBPin_A, 150);
  analogWrite(motorBPin_B, 0);
  delay(200);
  stopRobot();
}

// Function to turn slightly 
void turnLeftLine() {
  analogWrite(motorAPin_A, 150);
  analogWrite(motorAPin_B, 150);
  analogWrite(motorBPin_A, 150);
  analogWrite(motorBPin_B, 0);
  delay(250);
  stopRobot();
}

// Function to turn slightly 
void turnRightLine() {
  analogWrite(motorAPin_A, 0);
  analogWrite(motorAPin_B, 150);
  analogWrite(motorBPin_A, 150);
  analogWrite(motorBPin_B, 150);
  delay(250);
  stopRobot();
}

// Function to move backwards
void moveBackLine() {
  analogWrite(motorAPin_A, 150);
  analogWrite(motorAPin_B, 0);
  analogWrite(motorBPin_A, 0);
  analogWrite(motorBPin_B, 150);
  delay(200);
}

// Function to search for line
void searchForLine(int sensorLeft, int sensorRight) {
  if (sensorLeft > sensorRight) {
    turnLeftLine();
    delay(100); // Small left turn
      if (sensorLeft > whiteThreshold1 || sensorRight > whiteThreshold1) {
          return; // Found the line
      }
  } else if (sensorLeft < sensorRight) {
    turnRightLine();
    delay(100);
        if (sensorLeft > whiteThreshold1 || sensorRight > whiteThreshold1) {
          return; // Found the line
        }         
  }
}

void moveForwardStartLine() {
    analogWrite(motorAPin_A, 0);
    analogWrite(motorAPin_B, 255);
    analogWrite(motorBPin_A, 255);
    analogWrite(motorBPin_B, 0);
    delay(1000);
    stopRobot();
}

/*

void searchForLine(int sensorLeft, int sensorRight) {

  if (sensorLeft > sensorRight) {
    turnLeftLine();
    delay(100); // Small left turn
      if (sensorLeft > whiteThreshold || sensorRight > whiteThreshold) {
          return; // Found the line
      }
  } else if (sensorLeft < sensorRight) {
    turnRightLine();
    delay(100);
        if (sensorLeft > whiteThreshold || sensorRight > whiteThreshold) {
          return; // Found the line
        }         
  }
}
    // If not found, try turning right
    for (int i = 0; i < 20; i++) { // Turn right more since we turned left first
        turnRightLine();
        delay(100);
        if (analogRead(sensorLeft) > whiteThreshold || analogRead(sensorRight) > whiteThreshold) {
            return; // Found the line
        }
    }

    // If still not found, move backward slightly and retry
    moveBackLine();
    delay(200);
    searchForLine(sensorLeft, sensorRight); // Retry
*/


// **Smooth Forward Motion with Differential Speed**
void moveForwardWall(int leftSpeed, int rightSpeed) {
    analogWrite(motorAPin_A, 0);
    analogWrite(motorAPin_B, leftSpeed);
    analogWrite(motorBPin_A, rightSpeed);
    analogWrite(motorBPin_B, 0);
    delay(400);
    stopRobot();
}

// **Turn Right (Outside Corner)**
void turnRightWall() {
    analogWrite(motorAPin_A, 0);
    analogWrite(motorAPin_B, 255);
    analogWrite(motorBPin_A, 255);
    analogWrite(motorBPin_B, 255);
    delay(200); // Adjust based on testing

}

// **Turn Left (Inside Corner)**
void turnLeftWall() {
    analogWrite(motorAPin_A, 255);
    analogWrite(motorAPin_B, 255);
    analogWrite(motorBPin_A, 255);
    analogWrite(motorBPin_B, 0);
    delay(500); // Adjust based on testing
    stopRobot();
}

void backwardWall() {
    analogWrite(motorAPin_A, 255);
    analogWrite(motorAPin_B, 0);
    analogWrite(motorBPin_A, 0);  
    analogWrite(motorBPin_B, 255); 
    delay(300); // Adjust based on testing
    stopRobot();
}

void forwardWall() {
    analogWrite(motorAPin_A, 0);
    analogWrite(motorAPin_B, 255);
    analogWrite(motorBPin_A, 255);  
    analogWrite(motorBPin_B, 0); 
    delay(300); // Adjust based on testing
    stopRobot();
}

void stopRobot() {
  analogWrite(motorAPin_A, 255);
  analogWrite(motorAPin_B, 255);
  analogWrite(motorBPin_A, 255);
  analogWrite(motorBPin_B, 255);
}

// **Smooth Forward Motion with Differential Speed**
void moveForwardStart1() {
    analogWrite(motorAPin_A, 0);
    analogWrite(motorAPin_B, 255);
    analogWrite(motorBPin_A, 255);
    analogWrite(motorBPin_B, 0);
    delay(1400);
    stopRobot();
}

// **Smooth Forward Motion with Differential Speed**
void moveCurveStart() {
    analogWrite(motorAPin_A, 255);
    analogWrite(motorAPin_B, 255);
    analogWrite(motorBPin_A, 255);
    analogWrite(motorBPin_B, 0);
    delay(100);
    stopRobot();
}

void moveForwardStart2() {
    analogWrite(motorAPin_A, 0);
    analogWrite(motorAPin_B, 255);
    analogWrite(motorBPin_A, 255);
    analogWrite(motorBPin_B, 0);
    delay(1500);
    stopRobot();
}

// **Smooth Forward Motion with Differential Speed**
void moveForwardSearch() {
    analogWrite(motorAPin_A, 0);
    analogWrite(motorAPin_B, 255);
    analogWrite(motorBPin_A, 255);
    analogWrite(motorBPin_B, 0);
    delay(500);
    stopRobot();
}

// **Smooth Forward Motion with Differential Speed**
void moveForwardObject(int leftSpeed, int rightSpeed) {
    analogWrite(motorAPin_A, 0);
    analogWrite(motorAPin_B, leftSpeed);
    analogWrite(motorBPin_A, rightSpeed);
    analogWrite(motorBPin_B, 0);
    delay(300);
    stopRobot();
}


// **Turn Left
void turnLeft() {
    analogWrite(motorAPin_A, 255);
    analogWrite(motorAPin_B, 255);
    analogWrite(motorBPin_A, 255);
    analogWrite(motorBPin_B, 0);
    delay(600); 
    stopRobot();
}

// **Turn Right
void turnRight() {
    analogWrite(motorAPin_A, 0);
    analogWrite(motorAPin_B, 255);
    analogWrite(motorBPin_A, 255);
    analogWrite(motorBPin_B, 255);
    delay(600); 
    stopRobot();
}

void turnLeftSearch() {
    analogWrite(motorAPin_A, 255);
    analogWrite(motorAPin_B, 255);
    analogWrite(motorBPin_A, 255);
    analogWrite(motorBPin_B, 0);
    delay(500); 
    stopRobot();
}

void turnRightSearch() {
    analogWrite(motorAPin_A, 0);
    analogWrite(motorAPin_B, 255);
    analogWrite(motorBPin_A, 255);
    analogWrite(motorBPin_B, 255);
    delay(500); 
    stopRobot();
}

void turnLeftSmall() {
  analogWrite(motorAPin_A, 255);
  analogWrite(motorAPin_B, 255);
  analogWrite(motorBPin_A, 255);
  analogWrite(motorBPin_B, 0);
  delay(200); 
  stopRobot();
}

void backward() {
    analogWrite(motorAPin_A, 255);
    analogWrite(motorAPin_B, 0);
    analogWrite(motorBPin_A, 0);  
    analogWrite(motorBPin_B, 255); 
    delay(300); 
    stopRobot();
}

