#include <PID_v1.h>

// Motor driver pins
#define IA1  9
#define IB1  10
#define IA2  11
#define IB2  12

// Sensor pins
#define L2   3
#define L1   4
#define MM   5
#define R1   6
#define R2   7

// Motor speed constants
#define FULL_SPEED 255
#define BASE_SPEED 150 // Reduced speed for better control

// PID parameters (you'll need to tune these)
#define KP 10.0 
#define KI 0.1  
#define KD 1.0  

// PID variables
double Setpoint = 0;  // Desired error (line centered)
double Input, Output;
PID myPID(&Input, &Output, &Setpoint, KP, KI, KD, DIRECT);

void setup() {
  // Initialize motor pins
  pinMode(IA1, OUTPUT);
  pinMode(IB1, OUTPUT);
  pinMode(IA2, OUTPUT);
  pinMode(IB2, OUTPUT);

  // Initialize sensor pins with pull-up resistors
  pinMode(L2, INPUT); digitalWrite(L2, HIGH);
  pinMode(L1, INPUT); digitalWrite(L1, HIGH);
  pinMode(MM, INPUT); digitalWrite(MM, HIGH);
  pinMode(R1, INPUT); digitalWrite(R1, HIGH);
  pinMode(R2, INPUT); digitalWrite(R2, HIGH);

  // Initialize PID controller
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-100, 100); // Limit PID output for motor control
}

void loop() {
  // 1. Read Sensors and Estimate Line Position
  Input = readLinePosition(); 

  // 2. Calculate PID Output
  myPID.Compute();

  // 3. Adjust Motor Speeds based on PID Output
  int leftSpeed = BASE_SPEED - Output;
  int rightSpeed = BASE_SPEED + Output;

  // Ensure speeds are within the valid range
  leftSpeed = constrain(leftSpeed, 0, FULL_SPEED);
  rightSpeed = constrain(rightSpeed, 0, FULL_SPEED);

  MOVE_L(leftSpeed);
  MOVE_R(rightSpeed);
}

// --- Helper Functions ---

// Reads sensor values and estimates the line position
int readLinePosition() {
  int sensorValues[] = {
    digitalRead(L2),
    digitalRead(L1),
    digitalRead(MM),
    digitalRead(R1),
    digitalRead(R2)
  };

  // Weighted average for line position estimation (adjust weights as needed)
  int weights[] = {-2, -1, 0, 1, 2}; 
  int weightedSum = 0;
  int totalWeight = 0;

  for (int i = 0; i < 5; i++) {
    if (sensorValues[i] == 0) { // Sensor sees the line
      weightedSum += weights[i];
      totalWeight++;
    }
  }

  if (totalWeight > 0) {
    return weightedSum / totalWeight;
  } else {
    return 0; // No line detected
  }
}

// Controls the left motor
void MOVE_L(unsigned char P) {
  analogWrite(IA2, 255 - P);
  digitalWrite(IB2, HIGH);
}

// Controls the right motor
void MOVE_R(unsigned char P) {
  analogWrite(IA1, 255 - P);
  digitalWrite(IB1, HIGH);
}