#include <PID_v1.h> 
#include <Motor.h> // Include a motor driver library

// Motor driver pins (adjust according to your motor driver library)
#define MOTOR_L_PIN1 9   
#define MOTOR_L_PIN2 10  
#define MOTOR_R_PIN1 11  
#define MOTOR_R_PIN2 12  

// Sensor pins
#define L 3
#define M 5
#define R 7

// Motor speed constants
#define FULL_SPEED 255 
#define BASE_SPEED 150 

// PID parameters
#define KP 10.0
#define KI 0.1
#define KD 1.0

// PID variables
double Setpoint = 0;
double Input, Output; 
PID myPID(&Input, &Output, &Setpoint, KP, KI, KD, DIRECT); 

// Create motor objects using the Motor library
Motor leftMotor(MOTOR_L_PIN1, MOTOR_L_PIN2); // Replace with your motor driver pin configuration
Motor rightMotor(MOTOR_R_PIN1, MOTOR_R_PIN2);

void setup() {
  // Initialize sensor pins as inputs with pull-up resistors enabled
  pinMode(L, INPUT); digitalWrite(L, HIGH);
  pinMode(M, INPUT); digitalWrite(M, HIGH);
  pinMode(R, INPUT); digitalWrite(R, HIGH);

  // Initialize PID controller
  myPID.SetMode(AUTOMATIC); 
  myPID.SetOutputLimits(-100, 100); 
}

void loop() {
  // 1. Read Sensors and Estimate Line Position
  Input = readLinePosition(); 

  // 2. Calculate PID Output
  myPID.Compute(); 

  // 3. Adjust Motor Speeds based on PID Output
  int leftSpeed = BASE_SPEED - Output; 
  int rightSpeed = BASE_SPEED + Output; 

  // Ensure speeds are within the valid range (0 to 255)
  leftSpeed = constrain(leftSpeed, 0, FULL_SPEED);
  rightSpeed = constrain(rightSpeed, 0, FULL_SPEED);

  // Set motor speeds using the motor library
  leftMotor.speed(leftSpeed);  
  rightMotor.speed(rightSpeed); 
}

// --- Helper Functions ---

// Reads sensor values and estimates the line position
int readLinePosition() {
  int sensorValues[] = {
    digitalRead(L), 
    digitalRead(M), 
    digitalRead(R)
  };

  // Weighted average for line position estimation (adjust weights as needed)
  int weights[] = {-1, 0, 1}; 
  int weightedSum = 0; 
  int totalWeight = 0; 

  for (int i = 0; i < 3; i++) {
    if (sensorValues[i] == 0) { 
      weightedSum += weights[i]; 
      totalWeight++; 
    }
  }

  if (totalWeight > 0) {
    return weightedSum / totalWeight; 
  } else {
    return 0; 
  }
}