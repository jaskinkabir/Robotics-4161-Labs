#include "SimpleRSLK.h"

#define LEFT_MOTOR_SPEED_RAW    50    // Raw Speed 
#define RIGHT_MOTOR_SPEED_RAW   50    // Raw Speed 
#define RIGHT_MOTOR_SPEED       20    // Speed Percentage
#define LEFT_MOTOR_SPEED        20    //Speed Percentage
#define LEFT_TURN_SPEED     12    // Speed percentage
#define RIGHT_TURN_SPEED    12    // Speed percentage

// Value for turning directions (do not change)
#define CCW                  2     // rotate robot counter clockwise
#define CW                   1     // rotate robot clockwise

// RSLK specified mechanics (do not change)
#define wheelDiameter       6.999      // in centimeters
#define cntPerRevolution    360        // Number of encoder (rising) pulses every time the 

#define BASE_R         7 // Radius of the wheel base on the RSLK robot   
#define BASE_D         14
#define CIRCLE_R_IN           18
#define CIRCLE_R_CM           CIRCLE_R_IN*2.54
#define CIRCLE_D_CM           CIRCLE_R_CM * 2
#define LEFT_TRAVEL_DISTANCE  (CIRCLE_R_CM - BASE_R)*2*PI*0.75
#define RIGHT_TRAVEL_DISTANCE (CIRCLE_R_CM + BASE_R)*2*PI*0.75
#define OUTSIDE_WHEEL_RATIO   (CIRCLE_D_CM + BASE_D)/(CIRCLE_D_CM - BASE_D)

void setup() {
  Serial.begin(9600);         // Start serial com at 9600 baud rate
  setupRSLK();                // Initialize RSLK functions and classes
  setupWaitBtn(LP_LEFT_BTN);  // Setup left button on Launchpad
  setupLed(RED_LED);          // Red led in rgb led
  setupLed(GREEN_LED);        // Greem led in rgb led                       
}

void loop() {
    /* Amount of encoder pulses needed to achieve distance */
   String btnMsg = "Expected Total Pulse Count: ";
  // put your main code here, to run repeatedly: 
  waitBtnPressed(LP_LEFT_BTN,btnMsg,RED_LED);
  //rotate(CCW, 90);
  //driveCircle(LEFT_MOTOR_SPEED, RIGHT_MOTOR_SPEED, LEFT_TRAVEL_DISTANCE, RIGHT_TRAVEL_DISTANCE);
  //forward(500);

    forward(45.72);
    rotate(CCW, 45);
    forward(45.72);
    rotate(CCW, 90);
    driveCircle(LEFT_MOTOR_SPEED_RAW, RIGHT_MOTOR_SPEED_RAW, LEFT_TRAVEL_DISTANCE, RIGHT_TRAVEL_DISTANCE);
    rotate(CCW, 90);
    forward(45.72);
    rotate(CCW, 45);
    forward(45.72);
    rotate(CCW, 180);
}

void startProgram(){
  /* Setup message to print to serial montor */
  String btnMsg = "Push left button on Launchpad to start lab program.\n";
  waitBtnPressed(LP_LEFT_BTN,btnMsg,RED_LED); // Function to setup button, msg, LED
  /* Using an LED as a DELAY */
  blinkLED(1000);                             // Cause LED to blink for a period of one second
  blinkLED(1000);                             // Cause LED to blink for a period of one second
}

int blinkLED(int period){
  int pause = period/2;           // Determine on/off time
  digitalWrite(GREEN_LED, HIGH);  // Turn LED on
  delay(pause);                   // Time the LED is on
  digitalWrite(GREEN_LED, LOW);   // Turn LED off
  delay(pause);                   // Time LED is off
}

void rotate(int rotate_dir, int rotate_deg) {
  //Put your code here that will rely on motor encoders for accurately rotating in place
  uint32_t robot_base_diam = 14.0;
  float distance = (robot_base_diam * PI) * (rotate_deg / 360.0);
  uint32_t targetCount = countForDistance(wheelDiameter, cntPerRevolution, distance);
  resetLeftEncoderCnt();
  resetRightEncoderCnt();

  int leftCount = 0;
  int rightCount = 0;

  switch (rotate_dir) {
    case CW:
      setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
      setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
      
    case CCW:  
      setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
      setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);

  }
  enableMotor(BOTH_MOTORS);
  setMotorSpeed(LEFT_MOTOR, LEFT_TURN_SPEED);
  setMotorSpeed(RIGHT_MOTOR, RIGHT_TURN_SPEED);
  while (leftCount < targetCount || rightCount < targetCount) {
    leftCount = getEncoderLeftCnt();
    rightCount = getEncoderRightCnt();
  }
  disableMotor(BOTH_MOTORS);
  delay(2000);

}

int pid(int setpoint, int input, double kp, double ki, double kd) {
  static unsigned long lastTime = millis();
  static int lastErrorCalculated = input - setpoint;
  static double cumulativeError = 0;

  unsigned long currentTime = millis();
  double deltaTime = (double)(currentTime - lastTime);
  lastTime = currentTime;

  int error = input - setpoint;

  float prop = kp * error;
  cumulativeError += error * deltaTime;

  float integral = cumulativeError * ki;

  float derivative = ((error - lastErrorCalculated) / deltaTime) * kd;

  lastErrorCalculated = error;

  int adjustment = prop + integral + derivative;
  

  //Serial.println("e: " + String(error) + " p: " + String(prop) + " i: " + String(integral) +  " d: " + String(derivative) + " sum: " +  String(adjustment));
  return adjustment;

}

void forward(float travel_dist) {

  int minSpeed = 5;
  int maxSpeed = 65;

  float kp = 0.396;
  float ki = 0.0005;
  float kd = 49.8;
  //Put your code here that will rely on motor encoders to drive your robot straight for a specified distance
  resetLeftEncoderCnt();
  resetRightEncoderCnt();

  int targetCount = countForDistance(wheelDiameter, cntPerRevolution, travel_dist);

  int leftSpeed = LEFT_MOTOR_SPEED;
  int rightSpeed = RIGHT_MOTOR_SPEED;

  setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
  setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);

  int rightCnt = 0;
  int leftCnt = 0;

  enableMotor(BOTH_MOTORS);

  while ((rightCnt <= targetCount || leftCnt <= targetCount)) {
    rightCnt = getEncoderRightCnt();
    leftCnt = getEncoderLeftCnt();

    int error = rightCnt - leftCnt;

    // Difference between counts should be 0, so setpoint is 0
    int adjustment = pid(0, error, kp, ki, kd);

    leftSpeed += adjustment;

    if (leftSpeed < minSpeed) {
      //blinkRedLED(50);
      leftSpeed = minSpeed;
    }

    if (leftSpeed > maxSpeed) {
      leftSpeed = maxSpeed;
      //blinkLED(50);
    }

    setMotorSpeed(LEFT_MOTOR, leftSpeed);
    setMotorSpeed(RIGHT_MOTOR, rightSpeed);

    delay(50);
  }

  disableMotor(BOTH_MOTORS);
  delay(2000);
}

/* Function Name: countForDistance
   Input: 2 int input variables, 1 float input
   Return: int value
   Details: Function called to calculate the number of pulses need to travel a specified
            distance by the user input variable "distance."
*/
uint32_t countForDistance(float wheel_diam, uint16_t cnt_per_rev, float distance) {
  float temp = (wheel_diam * PI) / cnt_per_rev;
  temp = distance / temp;
  return int(temp);
}

void driveCircle(int Left_Wheel_Speed, int Right_Wheel_Speed, float Left_Travel_Dist, float Right_Travel_Dist){
//  int minSpeed = 5;
//  int maxSpeed = 65;
//
//  float kp = 50.0;
//  float ki = 0.4;
//  float kd = 0.0;
    
  int rightTargetSpeed = (Left_Wheel_Speed * (OUTSIDE_WHEEL_RATIO)) + 0.5;
  int leftTargetSpeed = Left_Wheel_Speed;

  int leftTargetCnt = countForDistance(wheelDiameter, cntPerRevolution, Left_Travel_Dist);
  int rightTargetCnt = countForDistance(wheelDiameter, cntPerRevolution, Right_Travel_Dist);

//  int difference = rightTargetCnt - leftTargetCnt;
  
  resetLeftEncoderCnt();
  resetRightEncoderCnt();

  setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
  setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
  
  int leftCnt = 0;
  int rightCnt = 0;
  
  enableMotor(BOTH_MOTORS);

  while((leftCnt <= leftTargetCnt) || (rightCnt <= rightTargetCnt)){
    // TO-DO: Impliment PID.....
    
    rightCnt = getEncoderRightCnt();
    leftCnt = getEncoderLeftCnt();

//    int error = rightCnt - leftCnt;
//
//    int adjustment = pid(difference, error, kp, ki, kd);
//
//    rightTargetSpeed +=adjustment;
//
//    if(rightTargetSpeed < minSpeed){
//      rightTargetSpeed = minSpeed;
//    }
//
//    if(rightTargetSpeed > maxSpeed){
//      rightTargetSpeed = maxSpeed;
//    }
     setRawMotorSpeed(LEFT_MOTOR, leftTargetSpeed);
     setRawMotorSpeed(RIGHT_MOTOR, rightTargetSpeed);
    }
  disableMotor(BOTH_MOTORS);
  delay(2000);
}
