#include "SimpleRSLK.h"

#define DELAY_MS            250   // delay in milliseconds

// Default pwm signals (percentage-% of power 0-100) for both RSLK motor.
// Change these values as needed
#define LEFT_MOTOR_SPEED    100    // Speed percentage
#define RIGHT_MOTOR_SPEED   100    // Speed percentage
#define LEFT_TURN_SPEED     30    // Speed percentage
#define RIGHT_TURN_SPEED    30    // Speed percentage

// Value for turning directions (do not change)
#define CCW                  1     // rotate robot counter clockwise
#define CW                   2     // rotate robot clockwise

// RSLK specified mechanics (do not change)
#define wheelDiameter       6.999      // in centimeters
#define cntPerRevolution    360        // Number of encoder (rising) pulses every time the wheel turns completely

/* Place code here to only run once ***********************************************/
void setup() {
  //Do not edit this setup function
  Serial.begin(19200);         // Start serial com at 9600 baud rate
  setupRSLK();                // Initialize RSLK functions and classes
  setupWaitBtn(LP_LEFT_BTN);  // Setup left button on Launchpad
  setupLed(RED_LED);          // Red led in rgb led
  setupLed(GREEN_LED);        // Red led in rgb led
}

/* Place code here to run forever/loop/repeat *************************************/
void loop() {
  startProgram();   // wait until left MSP 432 button is pressed to start program

  /************ after reading, you may delete this for your testing and use ***********
    Test your algorithm and/or functions by placing them here just like the example below.
    It does not matter what is in this loop when you submit. It will be replaced by the
    instructor or TA with a specific algorithm of their choosing.

    forward(10);
    rotate(CCW, 360);
    forward(90);
  ************************************************************************************/
  forward(200);
}

/* Function Name: rotate
   Input: 2 int inputs
   Return: void
   Details:  Will rotate the robot the specified number of degrees in the specified direction
   Uses the encoders to ensure this number of degrees is reached as closely as possible
*/

void rotate(int rotate_dir, int rotate_deg) {
  //Put your code here that will rely on motor encoders for accurately rotating in place
  uint32_t robot_base_diam = 14.0;
  uint32_t distance = (robot_base_diam * PI) * (rotate_deg / 360.0);
  uint32_t targetCount = countForDistance(wheelDiameter, cntPerRevolution, distance);
  resetLeftEncoderCnt();
  resetRightEncoderCnt();

  int leftCount = 0;
  int rightCount = 0;

  switch (rotate_dir) {
    case CCW:
      setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
      setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);

    case CW:

      setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
      setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);

  }
  enableMotor(BOTH_MOTORS);
  setRawMotorSpeed(LEFT_MOTOR, LEFT_TURN_SPEED);
  setRawMotorSpeed(RIGHT_MOTOR, RIGHT_TURN_SPEED);
  while (leftCount < targetCount || rightCount < targetCount) {
    leftCount = getEncoderLeftCnt();
    rightCount = getEncoderRightCnt();
  }
  disableMotor(BOTH_MOTORS);
  delay(2000);

}

/* Function Name: pid
   Input: 2 int inputs, 3 double inputs
   Return: int value
   Details: Implements PID Control to calculate an
   adjustment to be added to the input of a system based
   on the error detected at the output of the system.
   Requires 3 input parameters to be tuned for the system.
*/
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


/* Function Name: forward
   Input: 1 float input
   Return: void
   Details: Function called to command robot to move forward for the distance--in centimeters-- specified by the user.
   Uses PID Control to ensure robot drives straight
*/
void forward(float travel_dist) {


  int minSpeed = 12;
  int maxSpeed = 200;


  
  float kp = .242;
  float ki = 0.00039;
  float kd = 56;
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
    rightSpeed -= adjustment;

    if (leftSpeed < minSpeed) {
      //blinkRedLED(50);

      leftSpeed = minSpeed;
    }


    if (leftSpeed > maxSpeed) {
      leftSpeed = maxSpeed;

      //blinkLED(50);
    }

    if (rightSpeed < minSpeed) rightSpeed = minSpeed;
    if (rightSpeed > maxSpeed) rightSpeed = maxSpeed;


    setRawMotorSpeed(LEFT_MOTOR, leftSpeed);
    setRawMotorSpeed(RIGHT_MOTOR, rightSpeed);

    delay(50);
    Serial.println("Left Enc: " + String(leftCnt) + "  Right Enc:  " + String(rightCnt));
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
uint32_t countForDistance(float wheel_diam, uint16_t cnt_per_rev, uint32_t distance) {
  float temp = (wheel_diam * PI) / cnt_per_rev;
  temp = distance / temp;
  return int(temp);
}

/* Function Name: startProgram
   Input: void
   Details: Function called to wait for a button to be pressed
            in order to start the robot program
*/
void startProgram() {
  /* Setup message to print to serial montor */
  String btnMsg = "Push left button on Launchpad to start lab program.\n";
  //String btnMsg = "";
  waitBtnPressed(LP_LEFT_BTN, btnMsg, RED_LED); //Setup button, msg, LED
  /* Using an LED as a DELAY */
  blinkLED(1000); //Cause LED to blink for a period of one second
  blinkLED(1000); //Cause LED to blink for a period of one second
}

/* Function Name: blinkLED
   Input: integer (period) in milliseconds
   Details: Function call that will blink a colored LED for a period specified.
*/
int blinkLED(int period) {
  int pause = period / 2;         // Determine on/off time
  digitalWrite(GREEN_LED, HIGH);  // Turn LED on
  delay(pause);                   // Time the LED is on
  digitalWrite(GREEN_LED, LOW);   // Turn LED off
  delay(pause);                   // Time LED is off
}

/* Function Name: blinkRedLED
   Input: integer (period) in milliseconds
   Details: Function call that will blink a red LED for a period specified.
*/
int blinkRedLED(int period) {
  int pause = period / 2;         // Determine on/off time
  digitalWrite(RED_LED, HIGH);  // Turn LED on
  delay(pause);                   // Time the LED is on
  digitalWrite(RED_LED, LOW);   // Turn LED off
  delay(pause);                   // Time LED is off
}
