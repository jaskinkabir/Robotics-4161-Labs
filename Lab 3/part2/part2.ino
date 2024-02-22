/* Edited by young padawan Joey Phillips and master Jedi Dr. Conrad 2-9-24
 * Energia Robot Library for Texas Instruments' Robot System Learning Kit (RSLK)
 * Simplified drive forward Example
 *
 * Summary:
 * This example has the TI Robotic System Learning Kit (TI RSLK) ramp up to speed, 
 * drive forward at a constant speed, and then ramp down to specified speed.
 *
 * How to run:
 * 1) Push left button on Launchpad to start the demo
 * 2) Robot will drive forward after a short LED blinking delay.
 * 3) Push left button to start the demo again
 */

#include "SimpleRSLK.h"

/* Timing delays in milliseconds */
#define MOTOR_DELAY1          6600 // <--- Adjust delay
#define MOTOR_DELAY2          600  // <--- Adjust delay
#define STOP_MOTOR_DELAY      1000

/* Default pwm signals (percentage-% of power 0-100) for both RSLK motor. */
int DEFAULT_LEFT_SPEED = 3;   // <--- Adjust main speed
int DEFAULT_RIGHT_SPEED = 4;  // <--- Adjust main speed
/* Used to reset the motor speeds to run the program again */
int RESET_LEFT_SPEED = DEFAULT_LEFT_SPEED;
int RESET_RIGHT_SPEED = DEFAULT_RIGHT_SPEED;


/* Place code here to only run once ***********************************************/
void setup() {
  Serial.begin(9600);         // Start serial com at 9600 baud rate
  setupRSLK();                // Initialize RSLK functions and classes
  setupWaitBtn(LP_LEFT_BTN);  // Setup left button on Launchpad
  setupLed(RED_LED);          // Red led in rgb led
  setupLed(GREEN_LED);        // Greem led in rgb led
}

/* Place code here to run forever/loop/repeat *************************************/
void loop() {
  /* Function to wait until left MSP 432 button is pressed to start program */
  startProgram();   // wait until left MSP 432 button is pressed to start program
  rampUpMotors();   // ramp motor speeds up over a period of time
  //moveForward();    // move robot forward at a constant speed for a period of time
  blinkLED(1000);
  delay(MOTOR_DELAY1-2000);         // The time the motors should run, in milliseconds
  blinkLED(1000);
  rampDownMotors(); // ramp motor speeds down over a period of time
  stopMotors();     // stop robot/motors
  resetMotorSpeeds();    // reset motors back to default speeds
}

/* Function Name: rampUpMotors
 * Input: void
 * Details: Function called to increase the motor speeds over a period of time. The 
 *          function represents 11 stages of power increase.
*/
void rampUpMotors(){
  for(int i = 0; i < 10; i++){  // Loop 10 times
    DEFAULT_LEFT_SPEED += 2;      // increment left speed by 1
    DEFAULT_RIGHT_SPEED += 2;     // increment right speed by 1
    moveForward();              // move robot forward at speed set
    delay(MOTOR_DELAY2);        // The time the motors should run, in milliseconds
  }
}

/* Function Name: rampDownMotors
 * Input: void
 * Details: Function called to decrease the motor speeds over a period of time. The 
 *          function represents 10 stages of power decrease.
*/
void rampDownMotors(){
  for(int i = 10; i > 0; i--){  // Loop 10 times
    DEFAULT_LEFT_SPEED -= 2;      // decrement left speed by 1
    DEFAULT_RIGHT_SPEED -= 2;     // decrement right speed by 1
    moveForward();              // move robot forward at speed set
    delay(MOTOR_DELAY2);         // The time the motors should run, in milliseconds
  }
}

/* Function Name: resetMotorSpeeds
 * Input: void
 * Details: Function called to reset the motor speeds back to a default value. Since
 *          the motor speeds are increased and decreased during the program, we want
 *          to make sure our values are reset properly before the program begins.
*/
void resetMotorSpeeds(){
  /* Reset motor speeds to original specified speeds */
  DEFAULT_LEFT_SPEED = RESET_LEFT_SPEED;
  DEFAULT_RIGHT_SPEED = RESET_RIGHT_SPEED;
}

/* Function Name: moveForward
 * Input: void
 * Details: Function called to move the robot forward 
 *          for a specified time and then stop.
*/
void moveForward(){
  enableMotor(BOTH_MOTORS);                         // "Turn on" the motor
  setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD); // Cause the robot to drive forward
  setMotorSpeed(LEFT_MOTOR,DEFAULT_LEFT_SPEED);       // Set motor left motor speed
  setMotorSpeed(RIGHT_MOTOR,DEFAULT_RIGHT_SPEED);     // Set motor right motor speed
}

/* Function Name: stopMotors
 * Input: void
 * Details: Function called to disable or stop the motors/robot.
*/
void stopMotors(){
  disableMotor(BOTH_MOTORS); // Stop both motors
  delay(STOP_MOTOR_DELAY);   // Time the motors are stopped
}

/* Function Name: startProgram
 * Input: void
 * Details: Function called to wait for a button to be pressed
 *          in order to start the robot program
*/
void startProgram(){
  /* Setup message to print to serial montor */
  String btnMsg = "Push left button on Launchpad to start lab program.\n";
  waitBtnPressed(LP_LEFT_BTN,btnMsg,RED_LED); // Function to setup button, msg, LED
  /* Using an LED as a DELAY */
  blinkLED(1000);                             // Cause LED to blink for a period of one second
  blinkLED(1000);                             // Cause LED to blink for a period of one second
}

/* Function Name: blinkLED
 * Input: integer (period) in milliseconds
 * Details: Function call that will blink a colored LED for a period specified.
*/
int blinkLED(int period){
  int pause = period/2;           // Determine on/off time
  digitalWrite(GREEN_LED, HIGH);  // Turn LED on
  delay(pause);                   // Time the LED is on
  digitalWrite(GREEN_LED, LOW);   // Turn LED off
  delay(pause);                   // Time LED is off
}
