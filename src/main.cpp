#include "vex.h"

using namespace vex;

//motor and controller setup

motor leftFront(PORT11, ratio6_1, true);        //left motors
motor leftBack(PORT12, ratio6_1, true);         //left motors are reversed because they are on the opposite side of the robot
motor rightFront(PORT13, ratio6_1, false);      //right motors
motor rightBack(PORT14, ratio6_1, false);

motor_group leftDrive = {leftFront, leftBack};      //grouped together since motors on the same side should move the same direction
motor_group rightDrive = {rightFront, rightBack};

motor intakeMotor(PORT15);

motor armMotor(PORT16);     
bool armExtended = false;      //tracks if the arm is extended or not

motor clawMotor(PORT17, true);
bool clawOpen = false;          //tracks if the claw is open or not

motor lift_left(PORT18, true);
motor lift_right(PORT19);
int liftPower = 60;       //power to run the lift motors at

motor monitored_motors[] = {leftFront, leftBack, rightFront, rightBack, intakeMotor, armMotor, clawMotor, lift_left, lift_right};  //array of motors to be monitored for overheating
const char* motorNames[] = {"LFD", "LBD", "RFD", "RBD", "I", "A", "C", "LL", "LR"};      //names of the motors to be monitored for overheating

int motorCount = sizeof(monitored_motors) / sizeof(monitored_motors[0]);  //number of motors to be monitored for overheating

motor_group liftMotors = {lift_left, lift_right};

controller Controller1;

brain Brain;

bool buttonAPressed = false;    //variable used for debounce of button A
bool buttonBPressed = false;    //variable used for debounce of button B
bool buttonUpPressed = false;   //variable used for debounce of up button
bool buttonDownPressed = false; //variable used for debounce of down button


const int DEADZONE = 10;    //Area that the joysticks must be outside of to register movement. Used to prevent accidental movement when the joysticks are near the center.

int applyScaling(int value) {    //Applying the deadzone to joystick values
  float scaled = 0.0f;   //variable to store the scaled value of the joystick
  
  if (abs(value) <= DEADZONE) {     //If joystick is within the deadzone, return 0 to prevent movement
    return 0;
  }
  else{
    if (value > 0) {      //If joystick is outside of the deadzone, scale the value to be between 0 and 100 or 0 and -100 depending on the direction of the joystick
      scaled = (value - DEADZONE) * 100.0f / (100.0f - DEADZONE);  // scale the value to be between 0 and 100
    }
    else{
      scaled = (value + DEADZONE) * 100.0f / (100.0f - DEADZONE);  // scale the value to be between 0 and -100
    }
  }

  scaled /= 100.0f;  // scale the value to be between 0 and 1.0 or 0 and -1.0
  float curved = (0.3f * scaled) + (0.7f * scaled * scaled * scaled);  // apply a cubic and linear blend to the value to make it less sensitive at low speeds and more sensitive at high speeds
  return curved * 100;  // scale the value back to be between 0 and 100 or 0 and -100
}


void controlLiftSpeed()  {
  bool speedChanged = false;   //variable to track if the lift speed has changed

  if (Controller1.ButtonUp.pressing()) {
    if (!buttonUpPressed && liftPower < 100) {   //if the up button is pressed and the lift speed is less than 100, increase the lift speed by 1
      buttonUpPressed = true;
      liftPower += 1;
      speedChanged = true;
    }
  }
  else {
    buttonUpPressed = false;
  }
  
  if (Controller1.ButtonDown.pressing()) {
    if (!buttonDownPressed && liftPower > 0) {
      buttonDownPressed = true;
      liftPower -= 1;
      speedChanged = true;
    }
  }
  else {
    buttonDownPressed = false;
  }

  if (speedChanged) {
    Controller1.Screen.clearLine(1);
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("Lift Speed: %d%%", liftPower);
  }
}


void monitorMotorTemperature() {
  static char lastBuffer[64] = "";  //buffer to store the last temperature readings to prevent spamming the controller screen with the same message
  char buffer[64] = "";  //buffer to store the current temperature readings
  bool firstOverheated = true;  //variable to track if this is the first motor that is overheating

  for (int i = 0; i < motorCount; i++) {
    if (monitored_motors[i].temperature(celsius) > 45) {  //check if the motor is overheating
      if (firstOverheated) {
        strcat(buffer, "HOT:");  //if this is the first motor that is overheating, add "HOT:" to the buffer
        firstOverheated = false;
      }
      else {
        strcat(buffer, " ");  //if this is not the first motor that is overheating, add a space to the buffer
      }
      strcat(buffer, motorNames[i]);  //add the name of the motor that is overheating to the buffer
    }
  }

  if (strcmp(buffer, lastBuffer) != 0) {        //If current buffer is different from last buffer, print new buffer and update last buffer
    Controller1.Screen.clearLine(2);
    Controller1.Screen.setCursor(2, 1);
    Controller1.Screen.print("%s", buffer);     //print new list of overheated motors to the controller screen
    strcpy(lastBuffer, buffer);                 //update last buffer to current buffer

    if (strlen(buffer) > 0) {                   //check if any motors are actually overheating before vibrating the controller
      Controller1.rumble(".-.");                //vibrate the controller to alert the user that a motor is overheating
    }
  }
}


void monitorBattery() {
  static int lastBatteryLevel = 0;    //store the last battery level to detect changes
  int batteryLevel = Brain.Battery.capacity(percentUnits::pct);    //get the battery percentage
  if (batteryLevel != lastBatteryLevel) {             //check if battery level has changed since last check
    lastBatteryLevel = batteryLevel;          //update the last battery level to the current battery level
    Controller1.Screen.clearLine(3);
    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.print("Battery: %d%%", batteryLevel);    //display the battery percentage on the controller screen
  }
}

// A global instance of the competition
competition Competition;

//Setup code

void pre_auton(void) {
  clawMotor.setMaxTorque(30, percent);
  clawMotor.spin(forward, 10, percent);
  int timeout = 0;
  wait(100, msec);  //wait for the claw to start closing

  while (fabs(clawMotor.velocity(rpm)) > 0.1 && timeout < 100) {    //wait until the claw is fully closed or a timeout of 2 seconds is reached to prevent the robot from getting stuck
    wait(20, msec);  //wait for the claw to fully close
    timeout++;
  }
  clawMotor.setMaxTorque(40, percent);
  clawMotor.stop(hold);
  clawMotor.resetPosition();
  clawMotor.setStopping(hold);

  armMotor.setStopping(hold);
  liftMotors.setStopping(hold);

  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("Lift Speed: %d%%", liftPower);
}


//Autonomous mode

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}


//User controls

void usercontrol(void) {
  buttonAPressed = false;
  buttonBPressed = false;
  buttonUpPressed = false;
  buttonDownPressed = false;
  
  bool liftWasMoving = true;  //variable to track if the lift was moving in the last iteration of the loop

  while (true) {
    int forwardSpeed = applyScaling(Controller1.Axis3.position());  // front and backward
    int turnSpeed = applyScaling(Controller1.Axis1.position());     //left and right 
    
    if (forwardSpeed < 0) {
      turnSpeed *= -1;     //Invert the turn speed when going backwards
    }
    
    int leftSpeed = forwardSpeed + turnSpeed;            //calculate the power to use in the left side 
    int rightSpeed = forwardSpeed - turnSpeed;           //calculate the power to use in the right side

    int maxSpeed = std::max(abs(leftSpeed), abs(rightSpeed));  //find the maximum speed of the two sides
    if (maxSpeed > 100) {     //if the maximum speed is greater than 100, scale both sides down to be within the range of -100 to 100
      leftSpeed = leftSpeed * 100 / maxSpeed;
      rightSpeed = rightSpeed * 100 / maxSpeed;
    }

    leftDrive.spin(forward, leftSpeed, percent);          //spin the left side motors at the calculated speed
    rightDrive.spin(forward, rightSpeed, percent);        //spin the right side motors at the calculated speed

    int intake = Controller1.ButtonR2.pressing() - Controller1.ButtonL2.pressing();  //if R2 is pressed, intake = 1, if L2 is pressed, intake = -1, if both are pressed, intake = 0
    intake *= 100;           //scale the intake value to run the motor at full speed
    intakeMotor.spin(forward, intake, percent);     //run the intake motor

    int lift = Controller1.ButtonR1.pressing() - Controller1.ButtonL1.pressing();
    lift *= liftPower;     //scale the lift value to run the motor at the set speed
    if (lift == 0) {
      if (liftWasMoving) {    //if the lift was moving in the last iteration of the loop, stop the lift motors and hold their position
        liftMotors.stop(hold);     //if neither button is pressed, hold the lift in place
        liftWasMoving = false;    //set the variable to false since the lift is no longer moving
      }
    }
    else {
      liftMotors.spin(forward, lift, percent);
      liftWasMoving = true;    //if the lift is moving, set the variable to true
    }

    if (Controller1.ButtonA.pressing()) {
      if (!buttonAPressed) {
        buttonAPressed = true;
        if (armExtended) {
          armExtended = false;
          armMotor.spinToPosition(0, degrees, 70, velocityUnits::pct, false);
        }
        else {
          armExtended = true;
          armMotor.spinToPosition(90, degrees, 70, velocityUnits::pct, false);
        }
      }
    }
    else {
      buttonAPressed = false;
    }
    
    if (Controller1.ButtonB.pressing()) {
      if (!buttonBPressed) {
        buttonBPressed = true;
        if (clawOpen) {
          clawOpen = false;
          clawMotor.setMaxTorque(40, percent);
          clawMotor.spinToPosition(16, degrees, 100, velocityUnits::pct, false);
        }
        else {
          clawOpen = true;
          clawMotor.setMaxTorque(100, percent);
          clawMotor.spinToPosition(146, degrees, 100, velocityUnits::pct, false);
        }
      }
    }

    else {
      buttonBPressed = false;
    }

    controlLiftSpeed();     //call the function to control and display the lift speed
    
    monitorMotorTemperature();   //call the function to monitor the motor temperatures

    monitorBattery();          //call the function to monitor the battery level

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {

  
  
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);


  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
