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
motor liftMotor(PORT16);

motor armMotor(PORT17);     
bool extended = false;      //tracks if the arm is extended or not

motor clawMotor(PORT18);
bool open = false;          //tracks if the claw is open or not

controller Controller1;

const int DEADZONE = 10;    //Area that the joysticks must be outside of to register movement. Used to prevent accidental movement when the joysticks are near the center.

int applyDeadzone(int value) {    //Applying the deadzone to joystick values
    if (abs(value) <= DEADZONE) {     //If joystick is within the deadzone, return 0 to prevent movement
      return 0;
    }
    else{         
      if (value > 0) {      //If joystick is outside of the deadzone, scale the value to be between 0 and 100 or 0 and -100 depending on the direction of the joystick
        return (value - DEADZONE) * 100 / (100 - DEADZONE);  // scale the value to be between 0 and 100
      }
      else{
        return (value + DEADZONE) * 100 / (100 - DEADZONE);  // scale the value to be between 0 and -100
      }
    } 
}

// A global instance of the competition
competition Competition;

//Setup code

void pre_auton(void) {

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}


//Autonomous mode

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}


//User controls

bool buttonAPressed = false;    //variable used for debounce of button A
bool buttonBPressed = false;    //variable used for debounce of button B

void usercontrol(void) {
  // User control code here, inside the loop

  while (true) {
    int forwardSpeed = applyDeadzone(Controller1.Axis3.position());  // front and backward
    int turnSpeed = applyDeadzone(Controller1.Axis1.position());     //left and right 
    
    if (forwardSpeed < 0) {
      turnSpeed *= -1;     //Invert the turn speed when going backwards
    }
    
    int leftSpeed = forwardSpeed + (turnSpeed / 2);            //calculate the power to use in the left side 
    int rightSpeed = forwardSpeed - (turnSpeed / 2);           //calculate the power to use in the right side

    leftDrive.spin(forward, leftSpeed, percent);          //spin the left side motors at the calculated speed
    rightDrive.spin(forward, rightSpeed, percent);        //spin the right side motors at the calculated speed

    int intake = Controller1.ButtonR2.pressing() - Controller1.ButtonL2.pressing();  //if R2 is pressed, intake = 1, if R1 is pressed, intake = -1, if both are pressed, intake = 0
    intake *= 100;
    intakeMotor.spin(forward, intake, percent);     //run the intake motor

    int lift = Controller1.ButtonR1.pressing() - Controller1.ButtonL1.pressing();
    lift *= 100;
    liftMotor.spin(forward, lift, percent);

    if (Controller1.ButtonB.pressing()) {
      if (!buttonBPressed) {
        buttonBPressed = true;
        if (extended) {
          extended = false;
          armMotor.spinToPosition(0, degrees, 70, velocityUnits::pct, false);
        }
        else {
          extended = true;
          armMotor.spinToPosition(90, degrees, 70, velocityUnits::pct, false);
        }
      }
    }
    else {
      buttonBPressed = false;
    }
    
    if (Controller1.ButtonA.pressing()) {
      if (!buttonAPressed) {
        buttonAPressed = true;
        if (open) {
          open = false;
          clawMotor.spinToPosition(0, degrees, 100, velocityUnits::pct, false);
        }
        else {
          open = true;
          clawMotor.spinToPosition(40, degrees, 100, velocityUnits::pct, false);
        }
      }
    }

    else {
      buttonAPressed = false;
    }

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
