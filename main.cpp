/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       2772031                                                   */
/*    Created:      10/14/2026, 11:52:10 AM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

//set up the motors 4 
  motor LeftFront(PORT2, ratio6_1, false);      //18_1 for green motors   // 6_1 is for blue motors 
  motor LeftBack(PORT4, ratio6_1, false);            ///for the left motor becuase the motor on the left is reversed
  motor RightFront(PORT6, ratio6_1, true);
  motor RightBack(PORT8, ratio6_1, true);          //right motors

  motor_group LeftDrive = (LeftFront, LeftBack);   //group together since motor on one side should move the same direciton
  motor_group LeftDrive = (RightFront, RightBack);
  

  
  const int DEADZONE = 10;

int applyDeadzone(int value) {
    return (abs(value) > DEADZONE) ? value : 0;            //avoid joy stick error click 
}
// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  controller Controller1;

  while (true) {
    int forwardSpeed = applyDeadzone(Controller1.Axis3.position());  // front and backward
    int turnSpeed = applyDeadzone(Controller1.Axis4.position());     //left and right 

    int leftSpeed = forwardSpeed + turnSpeed;            //the brain calculate the power used in the left side 
    int rightSpeed = forwardSpeed - turnSpeed;           //the brain calculates the power used in the right side

    LeftDrive.spin(forward, leftSpeed, percent);
    RightDrive.spin(forward, rightSpeed, percent);

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

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
