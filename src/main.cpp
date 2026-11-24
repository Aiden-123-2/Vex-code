#include "vex.h"

using namespace vex;

//motor and controller setup

motor leftFront(PORT11, ratio12_1, true);        //left motors
motor leftBack(PORT12, ratio12_1, true);         //left motors are reversed because they are on the opposite side of the robot
motor rightFront(PORT13, ratio12_1, false);      //right motors
motor rightBack(PORT14, ratio12_1, false);

motor_group leftDrive = {leftFront, leftBack};      //grouped together since motors on the same side should move the same direction
motor_group rightDrive = {rightFront, rightBack};
int drivePower = 100;

motor intakeMotor(PORT15);

motor armMotor(PORT16);     
bool armExtended = false;      //tracks if the arm is extended or not
int armPower = 60;

motor clawMotor(PORT17, true);
bool clawOpen = false;          //tracks if the claw is open or not

motor lift_left(PORT18, true);
motor lift_right(PORT19);
int liftPower = 60;       //power to run the lift motors at

motor monitored_motors[] = {leftFront, leftBack, rightFront, rightBack, intakeMotor, armMotor, clawMotor, lift_left, lift_right};  //array of motors to be monitored for overheating
const char* motorNames[] = {"LFD", "LBD", "RFD", "RBD", "I", "A", "C", "LL", "LR"};      //names of the motors to be monitored for overheating

int motorCount = sizeof(monitored_motors) / sizeof(monitored_motors[0]);  //number of motors to be monitored for overheating

motor_group liftMotors = {lift_left, lift_right};

char lastBuffer[64] = "";  //buffer to store the last temperature readings to prevent spamming the controller screen with the same message
int selected = 1;
bool speedChanged = true;
int lastBatteryLevel = -1;

controller Controller1;

brain Brain;

bool buttonAPressed = false;    //variable used for debounce of button A
bool buttonBPressed = false;    //variable used for debounce of button B
bool buttonUpPressed = false;   //variable used for debounce of up button
bool buttonDownPressed = false; //variable used for debounce of down button
bool buttonRightPressed = false;  //variable used for debounce of right button
bool buttonLeftPressed = false;   //variable used for debounce of left button
bool buttonXPressed = false;    //variable used for debounce of button X

int screen = 1;
int lastScreen = screen;

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

bool wasPressed(bool isPressing, bool &flag) {  //function to check if a button was pressed and released
  if (isPressing) {
    if (!flag) {
      flag = true;
      return true;  //button was pressed
    }
  }
  else {
    flag = false;  //button was released
  }
  return false;  //button was not pressed
}

void controlMotorSpeeds()  {
  static int count = 0;
  if (wasPressed(Controller1.ButtonX.pressing(), buttonXPressed)) {
    selected++;
    if (selected > 3) {
      selected = 1;
    }
    
    speedChanged = true;
  }

  if (wasPressed(Controller1.ButtonUp.pressing(), buttonUpPressed)) {  //if button up is pressed and released, increase the lift speed by 1
    if (selected == 1) liftPower++;
    if (selected == 2) armPower++;
    if (selected == 3) drivePower++;
    speedChanged = true;
  }

  else if (Controller1.ButtonUp.pressing()) {
    count++;
    if (count % 7 == 0) {
      if (selected == 1) liftPower++;
      if (selected == 2) armPower++;
      if (selected == 3) drivePower++;
      speedChanged = true;
    }
  }

  else if (wasPressed(Controller1.ButtonDown.pressing(), buttonDownPressed)) {
    if (selected == 1) liftPower--;
    else if (selected == 2) armPower--;
    else if (selected == 3) drivePower--;
    speedChanged = true;
  }
  
  else if (Controller1.ButtonDown.pressing()) {
    count++;
    if (count % 7 == 0) {
      if (selected == 1) liftPower--;
      if (selected == 2) armPower--;
      if (selected == 3) drivePower--;
      speedChanged = true;
    }
  }

  else {
    count = 0;
  }

  liftPower = (liftPower > 100) ? 100 : (liftPower < 20) ? 20 : liftPower;
  armPower = (armPower > 100) ? 100 : (armPower < 20) ? 20 : armPower;
  drivePower = (drivePower > 100) ? 100 : (drivePower < 20) ? 20 : drivePower;

  if (speedChanged) {
    Controller1.Screen.clearScreen();

    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("%sLift Power: %d%%", (selected == 1) ? ">" : "", liftPower);

    Controller1.Screen.setCursor(2, 1);
    Controller1.Screen.print("%sArm Power: %d%%", (selected == 2) ? ">" : "", armPower);

    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.print("%sDrive Power: %d%%", (selected == 3) ? ">" : "", drivePower);

    speedChanged = false;
  }
}


void monitorMotorTemps() {
  char buffer[64] = "";  //buffer to store the displayed message
  char motorTemp[8] = "";

  for (int i = 0; i < motorCount; i++) {
    double temp = monitored_motors[i].temperature(celsius);  //get the temperature of the motor in celsius
      snprintf(motorTemp, sizeof(motorTemp), "%d° ", (int)temp);  //convert the temperature to a string
      strcat(buffer, motorTemp);
  }

  if (strcmp(buffer, lastBuffer) != 0) {        //If current buffer is different from last buffer, print new buffer and update last buffer
    strcpy(lastBuffer, buffer);
    char line1[20] = "";
    strncpy(line1, buffer, 19);
    line1[19] = '\0';  //ensure that the strings are null terminated

    char line2[20] = "";
    strncpy(line2, buffer + 19, 19);
    line2[19] = '\0';
    
    Controller1.Screen.clearLine(1);
    Controller1.Screen.clearLine(2);
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("%s", line1);
    Controller1.Screen.setCursor(2, 1);
    Controller1.Screen.print("%s", line2);
  }
}


void monitorBattery() {
  int batteryLevel = Brain.Battery.capacity(percentUnits::pct);    //get the battery percentage

  if (batteryLevel != lastBatteryLevel) {             //check if battery level has changed since last check
    lastBatteryLevel = batteryLevel;          //update the last battery level to the current battery level
    Controller1.Screen.clearLine(3);
    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.print("Battery: %d%%", batteryLevel);    //display the battery percentage on the controller screen
  }
}

void displayScreen() {
  lastScreen = screen;
  
  if (wasPressed(Controller1.ButtonRight.pressing(), buttonRightPressed)) {  //if button right is pressed and released, go to the next screen
    screen++;
    if (screen > 2) {   //if the screen number is greater than 2, reset it to 1
      screen = 1;
    }
  }

  if (wasPressed(Controller1.ButtonLeft.pressing(), buttonLeftPressed)) {  //if button left is pressed and released, go to the previous screen
    screen--;
    if (screen < 1) {   //if the screen number is less than 1, reset it to 2
      screen = 2;
    }
  } 

  if (screen == 1) {
    if (screen != lastScreen) {
      speedChanged = true;
    }
    controlMotorSpeeds();
  }

  else if (screen == 2) {
    if (screen != lastScreen) {
      Controller1.Screen.clearScreen();
      lastBuffer[0] = '\0';
      lastBatteryLevel = -1;
    }
    monitorMotorTemps();
    monitorBattery();
  }
}

// A global instance of the competition
competition Competition;

//Setup code

void pre_auton(void) {
  clawMotor.setMaxTorque(30, percent);
  clawMotor.spin(reverse, 10, percent);
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
  clawOpen = false;

  armMotor.resetPosition();
  armMotor.setStopping(hold);
  armExtended = false;

  liftMotors.setStopping(hold);

  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print(">Lift Power: %d%%", liftPower);
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("Arm Power: %d%%", armPower);
  Controller1.Screen.setCursor(3, 1);
  Controller1.Screen.print("Drive Power: %d%%", drivePower);
}


//Autonomous mode
const double Wheel_Diameter = 4.0;          //for the inches of diameter of our drivetrain wheels
const double Gear_Ratio = 48.0/36.0;        // wheel 48 motor 36 ratio
double inchesToDegrees(double inches){
  return inches/(Wheel_Diameter *3.14159265358)*360.0 * Gear_Ratio;         //wheel to motor
}
void driveInches(double inches, int speed){
  double deg = inchesToDegrees (inches);
  leftDrive.resetPosition();
  rightDrive.resetPosition();
  leftDrive.spinFor(forward, deg, degress, speed, velocityUnits::pct,false);
  rightDrive.spinFor(forward, deg, degress, speed, velocityUnits::pct);
  
}

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  leftDrive.setStopping(brake);
  rightDrive.setStopping(brake);
  driveInches(24.0, 50);                     //example
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print("L:%.0f R:%.0f", leftDrive.position(degrees), rightDrive.position(degrees) );            //display the degree spinned 
  
  // ..........................................................................
}


//User controls

void usercontrol(void) {
  buttonAPressed = false;
  buttonBPressed = false;
  buttonUpPressed = false;
  buttonDownPressed = false;
  buttonRightPressed = false;
  buttonLeftPressed = false;
  buttonXPressed = false;
  
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
    if (maxSpeed > drivePower) {     //if the maximum speed is greater than 100, scale both sides down to be within the range of -100 to 100
      leftSpeed = leftSpeed * drivePower / maxSpeed;
      rightSpeed = rightSpeed * drivePower / maxSpeed;
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

    if (wasPressed(Controller1.ButtonA.pressing(), buttonAPressed)) {  //if button A is pressed and released, toggle the arm position
      if (armExtended) {
        armExtended = false;
        armMotor.spinToPosition(0, degrees, armPower, velocityUnits::pct, false);
      }
      else {
        armExtended = true;
        armMotor.spinToPosition(90, degrees, armPower, velocityUnits::pct, false);
      }
    }
    
    if (wasPressed(Controller1.ButtonB.pressing(), buttonBPressed)) {  //if button B is pressed and released, toggle the claw position
      if (clawOpen) {
        clawOpen = false;
        clawMotor.setMaxTorque(100, percent);
        clawMotor.spinToPosition(100, degrees, 100, velocityUnits::pct, false);
      }
      else {
        clawOpen = true;
        clawMotor.setMaxTorque(100, percent);
        clawMotor.spinToPosition(146, degrees, 100, velocityUnits::pct, false);
      }
    }

    if (clawMotor.isDone()) {
      clawMotor.setMaxTorque(40, percent);
    }

    displayScreen();

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
