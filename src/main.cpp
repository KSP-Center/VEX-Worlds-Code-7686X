// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// FLD                  motor         8               
// MLD                  motor         13              
// BLD                  motor         1               
// FRD                  motor         16              
// MRD                  motor         11              
// BRD                  motor         12              
// IntakeFS             motor         4               
// Hooks                motor         2               
// Inertial1            inertial      14              
// PDoinker             digital_out   F               
// PMoGo                digital_out   H               
// PIntake              digital_out   E               
// LB                   motor         7               
// Color_Sort_Optical   optical       10              
// Lady_Rotation        rotation      21              
// Vertical_Odom        rotation      9               
// PDoinker2            digital_out   G               
// PColorSort           digital_out   D               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// FLD                  motor         8               
// MLD                  motor         13              
// BLD                  motor         1               
// FRD                  motor         16              
// MRD                  motor         11              
// BRD                  motor         12              
// IntakeFS             motor         4               
// Hooks                motor         2               
// Inertial1            inertial      14              
// PDoinker             digital_out   F               
// PMoGo                digital_out   H               
// PIntake              digital_out   E               
// LB                   motor         7               
// Color_Sort_Optical   optical       10              
// Lady_Rotation        rotation      21              
// Vertical_Odom        rotation      9               
// PDoinker2            digital_out   G               
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "vex.h"

using namespace vex;
competition Competition;

bool spinHooks;

float kP = 0.095;
float kI = 0.0;
float kD = 0.015;
float error = 0.0;
float prevError = 0.0;
float integral = 0.0;
float derivative = 0.0;
float targetPosition = -38.0;
float currentPosition = 0.0;
float power = 0.0;

void ladybrownPID() {
  currentPosition = Lady_Rotation.position(degrees) / 3;
  error = targetPosition - currentPosition;
  integral += error;
  derivative = error - prevError;
  power = kP*error + kI*integral + kD*derivative;
  LB.spin(forward, power, volt);
  wait(20, msec);
}

/*void colorSort(bool isRed) {
  optical::rgbc color = Color_Sort_Optical.getRgb();
  if (Color_Sort_Optical.isNearObject()) {
    if (isRed) {
      if (color.red > color.blue) {
        wait(51, msec);
        spinHooks = 0;
      }
    } else {
      if (color.blue > color.red) {
        wait(51, msec);
        spinHooks = 0;
      }
    }
  } else  {
    spinHooks = 1;
  }
}*/

/*---------------------------------------------------------------------------*/
/*                             VEXcode Config                                */
/*                                                                           */
/*  Before you do anything else, start by configuring your motors and        */
/*  sensors. In VEXcode Pro V5, you can do this using the graphical          */
/*  configurer port icon at the top right. In the VSCode extension, you'll   */
/*  need to go to robot-config.cpp and robot-config.h and create the         */
/*  motors yourself by following the style shown. All motors must be         */
/*  properly reversed, meaning the drive should drive forward when all       */
/*  motors spin forward.                                                     */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                             JAR-Template Config                           */
/*                                                                           */
/*  Where all the magic happens. Follow the instructions below to input      */
/*  all the physical constants and values for your robot. You should         */
/*  already have configured your motors.                                     */
/*---------------------------------------------------------------------------*/

Drive chassis(

//Pick your drive setup from the list below:
//ZERO_TRACKER_NO_ODOM
//ZERO_TRACKER_ODOM
//TANK_ONE_FORWARD_ENCODER
//TANK_ONE_FORWARD_ROTATION
//TANK_ONE_SIDEWAYS_ENCODER
//TANK_ONE_SIDEWAYS_ROTATION
//TANK_TWO_ENCODER
//TANK_TWO_ROTATION
//HOLONOMIC_TWO_ENCODER
//HOLONOMIC_TWO_ROTATION
//
//Write it here:
TANK_ONE_FORWARD_ROTATION,

//Add the names of your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).
//You will input whatever motor names you chose when you configured your robot using the sidebar configurer, they don't have to be "Motor1" and "Motor2".

//Left Motors:
motor_group(FLD, MLD, BLD),

//Right Motors:
motor_group(FRD, MRD, BRD),

//Specify the PORT NUMBER of your inertial sensor, in PORT format (i.e. "PORT1", not simply "1"):
PORT14,

//Input your wheel diameter. (4" omnis are actually closer to 4.125"):
3.25,

//External ratio, must be in decimal, in the format of input teeth/output teeth.
0.75,

//Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
//For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
360,

/*---------------------------------------------------------------------------*/
/*                                  PAUSE!                                   */
/*                                                                           */
/*  The rest of the drive constructor is for robots using POSITION TRACKING. */
/*  If you are not using position tracking, leave the rest of the values as  */
/*  they are.                                                                */
/*---------------------------------------------------------------------------*/

//If you are using ZERO_TRACKER_ODOM, you ONLY need to adjust the FORWARD TRACKER CENTER DISTANCE.

//FOR HOLONOMIC DRIVES ONLY: Input your drive motors by position. This is only necessary for holonomic drives, otherwise this section can be left alone.
//LF:      //RF:    
PORT1,     -PORT2,

//LB:      //RB: 
PORT3,     -PORT4,

//If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
//If this is a rotation sensor, enter it in "PORT1" format, inputting the port below.
//If this is an encoder, enter the port as an integer. Triport A will be a "1", Triport B will be a "2", etc.
PORT9,

//Input the Forward Tracker diameter (reverse it to make the direction switch):
2.75,

//Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
//For a zero tracker tank drive with odom, put the positive distance from the center of the robot to the right side of the drive.
//This distance is in inches:
7,

//Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
1,

//Sideways tracker diameter (reverse to make the direction switch):
-2.75,

//Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
5.5

);

int current_auton_selection = 2;
bool auto_started = 0;

/** 
 * Function before autonomous. It prints the current auton number on the screen
 * and tapping the screen cycles the selected auton by 1. Add anything else you
 * may need, like resetting pneumatic components. You can rename these autons to
 * be more descriptive, if you like.
 */

void pre_auton() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  default_constants();

  // setup for user control
  LB.setVelocity(100, percent);
  LB.setMaxTorque(100, percent);
  IntakeFS.setVelocity(100, percent);
  IntakeFS.setMaxTorque(100, percent);
  Hooks.setVelocity(90, percent);
  Hooks.setMaxTorque(100, percent);
  Lady_Rotation.setPosition(0, degrees);

  Color_Sort_Optical.setLight(ledState::on);

  while(!auto_started){
    Brain.Screen.clearScreen();
    Controller1.Screen.clearScreen();
    Brain.Screen.printAt(5, 20, "7686X Phoenix Rising");
    Brain.Screen.printAt(5, 40, "Battery Percentage:");
    //Controller1.Screen.setCursor(1,3);
    Brain.Screen.printAt(5, 60, "%d", Brain.Battery.capacity());
    Brain.Screen.printAt(5, 80, "Chassis Heading Reading:");
    Brain.Screen.printAt(5, 100, "%f", chassis.get_absolute_heading());
    Brain.Screen.printAt(5, 120, "Selected Auton:");
    switch(current_auton_selection){
      case 0:
        Brain.Screen.printAt(5, 140, "Red Neg 1+4");
        controller(primary).Screen.setCursor(1,1);
        controller(primary).Screen.print("Red Neg 1+4");
        break;
      case 1:
        Brain.Screen.printAt(5, 140, "Blue Neg 1+4");
        controller(primary).Screen.setCursor(1,1);
        controller(primary).Screen.print("Blue Neg 1+4");
        break;
      case 2:
        Brain.Screen.printAt(5, 140, "Red Negative 1+4");
        controller(primary).Screen.setCursor(1,1);
        controller(primary).Screen.print("Red Neagtive 1+4");
        break;
      case 3:
        Brain.Screen.printAt(5, 140, "Red Goal Rush");
        controller(primary).Screen.setCursor(1,1);
        controller(primary).Screen.print("Red Goal Rush");
        break;
      case 4:
        Brain.Screen.printAt(5, 140, "Blue Goal Rush");
        controller(primary).Screen.setCursor(1,1);
        controller(primary).Screen.print("Blue Goal Rush");
        break;
      case 5:
        Brain.Screen.printAt(5, 140, "Red 4 Ring");
        controller(primary).Screen.setCursor(1,1);
        controller(primary).Screen.print("Red 4 Ring");
        break;
      case 6:
        Brain.Screen.printAt(5, 140, "Skills");
        controller(primary).Screen.setCursor(1,1);
        controller(primary).Screen.print("Skills");
        break;
      case 7:
        Brain.Screen.printAt(5, 140, "Left Safe");
        controller(primary).Screen.setCursor(1,1);
        controller(primary).Screen.print("Left Safe");
        break;
      case 8:
        Brain.Screen.printAt(5, 140, "Right Safe");
        controller(primary).Screen.setCursor(1,1);
        controller(primary).Screen.print("Right Safe");
        break;
    }
    Brain.Screen.printAt(5, 160, "%f",Lady_Rotation.position(rev));
    if(Brain.Screen.pressing() or controller(primary).ButtonRight.pressing()){
      while(Brain.Screen.pressing() or controller(primary).ButtonRight.pressing()) {}
      current_auton_selection++;
    } else if (current_auton_selection == 9){
      current_auton_selection = 0;
    }
    /*if(controller(primary).ButtonLeft.pressing()){
      while(controller(primary).ButtonLeft.pressing()) {}
      current_auton_selection--;
    } else if (current_auton_selection == 9){
      current_auton_selection = 0;
    }
   */ 
    wait(10, msec);
  }
}

/**
 * Auton function, which runs the selected auton. Case 0 is the default,
 * and will run in the brain screen goes untouched during preauton. Replace
 * drive_test(), for example, with your own auton function you created in
 * autons.cpp and declared in autons.h.
 */

void autonomous(void) {
  auto_started = 1;
  switch(current_auton_selection){ 
    case 0:
      blue4ring();
      break;
    case 1:         
      right2stake();
      break;
    case 2:
      left2stake();
      break;
    case 3:
      blue_SAWP();
      break;
    case 4:
      red_SAWP();
      break;
    case 5:
      red4ring();
      break;
    case 6:
      skills();
      break;
    case 7:
      left_safe();
      break;
    case 8:
      right_safe();
      break;
    
 }
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



//Driver Toggles
bool mogo_state, doinker_state1, doinker_state2, intake_state, intake_raiser_state, lady_brown_state;

void usercontrol() {
  // User control code here, inside the loop
  //mogo_state = true;
Lady_Rotation.setPosition(0, degrees);

  while (1) {
    //colorSort(1);
    // intake roller and hooks
    if (Controller1.ButtonL1.pressing()) {
      if (!spinHooks) {
        IntakeFS.stop(hold);
        Hooks.stop(hold);
        wait(10, msec);
      } else {
        IntakeFS.spin(forward);
        Hooks.spin(forward);
      }
    } else if (Controller1.ButtonR1.pressing()) {
      if (!spinHooks) {
        IntakeFS.stop(hold);
        Hooks.stop(hold);
        wait(10, msec);
      } else {
        IntakeFS.spin(reverse);
        Hooks.spin(reverse);
      }
    } else {
      IntakeFS.stop();
      Hooks.stop();
    }
    
    // mogo clamp
    if (Controller1.ButtonB.pressing()) {
      mogo_state = !mogo_state;
      PMoGo.set(mogo_state);
      wait(200, msec);
    }
    
    // doinker
    if (Controller1.ButtonLeft.pressing()) {
      doinker_state1 = !doinker_state1;
      PDoinker.set(doinker_state1);
      wait(200, msec);
    }

    // doinker 2
    if (Controller1.ButtonRight.pressing()) {
      doinker_state2 = !doinker_state2;
      PDoinker2.set(doinker_state2);
      wait(200, msec);
    }
    
    // doinker 2 non-toggle
    if (Controller1.ButtonL2.pressing()) {
      PDoinker2.set(true);
    } else if (!Controller1.ButtonL2.pressing()) {
      PDoinker2.set(false);
    }

    // lady brown
    if (Controller1.ButtonR2.pressing()) {
      LB.spin(reverse);
    } else if (Controller1.ButtonX.pressing()) {
      LB.spin(forward);
    } else if (Controller1.ButtonY.pressing()) {
      ladybrownPID();
    } else if (!Controller1.ButtonR2.pressing() && !Controller1.ButtonL2.pressing()) {
      LB.stop();
      LB.setBrake(hold);
    }

    //Replace this line with chassis.control_tank(); for tank drive 
    //or chassis.control_holonomic(); for holo drive.
    chassis.control_arcade();

    wait(20, msec); // Sleep the task for a short amount of time to
                   
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
  while (1) {
    wait(100, msec);
  }
}
