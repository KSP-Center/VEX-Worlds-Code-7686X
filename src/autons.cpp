#include "vex.h"

/**
 * Resets the constants for auton movement.
 * Modify these to change the default behavior of functions like
 * drive_distance(). For explanations of the difference between
 * drive, heading, turning, and swinging, as well as the PID and
 * exit conditions, check the docs.
 */

void default_constants(){
  // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
  chassis.set_drive_constants(10, 1.5, 5, 7.5, 0);
  chassis.set_heading_constants(6, 0.3, 0, 0, 0);
  chassis.set_turn_constants(12, .45, .015, 5, 15);
  chassis.set_swing_constants(12, .45, .015, 5, 15);

  // Each exit condition set is in the form of (settle_error, settle_time, timeout).
  chassis.set_drive_exit_conditions(1.5, 300, 600);
  chassis.set_turn_exit_conditions(1, 300, 600);
  chassis.set_swing_exit_conditions(1, 300, 750);
}

/**
 * Sets constants to be more effective for odom movements.
 * For functions like drive_to_point(), it's often better to have
 * a slower max_voltage and greater settle_error than you would otherwise.
 */

void odom_constants(){
  default_constants();
  chassis.heading_max_voltage = 10;
  chassis.drive_max_voltage = 8;
  chassis.drive_settle_error = 3;
  chassis.boomerang_lead = .5;
  chassis.drive_min_voltage = 0;
}
bool ColorSortActive = true;
int blueTeamColorSortTask() {
  while(true) {
    if (ColorSortActive) {
      if (Color_Sort_Optical.isNearObject()) {
        if (Color_Sort_Optical.color() == red) {
          PColorSort.set(1);
          wait(350, msec);
          PColorSort.set(0);
          /*Hooks.setVelocity( 10, percent);
          wait (350, msec);
          Hooks.setVelocity(-100, percent);*/


        }
      } else {
        PColorSort.set(0);
      }
    }
    wait(2, msec);
  }
  return 0;
}
int redTeamColorSortTask() {
  while(true) {
    if (ColorSortActive) {
      if (Color_Sort_Optical.isNearObject()) {
        if (Color_Sort_Optical.color() == blue) {
          PColorSort.set(1);
          wait(350, msec);
          PColorSort.set(0);
          /*Hooks.setVelocity( 10, percent);
          wait (350, msec);
          Hooks.setVelocity(-100, percent);*/


        }
      } else {
        PColorSort.set(0);
      }
    }
    wait(2, msec);
  }
  return 0;
}
/*void colorSort(bool isRed) {
  if (isRed) {
    if (Color_Sort_Optical.color() > 0 && Color_Sort_Optical.color() < 10) {
      // do nothing
    } else {
      Hooks.stop(hold);
    }
  } else {
    // do nothing for now
  }
}*/

// Red Negative 1+4
void blue4ring(){
  chassis.drive_distance(-2);
  chassis.right_swing_to_angle(-40);
  LB.spinFor(-500, degrees);
  chassis.drive_distance(-5);
  LB.spinFor(600, degrees);
  chassis.right_swing_to_angle(-20);
  chassis.drive_distance(-17, -15, 8, 6);
  chassis.drive_distance(-14, -15, 5, 6);
  chassis.drive_stop(hold);
  PMoGo.set(1);
  chassis.turn_to_angle(135);
  chassis.drive_distance(9);
  IntakeFS.spin(reverse);
  Hooks.spin(reverse);
  chassis.drive_distance(26, 90, 7, 6);
  chassis.drive_distance(10, 90, 7, 6);
  chassis.drive_distance(-14);
  chassis.right_swing_to_angle(0);
  chassis.drive_distance(15);
  chassis.drive_stop(hold);
  wait(600, msec);
  chassis.turn_to_angle(-65);
  PIntake.set(1);
  chassis.drive_distance(25);
  chassis.drive_distance(17, -65, 7, 6);
  chassis.drive_stop(hold);
  wait(300, msec);
  PIntake.set(0);
  chassis.turn_to_angle(-180);
  chassis.drive_distance(10);
  chassis.drive_stop(hold);
  wait(300, msec);
  LB.spinFor(-500, degrees);
}

// Blue Negative 1+4 (experimental)
void red4ring() {
  chassis.drive_distance(-2);
  chassis.left_swing_to_angle(40);
  LB.spinFor(-500, degrees);
  chassis.drive_distance(-5);
  LB.spinFor(600, degrees);
  chassis.right_swing_to_angle(-20);
  chassis.drive_distance(-17, 15, 8, 6);
  chassis.drive_distance(-14, 15, 5, 6);
  chassis.drive_stop(hold);
  PMoGo.set(1);
  chassis.turn_to_angle(-135);
  chassis.drive_distance(9);
  IntakeFS.spin(reverse);
  Hooks.spin(reverse);
  chassis.drive_distance(26, -90, 7, 6);
  chassis.drive_distance(10, -90, 7, 6);
  chassis.drive_distance(-14);
  chassis.right_swing_to_angle(0);
  chassis.drive_distance(15);
  chassis.drive_stop(hold);
  wait(600, msec);
  chassis.turn_to_angle(65);
  PIntake.set(1);
  chassis.drive_distance(25);
  chassis.drive_distance(17, 65, 7, 6);
  chassis.drive_stop(hold);
  wait(300, msec);
  PIntake.set(0);
  chassis.turn_to_angle(180);
  chassis.drive_distance(10);
  chassis.drive_stop(hold);
  wait(300, msec);
  LB.spinFor(-500, degrees);
}

// Color Sort Test
void right2stake(){
  vex::task blueTeamColorSort(blueTeamColorSortTask);
  PMoGo.set(1);
  Hooks.setVelocity(500, rpm);
  Hooks.spin(reverse);
  IntakeFS.spin(reverse);
}

// Red Negative v2
void left2stake(){
}

// Red Goal Rush v2
void blue_SAWP(){
  vex::task redTeamColorSort(redTeamColorSortTask);
  IntakeFS.spin(reverse);
  chassis.drive_distance(40.25, -27, 10, 6, 1.5, 300, 900);
  chassis.drive_stop(hold);
  chassis.turn_to_angle(19);
  LB.spinFor(-600, degrees);
  chassis.drive_distance(3);
  chassis.turn_to_angle(120);
  LB.spinFor(-100, degrees);
  chassis.turn_to_angle(60);
  chassis.drive_distance(-18, 60, 6, 6);
  chassis.drive_stop(coast);
  wait(200, msec);
  PMoGo.set(1);
  wait(175, msec);
  Hooks.spin(reverse);
  wait(200, msec);
  LB.spinFor(825, degrees);
  Hooks.stop();
  IntakeFS.stop();
  chassis.turn_to_angle(110);
  PDoinker2.set(1);
  chassis.drive_distance(55, 135, 10, 6, 1.5, 300, 1200);
  chassis.turn_to_angle(30);
  PDoinker2.set(0);
  IntakeFS.spin(forward);
  wait(100, msec);
  IntakeFS.spin(reverse);
  Hooks.spin(reverse);
  chassis.drive_distance(30);
  chassis.drive_stop(hold);
}        

// Blue Goal Rush v2
void red_SAWP() {
vex::task blueTeamColorSort(blueTeamColorSortTask);
IntakeFS.spin(reverse);
chassis.drive_distance(39, 27, 10, 6, 1.5, 300, 900);
chassis.drive_stop(hold);
chassis.turn_to_angle(-21);
LB.spinFor(-600, degrees);
chassis.drive_distance(3);
chassis.turn_to_angle(-120);
LB.spinFor(-100, degrees);
chassis.turn_to_angle(-45);
chassis.drive_distance(-18, -65, 6, 6);
chassis.drive_stop(coast);
wait(200, msec);
PMoGo.set(1);
wait(175, msec);
Hooks.spin(reverse);
wait(200, msec);
LB.spinFor(825, degrees);
Hooks.stop();
IntakeFS.stop();
chassis.turn_to_angle(-110);
PDoinker.set(1);
chassis.drive_distance(55, -135, 10, 6, 1.5, 300, 1200);
chassis.turn_to_angle(-30);
PDoinker.set(0);
IntakeFS.spin(forward);
wait(100, msec);
IntakeFS.spin(reverse);
Hooks.spin(reverse);
chassis.drive_distance(30);
chassis.drive_stop(hold);

}

// Blue Goal Rush v1 (unchanged version) (we probably will not be using using this)
void left_safe(){
  Hooks.setVelocity(100, percent);
  PDoinker.set(1);
  IntakeFS.spin(reverse);
  chassis.turn_to_angle(20);
  chassis.drive_distance(33);
  wait(200, msec);
  PDoinker.set(0);
  wait(400, msec);
  chassis.drive_distance(-14);
  wait(200, msec);
  PDoinker.set(1);
  wait(200, msec);
  chassis.drive_distance(-3);
  PDoinker.set(0);
  chassis.turn_to_angle(190);
  chassis.drive_distance(-18, 190, 6, 6);
  wait(400, msec);
  PMoGo.set(1);
  wait(200, msec);
  chassis.drive_distance(9);
  chassis.drive_stop(hold);
  Hooks.spinFor(-1.5, rev);
  wait(200, msec);
  chassis.turn_to_angle(0);
  chassis.drive_distance(-10);
  wait(200, msec);
  PMoGo.set(0);
  IntakeFS.stop(coast);
  wait(400, msec);
  chassis.turn_to_angle(270);
  chassis.drive_distance(-15);
  chassis.drive_distance(-12, 270, 4, 6);
  wait(600, msec);
  PMoGo.set(1);
  chassis.drive_stop(hold);
  Hooks.spin(reverse);
  wait(500, msec);
  chassis.turn_to_angle(220);
  chassis.drive_distance(30);
}

// Blue Goal Rush v1
void right_safe(){
  Hooks.setVelocity(100, percent);
  PDoinker2.set(1);
  IntakeFS.spin(reverse);
  chassis.turn_to_angle(-20);
  chassis.drive_distance(33);
  wait(200, msec);
  PDoinker2.set(0);
  wait(400, msec);
  chassis.drive_distance(-14);
  wait(200, msec);
  PDoinker2.set(1);
  wait(200, msec);
  chassis.drive_distance(-3);
  PDoinker2.set(0);
  chassis.turn_to_angle(-190);
  chassis.drive_distance(-18, -190, 6, 6);
  wait(400, msec);
  PMoGo.set(1);
  wait(200, msec);
  chassis.drive_distance(9);
  chassis.drive_stop(hold);
  Hooks.spinFor(-1.5, rev);
  wait(200, msec);
  chassis.turn_to_angle(0);
  chassis.drive_distance(-10);
  wait(200, msec);
  PMoGo.set(0);
  IntakeFS.stop(coast);
  wait(400, msec);
  chassis.turn_to_angle(-270);
  chassis.drive_distance(-15);
  chassis.drive_distance(-12, -270, 4, 6);
  wait(600, msec);
  PMoGo.set(1);
  chassis.drive_stop(hold);
  Hooks.spin(reverse);
  wait(500, msec);
  chassis.turn_to_angle(-220);
  chassis.drive_distance(30);
}

// Skills
void skills() {

}
