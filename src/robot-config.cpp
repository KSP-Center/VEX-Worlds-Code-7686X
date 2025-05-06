#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor FLD = motor(PORT8, ratio6_1, false);
motor MLD = motor(PORT13, ratio6_1, true);
motor BLD = motor(PORT1, ratio6_1, true);
motor FRD = motor(PORT16, ratio6_1, true);
motor MRD = motor(PORT11, ratio6_1, false);
motor BRD = motor(PORT12, ratio6_1, false);
motor IntakeFS = motor(PORT4, ratio18_1, false);
motor Hooks = motor(PORT2, ratio6_1, true);
inertial Inertial1 = inertial(PORT14);
digital_out PDoinker = digital_out(Brain.ThreeWirePort.F);
digital_out PMoGo = digital_out(Brain.ThreeWirePort.H);
digital_out PIntake = digital_out(Brain.ThreeWirePort.E);
motor LB = motor(PORT7, ratio18_1, false);
optical Color_Sort_Optical = optical(PORT10);
rotation Lady_Rotation = rotation(PORT21, false);
rotation Vertical_Odom = rotation(PORT9, false);
digital_out PDoinker2 = digital_out(Brain.ThreeWirePort.G);
digital_out PColorSort = digital_out(Brain.ThreeWirePort.D);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}