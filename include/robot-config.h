using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor FLD;
extern motor MLD;
extern motor BLD;
extern motor FRD;
extern motor MRD;
extern motor BRD;
extern motor IntakeFS;
extern motor Hooks;
extern inertial Inertial1;
extern digital_out PDoinker;
extern digital_out PMoGo;
extern digital_out PIntake;
extern motor LB;
extern optical Color_Sort_Optical;
extern rotation Lady_Rotation;
extern rotation Vertical_Odom;
extern digital_out PDoinker2;
extern digital_out PColorSort;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );