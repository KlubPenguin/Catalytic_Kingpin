#include "main.h"


/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

// Chassis constructor
Drive chassis (
  // Left Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  {-6, 4, -2}

  // Right Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  ,{16, -14, 13}

  // IMU Port
  ,21

  // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
  //    (or tracking wheel diameter)
  ,2.75

  // Cartridge RPM
  //   (or tick per rotation if using tracking wheels)
  ,360

  // External Gear Ratio (MUST BE DECIMAL)
  //    (or gear ratio of tracking wheel)
  // eg. if your drive is 84:36 where the 36t is powered, your RATIO would be 2.333.
  // eg. if your drive is 36:60 where the 60t is powered, your RATIO would be 0.6.
  ,1.0


  // Uncomment if using tracking wheels
  
  // Left Tracking Wheel Ports (negative port will reverse it!)
   ,{1, 2} // 3 wire encoder
  // ,8 // Rotation sensor

  // Right Tracking Wheel Ports (negative port will reverse it!)
   ,{1, 2} // 3 wire encoder
  // ,-9 // Rotation sensor
  

  // Uncomment if tracking wheels are plugged into a 3 wire expander
  // 3 Wire Port Expander Smart Port
  //,1
);




/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
//D BACK RIGHT, FRONT RIGHT G, LEFT FRONT C, BACK LEFT D, HANG F
  pros::ADIDigitalOut leftWing('C');
  bool leftWingLock = 0;
  int leftWingMode = 0;

  pros::ADIDigitalOut rightWing('H');
  bool rightWingLock = 0;
  int rightWingMode = 0;

  pros::ADIDigitalOut backLeftWing('D');
  bool backLeftWingLock = 0;
  int backLeftWingMode = 0;

  pros::ADIDigitalOut backRightWing('G');
  bool backRightWingLock = 0;
  int backRightWingMode = 0;

  pros::Motor catapult(-1);
  int cataStart = 0;
  pros::Rotation cataRotation(12);
  pros::Distance cataDistance(17);
  int autoLock = 0;
  int semiAutoLock = 0;
  int setFireLock = 0;
  int cataShots = 0;
  int cataFired = 0;
  float cataReference = 0;
  std::string catapultMode = "loading";

void imuPrinter(){
while(1){
 if(pros::competition::is_disabled){
 
pros::lcd::print(2, "Catapult: %d",  cataRotation.get_angle()/100);

pros::lcd::print(6, "IMU: %f", chassis.get_gyro());
 }
pros::delay(20);
}
}

void initialize() {
  
  ez::print_ez_template();
  
  pros::delay(500); // Stop the user from doing anything while legacy ports configure.
  chassis.toggle_auto_print(true);
  // Configure your chassis controls
  chassis.toggle_modify_curve_with_controller(true); // Enables modifying the controller curve with buttons on the joysticks
  chassis.set_active_brake(0); // Sets the active brake kP. We recommend 0.1.
  chassis.set_curve_default(0, 0); // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)  
  //default_constants(); // Set the drive to your own constants from autons.cpp!

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.set_left_curve_buttons (pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT); // If using tank, only the left side is used. 
  // chassis.set_right_curve_buttons(pros::E_CONTROLLER_DIGITAL_Y,    pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  
  
  /*
  ez::as::auton_selector.add_autons({
    //Auton("Example Drive\n\nDrive forward and come back.", drive_example),
  });
  */
 
 ez::as::auton_selector.add_autons({
    Auton("Loading Side Win Point: 155", wpLoadingSide),
    Auton("Far Side Six Ball: -90", far6ball3),
    Auton("Skills: -135", skills),
  });
  
  // Initialize chassis and auton selector
  chassis.initialize();

  ez::as::initialize();


  chassis.imu_calibrate();
pros::Task imuPrint(imuPrinter);
}
//
/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {

   
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
int hangClock = 0;
int cataClock = 0;
void clockFXN(int delay){
pros::delay(delay); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
cataClock = cataClock + delay;
hangClock = hangClock + delay;
}

void autonomous() {

  pros::lcd::print(4, "Started");
   // Resets PID targets to 0
  chassis.reset_gyro(); // Reset gyro position to 0
  chassis.reset_drive_sensor(); // Reset drive sensors to 0
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps autonomous consistency.

  ez::as::auton_selector.call_selected_auton(); // Calls selected auton from autonomous selector.
  //pros::Task clocks(clockFXN(20));  
  //drive_example();





}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */





  pros::ADIDigitalOut hang('F');
  bool hangLock = 0;
  bool hangMode = 0;
//true is up
  int hangStage = 1;
void hangFXN(int delay){
/*if(((master.get_digital(DIGITAL_Y)) || (hangStage == 2)) && (hangLock == 0)){

if(hangStage == 1){
hang.set_value(false);
hangClock = 0;
hangStage = 2;


} else if ((hangStage == 2)  && (hangClock > delay)){
hang.set_value(true);
hangStage = 1;
hangLock = 1;
} 
}


if ((!master.get_digital(DIGITAL_Y)) && (hangLock == 1)){
hangLock = 0;

}

*/
/*
if((master.get_digital(DIGITAL_Y)) && (hangLock) == 0 && (hangMode == 0)){
hangMode = 1;
hang.set_value(true);
} else if (master.get_digital(DIGITAL_Y) && (hangLock == 0) && (hangMode == 1)){
hangMode = 0;
hang.set_value(false);
}

if(master.get_digital(DIGITAL_Y)){
hangLock = 1;
} else{
hangLock = 0;
}



if ((!master.get_digital(DIGITAL_Y)) && (hangLock == 1)){
hangLock = 0;

}
*/
}
















pros::Motor intake(5);
bool downIntakeLock = 0;
bool upIntakeLock = 0;
int intakeSpeed = 0;
void intakeFXN(void){
if(master.get_digital(DIGITAL_L2)){
intakeSpeed = -1;
master.print(1, 0, "Intake: Outaking"); 
} else if (master.get_digital(DIGITAL_L1)){
intakeSpeed = 1;
master.print(1, 0, "Intake: Intaking"); 
} else {
intakeSpeed = 0;
master.print(2, 0, "Intake: Coasting"); 
}
intake = 127 * intakeSpeed;
}


int latVolts = 0;
int turnVolts = 0;

  pros::Motor frontLeft(-6);
  pros::Motor midLeft(4);
  pros::Motor backLeft(-2);
  pros::Motor_Group leftMotors({frontLeft, midLeft, backLeft});

  pros::Motor frontRight(16);
  pros::Motor midRight(-14);
  pros::Motor backRight(13);
  pros::Motor_Group rightMotors({frontRight, midRight, backRight});

int rightSpeed;
int leftSpeed;
int maxDrive;
string drivePreset = "normal";
string driveState = "inactive";





void opcontrol() {







 /* 


master.clear();
master.clear_line(1);
master.clear_line(2);
wings.set_value(false);

int latVolts = 0;
int turnVolts = 0;

  pros::Motor frontLeft(-11);
  pros::Motor midLeft(-14);
  pros::Motor backLeft(-17);
  pros::Motor_Group leftMotors({frontLeft, midLeft, backLeft});

  pros::Motor frontRight(16);
  pros::Motor midRight(18);
  pros::Motor backRight(20);
  pros::Motor_Group rightMotors({frontRight, midRight, backRight});
  */
catapultMode = "loading";
leftWing.set_value(false);
rightWing.set_value(false);
hangDown();
	while (true) {

if(catapultMode == "setfire"){
master.print(1, 0, "Shots Left: %d", cataShots - cataFired, "      ");
} else {
   master.print(1, 0, "Catapult: %s", catapultMode, "       "); 
}
/*

if((abs(master.get_analog(ANALOG_LEFT_Y)) > 12) && (abs(master.get_analog(ANALOG_RIGHT_X)) < 12)){
latVolts = master.get_analog(ANALOG_LEFT_Y);
turnVolts = 0;
} else if ((abs(master.get_analog(ANALOG_LEFT_Y)) > 12) && (abs(master.get_analog(ANALOG_RIGHT_X)) > 12)){
latVolts = master.get_analog(ANALOG_LEFT_Y);
turnVolts = master.get_analog(ANALOG_RIGHT_X);
} else if((abs(master.get_analog(ANALOG_LEFT_Y)) < 12) && (abs(master.get_analog(ANALOG_RIGHT_X)) > 12)){
turnVolts = master.get_analog(ANALOG_RIGHT_X);
latVolts = 0;
} else{
turnVolts = 0;
latVolts = 0;
}

rightMotors = latVolts - turnVolts;
leftMotors = latVolts + turnVolts;
*/

/*
if(master.get_digital(DIGITAL_R2)){
downIntakeLock = 1;
} else if (!master.get_digital(DIGITAL_R2)){
downIntakeLock = 0;
}

if(master.get_digital(DIGITAL_R1)){
upIntakeLock = 1;
} else if (!master.get_digital(DIGITAL_R1)){
upIntakeLock = 0;
}

if(master.get_digital(DIGITAL_R2) && (downIntakeLock == 0) && (intakeSpeed != 1)){
intakeSpeed = 1;
} else if(master.get_digital(DIGITAL_R1) && (upIntakeLock == 0) && (intakeSpeed != -1)){
intakeSpeed = -1;
} else if(master.get_digital(DIGITAL_R2) && (upIntakeLock == 0) && (intakeSpeed == 1)){
intakeSpeed = 0;
} else if(master.get_digital(DIGITAL_R1) && (downIntakeLock == 0) && (intakeSpeed == -1)){
intakeSpeed = 0;
}
*/

if((master.get_digital(DIGITAL_X)) && (hangLock) == 0 && (hangMode == 0)){
hangMode = 1;
hang.set_value(true);
} else if ((master.get_digital(DIGITAL_X)) && (hangLock == 0) && (hangMode == 1)){
hangMode = 0;
hang.set_value(false);
}

if(master.get_digital(DIGITAL_X)){
hangLock = 1;
} else{

hangLock = 0;
}


if((abs(master.get_analog(ANALOG_LEFT_Y)) > 12) && (abs(master.get_analog(ANALOG_RIGHT_X)) < 12)){
latVolts = master.get_analog(ANALOG_LEFT_Y);
turnVolts = 0;
} else if ((abs(master.get_analog(ANALOG_LEFT_Y)) > 12) && (abs(master.get_analog(ANALOG_RIGHT_X)) > 12)){
latVolts = master.get_analog(ANALOG_LEFT_Y);
turnVolts = master.get_analog(ANALOG_RIGHT_X);
} else if((abs(master.get_analog(ANALOG_LEFT_Y)) < 12) && (abs(master.get_analog(ANALOG_RIGHT_X)) > 12)){
turnVolts = master.get_analog(ANALOG_RIGHT_X);
latVolts = 0;
} else{
turnVolts = 0;
latVolts = 0;
}




rightSpeed = latVolts - turnVolts;
leftSpeed = latVolts + turnVolts;
maxDrive = max(abs(rightSpeed), abs(leftSpeed))/127;


if(max(abs(rightSpeed), abs(leftSpeed)) > 127){
rightMotors = rightSpeed/maxDrive;
leftMotors = leftSpeed/maxDrive;
} else {
rightMotors = rightSpeed;
leftMotors = leftSpeed;
}





//LEFT WING 

if(master.get_digital(DIGITAL_B) && (leftWingLock) == 0 && (leftWingMode == 0)){
leftWingMode = 1;
leftWing.set_value(true);
} else if (master.get_digital(DIGITAL_B) && (leftWingLock == 0) && (leftWingMode == 1)){
leftWingMode = 0;
leftWing.set_value(false);
}

if(master.get_digital(DIGITAL_B)){
leftWingLock = 1;
} else{
leftWingLock = 0;
}


//RIGHT WING
if(master.get_digital(DIGITAL_Y)&& (rightWingLock) == 0 && (rightWingMode == 0)){
rightWingMode = 1;
rightWing.set_value(true);
} else if (master.get_digital(DIGITAL_Y)  && (rightWingLock == 0) && (rightWingMode == 1)){
rightWingMode = 0;
rightWing.set_value(false);
}

if(master.get_digital(DIGITAL_Y)){
rightWingLock = 1;
} else{
rightWingLock = 0;
}



//LEFT WING 

if(master.get_digital(DIGITAL_DOWN) && (backLeftWingLock) == 0 && (backLeftWingMode == 0)){
backLeftWingMode = 1;
backLeftWing.set_value(true);
} else if (master.get_digital(DIGITAL_DOWN) && (backLeftWingLock == 0) && (backLeftWingMode == 1)){
backLeftWingMode = 0;
backLeftWing.set_value(false);
}

if(master.get_digital(DIGITAL_DOWN)){
backLeftWingLock = 1;
} else{
backLeftWingLock = 0;
}


//RIGHT WING
if(master.get_digital(DIGITAL_RIGHT)&& (backRightWingLock) == 0 && (backRightWingMode == 0)){
backRightWingMode = 1;
backRightWing.set_value(true);
} else if (master.get_digital(DIGITAL_RIGHT)  && (backRightWingLock == 0) && (backRightWingMode == 1)){
backRightWingMode = 0;
backRightWing.set_value(false);
}

if(master.get_digital(DIGITAL_RIGHT)){
backRightWingLock = 1;
} else{
backRightWingLock = 0;
}

intakeFXN();


//AT START OF MATCH CHECKS FOR CATAPULT


if(cataStart == 0){
 
if(cataRotation.get_angle()/100 >= 80){
catapultMode = "resetting";
}else{

catapultMode = "loading";
cataStart = 1;
}} else if (cataStart == 1){
/* if(catapultMode == "loading" && (master.get_digital(DIGITAL_R2))){
catapultMode = "resetting";
} else */ if ((catapultMode == "resetting")  && ((cataRotation.get_angle()/100) < 80) && (cataClock > 200)) {
catapultMode = "loading";
} /*else if (catapultMode == "blocking" && master.get_digital(DIGITAL_L2)){
catapultMode = "lowering";
}  else if (catapultMode == "lowering" && (cataRotation.get_angle()/100) <= 263 && (cataRotation.get_angle()/100) < 268){
catapultMode = "loading";
}*/ else if (master.get_digital(DIGITAL_R1) && (autoLock == 0)){
cataStart = 2;
autoLock = 1;
} else if (master.get_digital(DIGITAL_R2) && (semiAutoLock == 0)){
cataStart = 3;
semiAutoLock = 1;
catapultMode = "waiting";
} 
/* else if (master.get_digital(DIGITAL_UP) && (setFireLock == 0)){
cataStart = 5;
setFireLock = 1;
catapultMode = "autofire";
cataShots = 1;
cataFired = 0;
}  */ 

} else if(cataStart == 2){
catapultMode = "autofire";
if(master.get_digital(DIGITAL_R1) && (autoLock == 0)){
cataStart = 0;
autoLock = 1;
}

} else if (cataStart == 3){
if ((catapultMode == "distancefire") && ( 75 >= (cataRotation.get_angle()/100)) &&  (cataClock > 200)){
cataClock = 0;
catapultMode = "waiting";

} else if ((catapultMode == "waiting") && (cataDistance.get() < 50)){
catapultMode = "distancefire";
cataClock = 0;
} 



if(master.get_digital(DIGITAL_R2) && (semiAutoLock == 0)){
cataStart = 0;
semiAutoLock = 1;
}

if(master.get_digital(DIGITAL_R2) && (semiAutoLock == 0)){
cataStart = 0;
semiAutoLock = 1;
}

}/* else if (cataStart == 5){

if(cataShots > cataFired){
if((cataClock >= 300) && (cataRotation.get_angle()/100) >= 115){
cataClock = 0;
cataFired++;
} 

catapultMode = "setfire";


} else {
cataStart = 0;
catapultMode = "waiting";  
}

if(master.get_digital(DIGITAL_Y) && (setFireLock == 0)){
cataStart = 0;
setFireLock = 1;
}

if(master.get_digital(DIGITAL_UP) && (setFireLock == 0)){
cataShots++;
setFireLock = 1;
} 


} */





  
//SCRAPPED CATA FXN
/*else if (cataStart == 4){
if ((catapultMode == "distancefire") && ( 230 > (cataRotation.get_angle()/100)) &&  (cataClock > 400)){
cataClock = 0;
catapultMode = "waiting";

} else if ((catapultMode == "waiting") && (cataClock > 3000)){
cataStart = 0;
} 
  
} */

if(!master.get_digital(DIGITAL_R1)){
autoLock = 0;
}


if(!master.get_digital(DIGITAL_R2)){
semiAutoLock = 0;
}


if(master.get_digital(DIGITAL_R2)){
cataClock = 0;
}



if(!master.get_digital(DIGITAL_Y) && !master.get_digital(DIGITAL_UP)){
setFireLock = 0;
}


if(catapultMode == "resetting"){
catapult = 90;
} else if (catapultMode == "loading" || catapultMode == "blocking" || catapultMode == "waiting"){
catapult = 0;
} else if (catapultMode == "lowering"){
catapult = 89;
} else if (catapultMode == "setfire"  || catapultMode == "distancefire"){
catapult = 127;
} else if (catapultMode == "autofire"){
catapult = 114;
} 
    pros::lcd::print(1, "Catapult Temp: %f",  ((1.8 * catapult.get_temperature()) + 32));

    pros::lcd::print(2,"Catapult Shots: %d",  cataShots);

    pros::lcd::print(3, "Cata Angle %d", cataRotation.get_angle()/100);
    
		pros::lcd::print(4, "Distance: %d", cataDistance.get()); 

		pros::lcd::print(5, "Mode: %s", catapultMode);      

    pros::lcd::print(7, "Cata Clock: %d", cataClock);

   



clockFXN(20);

	}
}
