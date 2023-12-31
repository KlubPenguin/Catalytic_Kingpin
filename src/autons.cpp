#include "main.h"
#include "autons.hpp"





///
// Constants
///

// It's best practice to tune constants when the robot is empty and with heavier game objects, or with lifts up vs down.
// If the objects are light or the cog doesn't change much, then there isn't a concern here.

void hang_constants(){
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 15, 0, 45, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.7, 0, 5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.7, 0, 5, 0);
  chassis.set_pid_constants(&chassis.turnPID, 8, 0.01, 60, 15);
  chassis.set_pid_constants(&chassis.swingPID, 3, 0, 10, 0);


}

void default_constants() {
  
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 9, 0, 50, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.4, 0, 5.5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.4, 0, 5.5, 0);
  chassis.set_pid_constants(&chassis.turnPID, 4.4, 0.003, 30, 15);
  chassis.set_pid_constants(&chassis.swingPID, 3.2, 0, 11, 0);

  
}

void small_error_constants(){
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 9, 0, 50, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 1.2, 0.01, 6, 20);
  chassis.set_pid_constants(&chassis.backward_drivePID, 1.2, 0.01, 6, 20);
  chassis.set_pid_constants(&chassis.turnPID, 8, 0.01, 60, 15);
  chassis.set_pid_constants(&chassis.swingPID, 8, 0, 9, 0);


}

void modified_exit_conditions(){
  chassis.set_exit_condition(chassis.turn_exit,  100, 60,  100, 7,   50, 100);
  chassis.set_exit_condition(chassis.swing_exit, 100, 3,  500, 7,   500, 500);
  chassis.set_exit_condition(chassis.drive_exit, 100,  50, 300, 150, 500, 500);

}

void ram_conditions(){
  chassis.set_exit_condition(chassis.turn_exit,  100, 60,  100, 7,   50, 100);
  chassis.set_exit_condition(chassis.swing_exit, 100, 3,  500, 7,   500, 500);
  chassis.set_exit_condition(chassis.drive_exit, 100,  24, 100, 150, 100, 300);

}

void odom(double wheelSize){

  const int ENCODER_TICKS = 360;

  const double pi = 3.14159265358979323846;

  pros::ADIEncoder trackingWheel('A', 'B', false);

  pros::Imu inertial(21);

  double x = 0;
  double y = 0;
  double angle = 0;



  double sectorLength, initialAngle, finalAngle, 
    deltaTheta, avgAngle, lineSectorRatio, absLength;

  while(1){
    //RESET SENSORS
    trackingWheel.reset();

    initialAngle = (double) (180 * inertial.get_rotation()/pi);

    pros::delay(5);  
    //FINAL SENSOR VALUES
    finalAngle = (double) (180 * inertial.get_rotation()/pi);

    //ANGULAR DIFFERENCE 
    deltaTheta = finalAngle - initialAngle;
    avgAngle = (finalAngle + initialAngle)/2.0;

    //SECTOR LENGTH
    sectorLength = (double)  (wheelSize * pi * ((double) (trackingWheel.get_value())/ENCODER_TICKS));

    //LINE SECTOR RATIO
    if(deltaTheta != 0){
      lineSectorRatio = (pow(2 - 2 * cos(deltaTheta), 0.5)/deltaTheta);
    } else {
      lineSectorRatio = 1;
    }
    //ABSOLUTE DISTANCE
    absLength = lineSectorRatio * sectorLength;

    //POSITION  
    y +=  absLength * sin(avgAngle) ;
    x +=  absLength * cos(avgAngle);
    angle = inertial.get_rotation();

    pros::lcd::print(1, "X:%f", x); 
    pros::lcd::print(2, "Y:%f", y); 
    pros::lcd::print(3, "Angle:%f", angle);
    pros::lcd::print(4, "Enocder:%d", trackingWheel.get_value());  
    pros::lcd::print(5, "Delta Theta:%f", deltaTheta);  
    
  }
}

void waitTimes(){
  int totalTime = 0;
  int waitingTime = 0;
  while(1){
    if(abs(chassis.right_velocity()) <= 25 && abs(chassis.left_velocity()) <= 25){
      waitingTime += 20;
    }
    totalTime += 20;
    pros::lcd::print(7, "Pct Waiting:%f", static_cast<double>(waitingTime)); 
    pros::delay(20);
    
  }
}

void imuChecker(){
double maxLeft = 0;
double maxRight = 0;
while(1){
if(chassis.get_gyro() > maxRight){
maxRight = chassis.get_gyro();
} else {
maxRight = maxRight;
}


if(chassis.get_gyro() < maxLeft){
maxLeft = chassis.get_gyro();
} else {
maxLeft = maxLeft;
}

pros::lcd::print(6, "IMU: %f", chassis.get_gyro());
pros::lcd::print(4, "Left Veer:%f", maxLeft);
pros::lcd::print(5, "Right Veer:%f", maxRight);
pros::lcd::print(3, "Distance: %f", (chassis.right_sensor() / chassis.get_tick_per_inch()));
pros::delay(20);
}
}







void intakeFWDSpin(){
pros::Motor intake(-5);
while(1){
if(abs((chassis.right_velocity())) >= 5 || abs(chassis.left_velocity()) >= 5){
 intake = 127; 
} else {
  intake = 0;
}
pros::delay(20);
}
}

int hangUp(){
pros::ADIDigitalOut hang('E');
hang.set_value(true);
return(0);
}


int hangDown(){
pros::ADIDigitalOut hang('E');
hang.set_value(false);
return(0);
}


int wingsIn(){
pros::ADIDigitalOut leftWing('C');
pros::ADIDigitalOut rightWing('H');
leftWing.set_value(false);
rightWing.set_value(false);
return(0);
}

int rightWingIn(){
pros::ADIDigitalOut rightWing('H');
rightWing.set_value(false);
return(0);  
}

int leftWingIn(){
pros::ADIDigitalOut leftWing('C');
leftWing.set_value(false);
return(0);  
}

int wingsOut(){
pros::ADIDigitalOut leftWing('C');
pros::ADIDigitalOut rightWing('H');
leftWing.set_value(true);
rightWing.set_value(true);
return(0);
}

int rightWingOut(){
pros::ADIDigitalOut rightWing('H');
rightWing.set_value(true);
return(0);  
}

int leftWingOut(){
pros::ADIDigitalOut leftWing('C');
leftWing.set_value(true);
return(0);  
}

int backWingsIn(){
pros::ADIDigitalOut backLeftWing('D');
pros::ADIDigitalOut rightWing('G');
backLeftWing.set_value(false);
rightWing.set_value(false);
return(0);
}

int backRightWingIn(){
pros::ADIDigitalOut rightWing('G');
rightWing.set_value(false);
return(0);  
}

int backLeftWingIn(){
pros::ADIDigitalOut backLeftWing('D');
backLeftWing.set_value(false);
return(0);  
}

int backWingsOut(){
pros::ADIDigitalOut backLeftWing('D');
pros::ADIDigitalOut rightWing('G');
backLeftWing.set_value(true);
rightWing.set_value(true);
return(0);
}

int backRightWingOut(){
pros::ADIDigitalOut rightWing('G');
rightWing.set_value(true);
return(0);  
}

int backLeftWingOut(){
pros::ADIDigitalOut backLeftWing('D');
backLeftWing.set_value(true);
return(0);  
}

void intakeBACKSpin(){
pros::Motor intake(-5);
while(1){
if(abs((chassis.right_velocity())) >= 40 || abs(chassis.left_velocity()) >= 40){
 intake = -127; 
} else {
  intake = 0;
}
pros::delay(20);
}
}


void intakeSpin(int distance, string direction){
pros::Motor intake(-5);
intake.set_zero_position(0);
while(abs(intake.get_position()) < distance){
if(direction == "fwd"){
intake = 127;
} else if(direction == "back"){
intake = -127;
}
}
pros::lcd::print(4, "Intake:%f", intake.get_position()); 

intake.set_zero_position(0);
intake = 0;

}


int catapultLower(){
pros::Motor catapult(-1);
pros::Rotation cataRotation(12);
int clock = 0;
pros::lcd::print(6, "Clock: %d",  clock);
pros::lcd::print(5, "Catapult: %d",  cataRotation.get_angle()/100);
pros::delay(100);
while((cataRotation.get_angle()/100) > 85){
catapult = 110;




pros::lcd::print(6, "Clock: %d",  clock);
pros::lcd::print(5, "Catapult: %d",  cataRotation.get_angle()/100);
pros::delay(20);
clock += 20;
}
catapult = 0;
return(0);
}

void catapultFire(int shots){
pros::Distance cataDistance(17);
pros::Motor catapult(-1);
pros::Rotation cataRotation(12);
int fired = 0;
int clock = 420;
string catapultSetting = "delay";
int skillsCataTime = 0;
while(shots > fired){
if (( 85 >= (cataRotation.get_angle()/100)) &&  (skillsCataTime > 250) && (catapultSetting == "fire")){
skillsCataTime = 0;
catapult = 0;
catapultSetting = "delay";
fired++;

} else if ((cataDistance.get() < 50) && (catapultSetting == "delay")){
catapult = 127;
skillsCataTime = 0;
catapultSetting = "fire";
} 

pros::delay(20);
skillsCataTime += 20;

}
catapult = 0;
}

void catapultInfiniteFire(){
pros::Motor catapult(-1);

while(1){
catapult = 90;
}
}






void farSide(){

wingsIn();
pros::lcd::print(4, "Started");
hang_constants();
chassis.set_angle(45);

catapultLower(); 
pros::delay(1000);



pros::Task intakeKeep(intakeFWDSpin);


wingsOut();

chassis.set_drive_pid(12, DRIVE_SPEED, true);
chassis.wait_drive();

chassis.set_drive_pid(-8, DRIVE_SPEED, true);
chassis.wait_drive();

wingsIn();

chassis.set_drive_pid(16, DRIVE_SPEED, true);
chassis.wait_drive();

intakeKeep.suspend();

pros::Task intakeRev(intakeBACKSpin);

chassis.set_swing_pid(RIGHT_SWING, 0, SWING_SPEED);
chassis.wait_drive();

chassis.set_drive_pid(10, DRIVE_SPEED, true);
chassis.wait_drive();

chassis.set_drive_pid(-10, DRIVE_SPEED, true);
chassis.wait_drive();

chassis.set_drive_pid(12, DRIVE_SPEED, true);
chassis.wait_drive();

intakeRev.suspend();



chassis.set_drive_pid(-11, DRIVE_SPEED, true);
chassis.wait_drive();

chassis.set_turn_pid(-69, TURN_SPEED);
chassis.wait_drive();

chassis.set_drive_pid(40, DRIVE_SPEED, true);
chassis.wait_drive();



chassis.set_swing_pid(RIGHT_SWING, -90, SWING_SPEED);
chassis.wait_drive();

pros::Task intakeTake(intakeFWDSpin);


chassis.set_drive_pid(15, DRIVE_SPEED, true);
chassis.wait_drive();


chassis.set_drive_pid(-5, DRIVE_SPEED, true);
chassis.wait_drive();

intakeTake.suspend();
intakeSpin(0, "back");

chassis.set_turn_pid(45, TURN_SPEED);
chassis.wait_drive();

chassis.set_drive_pid(20, DRIVE_SPEED, true);
chassis.wait_drive();

chassis.set_turn_pid(90, TURN_SPEED);
chassis.wait_drive();



pros::Task intakeRev2(intakeBACKSpin);

pros::delay(250);

chassis.set_drive_pid(17, DRIVE_SPEED, true);
chassis.wait_drive();

intakeRev2.suspend();
intakeSpin(0, "back");

chassis.set_drive_pid(-5, DRIVE_SPEED, true);
chassis.wait_drive();

/*
chassis.set_swing_pid(LEFT_SWING, 60, SWING_SPEED);
chassis.wait_drive();

chassis.set_drive_pid(2, DRIVE_SPEED, true);
chassis.wait_drive();

chassis.set_swing_pid(RIGHT_SWING, 45, SWING_SPEED);
chassis.wait_drive();

chassis.set_drive_pid(2, DRIVE_SPEED, true);
chassis.wait_drive();

chassis.set_swing_pid(RIGHT_SWING, 5, SWING_SPEED);
chassis.wait_drive();


*/






pros::lcd::print(4, "Finished 3-Ball");

}

void skillsPractice(){

wingsIn();
modified_exit_conditions();
pros::lcd::print(4, "Started");
default_constants();
chassis.set_angle(-45);


pros::Task intakeIn(intakeFWDSpin);
chassis.set_drive_pid(-15, DRIVE_SPEED, true);
chassis.wait_drive();
intakeIn.suspend();
intakeSpin(0, "fwd");

chassis.set_swing_pid(LEFT_SWING, -90, SWING_SPEED);
chassis.wait_drive();


chassis.set_drive_pid(-17, 127, true);
chassis.wait_drive();

small_error_constants();
chassis.set_drive_pid(15, DRIVE_SPEED, true);
chassis.wait_drive();


chassis.set_turn_pid(23, TURN_SPEED);
chassis.wait_drive();



leftWingOut();

catapultLower(); 
catapultFire(44);

/*

catapultLower();

rightWingIn();

//DRIVE ACROSS FIELD

chassis.set_drive_pid(-2, DRIVE_SPEED, true);
chassis.wait_drive();

chassis.set_turn_pid(32, TURN_SPEED);
chassis.wait_drive();
*/
}

void skills(){


skillsPractice();

pros::delay(500);

leftWingIn();



//EXIT LOADING ZONE
chassis.set_drive_pid(10, DRIVE_SPEED, true);
chassis.wait_drive();

chassis.set_turn_pid(-45, TURN_SPEED);
chassis.wait_drive();

default_constants();
chassis.set_drive_pid(28, DRIVE_SPEED, true);
chassis.wait_drive();


//DRIVE TO OTHER SIDE
pros::Task intakeOut(intakeBACKSpin);
chassis.set_swing_pid(LEFT_SWING, 0, SWING_SPEED);
chassis.wait_drive();

chassis.set_drive_pid(80, DRIVE_SPEED, true);
chassis.wait_drive();

chassis.set_turn_pid(-90, TURN_SPEED);
chassis.wait_drive();

chassis.set_drive_pid(-24, DRIVE_SPEED, true);
chassis.wait_drive();


//PREPARE FOR LEFT SIDE PUSH
chassis.set_turn_pid(0, TURN_SPEED);
chassis.wait_drive();

chassis.set_drive_pid(-28, DRIVE_SPEED, true);
chassis.wait_drive();

chassis.set_turn_pid(-130, TURN_SPEED);
chassis.wait_drive();

//PUSH LEFT SIDE
leftWingOut();

chassis.set_drive_pid(-40, DRIVE_SPEED, true);
chassis.wait_drive();

chassis.set_drive_pid(15, DRIVE_SPEED, true);
chassis.wait_drive();

chassis.set_drive_pid(-30, DRIVE_SPEED, true);
chassis.wait_drive();

chassis.set_drive_pid(15, DRIVE_SPEED, true);
chassis.wait_drive();
leftWingIn();
//HEAD TO RIGHT SIDE
chassis.set_turn_pid(-180, TURN_SPEED);
chassis.wait_drive();


chassis.set_drive_pid(23, DRIVE_SPEED, true);
chassis.wait_drive();

chassis.set_turn_pid(-270, TURN_SPEED);
chassis.wait_drive();

chassis.set_drive_pid(48, DRIVE_SPEED, true);
chassis.wait_drive();

chassis.set_turn_pid(-210, TURN_SPEED);
chassis.wait_drive();

//PLOW RIGHT SIDE
wingsOut();
chassis.set_drive_pid(-30, DRIVE_SPEED, true);
chassis.wait_drive();

chassis.set_drive_pid(15, DRIVE_SPEED, true);
chassis.wait_drive();

chassis.set_drive_pid(-20, DRIVE_SPEED, true);
chassis.wait_drive();

chassis.set_drive_pid(15, DRIVE_SPEED, true);
chassis.wait_drive();
wingsIn;
//EXIT RIGHT SIDE

chassis.set_swing_pid(LEFT_SWING, -165, SWING_SPEED);
chassis.wait_drive();

chassis.set_drive_pid(20, DRIVE_SPEED, true);
chassis.wait_drive();

chassis.set_turn_pid(-180, TURN_SPEED);
chassis.wait_drive();

wingsOut();
chassis.set_drive_pid(-40, DRIVE_SPEED, true);
chassis.wait_drive();

chassis.set_drive_pid(20, DRIVE_SPEED, true);
chassis.wait_drive();

intakeOut.suspend();
intakeSpin(0, "fwd");
//THE REST OF SKILLS

pros::lcd::print(4, "Finished Skills");
}


void far6ball(){

  //START INTAKING, hang DOWN, TO THE LEFT
  default_constants();
  chassis.set_angle(-90);
  pros::Task intakeKeep(intakeFWDSpin);


  //GRAB BORDER TRIBALL
  chassis.set_drive_pid(5, DRIVE_SPEED, true);
  chassis.wait_drive();


  //PUSH ALLY BALL
  chassis.set_drive_pid(-42, DRIVE_SPEED, true);
  chassis.wait_drive();
  intakeKeep.suspend();

  //TAKE LOADING BALL
  chassis.set_swing_pid(LEFT_SWING, -135, SWING_SPEED);
  chassis.wait_drive();
  leftWingOut();

  chassis.set_drive_pid(-10, DRIVE_SPEED, true);
  chassis.wait_drive();
  leftWingIn();

  chassis.set_drive_pid(-5, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_swing_pid(LEFT_SWING, -170, SWING_SPEED);
  chassis.wait_drive();

  //RAM TWO BALLS IN GOAL
  chassis.set_drive_pid(-19, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_drive_pid(15, DRIVE_SPEED, true);
  chassis.wait_drive();

  //PLACE FINAL BALL

  chassis.set_turn_pid(10, TURN_SPEED);
  chassis.wait_drive();

  intakeSpin(50, "back");
  pros::Task intakeOut(intakeBACKSpin);
  chassis.set_drive_pid(23, DRIVE_SPEED, true);
  chassis.wait_drive(); 

  chassis.set_drive_pid(-20, DRIVE_SPEED, true);
  chassis.wait_drive(); 

  intakeOut.suspend();
  intakeSpin(0, "fwd");

  //GET FOURTH BALL 
  chassis.set_turn_pid(-70, TURN_SPEED);
  chassis.wait_drive();

  pros::Task intakeKeep2(intakeFWDSpin);
  chassis.set_drive_pid(55, DRIVE_SPEED, true);
  chassis.wait_drive(); 
  
  intakeKeep2.suspend();
  //SEND FORUTH BALL
  chassis.set_turn_pid(70, TURN_SPEED);
  chassis.wait_drive(); 

  intakeSpin(300, "back");

  //GET FIFTH BALL
  chassis.set_turn_pid(-20, TURN_SPEED);
  chassis.wait_drive(); 

  pros::Task intakeKeep3(intakeFWDSpin);
  chassis.set_drive_pid(30, DRIVE_SPEED, true);
  chassis.wait_drive(); 

  intakeKeep3.suspend();

  //FACE GOAL
  chassis.set_turn_pid(100, TURN_SPEED);
  chassis.wait_drive();

  intakeSpin(200, "back");

  chassis.set_turn_pid(-90, TURN_SPEED);
  chassis.wait_drive();

  wingsOut();

  //FINISH AUTON
  
  chassis.set_drive_pid(-30, DRIVE_SPEED, true);
  chassis.wait_drive(); 

  chassis.set_drive_pid(20, DRIVE_SPEED, true);
  chassis.wait_drive(); 

  chassis.set_drive_pid(-30, DRIVE_SPEED, true);
  chassis.wait_drive(); 

  chassis.set_drive_pid(20, DRIVE_SPEED, true);
  chassis.wait_drive(); 

  chassis.set_turn_pid(-180, TURN_SPEED);
  chassis.wait_drive();

  hangUp();

  intakeSpin(0, "fwd");
}


void far6ball2(){

  //START INTAKING, hang DOWN, TO THE LEFT
  default_constants();
  modified_exit_conditions();
  chassis.set_angle(-90);
  pros::Task intakeKeep(intakeFWDSpin);


  //GRAB BORDER TRIBALL
  small_error_constants();
  chassis.set_drive_pid(6, DRIVE_SPEED, true);
  chassis.wait_drive();


  //PUSH ALLY BALL
  default_constants();
  chassis.set_drive_pid(-40, DRIVE_SPEED, true);
  chassis.wait_drive();
  intakeKeep.suspend();
  intakeSpin(0, "back");

  //TAKE LOADING BALL
  chassis.set_swing_pid(LEFT_SWING, -135, SWING_SPEED);
  chassis.wait_drive();
  wingsOut();


  small_error_constants();
  chassis.set_drive_pid(-10, DRIVE_SPEED, true);
  chassis.wait_drive();
  leftWingIn();

  chassis.set_drive_pid(-5, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_swing_pid(LEFT_SWING, -170, SWING_SPEED);
  chassis.wait_drive();

  //PLACE 2 BALLS IN GOAL

  wingsIn();

  default_constants();
  chassis.set_drive_pid(-22, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_drive_pid(16.5, DRIVE_SPEED, true);
  chassis.wait_drive(); 


  //PLACE 3RD BALL IN GOAL

  
  //chassis.set_drive_pid(3, DRIVE_SPEED, true);
  //chassis.wait_drive();

  default_constants();
  chassis.set_turn_pid(10, TURN_SPEED);
  chassis.wait_drive();

  intakeSpin(50, "back");
  pros::Task intakeOut(intakeBACKSpin);
  chassis.set_drive_pid(23.5, DRIVE_SPEED, true);
  chassis.wait_drive(); 

  chassis.set_drive_pid(-20.5, DRIVE_SPEED, true);
  chassis.wait_drive(); 

  intakeOut.suspend();
  intakeSpin(0, "fwd");

  //GET FOURTH BALL 
  chassis.set_turn_pid(-68, TURN_SPEED);
  chassis.wait_drive();

  pros::Task intakeKeep2(intakeFWDSpin);
  chassis.set_drive_pid(55, DRIVE_SPEED, true);
  chassis.wait_drive(); 
  
  intakeKeep2.suspend();
  //SEND FORUTH BALL
  chassis.set_turn_pid(80, TURN_SPEED);
  chassis.wait_drive(); 

  intakeSpin(400, "back");

  //GET FIFTH BALL
  chassis.set_turn_pid(-25, TURN_SPEED);
  chassis.wait_drive(); 

  pros::Task intakeKeep3(intakeFWDSpin);
  chassis.set_drive_pid(33, DRIVE_SPEED, true);
  chassis.wait_drive(); 

  intakeKeep3.suspend();

  //FACE GOAL
  chassis.set_turn_pid(100, TURN_SPEED);
  chassis.wait_drive();

  intakeSpin(200, "back");

  chassis.set_turn_pid(-90, TURN_SPEED);
  chassis.wait_drive();

  wingsOut();

  //FINISH AUTON
  
  chassis.set_drive_pid(-40, DRIVE_SPEED, true);
  chassis.wait_drive(); 

  chassis.set_drive_pid(20, DRIVE_SPEED, true);
  chassis.wait_drive(); 

  chassis.set_turn_pid(-180, TURN_SPEED);
  chassis.wait_drive();

 // hangUp();

  intakeSpin(0, "fwd");

}

void far6ball3(){


  //START INTAKING, hang DOWN, TO THE LEFT
  default_constants();
  modified_exit_conditions();
  chassis.set_angle(-90);
  pros::Task intakeKeep(intakeFWDSpin);


  //GRAB BORDER TRIBALL
  small_error_constants();
  chassis.set_drive_pid(6, DRIVE_SPEED, true);
  chassis.wait_drive();


  //PUSH ALLY BALL
  chassis.set_drive_pid(-6, 127, true);
  chassis.wait_drive();

  intakeSpin(0, "back");
  default_constants();
  chassis.set_drive_pid(-35, DRIVE_SPEED, true);
  chassis.wait_drive();
  intakeKeep.suspend();

  //TAKE LOADING BALL
  chassis.set_swing_pid(LEFT_SWING, -135, SWING_SPEED);
  chassis.wait_drive();
  wingsOut();


  chassis.set_drive_pid(-12, DRIVE_SPEED, true);
  chassis.wait_drive();
  leftWingIn();

  small_error_constants();
  chassis.set_drive_pid(-5, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_swing_pid(LEFT_SWING, -170, SWING_SPEED);
  chassis.wait_drive();

  //PLACE 2 BALLS IN GOAL

  wingsIn();
  default_constants();
  chassis.set_drive_pid(-19, 127, true);
  chassis.wait_drive();

  chassis.set_drive_pid(16.5, DRIVE_SPEED, true);
  chassis.wait_drive(); 


  //PLACE 3RD BALL IN GOAL

  
  //chassis.set_drive_pid(3, DRIVE_SPEED, true);
  //chassis.wait_drive();

  small_error_constants();
  chassis.set_turn_pid(10, TURN_SPEED);
  chassis.wait_drive();

  default_constants();
  intakeSpin(50, "back");
  pros::Task intakeOut(intakeBACKSpin);
  chassis.set_drive_pid(22, 127, true);
  chassis.wait_drive(); 

  small_error_constants();
  chassis.set_drive_pid(-20.5, DRIVE_SPEED, true);
  chassis.wait_drive(); 

  intakeOut.suspend();
  intakeSpin(0, "fwd");

  //GET FOURTH BALL 
  chassis.set_turn_pid(-68, TURN_SPEED);
  chassis.wait_drive();

  pros::Task intakeKeep2(intakeFWDSpin);
  chassis.set_drive_pid(55, DRIVE_SPEED, true);
  chassis.wait_drive(); 
  
  intakeKeep2.suspend();
  //SEND FOURTH BALL
  chassis.set_turn_pid(75, 127);
  chassis.wait_drive(); 

  intakeSpin(400, "back");

  //GET FIFTH BALL
  small_error_constants();
  chassis.set_swing_pid(RIGHT_SWING, -40, SWING_SPEED);
  chassis.wait_drive(); 

  pros::Task intakeKeep3(intakeFWDSpin);
  chassis.set_drive_pid(25, 127, true);
  chassis.wait_drive(); 

  intakeKeep3.suspend();

  //FACE GOAL
  chassis.set_turn_pid(100, 127);
  chassis.wait_drive();



  //FINISH AUTON
  chassis.set_drive_pid(42, 127, true);
  chassis.wait_drive(); 


  intakeSpin(0, "fwd"); 

}




void far4Wingin(){

modified_exit_conditions();

wingsIn();
pros::lcd::print(4, "Started");
hang_constants();
chassis.set_angle(45);

catapultLower(); 
pros::delay(1000);



pros::Task intakeKeep(intakeFWDSpin);


rightWingOut();

chassis.set_drive_pid(15, DRIVE_SPEED, true);
chassis.wait_drive();

chassis.set_turn_pid(15, 80);
chassis.wait_drive();

rightWingIn();

chassis.set_turn_pid(45, TURN_SPEED);
chassis.wait_drive();

chassis.set_drive_pid(1.5, DRIVE_SPEED, true);
chassis.wait_drive();

intakeKeep.suspend();

pros::Task intakeRev(intakeBACKSpin);

chassis.set_swing_pid(RIGHT_SWING, 0, SWING_SPEED);
chassis.wait_drive();



chassis.set_drive_pid(10, DRIVE_SPEED, true);
chassis.wait_drive();

chassis.set_drive_pid(-10, DRIVE_SPEED, true);
chassis.wait_drive();

chassis.set_turn_pid(-180, TURN_SPEED);
chassis.wait_drive();

chassis.set_drive_pid(-12, DRIVE_SPEED, true);
chassis.wait_drive();

intakeRev.suspend();



chassis.set_drive_pid(14, DRIVE_SPEED, true);
chassis.wait_drive();

chassis.set_turn_pid(-65, TURN_SPEED);
chassis.wait_drive();

pros::Task intakeTake(intakeFWDSpin);

chassis.set_drive_pid(45, DRIVE_SPEED, true);
chassis.wait_drive();



chassis.set_swing_pid(RIGHT_SWING, -90, SWING_SPEED);
chassis.wait_drive();





chassis.set_drive_pid(-5, DRIVE_SPEED, true);
chassis.wait_drive();

intakeTake.suspend();
intakeSpin(0, "back");

chassis.set_turn_pid(0, TURN_SPEED);
chassis.wait_drive();




chassis.set_drive_pid(11, DRIVE_SPEED, true);
chassis.wait_drive();

chassis.set_turn_pid(95, TURN_SPEED);
chassis.wait_drive();

small_error_constants();
chassis.set_exit_condition(chassis.drive_exit, 60,  20, 100, 100, 100, 100);

chassis.set_drive_pid(-3, DRIVE_SPEED, true);
chassis.wait_drive();



rightWingOut();

chassis.set_swing_pid(RIGHT_SWING, 90, SWING_SPEED);
chassis.wait_drive();

pros::Task intakeRev2(intakeBACKSpin);

hang_constants();
chassis.set_exit_condition(chassis.drive_exit, 80,  50, 300, 150, 100, 100);

pros::delay(250);

chassis.set_drive_pid(30, DRIVE_SPEED, true);
chassis.wait_drive();

intakeRev2.suspend();
intakeSpin(0, "back");

chassis.set_drive_pid(-15, DRIVE_SPEED, true);
chassis.wait_drive();

rightWingIn();

chassis.set_turn_pid(-90, TURN_SPEED);
chassis.wait_drive();



chassis.set_drive_pid(-15.5, DRIVE_SPEED, true);
chassis.wait_drive();

intakeSpin(0, "back");

chassis.set_drive_pid(3, DRIVE_SPEED, true);
chassis.wait_drive();


pros::lcd::print(4, "Finished 4-Ball");

}

void farSideRush(){

modified_exit_conditions();

wingsIn();
pros::lcd::print(4, "Started");
hang_constants();

chassis.set_angle(-35);
catapultLower();
rightWingOut(); 
pros::delay(1000);


rightWingIn();

pros::Task intakeIn(intakeFWDSpin);
chassis.set_drive_pid(64, DRIVE_SPEED, true);
chassis.wait_drive();

chassis.set_drive_pid(-2, DRIVE_SPEED, true);
chassis.wait_drive();

chassis.set_turn_pid(110, TURN_SPEED);
chassis.wait_drive();



leftWingOut();

intakeIn.suspend();

pros::Task intakeOut(intakeBACKSpin);

chassis.set_turn_pid(90, TURN_SPEED);
chassis.wait_drive();



chassis.set_drive_pid(25, DRIVE_SPEED, true);
chassis.wait_drive();

chassis.set_drive_pid(-10, DRIVE_SPEED, true);
chassis.wait_drive();

chassis.set_drive_pid(14, DRIVE_SPEED, true);
chassis.wait_drive();

chassis.set_drive_pid(-5, DRIVE_SPEED, true);
chassis.wait_drive();

intakeOut.suspend();

leftWingIn();

chassis.set_turn_pid(240, TURN_SPEED);
chassis.wait_drive();

pros::Task intakeIn2(intakeFWDSpin);

chassis.set_drive_pid(28, DRIVE_SPEED, true);
chassis.wait_drive();

chassis.set_swing_pid(LEFT_SWING, 150, SWING_SPEED);
chassis.wait_drive();

chassis.set_drive_pid(42, DRIVE_SPEED, true);
chassis.wait_drive();

rightWingOut();

intakeIn2.suspend();
intakeSpin(0, "back");
pros::Task intakeOut2(intakeBACKSpin);

chassis.set_swing_pid(RIGHT_SWING, 10, SWING_SPEED);
chassis.wait_drive();

small_error_constants();

rightWingIn();

chassis.set_drive_pid(-4, DRIVE_SPEED, true);
chassis.wait_drive();

chassis.set_turn_pid(210, TURN_SPEED);
chassis.wait_drive();

hang_constants();


chassis.set_drive_pid(-24, DRIVE_SPEED, true);
chassis.wait_drive();






chassis.set_drive_pid(12, DRIVE_SPEED, true);
chassis.wait_drive();

chassis.set_drive_pid(-20, DRIVE_SPEED, true);
chassis.wait_drive();

chassis.set_drive_pid(10, DRIVE_SPEED, true);
chassis.wait_drive();

intakeOut2.suspend();
intakeSpin(0, "fwd");

pros::lcd::print(4, "Finished 5-Ball");
}


void far3Ball(){

  //START INTAKING, hang DOWN, TO THE LEFT
  default_constants();
  chassis.set_angle(-90);
  pros::Task intakeKeep(intakeFWDSpin);


  //GRAB BORDER TRIBALL
  chassis.set_drive_pid(5, DRIVE_SPEED, true);
  chassis.wait_drive();


  //PUSH ALLY BALL
  chassis.set_drive_pid(-43, 90, true);
  chassis.wait_drive();
  intakeKeep.suspend();

  //TAKE LOADING BALL
  chassis.set_swing_pid(LEFT_SWING, -135, SWING_SPEED);
  chassis.wait_drive();
  leftWingOut();

  chassis.set_drive_pid(-10, DRIVE_SPEED, true);
  chassis.wait_drive();
  leftWingIn();

  chassis.set_drive_pid(-5, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_swing_pid(LEFT_SWING, -170, SWING_SPEED);
  chassis.wait_drive();
  

  chassis.set_turn_pid(-180, TURN_SPEED);
  chassis.wait_drive();

  //RAM TWO BALLS IN GOAL
  chassis.set_drive_pid(-19, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_drive_pid(15, DRIVE_SPEED, true);
  chassis.wait_drive();

  //PLACE FINAL BALL
  chassis.set_turn_pid(10, TURN_SPEED);
  chassis.wait_drive();

  pros::Task intakeOut(intakeBACKSpin);
  chassis.set_drive_pid(23, DRIVE_SPEED, true);
  chassis.wait_drive(); 

  chassis.set_drive_pid(-20, DRIVE_SPEED, true);
  chassis.wait_drive(); 

  //RAM FINAL BALL
  chassis.set_turn_pid(-170, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-23, DRIVE_SPEED, true);
  chassis.wait_drive(); 

  chassis.set_drive_pid(15, DRIVE_SPEED, true);
  chassis.wait_drive();

  intakeOut.suspend();
  intakeSpin(0, "fwd");
   
}

void drive_example() {

 //catapultLower(); 
  //chassis.wait_drive();
  double angle;

  double avgPosition;

  double deviation;
  //catapultLower();
  //catapultFire(44);

//skills();

//skillsPractice();

//wpLoadingSide();

//farSide();

//far4Wingin();

//farSideRush();

//pros::Task intakeFWD(intakeFWDSpin);
//chassis.set_drive_pid(22, DRIVE_SPEED, true);
//chassis.wait_drive();
//intakeFWD.suspend();

//intakeSpin(2000, "back");


//pros::Task intakeBACK(intakeBACKSpin);
//chassis.set_drive_pid(-22, DRIVE_SPEED, true);
//chassis.wait_drive();
//intakeBACK.suspend();




//ROBOT CONTRACTED







  /*
  avgPosition = 0.5 * ((chassis.right_sensor() / chassis.get_tick_per_inch()) + (chassis.left_sensor() / chassis.get_tick_per_inch()));
  */
  default_constants();
  pros::Task imu(imuChecker);
  pros::Task wait(waitTimes);
  ram_conditions();

  pros::Task intakeOut(intakeBACKSpin);
  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_drive_pid(-12, DRIVE_SPEED, true);
  chassis.wait_drive();
  intakeOut.suspend();
  intakeSpin(0, "fwd");

/*
  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();

small_error_constants();

  chassis.set_drive_pid(6, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_drive_pid(-6, DRIVE_SPEED, true);
  chassis.wait_drive();

  

default_constants();

  chassis.set_drive_pid(-24, DRIVE_SPEED, true);
  chassis.wait_drive();

small_error_constants();

  chassis.set_drive_pid(6, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(10, TURN_SPEED);
  chassis.wait_drive();

default_constants();

  chassis.set_drive_pid(12, DRIVE_SPEED, true);
  chassis.wait_drive();

small_error_constants();

  chassis.set_turn_pid(-35, TURN_SPEED);
  chassis.wait_drive();



  chassis.set_turn_pid(-45, TURN_SPEED);
  chassis.wait_drive();

default_constants();

  chassis.set_drive_pid(12, DRIVE_SPEED, true);
  chassis.wait_drive(); 

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  small_error_constants();

  chassis.set_drive_pid(6, DRIVE_SPEED, true);
  chassis.wait_drive(); 

default_constants();

  chassis.set_turn_pid(-135, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_swing_pid(LEFT_SWING, -45, SWING_SPEED);
  chassis.wait_drive();  

small_error_constants();

  chassis.set_turn_pid(-55, TURN_SPEED);
  chassis.wait_drive();

default_constants();

  chassis.set_swing_pid(LEFT_SWING, 135, SWING_SPEED);
  chassis.wait_drive();  

small_error_constants();

  chassis.set_swing_pid(RIGHT_SWING, 90,  SWING_SPEED);
  chassis.wait_drive();

  chassis.set_swing_pid(LEFT_SWING, 80, SWING_SPEED);
  chassis.wait_drive(); 
*/



  imu.suspend();
  wait.suspend();
  /*
  avgPosition = 0.5 * ((chassis.right_sensor() / chassis.get_tick_per_inch()) + (chassis.left_sensor() / chassis.get_tick_per_inch())) - avgPosition;
  deviation = (chassis.right_sensor() / chassis.get_tick_per_inch()) - avgPosition; 
  pros::lcd::print(1, "Position:%f", avgPosition);
  pros::lcd::print(3, "Deviation:%f", deviation);

 


  angle = chassis.get_gyro();
  chassis.set_turn_pid(180, TURN_SPEED);
  chassis.wait_drive();  
  angle = chassis.get_gyro() - angle;
  pros::lcd::print(4, "Angle:%f", angle);  
  */
  /*

  chassis.set_drive_pid(22, DRIVE_SPEED, true);
  chassis.wait_drive();


  chassis.reset_gyro();
  angle = chassis.get_gyro();
  chassis.set_swing_pid(LEFT_SWING, -30, SWING_SPEED);
  chassis.wait_drive();  
  angle = chassis.get_gyro() - angle;
  pros::lcd::print(4, "Angle:%f", angle); 

  chassis.set_drive_pid(-19, DRIVE_SPEED, true);
  chassis.wait_drive();
  

  chassis.set_turn_pid(-90, TURN_SPEED);
  chassis.wait_drive();   
 

  chassis.set_drive_pid(22, DRIVE_SPEED, true);
  chassis.wait_drive();

 chassis.reset_gyro();
  angle = chassis.get_gyro();
  chassis.set_swing_pid(ez::LEFT_SWING, 30, SWING_SPEED);
  chassis.wait_drive();  
  angle = chassis.get_gyro() - angle;
  pros::lcd::print(4, "Angle:%f", angle);  

*/
}








//ONLY RUN AS TASK




