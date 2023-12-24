#include "main.h"
#include "autons.hpp"

void wpLoadingSide(){

//START CODE, WINGS IN, hang DOWN, INTAKE SPINNING, ORIENTED 155 DEGREES

wingsIn();
pros::lcd::print(4, "Started");
default_constants();
modified_exit_conditions();
chassis.set_angle(155);



/*

//DRIVE UP TO GOAL
ram_conditions();
chassis.set_drive_pid(-40, DRIVE_SPEED, true);
chassis.wait_drive();

small_error_constants();
modified_exit_conditions();
chassis.set_drive_pid(6, DRIVE_SPEED, true);
chassis.wait_drive();


chassis.set_turn_pid(180, TURN_SPEED);
chassis.wait_drive();





//HEAD TO LOADING BAR
chassis.set_drive_pid(6, DRIVE_SPEED, true);
chassis.wait_drive();

default_constants();
*/
chassis.set_turn_pid(315, TURN_SPEED);
chassis.wait_drive();


//TAKE TRIBALL OUT OF LOADING BAR

backLeftWingOut();


chassis.set_drive_pid(-6, DRIVE_SPEED, true);
chassis.wait_drive();




default_constants();

leftWingIn();
/*
chassis.set_turn_pid(90, TURN_SPEED);
chassis.wait_drive();

//HEAD TOWARDS HANGING BAR

pros::Task intakeRev2(intakeBACKSpin);
chassis.set_turn_pid(-240, TURN_SPEED);
chassis.wait_drive();

chassis.set_drive_pid(13, DRIVE_SPEED, true);
chassis.wait_drive();

chassis.set_turn_pid(-270, TURN_SPEED);
chassis.wait_drive();
intakeRev2.suspend();

chassis.set_drive_pid(31, DRIVE_SPEED, true);
chassis.wait_drive();






intakeSpin(200, "back");
hangUp();
//chassis.set_swing_pid(LEFT_SWING, 90, SWING_SPEED);
//chassis.wait_drive();



//wingsIn();
*/



pros::lcd::print(4, "Finished WP Loading Side");

}