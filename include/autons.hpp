#pragma once

extern Drive chassis;

void drive_example();
//DRIVE PID FUNCTIONS
void default_constants();

void hang_constants();
void small_error_constants();
void modified_exit_conditions();
void ram_conditions();

const int DRIVE_SPEED = 110;
const int SWING_SPEED = 110;
const int TURN_SPEED = 90;
//CHECKS DRIVE AND TURN FUNCTIONS TO THE BRAIN
void imuChecker();



//OTHER ROBOT FUNCTIONS
void intakeFWDSpin();
void intakeBACKSpin();
void intakeSpin(int distance, string direction);

int hangUp();
int hangDown();

int wingsIn();
int wingsOut();
int leftWingIn();
int leftWingOut();
int rightWingIn();
int rightWingOut();

int backWingsIn();
int backWingsOut();
int backLWingIn();
int backLeftWingOut();
int backLightWingIn();
int backRightWingOut();

int catapultLower();
void catapultFire(int shots);


//AUTONS

//1 BALL AUTON - 9 POINTS
void wpLoadingSide();

//3 BALL AUTON - 15 POINTS
void farSide();
void far3Ball();
//4 BALL AUTON - 20 POINTS
void far4Wingin();

//SKILLS CATA THEN SWEEP
void skills();

//5 BALL AUTON - 25 POINTS
void farSideRush();

//6 BALL AUTON - 30 POINTS
void far6ball();

void far6ball2();

void far6ball3();
//SPECIAL AUTONS

/**
 * ONLY RUNS FIRST PART OF SKILLS
 * GOES TO LOADING POSITION
 * FIRES CATAPULT AND WINGS
 * ENDS
*/
void skillsPractice();