#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
using namespace pros;
Controller master(E_CONTROLLER_MASTER);
//-------------------------------Initializaiton---------------------------//
// change port numbers before running
pros::Motor intake(11); //bottom
pros::Motor hood(-12); //top

adi::Pneumatics mogo_clamp(3,false);
adi::Pneumatics doinker(2,false);
adi::Pneumatics pto(1,false);
adi::Potentiometer pot(8);
pros::Rotation rotation_sensor(21);
//L1 raise L2 drop
//engage pto and reverse intake

bool mogo_clamp_extended = false;
bool doinker_extended = false;
bool pto_on = false;

//----Motor Groups---//
//change the port + motor gear
pros::MotorGroup leftMotor({-1,3,-4},pros::MotorGears::blue);
pros::MotorGroup rightMotor({6,7,-8},pros::MotorGears::blue);
//--------------------------------Configuration----------------------------//
// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotor, // left motor group
                              &rightMotor, // right motor group
                              14.3, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

// imu
pros::Imu imu(16);
// horizontal tracking wheel encoder
pros::Rotation horizontal_encoder(19);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, 2.5);

// vertical tracking wheel
pros::Rotation vertical_encoder(13);

lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, -0.7);

// odometry settings
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);


//-------------------------------------PID Tuning-----------------------------//

// lateral PID controller
lemlib::ControllerSettings lateral_controller(30, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              240, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);


// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              13, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);


//---------------------------------Function-------------------------//

void toggle_mogo_clamp() {
    if (mogo_clamp_extended) {
        mogo_clamp.retract();
    } else {
        mogo_clamp.extend();
    }
    mogo_clamp_extended = !mogo_clamp_extended;
}

void toggle_doinker() {
    if (doinker_extended) {
        doinker.retract();
    } else {
        doinker.extend();
    }
    doinker_extended = !doinker_extended;
}



// void toggle_arm(){
//     if(arm_extended){
//         arm.retract();
//     }else{
//         arm.extend();
//     }
//     arm_extended = !arm_extended;
// }





/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

int armState = 1;// 3states, 0 = loading, 1 = Ali, 2, wall , 3 afk
void armTask(){
    int target;
    int error;
    double kP = -0.1;
    while(true){
        if (armState != 3){
            pto.extend();
        }else {
            pto.retract();
        }
        if(armState == 0){
            target = 12900;
            error = target - rotation_sensor.get_position();
            hood.move(error*kP);
        } else if(armState == 1){
            target = 15600;
            error = target - rotation_sensor.get_position();
            hood.move(error*kP);
        } else if(armState == 2){
            target = 17824;
            error = target - rotation_sensor.get_position();
            hood.move(error*kP);
        } else {
            
        }
        delay(20);
    }
}
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
// initialize function. Runs on program startup
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    pto.retract();
    doinker.retract();
    mogo_clamp.retract();
    // print position to brain screen
    pros::Task arm_task(armTask);

    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
             pros::lcd::print(3, "Arm: %i", rotation_sensor.get_position());
            // delay to save resources
            pros::delay(20);
        }
    });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

void competition_initialize() {}


 //-------------------------ALL-----------------------//
void run(){
    chassis.setPose(-148.465,-70.034,90);
    chassis.moveToPoint(-115, -70,2000);
}
void red6(){
    mogo_clamp.extend();
    chassis.setPose(130,61.123,90);
    chassis.moveToPoint(100, 61.123, 1000,{.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    pros::delay(500);
	mogo_clamp.retract();
    //chassis.turnToHeading(340, 500);
    // intake.move(127);
    // hood.move(127);
    // chassis.moveToPoint(66.528, 107.589, 1000);
    // chassis.moveToPose(26, 110, 270, 1000);
    // chassis.moveToPose(42, 129, 220, 1000,{.forwards = false});
    // chassis.moveToPose(30, 129, 270, 1000);
    // chassis.moveToPose(130, 135, 55, 2000);
}
//extend mogo closes it and vice versa
void red5A(){
    mogo_clamp.extend();
    chassis.setPose(56.051,13.76,130);
    armState = 1;
    delay(1000);
    chassis.moveToPoint(66, 6, 1000);
    chassis.waitUntilDone();
    armState = 0;
    chassis.moveToPose(29.341, 32.228, 132, 2000,{.forwards = false});
    chassis.waitUntilDone();
    mogo_clamp.retract();
    armState = 3;
    chassis.turnToHeading(13, 1000);
    intake.move(127);
    hood.move(127);
    chassis.moveToPose(35, 55, 13, 1200);
    chassis.turnToHeading(-90, 1000);
    chassis.waitUntilDone();
    chassis.moveToPose(28, 54, -91, 1300);
    chassis.waitUntilDone();
    chassis.moveToPose(32, 53, -86, 1200,{.forwards = false});
    chassis.waitUntilDone();
    chassis.moveToPose(20,63, -70, 1000);
    chassis.waitUntilDone();



}
void autonomous() {
    red5A();
    //red6();
    //pto.extend();
    //run();
	//blueRush();
    
    
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
void opcontrol() {
	bool driveSplitARc = true; 
	bool tank = false;
    bool track = false;
    while (true) {
        // Tank Control Scheme
        if(master.get_digital_new_press(DIGITAL_UP)){
            driveSplitARc = !driveSplitARc; 
        }
        if(driveSplitARc){
            int left_y = master.get_analog(ANALOG_LEFT_Y);  // Left joystick for left side
            int right_y = master.get_analog(ANALOG_RIGHT_Y); // Right joystick for right side

            leftMotor.move(left_y);  // Left side motors
            rightMotor.move(right_y); // Right side motors

        }else{
            int turn = master.get_analog(ANALOG_RIGHT_X); // Turn direction
            int forward = master.get_analog(ANALOG_LEFT_Y); // Forward/backward

            // Move the motors
            leftMotor.move(forward + turn);  // Left side motors
            rightMotor.move(forward - turn); // Right side motors
		}
        // Move the motors
      


        // Intake and hood controls
        if (master.get_digital(DIGITAL_L1)) {
            intake.move(127);
            hood.move(127);
        } 
        // Redirect controls
        else if (master.get_digital(DIGITAL_L2)) {
            intake.move(-127);
            hood.move(-127);
        } else if(master.get_digital(DIGITAL_R1)){
            pto.extend();
            intake.move(127);
        }
        else if(master.get_digital(DIGITAL_R2)){
            pto.extend();
            intake.move(-127);
        }

        
        
        // Reverse controls
      /*  if (master.get_digital(DIGITAL_L2)) {
            intake.move(-127);
            hood.move(-127);
        }*/

        // Mogo hook pneumatics toggle
        if (master.get_digital_new_press(DIGITAL_B)) {
            toggle_mogo_clamp();
        }
        // Sweeper pneumatics toggle
        if (master.get_digital_new_press(DIGITAL_A)) {
            // toggle_doinker();
            doinker.toggle();
        }
        if(master.get_digital_new_press(DIGITAL_R2)){
            //toggle_pto();
            pto.toggle();
            track = !track;
            if(track){
                hood.move(0);
            }
            
        }
        // if(master.get_digital(DIGITAL_R1)){
        //     intake.move(127);
        // }

        // arm pneumatics toggle
        // if(master.get_digital_new_press(DIGITAL_B)){
        //     toggle_arm();
        // }
        pros::delay(20); // Run for 20 ms then update
	}
}