#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rotation.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "main.h"


/* Motor Configurations */
 pros::MotorGroup leftMotors({-14, 19, -20}, pros::MotorGearset::blue);
 pros::MotorGroup rightMotors({-5, 10, 11}, pros::MotorGearset::blue);
 pros::Motor tube(7,pros::MotorGearset::blue);
 pros::Motor intake(15, pros::MotorGearset::blue);
 pros::Rotation tubeEncoder(9);
 pros::adi::Pneumatics lift('C', false);
 pros::adi::Pneumatics descore('B', false);
 pros::adi::Pneumatics odomUp('D', true);
 pros::adi::Pneumatics scraper('A',false);
 


 const int tube_states = 2;
 int states[tube_states] {0, -90000};
 int current_state = 0;
 int target_state;

 void incrementState() {
    current_state = (current_state + 1) % tube_states;
    target_state = states[current_state];
 }
    

void tubeController() {
    double kP = 0.006;
    double error = target_state - tubeEncoder.get_position();
    double output = kP * error;
    tube.move_velocity(-output);
    
 }
 void tubecontroltask() {
    while (true) {
        tubeController();
        pros::delay(20);
    }
 }
 
 lemlib::Drivetrain drivetrain(&leftMotors,
							  &rightMotors,
							  10, //  10 inch track width
							  lemlib::Omniwheel::NEW_325, //  3.25 inch omniwheels
							  450, // 450 rpm
							  2 // horiz drift (CHANGE LATER)
							  );
// Inertial Sensor on port 10
pros::Imu imu(10);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(18);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-11);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -2);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);

// lateral motion controller
/*lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);
*/

const int lat_kp=10;
const int lat_kd=100; 

lemlib::ControllerSettings linearController(lat_kp, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              lat_kd, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);


// angular motion controller



lemlib::ControllerSettings angularController(8, // proportional gain (kP)
                                             0, // integral gain (kI)
                                            12, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
); 

/*
lemlib::ControllerSettings angularController(8, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
*/



// sensors for odometry
lemlib::OdomSensors sensors(
	nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

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

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "");
	pros::lcd::register_btn1_cb(on_center_button);
	chassis.calibrate();
    lift.set_value(true);
    odomUp.set_value(false);

    
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    tubeEncoder.reset_position();
    pros::Task tubeTask(tubecontroltask);
	 pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screens
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            pros::lcd::print(4, "Tube L: %d", tubeEncoder.get_position());
            // delay to save resources
            pros::lcd::print(6, "Tube: %d", tubeEncoder.get_position());
            pros::delay(50);
            /*
            pros::lcd::print(6, "Target Tube: %d", target_state);
            */


        }
    });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

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
/**void autonomous() {
    if(odomUp.is_extended());
    chassis.setPose(0,0,0); 
    chassis.calibrate();
    descore.toggle();
    chassis.moveToPoint(0,29,1500,{.forwards=true,.maxSpeed=60},false);
    scraper.toggle();
    chassis.turnToPoint(15,24,1500,{.forwards=true,.maxSpeed=60},false);
    intake.move_velocity(-12000);
    chassis.moveToPoint(15,24,1250,{.forwards=true,.maxSpeed=60},false);
    pros::delay(250);
    chassis.turnToPoint(-25,39,1000,{.forwards=false,.maxSpeed=60},false);
    chassis.moveToPoint(-25,39,1050,{.forwards=false,.maxSpeed=50},false);
    target_state = 250000;
    pros::delay(1000);
    chassis.turnToPoint(0,-7,1500,{.forwards=true,.maxSpeed=60},false);
    chassis.moveToPoint(0,-7,1500,{.forwards=true,.maxSpeed=60},false);
*/
    
void autonomous() {
    if(odomUp.is_extended());
    chassis.setPose(0,0,0); 
    chassis.calibrate();
    descore.toggle();
    intake.move_velocity(-12000);
    chassis.turnToPoint(18,30.25,1500,{.forwards=true,.maxSpeed=60},false);
    chassis.moveToPoint(18,30.25,1500,{.forwards=true,.maxSpeed=60},false);
    pros::delay(500);
    chassis.turnToPoint(41,6.25,1500,{.forwards=false,.maxSpeed=60},false);
    chassis.moveToPoint(41,6.25,1500,{.forwards=false,.maxSpeed=60},false);
    pros::delay(500);
    chassis.turnToPoint(32,17,1500,{.forwards=false,.maxSpeed=60},false);
    chassis.moveToPoint(32,17,1500,{.forwards=false,.maxSpeed=60},false);
    target_state = 250000;
    pros::delay(800);
    scraper.toggle();
    pros::delay(1000);
    chassis.moveToPoint(59,-12,1500,{.forwards=true,.maxSpeed=60},false);

}




   
    /*
    4 left side tube
   if(odomUp.is_extended());
    chassis.setPose(0,0,0); 
    chassis.calibrate();
    descore.toggle();
    pros::delay(1000);
    intake.move_velocity(-12000);
    chassis.turnToPoint(-9,21,1000,{.forwards=true},false);
    chassis.moveToPoint(-9,21,1000,{.forwards=true,.maxSpeed=50},false);
    chassis.turnToPoint(-18,-14,1000,{.forwards=true,},false);
    chassis.moveToPoint(-18,-14,1000,{.forwards=true,.maxSpeed=50},false);
    chassis.turnToPoint(-23,-13,1000,{.forwards=true,.maxSpeed=50},false);
    chassis.moveToPoint(-23,-13,1000,{.forwards=true,.maxSpeed=50},false);
    chassis.turnToPoint(-80,20,1000,{.forwards=false,},false);
    chassis.moveToPoint(-78,20,1000,{.forwards=false,},false);
    target_state = 250000;
    */
/*
    if(odomUp.is_extended());
    chassis.setPose(0,0,0); 
    chassis.calibrate();
    descore.toggle();
    intake.move_velocity(-12000);
    chassis.turnToPoint(9,22,1000,{.forwards=true},false);
    chassis.moveToPoint(9,22,1000,{.forwards=true,.maxSpeed=60},false);
    chassis.turnToPoint(-8,32,1000,{.forwards=true,},false);
    intake.move_velocity(0);
    chassis.moveToPoint(-8,32,1000,{.forwards=true,.maxSpeed=60},false);
    intake.move_velocity(12000);
    pros::delay(1500);
    chassis.moveToPoint(20,-20,1000,{.forwards=false,.maxSpeed=60},false);
*/

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
	pros::Controller controller(pros::E_CONTROLLER_MASTER);
    bool resetTube = false; 

	 while (true) {
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            if (!resetTube) {
                target_state = 0;
            }
            else {
                target_state = 145000;
                tubeEncoder.reset_position();
                resetTube = true;
            }
        }
        else{

            target_state = 250000;
            resetTube = true;
    
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake.move_velocity(-5500);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) 
        {
            intake.move_velocity(5500);
        } else {
            intake.move_velocity(0);
        }

        
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);
        

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
        lift.toggle();
        }
        
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
        tube.move_velocity(12000);
        }
    

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
        scraper.toggle();
        }


        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
            descore.toggle();
        }
        pros::delay(10);
    }
}