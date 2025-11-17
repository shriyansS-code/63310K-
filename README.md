//# 63310K-
#include "main.h"
#include "lemlib/api.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include <sys/syslimits.h>

/*
===========================================================
VEX V5 Robot Program — PROS + LemLib Template
Author: Bhargav Trivedi
===========================================================
📘 Overview:
This template combines PROS (for low-level robot control)
and LemLib (for odometry & motion control).

Sections include:
  1. Motor Configuration
  2. Sensor Configuration
  3. Pneumatics (ADI Example)
  4. LemLib Drivetrain & Controllers
  5. Initialization / Calibration
  6. Competition Functions
  7. Driver Control
===========================================================
*/

/* 
-----------------------------------------------------------
1️⃣ MOTOR CONFIGURATION
-----------------------------------------------------------
💡 NOTES:
- Port numbers correspond to the V5 Brain labels.
- Use negative ports to reverse motor direction.
- Motors can be grouped using `pros::MotorGroup` or defined individually.
*/

// Example of standalone motor declaration
// pros::Motor exampleMotor(7, pros::MotorGears::blue, false);

// Left motor group on ports 1, 2, 3 (1 & 3 reversed)
pros::MotorGroup left_motors({4,3,2}, pros::MotorGears::blue);

// Right motor group on ports 4, 5, 6 (5 reversed)
pros::MotorGroup right_motors({8,7,6}, pros::MotorGears::green);

// Standalone intake motor (port 10)
pros::Motor intake(10, pros::MotorGears::blue);


/*
-----------------------------------------------------------
2️⃣ SENSOR CONFIGURATION
-----------------------------------------------------------
💡 Includes IMU, tracking wheels, encoders, etc.
- Tracking wheels measure distance and direction.
- IMU provides inertial heading tracking.
*/

pros::Imu imu(10); // IMU on port 10

// Tracking encoders (rotation sensors or ADI encoders)
pros::Rotation horizontal_encoder(20);              // Horizontal tracking wheel
pros::adi::Encoder vertical_encoder('C', 'D', true); // Vertical tracking wheel (ADI ports C, D)

// Tracking wheel objects for LemLib odometry
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, -5.75);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, -2.5);


/*
-----------------------------------------------------------
3️⃣ PNEUMATICS (ADI EXAMPLE)
-----------------------------------------------------------
💡 Pneumatics are controlled via ADI digital outputs.
- Setting HIGH opens the solenoid.
- Example button control included in opcontrol().
*/

pros::adi::DigitalOut clamp('A');  // Pneumatic clamp on ADI port A
bool clampValue = false;           // Initial state of pneumatic clamp


/*
-----------------------------------------------------------
4️⃣ LEMLIB DRIVETRAIN & CONTROLLERS
-----------------------------------------------------------
💡 LemLib handles odometry, PID control, and autonomous motion.
- Adjust parameters based on robot geometry and wheel setup.
*/

lemlib::Drivetrain drivetrain(
    &left_motors,                // Left motor group
    &right_motors,               // Right motor group
    14.5,                          // Track width (inches)
    lemlib::Omniwheel::NEW_4,    // Wheel type (4" omni)
    800,                         // Max RPM
    2                            // Drift (measured experimentally)
);

// Odometry sensor setup
lemlib::OdomSensors sensors(
    &vertical_tracking_wheel,    // Vertical tracking wheel 1
    nullptr,                     // Vertical tracking wheel 2 (none)
    &horizontal_tracking_wheel,  // Horizontal tracking wheel 1
    nullptr,                     // Horizontal tracking wheel 2 (none)
    &imu                         // IMU
);

// PID Controller settings for forward/backward motion
lemlib::ControllerSettings lateral_controller(
    10, 0, 3, 3,  // kP, kI, kD, anti-windup
    1, 100,        // Small error (inches), timeout (ms)
    3, 500,        // Large error (inches), timeout (ms)
    20             // Max acceleration (slew)
);

// PID Controller settings for turning
lemlib::ControllerSettings angular_controller(
    2, 0, 10, 3,  // kP, kI, kD, anti-windup
    1, 100,        // Small error (degrees), timeout (ms)
    3, 500,        // Large error (degrees), timeout (ms)
    20             // Max acceleration (slew)
);

// Create LemLib chassis
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);


/*
-----------------------------------------------------------
5️⃣ INITIALIZATION & CALIBRATION
-----------------------------------------------------------
💡 Runs on startup to initialize devices and calibrate sensors.
*/

void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();  // Calibrate IMU & encoders

    // Task to continuously print pose data to the brain screen
    pros::Task screen_task([&]() {
        while (true) {
            pros::lcd::print(0, "X: %.2f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %.2f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %.2f", chassis.getPose().theta);
            pros::delay(50);
        }
    });
}


/*
-----------------------------------------------------------
6️⃣ COMPETITION TEMPLATE FUNCTIONS
-----------------------------------------------------------
💡 These are standard PROS functions for competition control.
*/

void disabled() {}
void competition_initialize() {}

void autonomous() {
    // Example autonomous routine
    chassis.moveToPoint(24, 0, 3000);  // Move forward 24"
    chassis.turnToHeading(90, 1000);   // Turn to 90°
}


/*
-----------------------------------------------------------
7️⃣ DRIVER CONTROL (OPCONTROL)
-----------------------------------------------------------
💡 Handles user control using the VEX Controller.
Includes examples for drive, motor control, and pneumatics.
*/

void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);

    while (true) {
        // --- Drive Controls ---
        // Arcade drive (single-stick)
        //chassis.arcade(master.get_analog(ANALOG_LEFT_Y), master.get_analog(ANALOG_RIGHT_X));

        // Example template for clarity (not executable):
        // chassis.arcade(int throttle, int turn);

        // Tank drive (two-stick)
         chassis.tank(master.get_analog(ANALOG_LEFT_Y), master.get_analog(ANALOG_RIGHT_Y));
        // Example template:
        // chassis.tank(int left, int right);


        // --- Intake Motor Control ---
        // L2 = forward, L1 = reverse
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intake.move(127);  // Full forward
        } 
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intake.move(-127); // Full reverse
        } 
        else {
            intake.brake();    // Stop (optional — can replace with .move(0))
        }


        // --- Pneumatics Toggle ---
        // A button toggles the pneumatic clamp
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            clampValue = !clampValue;
            clamp.set_value(clampValue);
        }

        pros::delay(20); // Delay to reduce CPU usage
    }
}
