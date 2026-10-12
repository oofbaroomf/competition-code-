#include "vex.h"
#include "robot-config.h"
using namespace vex;
// A global instance of brain used for printing to the V5 Brain screen
brain Brain;
// VEXcode device constructors
controller controller_1 = controller(primary);
// ============================================================================
// DRIVETRAIN
// Names/ports/gearsets/reversal match this year's python code exactly.
// Format: motor(port, gearSetting, reversed)
// gearSetting: ratio36_1 (red), ratio18_1 (green), ratio6_1 (blue/5.5w)
// ============================================================================
motor motor_18 = motor(PORT2, ratio6_1, true);
motor motor_19 = motor(PORT3, ratio6_1, true);
motor motor_205 = motor(PORT1, ratio18_1, true);// 5.5 is coded if needed, but not used in this year's code
motor_group left_chassis = motor_group(motor_18, motor_19);
motor motor_11 = motor(PORT6, ratio6_1, true);
motor motor_12 = motor(PORT5, ratio6_1, true);
motor motor_135 = motor(PORT4, ratio18_1, true);
motor_group right_chassis = motor_group(motor_11, motor_12);
// drivetrain(left, right, wheelTravel_in, trackWidth_in, wheelbase_in, units)
// trackWidther and wheelbase and
// update thes matched to python's TRACK_WIDTH = 10.75.
// wheelTravel_in and wheelbase_in are PLACEHOLDERS carried from the generic
// template - verify/measure your actual wheel diamete two numbers before relying on this for autonomous movement.
drivetrain Drivetrain = drivetrain(left_chassis, right_chassis, 3.17, 10.75, 1.5, inches);
// Matches python's imu1 = Inertial(Ports.PORT11)
inertial inertial_sensor = inertial(PORT11);
// ── Tracking wheels ──────────────────────────────────────────────────────
// Not used this year (using_horizontal_tracker / using_vertical_tracker are
// false below), but declared since motor-control.cpp's odometry code
// references these symbols at compile/link time regardless of whether
// they're used. Ports chosen to avoid conflicts with motors/sensors in use.
rotation horizontal_tracker = rotation(PORT14, true);
rotation vertical_tracker = rotation(PORT15, true);
// ── Distance sensors ──────────────────────────────────────────────────────
// Not used by this year's python code (no distance-based position resets),
// but motor-control.cpp's resetPositionWithSensor() calls reference these
// symbols at compile/link time, so they must exist even if not physically
// wired up. Ports chosen to be free (not used by any motor/sensor above).
distance front_sensor = distance(PORT7);
distance left_sensor = distance(PORT8);
distance right_sensor = distance(PORT12);
distance back_sensor = distance(PORT13);
// ============================================================================
// SUBSYSTEM DEVICES
// Names/ports/gearsets/reversal match this year's python code exactly.
// ============================================================================
motor motor_intakehalf = motor(PORT16, ratio6_1, true);// full motor
motor motor_clawhalf = motor(PORT20, ratio6_1, true);
motor motor_winch = motor(PORT17, ratio18_1, true);
motor motor_clawhalfflex = motor(PORT19, ratio6_1, true);
motor motor_winchhalf = motor(PORT18, ratio18_1, true); 
rotation rotation_wrist = rotation(PORT10, true);// fix port

// ============================================================================
// USER-CONFIGURABLE PARAMETERS (CHANGE BEFORE USING THIS TEMPLATE)
// ============================================================================
// Distance between the middles of the left and right wheels of the drive (in inches)
// Matched to python's TRACK_WIDTH = 10.75
double distance_between_wheels = 10.75;
// motor to wheel gear ratio * wheel diameter (in inches) * pi
// PLACEHOLDER - carried from the generic template. Verify your actual
// external gear ratio and wheel diameter and update this.
double wheel_distance_in = (36.0 / 48.0) * 3.17 * M_PI;
// PID Constants for movement
// distance_* : Linear PID for straight driving
// turn_* : PID for turning in place
// heading_correction_* : PID for heading correction during linear movement
// NOTE: these are placeholder values — run the auto-tuner or hand-tune before use.
double distance_kp = 0, distance_ki = 0, distance_kd = 0;
double turn_kp = 0, turn_ki = 0, turn_kd = 0;
double heading_correction_kp = 0, heading_correction_ki = 0, heading_correction_kd = 0;
// Enable or disable the use of tracking wheels
bool using_horizontal_tracker = false;
bool using_vertical_tracker = false;
// IGNORE THESE IF YOU ARE NOT USING TRACKING WHEELS
double horizontal_tracker_dist_from_center = 0;
double vertical_tracker_dist_from_center = 0;
double horizontal_tracker_diameter = 2.75; // in
double vertical_tracker_diameter = 2; // in
// ── Distance sensor position-reset offsets (not used this year) ─────────
double front_sensor_offsetX = 0.0;
double front_sensor_offsetY = 0.0;
double left_sensor_offsetX = 0.0;
double left_sensor_offsetY = 0.0;
double right_sensor_offsetX = 0.0;
double right_sensor_offsetY = 0.0;
double back_sensor_offsetX = 0.0;
double back_sensor_offsetY = 0.0;
double distance_between_sensors = 0.0;
// ============================================================================
// ADVANCED TUNING (OPTIONAL)
// ============================================================================
bool heading_correction = true; // Use heading correction when the bot is stationary
bool dir_change_start = true;
bool dir_change_end = true;
double min_output = 5;
double max_slew_accel_fwd = 24;
double max_slew_decel_fwd = 24;
double max_slew_accel_rev = 24;
double max_slew_decel_rev = 24;
double chase_power = 2.5;
// ============================================================================
// DO NOT CHANGE ANYTHING BELOW
// ============================================================================
bool RemoteControlCodeEnabled = true;
// ── DRIVE SCHEME ─────────────────────────────────────────────────────────
// TANK — Axis3 left side, Axis2 right side
// ARCADE — Axis3 forward, Axis4 turn (both on left stick)
// SPLIT_ARCADE — Axis3 forward, Axis1 turn (left stick fwd, right stick turn)
DriveScheme drive_scheme = SPLIT_ARCADE;
/**
o   Used to initialize code/tasks/devices added using tools in VEXcode Pro.
o   This should be called at the start of your int main function.
/
•   void vexcodeInit(void) {
o   // nothing to initialize
•   }
/** */
