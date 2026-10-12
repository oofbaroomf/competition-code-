#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "pid.h"
#include "motor-control.h"
#include "robot-config.h"
#include "utils.h"
#include "vex.h"
#include "autotune.h"

using namespace vex;

extern bool tuner_active;
extern bool tuner_finished;
extern bool tuner_done_early;
extern double tuner_kp;
extern double tuner_kd;
extern double tuner_cost;
extern char tuner_label[16];

// ============================================================================
// V5 MACROS
// ============================================================================

#define waitUntil(condition) \
  do { wait(5, msec); } while (!(condition))

#define repeat(iterations) \
  for (int iterator = 0; iterator < iterations; iterator++)

// ============================================================================
// UTILITY
// ============================================================================

void initializeRandomSeed() {
  int systemTime = Brain.Timer.systemHighResolution();
  double batteryCurrent = Brain.Battery.current();
  double batteryVoltage = Brain.Battery.voltage(voltageUnits::mV);

  int seed = int(batteryVoltage + batteryCurrent * 100) + systemTime;
  srand(seed);
}

// ============================================================================
// DRIVE CURVE
// ============================================================================
int apply_drive_curve(int raw) {
  const double c = 2.0;            // lower = gentler atan slope (was 3)
  const double blend = 0.8;        // mostly squared curve (was 0.5)
  const double deadband = 8.0;     // raw units
  const double max_output = 80.0;  // top speed cap in percent (was 90)

  if (fabs((double)raw) < deadband) {
    return 0;
  }

  // Normalize stick to -1..1 (V5 axis max is 127)
  double x = (double)raw / 127.0;
  if (x > 1.0) x = 1.0;
  if (x < -1.0) x = -1.0;

  double s = (x > 0) ? 1.0 : -1.0;   // x/|x|
  double ax = fabs(x);

  // Dual atan curve from your Desmos graph
  double atan_curve = atan((2.0 * x - s) * c) / (2.0 * atan(c)) + s / 2.0;
  atan_curve = fabs(atan_curve);

  // Squared curve for a steady, gradual build
  double squared = ax * ax;

  double y = (1.0 - blend) * atan_curve + blend * squared;

  double out = round(y * max_output);
  return (s > 0) ? (int)out : -(int)out;
}

// ============================================================================
// DISPLAY / STATUS TASK
// ============================================================================

void drawWatermark() {
  Brain.Screen.setFont(monoXXL);
  Brain.Screen.setPenColor(white);
  Brain.Screen.printAt(330, 230, "RW V2");
  Brain.Screen.setFont(mono15);
}

bool last_tuner_active = false;

void update() {
  double heading = normalizeTarget(getInertialHeading());

  bool showingTuner = tuner_active || tuner_finished;
  bool modeChanged = (showingTuner != last_tuner_active);

  if (modeChanged) {
    controller_1.Screen.clearScreen();
    last_tuner_active = showingTuner;
  }

  if (showingTuner) {
    controller_1.Screen.setCursor(1, 1);
    controller_1.Screen.print(
      tuner_done_early ? "GOOD VALUES!   " :
      (tuner_finished ? "TUNE DONE      " : "TUNING...      ")
    );

    controller_1.Screen.setCursor(2, 1);
    controller_1.Screen.print("Kp:%.3f Kd:%.3f  ", tuner_kp, tuner_kd);
  } else {
    controller_1.Screen.setCursor(1, 1);
    controller_1.Screen.print("Hdg: %.1f      ", inertial_sensor.heading());

    controller_1.Screen.setCursor(2, 1);
    controller_1.Screen.print(
      "Winch: %.1f    ",
      motor_winch.position(degrees)
    );

    controller_1.Screen.setCursor(3, 1);
    controller_1.Screen.print(
      "Batt: %d%%      ",
      Brain.Battery.capacity()
    );
  }

  Brain.Screen.clearScreen();

  if (showingTuner) {
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print(
      tuner_done_early ? "GOOD VALUES FOUND" :
      (tuner_finished ? "TUNING COMPLETE" : "TUNING IN PROGRESS")
    );

    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("%s  cost: %.3f", tuner_label, tuner_cost);

    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Kp: %.4f", tuner_kp);

    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("Kd: %.4f", tuner_kd);
  } else {
    Brain.Screen.print(
      "ODOM  x:%.1f y:%.1f h:%.1f\n",
      x_pos, y_pos, heading
    );

    Brain.Screen.print(
      "WINCH POSITION: %.1f\n",
      motor_winch.position(degrees)
    );

    Brain.Screen.print(
      "WINCH HALF: %.1f\n",
      motor_winchhalf.position(degrees)
    );

    Brain.Screen.print(
      "CLAW (ENCODER): %.1f\n",
      motor_clawhalf.position(degrees)
    );
  }

  drawWatermark();
}

int updateTask() {
  while (true) {
    update();
    this_thread::sleep_for(100);
  }

  return 0;
}

void logPositionHandler() {
  printf(
    "X: %.2f Y: %.2f Heading: %.2f\n",
    x_pos, y_pos, inertial_sensor.heading()
  );

  printf(
    "WINCH POSITION: %.2f\n",
    motor_winch.position(degrees)
  );

  printf(
    "CLAW: %.2f\n",
    motor_clawhalf.position(degrees)
  );
}

int logPosition() {
  printf("\nBEGIN RUN\n");

  while (true) {
    logPositionHandler();
    this_thread::sleep_for(100);
  }

  return 0;
}

void trackPosition() {
  resetChassis();
  trackOdom();
}

int trackPositionTask() {
  trackPosition();
  return 0;
}

// ============================================================================
// SUBSYSTEM SETTINGS
// ============================================================================

// WINCH

double WINCH_Y_POSITION = 200;
double WINCH_Y_TOLERANCE = 3.0;

// CLAW

double CLAW_TARGET_90 = -230;
double CLAW_Y_TARGET = -550;
double CLAW_Y_SPEED = 100;

double CLAW_A_LIMIT = 178;
double CLAW_A_SPEED = 100;

double CLAW_R2_LIMIT = 300;
double CLAW_R2_SPEED = 100;

// ============================================================================
// R2 STATE MACHINE
// ============================================================================

enum R2State {
  R2_IDLE,
  R2_OUTTAKING
};

R2State r2State = R2_IDLE;
int r2OuttakeTicks = 0;

// ============================================================================
// Y STATE
// ============================================================================

bool winchGoingToY = false;

// ============================================================================
// START R2
// ============================================================================

void startR2Macro() {
  // R2 starts the winch going UP and starts the outtake together.
  r2OuttakeTicks = 25;  // About 1/3 of the previous upward travel
  r2State = R2_OUTTAKING;
}

// ============================================================================
// UPDATE R2
// ============================================================================

void updateR2Macro() {
  if (r2State != R2_OUTTAKING) {
    return;
  }

  // WINCH UP
  motor_winch.setVelocity(80, percent);
  motor_winchhalf.setVelocity(80, percent);

motor_winch.spin(forward);
motor_winchhalf.spin(reverse);

  // OUTTAKE SIMULTANEOUSLY
  motor_clawhalfflex.setVelocity(100, percent);
  motor_clawhalfflex.spin(reverse);

  r2OuttakeTicks--;

  // Stop when the outtake interval finishes.
  if (r2OuttakeTicks <= 0) {
    motor_winch.stop(hold);
    motor_winchhalf.stop(hold);

    r2State = R2_IDLE;
  }
}

// ============================================================================
// WINCH UP / DOWN / HOLD
// ============================================================================

void liftyup() {
  motor_winch.setVelocity(80, percent);
  motor_winchhalf.setVelocity(80, percent);

  motor_winch.spin(reverse);
  motor_winchhalf.spin(forward);
}

void liftydown() {
  motor_winch.setVelocity(80, percent);
  motor_winchhalf.setVelocity(80, percent);

  motor_winch.spin(forward);
  motor_winchhalf.spin(reverse);
}

void liftyhold() {
  motor_winch.stop(hold);
  motor_winchhalf.stop(hold);
}

// ============================================================================
// START Y
// ============================================================================

void startYMacro() {
  // Cancel R2.
  r2State = R2_IDLE;

  // Move the claw to the original Y target.
  motor_clawhalf.setVelocity(CLAW_Y_SPEED, percent);
  motor_clawhalf.spinToPosition(CLAW_Y_TARGET, degrees, false);

  winchGoingToY = true;

  // Preserve the original Y winch targets.
  motor_winch.setVelocity(100, percent);
  motor_winchhalf.setVelocity(100, percent);

  motor_winch.spinToPosition(WINCH_Y_POSITION, degrees, false);
  motor_winchhalf.spinToPosition(-WINCH_Y_POSITION, degrees, false);
}

// ============================================================================
// UPDATE Y
// ============================================================================

void updateYMacro() {
  if (!winchGoingToY) {
    return;
  }

  double winchPosition = motor_winch.position(degrees);
  double winchHalfPosition = motor_winchhalf.position(degrees);

  bool winchArrived =
    fabs(winchPosition - WINCH_Y_POSITION) <= WINCH_Y_TOLERANCE;

  bool winchHalfArrived =
    fabs(winchHalfPosition + WINCH_Y_POSITION) <= WINCH_Y_TOLERANCE;

  // Finish when both winch motors reach their targets.
  if (winchArrived && winchHalfArrived) {
    motor_winch.stop(hold);
    motor_winchhalf.stop(hold);

    winchGoingToY = false;
  }
}

// ============================================================================
// RIGHT ARROW: CLAW TO 90
// ============================================================================

void rightArrowMacro() {
  motor_clawhalf.setVelocity(100, percent);
  motor_clawhalf.spinToPosition(CLAW_TARGET_90, degrees, false);
}

// ============================================================================
// DRIVER CONTROL
// ============================================================================

void telop() {
  vex::task updater(updateTask);
  vex::task tracking(trackPositionTask);
  vex::task logposition(logPosition);

  update();

  inertial_sensor.setRotation(0, degrees);

  left_chassis.setStopping(coast);
  right_chassis.setStopping(coast);

  bool r2Last = false;
  bool yLast = false;
  bool aLast = false;
  bool rightLast = false;
  bool l1Last = false;          // new
  bool intakeRunning = false;   // new
  while (true) {
    // DRIVE

    int left_pct = 0;
    int right_pct = 0;

    switch (drive_scheme) {
      case SPLIT_ARCADE:
      default: {
        int fwd = apply_drive_curve(controller_1.Axis1.position());
        int turn = apply_drive_curve(controller_1.Axis3.position());

        left_pct = fwd + turn;
        right_pct = fwd - turn;
        break;
      }
    }

    if (left_pct > 100) left_pct = 100;
    if (left_pct < -100) left_pct = -100;

    if (right_pct > 100) right_pct = 100;
    if (right_pct < -100) right_pct = -100;

    left_chassis.setVelocity(left_pct, percent);
    right_chassis.setVelocity(right_pct, percent);

    left_chassis.spin(forward);
    right_chassis.spin(forward);

    // BUTTONS

    bool l1 = controller_1.ButtonL1.pressing();
    bool l2 = controller_1.ButtonL2.pressing();
    bool r1 = controller_1.ButtonR1.pressing();
    bool r2 = controller_1.ButtonR2.pressing();

    bool buttonA = controller_1.ButtonA.pressing();
    bool buttonY = controller_1.ButtonY.pressing();
    bool buttonRight = controller_1.ButtonRight.pressing();

    // Y BUTTON

    if (buttonY && !yLast) {
      r2State = R2_IDLE;

      motor_winch.stop(coast);
      motor_winchhalf.stop(coast);
      motor_clawhalfflex.stop(coast);

      startYMacro();
    }

    yLast = buttonY;

    // R2 START: PRESS ONCE

    if (r2 && !r2Last) {
      if (r2State == R2_IDLE && !winchGoingToY) {
        startR2Macro();
      }
    }

    r2Last = r2;

    // UPDATE Y

    if (winchGoingToY) {
      updateYMacro();
    }

    // UPDATE R2
    // Manual winch controls take priority.

    if (!winchGoingToY && !r1 && !l2) {
      updateR2Macro();
    }

    // CLAWHALFFLEX
    // Intake at 75% except during R2 outtake.

    if (r2State == R2_OUTTAKING) {
      motor_clawhalfflex.setVelocity(100, percent);
      motor_clawhalfflex.spin(reverse);
    } else {
      motor_clawhalfflex.setVelocity(75, percent);
      motor_clawhalfflex.spin(forward);
    }

    // WINCH CONTROL

    if (r1) {
      winchGoingToY = false;
      r2State = R2_IDLE;

      liftyup();

    } else if (l2) {
      winchGoingToY = false;
      r2State = R2_IDLE;

      liftydown();

    } else if (winchGoingToY) {
      // Y position commands control the winch.

    } else if (r2State == R2_OUTTAKING) {
      // updateR2Macro() controls the winch moving up.

    } else {
      liftyhold();
    }

    // CLAW A

    if (buttonA) {
      if (motor_clawhalf.position(degrees) < CLAW_A_LIMIT) {
        motor_clawhalf.setVelocity(CLAW_A_SPEED, percent);
        motor_clawhalf.spin(forward);
      }

      aLast = true;

    } else if (aLast) {
      motor_clawhalf.stop(hold);
      aLast = false;
    }

    // INTAKE

    if (l1 && !l1Last) {
      intakeRunning = !intakeRunning;
    }
    l1Last = l1;

    if (intakeRunning) {
      motor_intakehalf.spin(reverse, 100, percent);
    } else {
      motor_intakehalf.stop(hold);
    }

    // RIGHT ARROW

    if (buttonRight && !rightLast) {
      rightArrowMacro();
    }

    rightLast = buttonRight;

    this_thread::sleep_for(20);
  }
}

// ============================================================================
// AUTONOMOUS
// ============================================================================

void auton() {
  vex::task tracking(trackPositionTask);
  vex::task updater(updateTask);
  vex::task logposition(logPosition);

  update();

  inertial_sensor.setRotation(0, degrees);
  correct_angle = normalizeTarget(0);
}

// ============================================================================
// MAIN
// ============================================================================

int main() {
  competition Competition = competition();

  Competition.autonomous(auton);
  Competition.drivercontrol(telop);

  return 0;
}
