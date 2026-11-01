#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>


#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;


// START IQ MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END IQ MACROS


// Robot configuration code.
inertial BrainInertial = inertial();
controller Controller = controller();
motor Left_Motor = motor(PORT6, true);
motor Right_Motor = motor(PORT1, false);
motor L3MotorA = motor(PORT2, true);
motor L3MotorB = motor(PORT5, false);
motor_group L3 = motor_group(L3MotorA, L3MotorB);

motor L4 = motor(PORT4, false);
motor Take_bag = motor(PORT3, true);
touchled TouchLED12 = touchled(PORT12);
distance Distance7 = distance(PORT7);


// generating and setting random seed
void initializeRandomSeed(){
  wait(100,msec);
  double xAxis = BrainInertial.acceleration(xaxis) * 1000;
  double yAxis = BrainInertial.acceleration(yaxis) * 1000;
  double zAxis = BrainInertial.acceleration(zaxis) * 1000;
  // Combine these values into a single integer
  int seed = int(
    xAxis + yAxis + zAxis
  );
  // Set the seed
  srand(seed);
}



void vexcodeInit() {

  // Initializing random seed.
  initializeRandomSeed();
}


// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

#pragma endregion VEXcode Generated Robot Configuration

/*
 ==============================================================================
  TEAM ZENITH  -  AUTONOMOUS
 ==============================================================================
  CONFIG = EXACT COPY of the Zenith CONTROLLER project. The "VEXcode Generated Robot Configuration" block above must
  stay identical to the controller project's block (VEXcode rewrites it from the Devices panel).

    Name         Port   Reverse   What it is
    Left_Motor   6      true      drive motor  = PHYSICAL RIGHT wheel
    Right_Motor  1      false     drive motor  = PHYSICAL LEFT wheel
    L3MotorA/B   2 / 5  true/false shooter motors, group "L3" (rubber-band pull-back)
    Take_bag     3      true      intake
    L4           4      false     arm (UP = motor FORWARD, ARM_SIGN = +1)
    Distance7    7      -         shooter-loaded sensor (small mm = pulled back)
    TouchLED12   12     -         status LED + START button

  The motion code uses DL (physical LEFT) and DR (physical RIGHT); SWAP_DRIVE_SIDES below flips them
  (UNVERIFIED on the robot: if the first "turn LEFT" really turns RIGHT, set SWAP_DRIVE_SIDES to 1).

  HOW TO RUN: start-up finishes -> LED GREEN -> TOUCH the Touch LED -> the autonomous routine starts.
   FDown = ABORT at any time.   EUp = retry the start-up after a RED-blinking failure.
   LED blinking PURPLE = auto finished.   RED flashes x3 = low battery warning only (never blocks anything).

  START-UP: PURPLE = shooter pulled back + sensor confirms loaded, THEN arm drives down to the bottom (= 0) |
   YELLOW blinking = motor angles reset + gyro calibration (do not touch) | GREEN = all OK | RED blinking = a check failed.

  MOTION LIBRARY (mm, degrees): driveStraight / turnTo / turnLeft with ONE absolute heading (gyro), sqrt slow-down
   curves, learned coast lead, soft start, timeouts.

  PLAN: Phase 1 eat L4 -> Phase 2 leave L4 + turn + straight -> Phase 3 intake / back / forward / back ->
   Phase 4: back against the wall WHILE shooting (shot #1), forward 40 + back 35 + shot #2, then the SPAM LOOP
   (forward 55 -> back 45 -> back against the wall while shooting -> repeat) until FDown.
   SHOOTING = same logic as the controller (DK_Zenith.cpp), running in the BACKGROUND (l3Update, called from every
   waitMs): L3 spins at once, the sensor says when it fired (no fixed shot time), L3 never stops = the reload starts at
   once, and the pre-load (auto-learned) finishes while the robot already drives. Brain row 8 shows "trv" / "pre":
   copy the final "pre" into L3_PRELOAD_DEG so every run starts tuned.
 ==============================================================================
*/

// Drive sides used by the motion code (DL = physical LEFT wheel, DR = physical RIGHT wheel).
// 0 = DL is Right_Motor (PORT1, false), DR is Left_Motor (PORT6, true)   <- derived from the controller's A-C / A+C
// 1 = swapped: use it if the first "turn LEFT" turns RIGHT.
#define SWAP_DRIVE_SIDES 0
#if SWAP_DRIVE_SIDES
motor& DL = Left_Motor;
motor& DR = Right_Motor;
#else
motor& DL = Right_Motor;
motor& DR = Left_Motor;
#endif

// ============================================================
// TUNABLE VALUES  (tune in this order: 1 WHEEL_TRAVEL_MM, 2 turns, 3 straight)
// ============================================================

// --- Geometry ---------------------------------------------------------------
// !! MEASURE THIS FIRST. Run the straight test (1000 mm) and measure the real distance.
//    new WHEEL_TRAVEL_MM = old WHEEL_TRAVEL_MM * (real distance / 1000)
const double WHEEL_TRAVEL_MM  = 200.0;  // mm the robot moves per ONE wheel revolution (IQ standard wheel ~200)
const double DRIVE_GEAR_RATIO = 1.0;    // motor revolutions per wheel revolution (1.0 = direct)

// --- Drive (straight) -------------------------------------------------------
const double DRIVE_MAX_SPEED  = 85.0;   // % (leaves headroom for the heading correction)
const double DRIVE_MIN_SPEED  = 18.0;   // % never slower than this until arrived. Both wheels must move at this speed (no crawling = no drifting right at the end)
const double DRIVE_ACCEL_STEP = 4.0;    // % per 10 ms loop (smaller = softer start, less slip/drift)
const double DRIVE_DECEL_GAIN = 2.7;    // slow-down curve: speed = MIN + GAIN * sqrt(remaining motor-degrees)
                                        //   bigger = brakes later/harder (faster, may overshoot); smaller = brakes earlier (gentler, slower)
const double KP_HEAD          = 1.8;    // % per degree of heading error (raise if it drifts sideways, lower if it wiggles)
const double KD_HEAD          = 2.0;    // damping of the heading correction (raise if it wiggles)
const double KI_HEAD          = 0.02;   // self-learning wheel balance (fraction per deg*sec). Fixes the steady drift to one side
const double HEAD_TRIM_MAX    = 0.15;   // max wheel imbalance it may learn (0.15 = 15 %)
const double HEAD_TRIM_INIT   = 0.00;   // start value. + = robot drifted LEFT before / - = drifted RIGHT (normally leave 0: it learns by itself)
const double KI_HEAD_FAST     = 2.0;    // % per (deg*sec): fast integral inside ONE move (removes the leftover drift, esp. at low speed near the end)
const double HEAD_I_MAX       = 6.0;    // % max of that integral
const double MAX_HEAD_CORR    = 25.0;   // % max steering correction (also limited to 6 + 0.4 x current power)
const double DRIVE_LEAD_INIT  = 0.06;   // sec: stops commanding this much BEFORE the target (coasting). Learned per direction.
const double DRIVE_LEAD_MAX   = 0.30;   // sec: learning limit
const double DIST_TOL_MM      = 5.0;    // "arrived" distance tolerance: the robot STOPS as soon as it is this close
const double DIST_FIX_MM      = 15.0;   // after stopping: only correct again if still off by MORE than this
const double DRIVE_FIX_SPEED  = 20.0;   // % speed of that one correction (must be fast enough that BOTH wheels move)
const int    DRIVE_REST_MS    = 100;    // wait after stopping, before measuring

// --- Turn (smooth + precise, no jerky reversing) --------------------------------
const double GYRO_SCALE       = 1.00;   // real angle / angle shown by gyro. If a "90" really turns 87 -> 87/90 = 0.967
const double TURN_MAX_SPEED   = 34.0;   // % top turning speed
const double TURN_MIN_SPEED   = 12.0;   // % crawl speed near the target (raise if it stalls short, lower if it jumps)
const double TURN_DECEL_GAIN  = 2.0;    // slow-down curve: speed = MIN + GAIN * sqrt(remaining degrees)  (smaller = gentler/slower)
const double TURN_ACCEL_STEP  = 3.0;    // % per 10 ms loop (soft start, no wheel slip)
const double KD_TURN          = 0.10;   // % per (deg/sec): extra braking while still rotating fast toward the target (raise if it overshoots)
const double TURN_TOL_DEG     = 1.0;    // the robot STOPS as soon as it is this close (or passes the target)
const double TURN_FIX_TOL_DEG = 1.0;    // after stopping: only correct again if still off by MORE than this
const double TURN_NUDGE_SPEED = 16.0;   // % speed of a correction "nudge"
const int    TURN_NUDGES      = 2;      // max number of nudges (each one is short, then the robot rests)
const double TURN_LEAD_INIT   = 0.08;   // sec: stops commanding this much BEFORE the target (coasting). Learned per direction.
const double TURN_LEAD_MAX    = 0.25;   // sec: learning limit
const int    TURN_REST_MS     = 120;    // wait after stopping so inertia dies before measuring

// --- Start-up (same values/logic as the controller code's calibrateAll) -----------
const double ARM_SIGN         = 1.0;    // Zenith: arm goes UP when the motor spins FORWARD (+1) -> DOWN = reverse
const int    L4_INIT_MS       = 1000;   // arm drives DOWN by TIME to the bottom stop
const double L3_STOP_MM       = 35.0;   // shooter loaded when Distance7 reads this or less
const double L3_PULL_SPEED    = 100.0;  // start-up pull-back speed (lower it, e.g. 60, if it overshoots the sensor and fires)
const int    L3_PULL_TIMEOUT_MS = 5000; // give up one pull-back try after this long
const int    L3_SETTLE_MS     = 400;    // after stopping, wait then re-check the shooter is STILL loaded
const int    L3_PULL_TRIES    = 3;      // max pull-back tries
const double ARM_HOME_TOL_DEG = 15.0;   // final check: arm must read within this of 0

// --- Shooter (same logic as the controller FUp) -------------------------------------
const int    L3_FAR_MS          = 60;     // sensor FAR this long in a row = L3 has released (fired) / needs a reload
const int    L3_FIRE_MIN_MS     = 200;    // ignore "fired" for this long after the shot starts (sensor noise)
const int    L3_FIRE_TIMEOUT_MS = 2000;   // safety: nothing fired after this long -> stop spinning and hold
const int    L3_RELOAD_TIMEOUT_MS = 4000; // safety: reload gives up after this (also the max wait for "loaded" before a shot)
// PRE-LOAD: after the sensor catch, pull MORE degrees and hold, so the next shot has no pull-down delay.
// AUTO (1) = learns it by itself, growing by at most L3_PRELOAD_STEP per shot (never jumps past the release).
// Start value: copy the "pre" number from Brain row 8 here after a run, so the next run starts already tuned.
#define L3_PRELOAD_AUTO 1
const double L3_PRELOAD_DEG    = 0.0;     // start value (AUTO 1) or fixed value (AUTO 0)
const double L3_PRELOAD_MARGIN = 50.0;    // degrees still left between the held position and the release (raise if it fires by itself, lower to cut the delay)
const double L3_PRELOAD_MAX    = 500.0;   // safety cap for the learned value
const double L3_PRELOAD_STEP   = 40.0;    // pre-load grows by at most this much per shot
const int    L3_PRELOAD_TIMEOUT_MS = 1500; // safety for the extra pull

// --- Arm (same numbers as the controller code) ------------------------------------
const double L4_UP_POS         = 600.0;   // arm up height (deg, up = positive) = controller value
const double L4_UP_SPEED       = 100.0;
const double L4_TOL_DEG        = 10.0;
const int    L4_UP_TIMEOUT_MS  = 4000;
const int    L4_STALL_GRACE_MS = 300;     // stall check (same as the controller): ignore the first ms of a move
const int    L4_STALL_CHECK_MS = 100;     // how often the arm position is compared
const double L4_STALL_DEG      = 3.0;     // less than this many degrees per check = stalled -> the move ends at once

// --- Battery warning (WARNING ONLY: never stops or blocks anything) ----------------
const int    LOW_BATTERY_PCT  = 30;     // Brain battery at or below this % = low
const int    BATTERY_FLASHES  = 3;      // red flashes per warning
const int    BATTERY_REPEAT_MS = 10000; // while waiting for the start touch: warn again every this many ms

// --- Common -----------------------------------------------------------------
const int    LOOP_MS          = 10;     // control loop period
const int    MOVE_PAUSE_MS    = 120;    // pause after each move so the robot is still

// ============================================================
// STATE
// ============================================================
double targetHeading = 0.0;   // the ONE absolute heading the robot should hold (deg, + = clockwise)
double driveLead[2] = { DRIVE_LEAD_INIT, DRIVE_LEAD_INIT };   // learned coast time, [0] = forward, [1] = backward
double turnPeakOver = 0.0;       // how far the last turn went PAST the target (deg), for the Brain screen
double headTrim = HEAD_TRIM_INIT;                 // learned wheel balance (drive)
double turnLead[2] = { TURN_LEAD_INIT, TURN_LEAD_INIT };   // learned coast time, [0] = right/clockwise, [1] = left
bool   autoRunning = false;       // true while the full autonomous routine runs
bool   autoAborted = false;       // set when FDown is pressed during the routine: every later step is skipped
int    logRow = 2;            // rolling Brain-screen row for move results

// shooter state machine (runs inside l3Update)
bool   L3_holding = true;         // L3 stopped with hold at the loaded position (start-up leaves it that way)
bool   L3_firing = false;         // a shot is in progress (spins until the sensor says it fired)
bool   L3_preloading = false;     // extra pull after the sensor caught it
bool   L3_preHeld = false;        // holding at the pre-loaded position (a FAR reading now = it released by itself)
bool   L3_preloadLocked = false;  // L3 once released by itself: the pre-load may only get smaller from now on
bool   L3_skipLearn = false;      // this shot must not change the learned pre-load
bool   L3_gaveUp = false;         // reload timed out (jam): L3 stays stopped until the sensor reads loaded or a new shot
double L3_preloadDeg = L3_PRELOAD_DEG;   // current pre-load (learned when AUTO)
double L3_preloadTarget = 0.0;
double L3_catchPos = 0.0;         // L3 position where the sensor caught it (0 right after start-up)
double L3_fireStartPos = 0.0;     // L3 position when the shot started
double L3_farPos = 0.0;           // L3 position at the first FAR reading (= where it released)
double L3_travel = 0.0;           // degrees from the held position to the release on the last shot
int    L3_farSince = -1;          // ms timestamp of the first FAR reading in a row (-1 = not FAR)
int    L3_fireMs = 0, L3_reloadMs = 0, L3_preMs = 0;

// ============================================================
// HELPERS
// ============================================================
void l3Update();   // defined with the shooter code below

// every wait in the motion code goes through here, so the shooter keeps reloading while the robot drives
void waitMs(int ms) {
  int t = 0;
  while (t < ms) {
    l3Update();
    wait(10, msec);
    t += 10;
  }
}

double clampd(double v, double limit) {
  if (v > limit) return limit;
  if (v < -limit) return -limit;
  return v;
}

double signd(double v) { return (v >= 0.0) ? 1.0 : -1.0; }

// FDown pressed while the routine runs = ABORT (everything stops, remaining steps are skipped)
bool abortNow() {
  if (autoRunning && Controller.ButtonFDown.pressing()) { autoAborted = true; }
  return autoAborted;
}

double heading() { return BrainInertial.rotation(degrees) * GYRO_SCALE; }

// Motor degrees per mm of robot travel
double degPerMM() { return 360.0 * DRIVE_GEAR_RATIO / WHEEL_TRAVEL_MM; }

void stopDrive() {
  DL.stop(brake);
  DR.stop(brake);
}

// wait until the gyro stops changing (robot really still), at least minMs, at most maxMs
void waitStill(int minMs, int maxMs) {
  waitMs(minMs);
  double prev = heading();
  int t = 0;
  while (t < maxMs) {
    waitMs(30);
    double h = heading();
    if (fabs(h - prev) < 0.15) break;
    prev = h;
    t += 30;
  }
}

// Same velocity ratio for both sides; if either side is over 100 %, scale BOTH (keeps the path straight)
void setDrive(double l, double r) {
  double m = fmax(fabs(l), fabs(r));
  if (m > 100.0) { l = l * 100.0 / m; r = r * 100.0 / m; }
  DL.setVelocity(l, percent);
  DR.setVelocity(r, percent);
  DL.spin(forward);
  DR.spin(forward);
}

// row 7: what the robot has learned so far (copy these into the *_INIT constants to start the next run already tuned)
void showLearned() {
  Brain.Screen.setCursor(7, 1);
  Brain.Screen.print("L D%.2f/%.2f T%.2f/%.2f b%.3f   ", driveLead[0], driveLead[1], turnLead[0], turnLead[1], headTrim);
}

void logDrive(double mm, double errMM, double errHead) {
  Brain.Screen.setCursor(logRow, 1);
  Brain.Screen.print("DRV %5.0f d%.0f h%.1f      ", mm, errMM, errHead);
  logRow++; if (logRow > 6) logRow = 2;
  showLearned();
}

void logTurn(double angle, double err, int ms) {
  Brain.Screen.setCursor(logRow, 1);
  Brain.Screen.print("TRN %4.0f e%.1f ov%.1f %dms  ", angle, err, turnPeakOver, ms);
  logRow++; if (logRow > 6) logRow = 2;
  showLearned();
}

// ============================================================
// SETUP: gyro calibration + zero
// ============================================================
void zeroHeading() {
  BrainInertial.setRotation(0.0, degrees);
  targetHeading = 0.0;
}

void initDrive() {
  DL.setStopping(brake);
  DR.setStopping(brake);
  DL.setMaxTorque(100.0, percent);
  DR.setMaxTorque(100.0, percent);
  DL.setVelocity(0.0, percent);
  DR.setVelocity(0.0, percent);
  DL.stop();
  DR.stop();
}

// ------------------------------------------------------------
// START-UP SEQUENCE   PURPLE -> YELLOW (blinking) -> GREEN   (RED blinking = failed, press EUp to retry)
// ------------------------------------------------------------
void blinkWait(int ms, color c1, color c2) {
  int t = 0;
  bool f = false;
  while (t < ms) {
    TouchLED12.setColor(f ? c1 : c2);
    f = !f;
    wait(200, msec);
    t += 200;
  }
}

// LOW BATTERY = quick RED flashes + a line on the Brain screen, then everything carries on as normal.
// Only called while the robot is idle (never during the start-up sequence or the auto).
bool batteryWarn(color restoreColor) {
  int pct = (int)Brain.Battery.capacity();
  if (pct > LOW_BATTERY_PCT) { return false; }
  Brain.Screen.setCursor(8, 1);
  Brain.Screen.print("LOW BATTERY %d%% - charge!  ", pct);
  for (int i = 0; i < BATTERY_FLASHES; i++) {
    TouchLED12.setColor(red);   wait(150, msec);
    TouchLED12.setColor(black); wait(150, msec);
  }
  TouchLED12.setColor(restoreColor);
  return true;
}

bool shooterLoaded() {   // 2 reads in a row <= L3_STOP_MM
  int good = 0;
  for (int i = 0; i < 2; i++) {
    if (Distance7.objectDistance(mm) <= L3_STOP_MM) { good++; }
    wait(10, msec);
  }
  return good == 2;
}

// Pull shooter back until Distance7 <= L3_STOP_MM, HOLD, wait, confirm it is STILL there.
bool shooterPullBackConfirmed() {
  if (!shooterLoaded()) {                     // already loaded -> do not spin it at all
    L3.setVelocity(L3_PULL_SPEED, percent);
    L3.spin(forward);
    int t = 0;
    int good = 0;
    while (t < L3_PULL_TIMEOUT_MS && good < 2) {
      if (Distance7.objectDistance(mm) <= L3_STOP_MM) { good++; } else { good = 0; }
      wait(10, msec);                         // polled every 10 ms so it cannot overshoot and fire
      t += 10;
    }
  }
  L3.stop(hold);
  L3.setVelocity(100.0, percent);
  wait(L3_SETTLE_MS, msec);
  for (int i = 0; i < 5; i++) {               // must stay at the sensor
    if (Distance7.objectDistance(mm) > L3_STOP_MM) { return false; }
    wait(20, msec);
  }
  return true;
}

// returns true only if every check passed
bool startupSequence() {
  // ---------- PHASE 1 : PURPLE ----------
  TouchLED12.setColor(purple);
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1); Brain.Screen.print("1/3 Shooter pull-back...");

  DL.stop(); DR.stop(); L3.stop(); Take_bag.stop(); L4.stop();
  DL.setStopping(brake); DR.setStopping(brake);
  L3.setStopping(brake); L4.setStopping(brake); Take_bag.setStopping(coast);
  L3.setMaxTorque(100.0, percent); L4.setMaxTorque(100.0, percent);
  Take_bag.setMaxTorque(100.0, percent);
  L3.setVelocity(100.0, percent); L4.setVelocity(100.0, percent); Take_bag.setVelocity(100.0, percent);

  bool shooterOk = false;
  for (int i = 0; i < L3_PULL_TRIES && !shooterOk; i++) { shooterOk = shooterPullBackConfirmed(); }
  Brain.Screen.setCursor(2, 1);
  if (!shooterOk) {
    Brain.Screen.print("FAIL: shooter NOT loaded (arm not moved)");
    return false;                              // arm is NOT moved: it could knock the shooter loose
  }
  Brain.Screen.print("Shooter loaded + holding: OK");

  Brain.Screen.setCursor(3, 1); Brain.Screen.print("2/3 Arm down to bottom...");
  if (ARM_SIGN > 0) { L4.spin(reverse); } else { L4.spin(forward); }   // DOWN, whichever way it is mounted
  wait(L4_INIT_MS, msec);
  L4.stop(hold);
  wait(300, msec);
  L4.setPosition(0.0, degrees);               // bottom = 0
  Brain.Screen.setCursor(4, 1); Brain.Screen.print("Arm at bottom, set to 0: OK");

  // ---------- PHASE 2 : YELLOW BLINKING ----------
  Brain.Screen.setCursor(5, 1); Brain.Screen.print("3/3 Reset + gyro (stay still)");
  TouchLED12.setColor(yellow);
  blinkWait(600, yellow, black);                 // robot settles before the gyro starts

  // every motor angle = 0
  L4.setPosition(0.0, degrees);
  DL.setPosition(0.0, degrees);
  DR.setPosition(0.0, degrees);
  L3MotorA.setPosition(0.0, degrees);
  L3MotorB.setPosition(0.0, degrees);
  Take_bag.setPosition(0.0, degrees);
  DL.setVelocity(0.0, percent);
  DR.setVelocity(0.0, percent);

  // gyro
  BrainInertial.calibrate();
  bool f = false;
  int guard = 0;
  while (BrainInertial.isCalibrating() && guard < 200) {   // max ~10 s
    TouchLED12.setColor(f ? yellow : black);
    f = !f;
    wait(250, msec);
    guard++;
  }
  zeroHeading();
  blinkWait(400, yellow, black);

  // ---------- VERIFY EVERYTHING ----------
  bool okShooter = shooterLoaded();                         // still loaded after the arm moved
  bool okArm     = fabs(L4.position(degrees)) <= ARM_HOME_TOL_DEG;
  bool okGyro    = (guard < 200) && fabs(BrainInertial.rotation(degrees)) < 0.5;
  bool okDrive   = fabs(DL.position(degrees)) < 5.0 && fabs(DR.position(degrees)) < 5.0;

  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1); Brain.Screen.print("Shooter loaded : %s", okShooter ? "OK" : "FAIL");
  Brain.Screen.setCursor(2, 1); Brain.Screen.print("Arm at 0       : %s", okArm ? "OK" : "FAIL");
  Brain.Screen.setCursor(3, 1); Brain.Screen.print("Gyro zero      : %s", okGyro ? "OK" : "FAIL");
  Brain.Screen.setCursor(4, 1); Brain.Screen.print("Drive encoders : %s", okDrive ? "OK" : "FAIL");
  return okShooter && okArm && okGyro && okDrive;
}

// repeat until OK. On failure: RED blinking, press EUp to retry. GREEN only when all OK.
void startupUntilReady() {
  while (!startupSequence()) {
    Brain.Screen.setCursor(6, 1); Brain.Screen.print("Press EUp to retry");
    while (!Controller.ButtonEUp.pressing()) { blinkWait(400, red, black); }
    while (Controller.ButtonEUp.pressing()) { wait(20, msec); }
  }
  Brain.Screen.setCursor(6, 1); Brain.Screen.print("ALL OK - READY");
  TouchLED12.setColor(green);
}

// ============================================================
// DRIVE STRAIGHT  (+mm forward, -mm backward) - holds targetHeading
// ============================================================
// One pass toward targetDeg (motor-degrees from the position set at the start of the move).
// Slows down smoothly to DRIVE_MIN_SPEED (both wheels keep moving), then cuts power + brakes when it is inside the
// tolerance, passes the target, or is "close enough that coasting would finish the job" (lead = learned coast time).
// stopRate (out) = speed toward the target when it stopped (motor-deg/sec), stopPos (out) = position then.
void drivePass(double targetDeg, double maxSpeed, int timeoutMs, double lead, double* stopRate, double* stopPos) {
  const double tolDeg = DIST_TOL_MM * degPerMM();
  const int N = 5;                                    // speeds are measured over 5 loops = no noise
  double hist[N];
  double posHist[N];
  const double pos0 = (DL.position(degrees) + DR.position(degrees)) / 2.0;
  for (int i = 0; i < N; i++) { hist[i] = targetHeading - heading(); posHist[i] = pos0; }
  int    idx      = 0;
  double lastPower = 0.0;
  double headI    = 0.0;                              // fast heading integral (this move only)
  int    t        = 0;
  const double dir = signd(targetDeg - pos0);
  *stopRate = 0.0;
  *stopPos  = pos0;

  while (t < timeoutMs) {
    if (abortNow()) break;
    double pos     = (DL.position(degrees) + DR.position(degrees)) / 2.0;
    double err     = targetDeg - pos;
    double headErr = targetHeading - heading();

    double rate = (pos - posHist[idx]) * 1000.0 / (LOOP_MS * N);   // motor-deg/sec
    posHist[idx] = pos;
    double approach = rate * dir;                                  // + = moving toward the target

    double leadDeg = (approach > 0.0) ? approach * lead : 0.0;     // how far it will still coast
    if (fabs(err) <= tolDeg + leadDeg || err * dir < 0.0) {        // arrived / coast will finish it / passed -> STOP
      *stopRate = (approach > 0.0) ? approach : 0.0;
      *stopPos  = pos;
      break;
    }

    // distance: constant-deceleration curve that reaches DRIVE_MIN_SPEED exactly at the tolerance edge
    double remaining = fmax(0.0, fabs(err) - tolDeg);
    double power = dir * fmin(maxSpeed, DRIVE_MIN_SPEED + DRIVE_DECEL_GAIN * sqrt(remaining));
    // soft start: limit how fast the power may GROW (slowing down is never limited)
    if (fabs(power) > fabs(lastPower) + DRIVE_ACCEL_STEP) power = dir * (fabs(lastPower) + DRIVE_ACCEL_STEP);
    lastPower = power;

    // heading: PD on the gyro (corr > 0 turns the robot clockwise)
    double headRate = (headErr - hist[idx]) / N;           // deg per loop
    hist[idx] = headErr;
    idx = (idx + 1) % N;

    // (a) slow self-learning wheel balance, proportional to the signed power (forward AND backward)
    if (fabs(headErr) < 6.0) {
      headTrim = clampd(headTrim + KI_HEAD * headErr * signd(power) * (LOOP_MS / 1000.0), HEAD_TRIM_MAX);
      // (b) fast integral for THIS move: removes what is left, even at low speed near the end
      headI = clampd(headI + KI_HEAD_FAST * headErr * (LOOP_MS / 1000.0), HEAD_I_MAX);
    }
    double corr = KP_HEAD * headErr + KD_HEAD * headRate + headI + headTrim * power;
    corr = clampd(corr, fmin(MAX_HEAD_CORR, 6.0 + 0.4 * fabs(power)));

    setDrive(power + corr, power - corr);

    waitMs(LOOP_MS);
    t += LOOP_MS;
  }
  stopDrive();
}

void driveStraight(double mm, double maxSpeed = DRIVE_MAX_SPEED) {
  const double targetDeg = mm * degPerMM();
  const int    li  = (mm >= 0.0) ? 0 : 1;                 // which direction's learned lead to use
  const double dir = signd(mm);
  double stopRate = 0.0, stopPos = 0.0;

  DL.setPosition(0.0, degrees);
  DR.setPosition(0.0, degrees);

  drivePass(targetDeg, maxSpeed, 1500 + (int)(fabs(mm) * 5.0), driveLead[li], &stopRate, &stopPos);
  if (autoAborted) return;
  waitMs(DRIVE_REST_MS);

  double pos = (DL.position(degrees) + DR.position(degrees)) / 2.0;

  // learn how far the robot still travels after the stop command in THIS direction
  if (stopRate > 60.0) {
    double coast = (pos - stopPos) * dir;
    double measured = clampd(coast / stopRate, DRIVE_LEAD_MAX);
    if (measured < 0.0) measured = 0.0;
    driveLead[li] = 0.5 * driveLead[li] + 0.5 * measured;
  }

  // at most ONE correction, only if clearly off (undershoot only: backing up after an overshoot just makes it wobble)
  double err = targetDeg - pos;
  if (err * dir > DIST_FIX_MM * degPerMM()) {
    drivePass(targetDeg, DRIVE_FIX_SPEED, 700, 0.0, &stopRate, &stopPos);
    waitMs(DRIVE_REST_MS);
    err = targetDeg - (DL.position(degrees) + DR.position(degrees)) / 2.0;
  }
  logDrive(mm, err / degPerMM(), targetHeading - heading());
  waitMs(MOVE_PAUSE_MS);
}

// ============================================================
// TURN TO ABSOLUTE ANGLE  (0 = start direction, + = clockwise / right)
// ============================================================
// One pass toward "angle". It STOPS commanding as soon as the robot is inside the tolerance, passes the
// target, or is "close enough that coasting will finish the job" (lead = learned coast time x turning speed).
// It never reverses to brake; it only takes power away while rotating fast.
// stopRate (out) = turning speed toward the target when it stopped (deg/sec), stopHeading (out) = heading then.
void turnPass(double angle, double maxSpeed, int timeoutMs, double lead, double* stopRate, double* stopHeading) {
  const int N = 5;                    // turning speed is measured over 5 loops (50 ms) = no noise from the gyro
  double hist[N];
  for (int i = 0; i < N; i++) hist[i] = heading();
  int    idx     = 0;
  double lastPow = 0.0;
  int    t       = 0;
  const double dir = signd(angle - heading());
  *stopRate = 0.0;
  *stopHeading = heading();

  while (t < timeoutMs) {
    if (abortNow()) break;
    double h   = heading();
    double err = angle - h;

    double rate = (h - hist[idx]) * 1000.0 / (LOOP_MS * N);   // deg/sec, + = clockwise
    hist[idx] = h;
    idx = (idx + 1) % N;
    double approach = rate * dir;                              // + = rotating toward the target

    double leadDeg = (approach > 0.0) ? approach * lead : 0.0; // how far it will still coast
    if (fabs(err) <= TURN_TOL_DEG + leadDeg || err * dir < 0.0) {   // arrived / coast will finish it / passed -> stop
      *stopRate = (approach > 0.0) ? approach : 0.0;
      *stopHeading = h;
      break;
    }

    double over = -err * dir;                                  // + = already past the target
    if (over > turnPeakOver) turnPeakOver = over;
    double mag = fmin(maxSpeed, TURN_MIN_SPEED + TURN_DECEL_GAIN * sqrt(fmax(0.0, fabs(err) - TURN_TOL_DEG)));
    if (approach > 0.0) mag -= KD_TURN * approach;             // extra braking, never below the crawl speed
    mag = fmax(fmin(TURN_MIN_SPEED, maxSpeed), mag);
    double power = dir * mag;

    // soft start only (never slows the way down)
    if (fabs(power) > fabs(lastPow) + TURN_ACCEL_STEP) power = dir * (fabs(lastPow) + TURN_ACCEL_STEP);
    lastPow = power;

    setDrive(power, -power);                                   // + power = clockwise

    waitMs(LOOP_MS);
    t += LOOP_MS;
  }
  stopDrive();
}

void turnTo(double angle, double maxSpeed = TURN_MAX_SPEED) {
  targetHeading = angle;
  const int startMs = (int)Brain.Timer.time(msec);

  double err0 = angle - heading();
  const int li = (err0 >= 0.0) ? 0 : 1;                        // which direction's learned lead to use
  const double dir = signd(err0);
  double stopRate = 0.0, stopHeading = heading();
  turnPeakOver = 0.0;

  turnPass(angle, maxSpeed, 1200 + (int)(fabs(err0) * 20.0), turnLead[li], &stopRate, &stopHeading);
  if (autoAborted) return;
  waitStill(TURN_REST_MS, 300);
  if (-(angle - heading()) * dir > turnPeakOver) turnPeakOver = -(angle - heading()) * dir;

  // learn how far the robot coasts after stopping in THIS direction (right and left turns can differ)
  if (stopRate > 15.0) {
    double coast = (heading() - stopHeading) * dir;            // degrees it travelled after the stop command
    double measured = clampd(coast / stopRate, TURN_LEAD_MAX);
    if (measured < 0.0) measured = 0.0;
    turnLead[li] = 0.4 * turnLead[li] + 0.6 * measured;
  }

  // a few short slow nudges, only if clearly off - never a long hunt
  for (int i = 0; i < TURN_NUDGES; i++) {
    if (fabs(angle - heading()) <= TURN_FIX_TOL_DEG) break;
    double r2, h2;
    turnPass(angle, TURN_NUDGE_SPEED, 250, 0.0, &r2, &h2);
    waitStill(TURN_REST_MS, 300);
  }
  logTurn(angle, angle - heading(), (int)Brain.Timer.time(msec) - startMs);
}

void turnLeft(double deg)  { turnTo(targetHeading - fabs(deg)); }

// ============================================================
// AUTONOMOUS ROUTINE  (see PLAN in the notes at the top)
// ============================================================
// ---- tunables of the routine (all times in ms, all distances in cm) ----
const int    STEP_PAUSE_MS     = 250;     // pause + label on the Brain screen BEFORE an action, so you can see which step is next (0 = off).
const bool   RECAL_FULL_GYRO   = false;   // false = after a turn only RE-ZERO the gyro (instant). true = full gyro recalibration (~2-3 s, robot must stand still)
const bool   RECAL_AFTER_TURN  = true;    // after a Phase 1 left turn: stand still, re-zero the gyro. false = keep absolute heading
const double SPEED_SLOW        = 50.0;    // % "set speed down to 50"
const double SPEED_FAST        = 100.0;   // % "set both motors back to 100"
const double BIAS_COAST_S      = 0.05;    // sec: a raw distance move (rawMoveCm) stops commanding this much BEFORE the target (coasting)

// from the first shot on, the forward / backward moves use these fixed wheel speeds (compensates the robot's pull to one side)
const double BIAS_LEFT         = 70.0;    // % physical LEFT wheel
const double BIAS_RIGHT        = 100.0;   // % physical RIGHT wheel
const int    ARM_UP_SETTLE_MS   = 100;    // pause right after the arm is up
const int    P1_PUSH_MS        = 1000;    // Phase 1: timed straight push into L4
const double Z_BACK1_CM        = 8.0;    // after L4: first back move, left = right = 80
const double Z_BACK1_LEFT      = 80.0;
const double Z_BACK1_RIGHT     = 80.0;
const double Z_BACK2_CM        = 10.0;    // second back move, physical LEFT 100 / physical RIGHT 30
const double Z_BACK2_LEFT      = 100.0;
const double Z_BACK2_RIGHT     = 30.0;
const int    P2_STRAIGHT_MS    = 1000;    // straight move after the left turn (eats bag 1)
const int    P3_INTAKE_MS      = 2000;    // after the straight run: robot stands still, intake runs this long, then stops
const int    P3_BACK1_MS       = 800;     // raw timed back, physical LEFT 100 / RIGHT 100
const double P3_BACK1_LEFT     = 100.0;
const double P3_BACK1_RIGHT    = 100.0;
const int    P3_DELAY_MS       = 300;     // pause between the back move and the forward move
const int    P3_FWD_MS         = 900;     // raw timed forward, physical LEFT 100 / RIGHT 30 (curves right)
const double P3_FWD_LEFT       = 100.0;
const double P3_FWD_RIGHT      = 30.0;
const int    P3_BACK2_MS       = 800;     // raw timed back, physical LEFT 100 / RIGHT 100
const double P3_BACK2_LEFT     = 100.0;
const double P3_BACK2_RIGHT    = 100.0;
const int    P3_BACK3_MS       = 1000;    // raw timed back, physical LEFT 100 / RIGHT 60
const double P3_BACK3_LEFT     = 100.0;
const double P3_BACK3_RIGHT    = 60.0;

// Phase 4: shots and the spam loop
const double SHOT_BACK_LEFT    = 100.0;   // backing against the wall while the shot is fired: physical LEFT / RIGHT %
const double SHOT_BACK_RIGHT   = 100.0;
const int    AFTER_SHOT_MS     = 200;     // the robot keeps backing this long after the release is detected (the reload is already running)
const int    POST_SHOT_DELAY_MS = 200;    // after a shot: intake starts, then this pause
const double P4_FWD_CM         = 40.0;    // after shot #1: forward (BIAS speeds)
const int    P4_WAIT1_MS       = 300;
const double P4_BACK_CM        = 35.0;    // then back (BIAS speeds)
const int    P4_WAIT2_MS       = 200;     // wait, then the intake stops
const double SPAM_FWD_CM       = 56.0;    // spam loop: forward distance (BIAS speeds)
const int    SPAM_FWD_EXTRA_MS = 200;     // ...then forward this much more
const int    SPAM_DELAY_MS     = 200;
const double SPAM_BACK_CM      = 58.0;    // spam loop: back distance (BIAS speeds)
const int    SPAM_BACK_EXTRA_MS = 200;    // ...then back this much more
const int    SPAM_WAIT_MS      = 200;     // wait, then the intake stops
const int    SPAM_MAX_CYCLES   = 0;       // 0 = spam FOREVER (until FDown abort). Set e.g. 3 to limit the cycles
const int    AUTO_TIME_LIMIT_MS = 0;      // spam loop stops starting new cycles after this time since START (0 = no limit; real match: 60000)

double driveSpeedNow = DRIVE_MAX_SPEED;   // "set motor speed" of the plan (used by fwd/timed moves)
int    autoStartMs   = 0;

double L4pos() { return ARM_SIGN * L4.position(degrees); }

void autoStatus(const char* what) {
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("AUTO %-20s", what);
}

// ---- delay (can be aborted) ----
void pauseMs(int ms) {
  int t = 0;
  while (t < ms && !abortNow()) { waitMs(20); t += 20; }
}

// Label + pause before an action: the step name stays on Brain line 1 so you know exactly which action is running.
int stepNo = 0;
void stepPause(const char* label, int ms = STEP_PAUSE_MS) {
  if (autoAborted) return;
  stepNo++;
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("#%02d %-22s", stepNo, label);
  pauseMs(ms);
}

// ---- intake ----
void intakeOn() { Take_bag.setVelocity(100.0, percent); Take_bag.spin(forward); }

// ---- arm (same as the controller's arm up) ----
void L4_moveTo(double pos, double speed, int timeoutMs) {
  L4.setVelocity(speed, percent);
  L4.spinToPosition(ARM_SIGN * pos, degrees, false);
  int t = 0;
  double lastPos = L4pos();
  while (fabs(L4pos() - pos) > L4_TOL_DEG && t < timeoutMs && !abortNow()) {
    waitMs(10);
    t += 10;
    if (t % L4_STALL_CHECK_MS == 0) {            // stalled (hit its stop / blocked): do not wait for the timeout
      double nowPos = L4pos();
      if (t >= L4_STALL_GRACE_MS && fabs(nowPos - lastPos) < L4_STALL_DEG) { break; }
      lastPos = nowPos;
    }
  }
}

void armUp() {                                   // go up and HOLD
  if (autoAborted) return;
  L4_moveTo(L4_UP_POS, L4_UP_SPEED, L4_UP_TIMEOUT_MS);
  L4.stop(hold);
}

// ---- shooter: SAME state machine as the controller's main loop, called from every waitMs() ----
// Idle (holding, no pre-load) it does nothing at all, so it never disturbs the drive loops.
void l3Update() {
  if (L3_holding && !L3_preloading && !L3_preHeld && !L3_firing) { return; }

  const int now = (int)Brain.Timer.time(msec);
  const bool isFar = Distance7.objectDistance(mm) > L3_STOP_MM;
  if (!isFar) {
    L3_farSince = -1;
    L3_gaveUp = false;
  } else if (L3_farSince < 0) {
    L3_farSince = now;
    L3_farPos = L3MotorA.position(degrees);       // first FAR reading = where it released
  }
  const bool needReload = (L3_farSince >= 0) && (now - L3_farSince >= L3_FAR_MS);

  // a shot ends when the sensor says it FIRED. L3 is NOT stopped: it keeps spinning = reload, no gap.
  if (L3_firing) {
    if (needReload && now - L3_fireMs >= L3_FIRE_MIN_MS) {
      L3_firing = false;
      L3_reloadMs = now;
      L3_travel = L3_farPos - L3_fireStartPos;     // degrees still left from the held position to the release
#if L3_PRELOAD_AUTO
      if (!L3_skipLearn) {                         // feedback: the travel still left should be ~ the margin
        double delta = L3_travel - L3_PRELOAD_MARGIN;
        if (delta > L3_PRELOAD_STEP) { delta = L3_PRELOAD_STEP; }   // grow slowly, shrink at once
        if (delta > 0.0 && L3_preloadLocked) { delta = 0.0; }       // after a self-release it may only shrink
        double learned = L3_preloadDeg + delta;
        if (learned < 0.0) { learned = 0.0; }
        if (learned > L3_PRELOAD_MAX) { learned = L3_PRELOAD_MAX; }
        L3_preloadDeg = learned;
      }
#endif
      Brain.Screen.setCursor(8, 1);
      Brain.Screen.print("L3 trv %3.0f pre %3.0f  ", L3_travel, L3_preloadDeg);   // copy "pre" into L3_PRELOAD_DEG
    } else if (now - L3_fireMs >= L3_FIRE_TIMEOUT_MS) {
      L3_firing = false;                           // never fired: falls through to hold below
      L3_reloadMs = now;
    }
  }

  if (L3_firing) {
    L3.spin(forward);
    L3_holding = false;
  } else if (needReload) {
    if (L3_preloading || L3_preHeld) {             // it released BY ITSELF during/after the pre-load: pre-load was too big
      L3_preloadDeg -= 2.0 * L3_PRELOAD_STEP;
      if (L3_preloadDeg < 0.0) { L3_preloadDeg = 0.0; }
      L3_preloadLocked = true;
    }
    L3_preloading = false;
    L3_preHeld = false;
    if (L3_gaveUp) {
      if (!L3_holding) { L3.stop(hold); L3_holding = true; }
    } else {
      if (L3_holding) { L3_holding = false; L3_reloadMs = now; }   // lost the sensor: start a reload
      else if (now - L3_reloadMs > L3_RELOAD_TIMEOUT_MS) { L3_gaveUp = true; }   // jam: stop trying
      L3.spin(forward);
    }
  } else if (L3_preloading) {
    L3.spin(forward);                              // extra pull after the sensor caught it, then HOLD
    if (L3MotorA.position(degrees) >= L3_preloadTarget || now - L3_preMs >= L3_PRELOAD_TIMEOUT_MS) {
      L3_preloading = false;
      L3_preHeld = true;
      L3.stop(hold);
    }
  } else if (!L3_holding) {                        // the sensor JUST caught it
    L3_catchPos = L3MotorA.position(degrees);
    if (L3_preloadDeg > 0.0) {
      L3_preloading = true;                        // keep spinning (no stop) for L3_preloadDeg more, then hold
      L3_preloadTarget = L3_catchPos + L3_preloadDeg;
      L3_preMs = now;
    } else {
      L3.stop(hold);                               // pulled back: HOLD it (once)
    }
    L3_holding = true;
  }
}

// ---- drive helpers ----
void goFwd(double cm) { if (!autoAborted) driveStraight(fabs(cm) * 10.0, driveSpeedNow); }

// Timed move with FIXED wheel speeds (signed %, + = forward). ramps up softly.
// holdHeading = true: gyro keeps it straight (use for "go straight"); false: raw speeds (wall squaring).
void moveTimed(int ms, double leftPct, double rightPct, bool holdHeading) {
  if (autoAborted) return;
  double cl = 0.0, cr = 0.0;
  int t = 0;
  while (t < ms && !abortNow()) {
    cl += clampd(leftPct  - cl, DRIVE_ACCEL_STEP);
    cr += clampd(rightPct - cr, DRIVE_ACCEL_STEP);
    double l = cl, r = cr;
    if (holdHeading) {
      double p = (cl + cr) / 2.0;
      double corr = KP_HEAD * (targetHeading - heading()) + headTrim * p;
      corr = clampd(corr, fmin(MAX_HEAD_CORR, 6.0 + 0.4 * fabs(p)));
      l += corr; r -= corr;
    }
    setDrive(l, r);
    waitMs(LOOP_MS);
    t += LOOP_MS;
  }
  stopDrive();
  waitMs(DRIVE_REST_MS);
  if (!holdHeading) targetHeading = heading();   // after a raw move the robot's heading is whatever it is now
}

void fwdTime(int ms) { moveTimed(ms, driveSpeedNow, driveSpeedNow, true); }   // gyro straight

// Distance move with FIXED wheel speeds (leftPct / rightPct, magnitudes), no gyro correction.
// cm > 0 = forward, cm < 0 = backward (same wheel ratio both ways). Soft start, brakes at the end.
void rawMoveCm(double cm, double leftPct, double rightPct) {
  if (autoAborted) return;
  DL.setPosition(0.0, degrees);
  DR.setPosition(0.0, degrees);
  const double dir = signd(cm);
  const double targetDeg = fabs(cm) * 10.0 * degPerMM();
  const int N = 5;
  double posHist[N];
  for (int i = 0; i < N; i++) posHist[i] = 0.0;
  int idx = 0;
  double cl = 0.0, cr = 0.0;
  int t = 0;
  const int timeoutMs = 1500 + (int)(fabs(cm) * 60.0);
  while (t < timeoutMs && !abortNow()) {
    double pos = fabs(DL.position(degrees) + DR.position(degrees)) / 2.0;
    double rate = (pos - posHist[idx]) * 1000.0 / (LOOP_MS * N);   // motor-deg/sec toward the target
    posHist[idx] = pos;
    idx = (idx + 1) % N;
    if (pos + fmax(0.0, rate) * BIAS_COAST_S >= targetDeg) break;
    cl += clampd(dir * leftPct  - cl, DRIVE_ACCEL_STEP);
    cr += clampd(dir * rightPct - cr, DRIVE_ACCEL_STEP);
    setDrive(cl, cr);
    waitMs(LOOP_MS);
    t += LOOP_MS;
  }
  stopDrive();
  waitMs(DRIVE_REST_MS);
  targetHeading = heading();                     // heading is whatever it is now
}
void biasFwd(double cm)   { rawMoveCm( fabs(cm), BIAS_LEFT, BIAS_RIGHT); }
void biasBack(double cm)  { rawMoveCm(-fabs(cm), BIAS_LEFT, BIAS_RIGHT); }
void biasBackTime(int ms) { moveTimed(ms, -BIAS_LEFT, -BIAS_RIGHT, false); }   // raw timed back

// ---- shooting ----
// L3 is loaded and holding (the previous reload / pre-load is finished)
bool shotReady() { return L3_holding && Distance7.objectDistance(mm) <= L3_STOP_MM; }

// start one shot: L3 spins at once. l3Update ends it when the sensor says it fired, then keeps spinning = reload.
void startShot() {
  L3_skipLearn = L3_preloading;                    // fired before the pre-load finished: says nothing about the pre-load
  L3_preloading = false;
  L3_preHeld = false;
  L3_gaveUp = false;
  L3_farSince = -1;
  L3_fireMs = (int)Brain.Timer.time(msec);
  L3_fireStartPos = L3MotorA.position(degrees);
  L3_firing = true;
  L3.setVelocity(100.0, percent);
  L3.spin(forward);                                // FIRE
}

// Keep driving BACKWARD (raw fixed wheel speeds, soft start, no gyro: it presses against the wall) and fire one shot as
// soon as L3 is loaded. The drive stops AFTER_SHOT_MS after the release is detected; the reload keeps running in the background.
void backAndShoot(double leftPct, double rightPct) {
  if (autoAborted) return;
  autoStatus("BACK + SHOOT");
  double cl = 0.0, cr = 0.0;
  int t = 0, endT = -1;
  bool started = false;
  const int limitMs = L3_RELOAD_TIMEOUT_MS + L3_FIRE_TIMEOUT_MS + 1000;   // safety
  while (t < limitMs && !abortNow()) {
    cl += clampd(-leftPct  - cl, DRIVE_ACCEL_STEP);
    cr += clampd(-rightPct - cr, DRIVE_ACCEL_STEP);
    setDrive(cl, cr);
    if (!started && (shotReady() || t >= L3_RELOAD_TIMEOUT_MS)) { startShot(); started = true; }   // not loaded after 4 s: fire anyway
    if (started && !L3_firing && endT < 0) { endT = t + AFTER_SHOT_MS; }                           // released -> 200 ms more
    if (endT >= 0 && t >= endT) break;
    waitMs(LOOP_MS);
    t += LOOP_MS;
  }
  stopDrive();
  targetHeading = heading();                       // heading is whatever it is now
}

// "Reset the brain": stand still, re-zero the gyro (optionally full recalibration) and the wheel encoders.
void resetCalibrateGyro() {
  if (autoAborted) return;
  autoStatus("RESET GYRO");
  stopDrive();
  if (RECAL_FULL_GYRO) {                          // slow: ~2-3 s
    BrainInertial.calibrate();
    int guard = 0;
    while (BrainInertial.isCalibrating() && guard < 200 && !abortNow()) { waitMs(50); guard++; }
  }
  zeroHeading();                                  // fast: heading = 0 from here
  DL.setPosition(0.0, degrees);
  DR.setPosition(0.0, degrees);
}

// turn, then (optionally) re-zero the gyro so the NEXT turn starts from a fresh, drift-free reference
void turnLeftDeg(double d) { if (autoAborted) return; turnLeft(d); if (RECAL_AFTER_TURN) resetCalibrateGyro(); }

void abortEverything() {
  DL.stop(brake);
  DR.stop(brake);
  Take_bag.stop();
  L3.stop(hold);
  L4.stop(hold);
}

// ------------------------------------------------------------
// THE PLAN
// ------------------------------------------------------------
void autoRoutine() {
  autoRunning = true;
  autoAborted = false;
  autoStartMs = (int)Brain.Timer.time(msec);
  TouchLED12.setColor(red);
  Brain.Screen.clearScreen();
  logRow = 2;
  stepNo = 0;
  zeroHeading();
  driveSpeedNow = DRIVE_MAX_SPEED;
  intakeOn();                                      // intake runs from the very start (startup + calibration already done)

  // Every action is preceded by stepPause("..."): label on the Brain + STEP_PAUSE_MS pause.

  // ===== PHASE 1: eat L4 =====
  stepPause("P1 fwd 3cm");            goFwd(3);
  stepPause("P1 turn LEFT 32");       turnLeftDeg(32);
  stepPause("P1 speed -> 50");        driveSpeedNow = SPEED_SLOW;
  stepPause("P1 fwd 3cm + 1.2s");     goFwd(3);
                                      fwdTime(P1_PUSH_MS);    // no delay between the two straight moves (push L4)
  stepPause("P1 arm up");             armUp();
  // (the arm stays UP from here on)

  // ===== PHASE 2: leave L4 =====
  stepPause("P2 back 15cm L80 R80", ARM_UP_SETTLE_MS);
                                      rawMoveCm(-Z_BACK1_CM, Z_BACK1_LEFT, Z_BACK1_RIGHT);       // raw, no gyro
  stepPause("P2 back 10cm L100 R30", 0);
                                      rawMoveCm(-Z_BACK2_CM, Z_BACK2_LEFT, Z_BACK2_RIGHT);

  // ===== PHASE 2a: a little forward, turn left 50, straight to eat bag 1 =====
  stepPause("P2 speed -> 100");       driveSpeedNow = SPEED_FAST;
  stepPause("P2 both 100, fwd 500ms"); moveTimed(500, SPEED_FAST, SPEED_FAST, false);   // left = right = 100 (raw, no gyro trim)
  stepPause("P2 turn LEFT 60");       turnLeft(60);           // relative to the heading after the move above (no gyro reset)
  stepPause("P2 fwd 1s at 100");      fwdTime(P2_STRAIGHT_MS); // both motors 100 (gyro keeps it straight): eats bag 1

  // ===== PHASE 3: intake 2 s, back, delay, forward, back (raw timed moves, no gyro) =====
  stepPause("P3 intake 2s", 0);       intakeOn();
                                      pauseMs(P3_INTAKE_MS);
                                      Take_bag.stop();
  stepPause("P3 back 800ms L100 R100", 0); moveTimed(P3_BACK1_MS, -P3_BACK1_LEFT, -P3_BACK1_RIGHT, false);
  stepPause("P3 delay 500", P3_DELAY_MS);
  stepPause("P3 fwd 900ms L100 R30", 0);   moveTimed(P3_FWD_MS, P3_FWD_LEFT, P3_FWD_RIGHT, false);
  stepPause("P3 back 800ms L100 R100", 0); moveTimed(P3_BACK2_MS, -P3_BACK2_LEFT, -P3_BACK2_RIGHT, false);
  stepPause("P3 back 1s L100 R60", 0);     moveTimed(P3_BACK3_MS, -P3_BACK3_LEFT, -P3_BACK3_RIGHT, false);

  // ===== PHASE 4: shot #1 (back against the wall while shooting), forward / back, shot #2 =====
  stepPause("P4 back + SHOOT #1", 0);  backAndShoot(SHOT_BACK_LEFT, SHOT_BACK_RIGHT);   // L/R 100, until the shot fired + 200 ms
  stepPause("P4 intake on", 0);        intakeOn();
                                       pauseMs(POST_SHOT_DELAY_MS);
  stepPause("P4 fwd 40cm L70 R100", 0); biasFwd(P4_FWD_CM);
                                       pauseMs(P4_WAIT1_MS);
  stepPause("P4 back 35cm L70 R100", 0); biasBack(P4_BACK_CM);
                                       pauseMs(P4_WAIT2_MS);
                                       Take_bag.stop();
  stepPause("P4 back + SHOOT #2", 0);  backAndShoot(SHOT_BACK_LEFT, SHOT_BACK_RIGHT);
  stepPause("P4 intake on", 0);        intakeOn();
                                       pauseMs(POST_SHOT_DELAY_MS);

  // ===== SPAM LOOP: forward 55cm + 200ms, delay, back 45cm + 200ms, wait, back + shoot, repeat =====
  int cycle = 0;
  while (!autoAborted) {
    if (AUTO_TIME_LIMIT_MS > 0 && (int)Brain.Timer.time(msec) - autoStartMs >= AUTO_TIME_LIMIT_MS) break;
    if (SPAM_MAX_CYCLES > 0 && cycle >= SPAM_MAX_CYCLES) break;
    cycle++;
    char lab[28];
    snprintf(lab, sizeof(lab), "S%d fwd 55cm+200ms", cycle);   stepPause(lab, 0);
                                       biasFwd(SPAM_FWD_CM);
                                       moveTimed(SPAM_FWD_EXTRA_MS, BIAS_LEFT, BIAS_RIGHT, false);
                                       pauseMs(SPAM_DELAY_MS);
    snprintf(lab, sizeof(lab), "S%d back 45cm+200ms", cycle);  stepPause(lab, 0);
                                       biasBack(SPAM_BACK_CM);
                                       biasBackTime(SPAM_BACK_EXTRA_MS);
                                       pauseMs(SPAM_WAIT_MS);
                                       Take_bag.stop();
    snprintf(lab, sizeof(lab), "S%d back + SHOOT", cycle);     stepPause(lab, 0);
                                       backAndShoot(SHOT_BACK_LEFT, SHOT_BACK_RIGHT);
                                       intakeOn();
                                       pauseMs(POST_SHOT_DELAY_MS);
  }

  // ===== END (FDown abort, or the cycle / time limit): everything stops =====
  Take_bag.stop();
  if (autoAborted) abortEverything();
  autoRunning = false;
  Brain.Screen.setCursor(6, 1);
  Brain.Screen.print("END hdg %.1f tgt %.1f      ", heading(), targetHeading);   // gyro heading vs wanted heading (0 = direction after the P1 turn)
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print(autoAborted ? "AUTO ABORTED. Restart.     " : "TEST END - stopped. Restart.");
  // finished: blink PURPLE forever (aborted: blink RED). Restart the program to run again.
  bool on = true;
  while (true) {
    TouchLED12.setColor(on ? (autoAborted ? red : purple) : black);
    on = !on;
    wait(300, msec);
  }
}

// ============================================================
// MAIN
// ============================================================
int main() {
  vexcodeInit();
  TouchLED12.setColor(purple);
  initDrive();
  batteryWarn(purple);        // low battery = red flashes + message, then start-up continues anyway
  startupUntilReady();        // PURPLE (shooter + arm) -> YELLOW blinking (reset + gyro) -> GREEN only if all OK

  // GREEN = ready. A player TOUCHES the Touch LED -> the autonomous routine starts.
  Brain.Screen.setCursor(7, 1); Brain.Screen.print("Touch the LED to START   ");
  int lastBatteryCheckMs = (int)Brain.Timer.time(msec);
  while (!TouchLED12.pressing()) {
    if ((int)Brain.Timer.time(msec) - lastBatteryCheckMs >= BATTERY_REPEAT_MS) {   // idle only: warn again, robot still fully usable
      lastBatteryCheckMs = (int)Brain.Timer.time(msec);
      batteryWarn(green);
    }
    wait(20, msec);
  }
  while (TouchLED12.pressing()) { wait(20, msec); }                  // starts when the hand lifts
  autoRoutine();                                                    // ends in an endless purple blink
  return 0;
}
