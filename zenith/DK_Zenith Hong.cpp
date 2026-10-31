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
motor Left_Motor = motor(PORT6, true);
motor Right_Motor = motor(PORT1, false);
controller Controller = controller();
motor L3MotorA = motor(PORT2, true);
motor L3MotorB = motor(PORT5, false);
motor_group L3 = motor_group(L3MotorA, L3MotorB);

motor Take_bag = motor(PORT3, true);
motor L4 = motor(PORT4, false);
distance Distance7 = distance(PORT7);
touchled TouchLED12 = touchled(PORT12);


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
  PROJECT NOTES  -  READ THIS FIRST  (for the next Claude session / teammate)
 ==============================================================================
  Robot : "Team Zenith"  -  VEX IQ (2nd gen), VEXcode IQ C++, driver-control only.
  Owner : Albert (mentor/programmer). He prefers SHORT answers, small tables,
          short reasons, and step-by-step refinement. He tests on the real robot
          and reports what he sees, so keep changes small and tell him what to tune.
  Sister robot: "Team Tieu Hoc 2" runs the SAME logic with different config names/ports
          (Left, Right, shoot, arm, intake, Distance4, drive A+C / A-C, ARM_SIGN -1, UP 800 / EUP 600).
          Tieu Hoc 2 shares: FDown maintenance shot, ground keeper, shooter-first confirmed start-up.
          THIS FILE = BACKUP "DK_Zenith Hong": same as DK_Zenith.cpp, but the broken controller's RUp / RDown buttons are
          replaced by EUp / EDown (EUp = arm up, EDown = arm down). Everything else is IDENTICAL - keep both files in sync.
          NOTE: L4_UP_POS on Zenith is a first guess - tune it on the robot.

  HARDWARE (device config is the VEXcode-generated block above - do NOT edit it,
  VEXcode regenerates it; if the user changes ports he pastes a new file)
  ---------------------------------------------------------------------------
   Left_Motor  PORT6  (true)    drive left
   Right_Motor PORT1  (false)   drive right
   L3          group = L3MotorA PORT2 (true) + L3MotorB PORT5 (false)   shooter (rubber-band pull-back)
   L4          PORT4  (false)   arm (lifts the mechanism)
   Take_bag    PORT3  (true)    intake
   Distance7   PORT7            distance sensor: reads small mm when the shooter is pulled back (loaded)
   TouchLED12  PORT12           status LED
  Naming: "L3" = shooter, "L4" = arm, "Take_bag" = intake (these are the real config names on Zenith).

  CONTROLS (Controller)
  ---------------------------------------------------------------------------
   AxisA / AxisC  drive (A = forward/back, C = turn). Left = A - C, Right = A + C. Deadband 5.
   LUp     toggle intake on/off (ignored during a shoot cycle).
   LDown   intake REVERSE while held (beats the toggle).
   EUp     (backup for RUp) L4 goes to L4_UP_POS and HOLDS. LED red. Blocked if already up.
   EDown   (backup for RDown) L4 goes down to 0 (only if it is up). LED green.
   FUp     SHOOT then RELOAD (automatic). One shot per press. Only when the shoot gate is OK (below).
   FDown   SHOOT but do NOT reload and do NOT touch the arm (for maintenance: the pulled-back shooter
           covers inner parts). Same gate as FUp. Sets `maintenance = true`.
           Press FDown AGAIN while in maintenance = RE-VALIDATE (runs calibrateAll again).

  FEATURES / HOW IT WORKS
  ---------------------------------------------------------------------------
  1) START-UP CALIBRATION  calibrateAll()  (controller dead + LED flashing red/yellow until done, then GREEN)
       a. stop everything, set stopping modes, torque 100, speed 100 on every motor
       b. SHOOTER FIRST: L3_pullBackConfirmed() pulls L3 back until Distance7 <= 35 mm (polled every 10 ms,
          2 reads in a row), stop(hold), wait 400 ms, re-check it is STILL <= 35 mm. Up to 3 tries.
       c. ONLY THEN the arm: if the shooter is confirmed, L4 drives DOWN by TIME (L4_INIT_MS) to the stop,
          stop(hold). If the shooter was NOT confirmed the arm is NOT moved (prevents knocking the shooter loose).
       d. all encoders set to 0, all state flags reset, LED green, controlsEnabled = true.
       Button callbacks are registered in main() only AFTER calibration.
  2) SHOOT GATE  (FUp and FDown): shootReady = armOk AND distOk
       armOk  : L4 not moving AND armPos() >= max(L4_SHOOT_MIN_POS 300, L4_upReached * 0.7)
       distOk : Distance7 <= L3_STOP_MM (35 mm) = shooter is loaded
     Brain screen shows: line1 "L4 deg/needed OK|LOW", line2 "Dist mm OK|FAR", line3 "SHOOT READY|BLOCKED"
     (or "MAINT: FDown=revalidate").
  3) SHOOT / RELOAD (main loop whenStarted1, non-blocking so driving never freezes):
       FUp : L3 spins forward at once. It is NOT time based: when the sensor reads FAR for 3 loops in a row
             (after L3_FIRE_MIN_MS) the shot has FIRED, and L3 simply keeps spinning forward = reload with no stop
             in between; when <= 35 mm it stop(hold) ONCE (a latch, L3_holding, because re-sending hold every
             loop makes it creep). L3_FIRE_TIMEOUT_MS = safety if nothing ever fires.
             PRE-LOAD: on that sensor catch it keeps pulling L3_preloadDeg more, then holds, so the next FUp only
             has to release (no ~1 s extra pull-down). FUp during the pre-load fires at once.
             AUTO (L3_PRELOAD_AUTO 1): learns SLOWLY. After every FUp it measures "trv" = degrees still left from
             the HELD position to the release, and changes pre-load by (trv - L3_PRELOAD_MARGIN), but it may GROW
             by at most L3_PRELOAD_STEP (40) per shot (so the first ~4-6 shots get faster and faster; it never
             jumps past the release). If trv is below the margin it shrinks at once. If L3 ever releases by
             itself during/after the pre-load, pre-load backs off 2 steps and may only shrink until restart.
             Brain line 4: "trv" (aim ~ the margin) and "pre" = current pre-load. When "pre" stops changing you
             can copy it into L3_PRELOAD_DEG and set L3_PRELOAD_AUTO 0 (fixed, no learning, every shot fast).
   BUG FIXED (twice): pre-load was learned from the sensor-catch position / in one big jump, ignoring how far
             the motor coasts when it stops -> it stopped past the release, L3 fired by itself and re-shot 1-4
             times after a single FUp. Do not go back to a one-jump pre-load.
       FDown: same forced spin for L3_MAINT_SHOT_MS, then stop(coast) and NO reload (maintenance mode).
  4) INTAKE: decided in ONE place in the loop: LDown reverse > Intake_F==1 forward > stop.
       FUp/FDown press -> STOP 1: Intake_F = 0 and `shootCycle = true` (LUp locked).
       When reload is finished and holding -> STOP 2: Intake_F = 1 again, shootCycle = false.
       (In maintenance mode there is no reload, so the intake stays OFF.)
  5) L4 BY POSITION (not time): L4_moveTo(pos, speed, timeout) uses spinToPosition (non-blocking) and waits
     until within L4_TOL_DEG, the timeout, or a STALL (arm moved < L4_STALL_DEG in L4_STALL_CHECK_MS, after
     L4_STALL_GRACE_MS). A stalled EUp holds where it is and L4_upReached = that real height.
     Time is used ONLY for the start-up homing.
  6) ARM_SIGN = +1: on Zenith "up" is the FORWARD direction (Tieu Hoc 2 is -1). All degree values in the code are POSITIVE = UP;
     armPos() = ARM_SIGN * L4.position(). If the arm ever goes the wrong way, flip ARM_SIGN to -1.
  7) GROUND KEEPER (main loop): if the arm is resting on the ground (not up, not busy) and creeps
     above L4_DRIFT_DEG (15), it is sent back to 0 once (spring/bounce).
  8) USE_DISTANCE_SENSOR (#define, 1 = on). Set 0 to run without the sensor (shoot gate then ignores distance).

  STATE FLAGS (globals)
  ---------------------------------------------------------------------------
   controlsEnabled  false during calibration -> all button events ignored
   L4_busy          an L4 move is running (blocks other L4 buttons + the shoot gate)
   L4_up            L4 is at the EUp position; EDown required before EUp again
   L4_upReached     height L4 REALLY reached on its last EUp (the shoot gate uses a fraction of it)
   shootCycle       between a shot and "reloaded + holding" (intake off, LUp locked)
   maintenance      FDown was used: L3 left released, no reload, FUp blocked, until re-validated
   Intake_F         1.0 = intake toggled on

  THINGS TO TUNE ON THE ROBOT (all in the TUNABLE VALUES section)
  ---------------------------------------------------------------------------
   L4_UP_POS                        real up position (watch the L4 line on the Brain screen)
   L4_INIT_MS 1000                  start-up arm homing time
   L4_STALL_DEG 3 / L4_STALL_GRACE_MS 300   stall detection in L4_moveTo (raise DEG if EDown ends too early)
   L3_PRELOAD_AUTO 1 / L3_PRELOAD_MARGIN 50   auto-learned extra pull after the sensor catch (kills the pull-down delay on FUp). See FUp notes.
   L3_FIRE_MIN_MS / L3_FIRE_TIMEOUT_MS   FUp "fired" detection filter / safety timeout
   L3_MAINT_SHOT_MS                 how long the FDown (maintenance) forced spin lasts
   L3_STOP_MM 35                    "loaded" distance threshold
   L3_PULL_SPEED 100                lower it (e.g. 60) if L3 overshoots the sensor and fires at start-up
   L4_SHOOT_MIN_POS / L4_SHOOT_FRACTION   how high the arm must be before shooting is allowed

  HISTORY OF BUGS ALREADY FIXED (do not reintroduce)
  ---------------------------------------------------------------------------
   - Turn direction was reversed -> sign of C swapped (Left_Motor = A - C, Right_Motor = A + C on Zenith).
   - (Tieu Hoc 2 only) arm moved UP at start-up -> ARM_SIGN = -1. Zenith uses +1.
   - Shooter fired at start-up and re-pulled 2-3 times -> sensor was polled every 100 ms (overshoot). Now 10 ms,
     confirmed after a settle, shooter FIRST, arm only after it is confirmed, 3-read debounce in the loop.
   - Arm crept up on the ground -> brake is weak. Now ground keeper (hold + active spinToPosition(0)).
   - Never call stop(hold) every loop on the shooter (it creeps) - latch it once (L3_holding).
   - Config names: use Left_Motor, Right_Motor, L3, L4, Take_bag, TouchLED12, Distance7 (NOT Tieu Hoc 2's Left/Right/shoot/arm/intake/Distance4).
   - (DK_Zenith.cpp only) the old time-based EUp / EDown (L4_locked, L4_EUP_POS, L4_EDOWN_MS, L4_BACK_*) were removed. In THIS file EUp / EDown simply take over the RUp / RDown behaviour.

  RULES FOR EDITING (Claude: follow these)
  ---------------------------------------------------------------------------
   - Never edit the "VEXcode Generated Robot Configuration" block.
   - Keep every motor at speed/torque 100 unless the user says otherwise.
   - Keep EUp/EDown behaviour identical to RUp/RDown in DK_Zenith.cpp unless asked.
   - Test-compile with a mock vex.h if you cannot run the robot; always tell the user what to tune.
   - Deliver the full file with SendUserFile and give a SHORT summary + a short "to tune" list.
 ==============================================================================
*/

// Include the IQ Library
#include "vex.h"
using namespace vex;

// ============================================================
// TUNABLE VALUES  (change these first when testing)
// ============================================================
const double MAX_SPEED   = 100.0;  // velocity (%) for every motor
const double MAX_TORQUE  = 100.0;  // max torque (%) for every motor
const double DEADBAND    = 5.0;    // joystick deadband (was 0 = no deadband)

// --- L4 arm: POSITIONS in degrees, measured from the start position (= 0) ---
// ARM_SIGN: +1 = the arm goes UP when the motor spins forward, -1 = it goes UP when it spins reverse.
// On Zenith the arm goes UP when the motor spins FORWARD (Tieu Hoc 2 is the opposite), so +1.
// All the degree numbers below stay POSITIVE = up; the sign is applied in one place (armPos / L4_moveTo).
// If the arm ever goes the wrong way, just flip this to -1.
const double ARM_SIGN = 1.0;
// !! L4_UP_POS is a STARTING GUESS. Watch the "L4: ... deg" line on the
// !! Brain screen and tune it to the real up position on your robot.
const double L4_UP_POS      = 600; // EUp top position
const double L4_SHOOT_MIN_POS  = 300.0; // floor: FUp never shoots below this many degrees
const double L4_SHOOT_FRACTION = 0.7;   // ...and L4 must be above this fraction of the height it ACTUALLY reached on its last EUp
const double L4_DOWN_POS    = 0.0;    // ground = start position
const double L4_TOL_DEG     = 10.0;   // "arrived" tolerance
const double L4_DRIFT_DEG   = 15.0;   // at ground (not up): if L4 creeps above this, it is pushed back to 0 automatically
const double L4_UP_SPEED    = 100.0;
const double L4_DOWN_SPEED  = 100.0;  // full speed down (was 50)
const int    L4_UP_TIMEOUT_MS   = 4000; // safety: stop waiting if it can't arrive
const int    L4_DOWN_TIMEOUT_MS = 6000;
// Stall check: if L4 moves less than L4_STALL_DEG between two checks (L4_STALL_CHECK_MS apart) it is stalled -> the move ends at once
const int    L4_STALL_GRACE_MS = 300;   // ignore stalls during the first ms of a move (motor still accelerating)
const int    L4_STALL_CHECK_MS = 100;   // how often the arm position is compared
const double L4_STALL_DEG      = 3.0;   // less than this many degrees per check = stalled (raise it if EDown ends too early)

// --- Start-up ---
const int    L4_INIT_MS = 1000;     // start-up: L4 reverses by TIME to its start position

// --- L3 (shoot) ---
// FUp: L3 spins until the distance sensor says it FIRED (reads FAR), then simply KEEPS spinning = reload (no stop in between).
const int    L3_FIRE_MIN_MS = 200;      // ignore "fired" for this long after the press (sensor noise)
const int    L3_FIRE_TIMEOUT_MS = 2000; // safety: if nothing fired after this long, stop and hold
// PRE-LOAD: after the reload catches the sensor, pull MORE degrees and hold there, so FUp only has to release (no pull-down delay).
// AUTO (1): the robot LEARNS the distance by itself. The FIRST shot after start-up is still slow (it measures the
// travel from the sensor catch to the release); from the 2nd shot on, the pre-load = measured travel - L3_PRELOAD_MARGIN.
// AUTO (0): uses L3_PRELOAD_DEG as a fixed number (0 = no pre-load).
#define L3_PRELOAD_AUTO 1
const double L3_PRELOAD_DEG = 0.0;      // start value (AUTO 1) or fixed value (AUTO 0)
const double L3_PRELOAD_MARGIN = 50.0;  // degrees still left between the held position and the release. Raise it if a shot ever fires by itself after reload; lower it (e.g. 30) to cut the remaining delay
const double L3_PRELOAD_MAX = 500.0;    // safety cap for the learned value
const double L3_PRELOAD_STEP = 40.0;    // the pre-load grows by at most this many degrees per shot (learns SLOWLY from below, never jumps past the release)
const int    L3_PRELOAD_TIMEOUT_MS = 1500; // safety for the extra pull
const int    L3_MAINT_SHOT_MS = 800; // forced spin when FDown is pressed (shoot WITHOUT reload) - tune so it ends released, not re-pulled
const double L3_STOP_MM = 35.0;    // L3 spins until the sensor reads this or less
const int    L3_PULL_TIMEOUT_MS = 5000; // start-up pull-back: give up after this long (per try) if the sensor never triggers
const double L3_PULL_SPEED = 100.0;     // start-up pull-back speed. If it still overshoots the sensor and fires, lower this (e.g. 60)
const int    L3_SETTLE_MS  = 400;       // after stopping, wait this long and re-check that it is STILL at the sensor
const int    L3_PULL_TRIES = 3;         // max pull-back tries at start-up (normally 1)

// --- Distance sensor ---
// The sensor is declared in the device config above: Distance7 on PORT4.
#define USE_DISTANCE_SENSOR 1

// ============================================================
// STATE
// ============================================================
float A, C, Intake_F;

bool controlsEnabled = false;  // false while calibrating: nothing on the controller works
bool L4_busy = false;          // an L4 move is running
bool L4_up = false;            // true = at up position, must EDown before EUp again
bool shootCycle = false;       // true from FUp press until L3 has pulled back and holds (intake locked off)
double L4_upReached = 0.0;     // height (deg) L4 actually reached on its last EUp, used by the shoot gate
bool maintenance = false;      // true after FDown: shooter was fired and left RELEASED (no reload). FDown again = re-validate (shooter first, then arm)

// ============================================================
// HELPERS
// ============================================================
float clampf(float v, float limit) {
  if (v > limit) return limit;
  if (v < -limit) return -limit;
  return v;
}

// Wait ms while flashing the touch LED red <-> yellow (calibration "glitch").
void glitchWait(int ms) {
  static bool flip = false;
  int t = 0;
  while (t < ms) {
    if (flip) { TouchLED12.setColor(red); }
    else      { TouchLED12.setColor(yellow); }
    flip = !flip;
    wait(100, msec);
    t += 100;
  }
}

// Arm height in "up = positive" degrees, whichever way the motor is mounted.
double armPos() { return ARM_SIGN * L4.position(degrees); }

// Spin L4 toward the ground (opposite of "up"), whichever way the motor is mounted.
void L4_spinDown() {
  if (ARM_SIGN > 0) { L4.spin(reverse); } else { L4.spin(forward); }
}

// Move L4 to a position (non-blocking command + wait with timeout).
// Also gives up early if the arm STALLS (hits its stop, or is blocked) instead of waiting the full timeout.
void L4_moveTo(double pos, double speed, int timeoutMs) {
  L4.setVelocity(speed, percent);
  L4.spinToPosition(ARM_SIGN * pos, degrees, false);
  int t = 0;
  double lastPos = armPos();
  while (fabs(armPos() - pos) > L4_TOL_DEG && t < timeoutMs) {
    wait(10, msec);
    t += 10;
    if (t % L4_STALL_CHECK_MS == 0) {
      double nowPos = armPos();
      if (t >= L4_STALL_GRACE_MS && fabs(nowPos - lastPos) < L4_STALL_DEG) { break; }   // not moving -> stalled
      lastPos = nowPos;
    }
  }
}

// START-UP / RE-VALIDATE: pull L3 back until Distance7 <= L3_STOP_MM, stop + HOLD, wait, confirm it is STILL there.
// Polls every 10 ms (the old 100 ms polling let L3 overshoot the sensor and fire).
// Returns true only if the distance is still OK after settling.
bool L3_pullBackConfirmed() {
#if USE_DISTANCE_SENSOR
  L3.setVelocity(L3_PULL_SPEED, percent);
  L3.spin(forward);
  int t = 0;
  int good = 0;
  while (t < L3_PULL_TIMEOUT_MS && good < 2) {            // 2 reads in a row <= threshold = reached
    if (Distance7.objectDistance(mm) <= L3_STOP_MM) { good++; } else { good = 0; }
    if (t % 100 == 0) {                                    // LED glitch
      static bool f = false;
      TouchLED12.setColor(f ? red : yellow);
      f = !f;
    }
    wait(10, msec);
    t += 10;
  }
  L3.stop(hold);
  L3.setVelocity(MAX_SPEED, percent);
  glitchWait(L3_SETTLE_MS);                                // settle while LED keeps flashing
  for (int i = 0; i < 5; i++) {                            // must stay at the sensor
    if (Distance7.objectDistance(mm) > L3_STOP_MM) { return false; }
    wait(20, msec);
  }
  return true;
#else
  return true;
#endif
}

// ============================================================
// START-UP CALIBRATION  (controller is locked until this finishes)
// ============================================================
void calibrateAll() {
  controlsEnabled = false;
  TouchLED12.setColor(red);

  // 1) Everything stops
  Left_Motor.stop();
  Right_Motor.stop();
  L3.stop();
  Take_bag.stop();
  L4.stop();

  // 2) Same settings on every motor (no leftover speed/torque from before)
  Left_Motor.setStopping(brake);
  Right_Motor.setStopping(brake);
  L3.setStopping(brake);
  L4.setStopping(brake);
  Take_bag.setStopping(coast);

  Left_Motor.setMaxTorque(MAX_TORQUE, percent);
  Right_Motor.setMaxTorque(MAX_TORQUE, percent);
  L3.setMaxTorque(MAX_TORQUE, percent);
  Take_bag.setMaxTorque(MAX_TORQUE, percent);
  L4.setMaxTorque(MAX_TORQUE, percent);

  Left_Motor.setVelocity(MAX_SPEED, percent);
  Right_Motor.setVelocity(MAX_SPEED, percent);
  L3.setVelocity(MAX_SPEED, percent);
  Take_bag.setVelocity(MAX_SPEED, percent);
  L4.setVelocity(MAX_SPEED, percent);

  // 3) SHOOTER FIRST: pull back until the sensor reads it, stop, HOLD, settle, and CONFIRM it is still there.
  //    The arm below does NOT move until this is confirmed (so the arm can't knock the shooter loose).
  bool shooterOk = true;
#if USE_DISTANCE_SENSOR
  shooterOk = false;
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("Pulling L3 back...");
  for (int tryNo = 0; tryNo < L3_PULL_TRIES && !shooterOk; tryNo++) {
    shooterOk = L3_pullBackConfirmed();
  }
  Brain.Screen.setCursor(2, 1);
  if (shooterOk) { Brain.Screen.print("L3 loaded + holding"); }
  else           { Brain.Screen.print("WARNING: L3 not confirmed, arm NOT moved"); }
#endif

  // 4) THEN the arm (only if the shooter is confirmed): L4 goes down BY TIME to the start position, then HOLDS
  L4.setPosition(0.0, degrees);
  if (shooterOk) {
    L4_spinDown();
    glitchWait(L4_INIT_MS);
    L4.stop(hold);
    glitchWait(300);                       // let it settle

    Brain.Screen.setCursor(3, 1);
    if (fabs(L4.position(degrees)) < 20.0) {
      Brain.Screen.print("WARNING: L4 did not move!");
    } else {
      Brain.Screen.print("L4 homed (%.0f deg)", L4.position(degrees));
    }
  }

  // 4b) Reset every encoder: this spot is now 0
  L4.setPosition(0.0, degrees);
  Left_Motor.setPosition(0.0, degrees);
  Right_Motor.setPosition(0.0, degrees);
  L3MotorA.setPosition(0.0, degrees);
  L3MotorB.setPosition(0.0, degrees);
  Take_bag.setPosition(0.0, degrees);

  // 5) Reset all state flags
  Intake_F = 0.0;
  L4_busy = false;
  L4_up = false;
  shootCycle = false;
  L4_upReached = 0.0;
  maintenance = false;
  glitchWait(200);

  // 6) Done: green, controller unlocked
  TouchLED12.setColor(green);   // green = EUp allowed
  controlsEnabled = true;
}

// ============================================================
// MAIN LOOP: drive + intake + L3 (shoot)
// ============================================================
// whenStarted1 = THE MAIN LOOP (runs forever, every 20 ms, called from main() after calibration):
//   drive -> intake -> L4 ground keeper -> shoot gate -> FUp / FDown handling -> shooter state machine -> Brain screen.
// Button events below run in their own threads; they only touch L4 + flags, never the drive/shooter.
int whenStarted1() {
  int  L3_shotTicks = 0;       // counts down in 20 ms ticks (non-blocking, drive keeps working)
  bool fUpWasPressed = false;
  bool fDownWasPressed = false;
  bool L3_holding = true;      // true = shooter already stopped with hold (latch the hold ONCE, re-latching every loop lets it creep)
  int  screenTick = 0;
  bool L4_driftFix = false;    // latch: keeper already sent the arm back to 0
  int  farTicks = 0;           // how many loops in a row the sensor read FAR (debounce, avoids a flicker triggering a reload spin)
  double farPos = 0.0;         // L3 position at the first FAR reading (= where it released)
  bool   L3_firing = false;    // FUp shot in progress (spins until the sensor says it fired)
  int    L3_fireTicks = 0;     // loops since the FUp press
  double L3_fireStartPos = 0.0;// L3 position at the FUp press
  double L3_travel = 0.0;      // measured travel from hold to release (shown on the Brain; use it to set L3_PRELOAD_DEG)
  bool   L3_preloading = false;// extra pull after the sensor caught it
  bool   L3_preHeld = false;   // holding at the pre-loaded position (a FAR reading now = it released by itself)
  bool   L3_skipLearn = false; // this shot must not change the learned pre-load
  bool   L3_preloadLocked = false; // L3 once released by itself during the pre-load: the pre-load may only get SMALLER from now on
  double L3_preloadDeg = L3_PRELOAD_DEG; // current pre-load (learned when AUTO)
  double L3_catchPos = 0.0;    // L3 position where the sensor caught it (0 right after calibration)
  double L3_preloadTarget = 0.0;
  int    L3_preloadTicks = 0;

  while (true) {
    // ---- Drive ----
    A = Controller.AxisA.position();
    C = Controller.AxisC.position();
    if (fabs(static_cast<float>(A)) < DEADBAND) { A = 0.0; }
    if (fabs(static_cast<float>(C)) < DEADBAND) { C = 0.0; }
    Left_Motor.setVelocity(clampf(A - C, MAX_SPEED), percent);
    Right_Motor.setVelocity(clampf(A + C, MAX_SPEED), percent);
    Left_Motor.spin(forward);
    Right_Motor.spin(forward);

    // ---- Intake: ONE place decides (LDown reverse > toggled forward > stop) ----
    if (Controller.ButtonLDown.pressing()) {
      Take_bag.spin(reverse);
    } else if (Intake_F == 1.0) {
      Take_bag.spin(forward);
    } else {
      Take_bag.stop();
    }

    // ---- L4 keeper: resting on the ground but creeping up (spring/bounce)? push it back to 0 once ----
    if (controlsEnabled && !L4_busy && !L4_up) {
      if (armPos() > L4_DRIFT_DEG) {
        if (!L4_driftFix) {
          L4_driftFix = true;
          L4.setStopping(hold);
          L4.setVelocity(MAX_SPEED, percent);
          L4.spinToPosition(0.0, degrees, false);
        }
      } else {
        L4_driftFix = false;
      }
    }

    // ---- L3 (shoot): FUp = spin until the sensor says it fired, then reload (one shot per press) ----
    bool fUp = Controller.ButtonFUp.pressing();
    // Shoot gate = ARM + DISTANCE
    //  arm:  not moving, above the floor, and above a fraction of the height it really reached
    //  dist: L3 is loaded (sensor at/below L3_STOP_MM)
    double shootMinPos = L4_upReached * L4_SHOOT_FRACTION;
    if (shootMinPos < L4_SHOOT_MIN_POS) { shootMinPos = L4_SHOOT_MIN_POS; }
    double armDeg = armPos();
    double distMm = 0.0;          // 0 = no sensor
    bool needReload = false;      // sensor says L3 is not pulled back yet
#if USE_DISTANCE_SENSOR
    distMm = Distance7.objectDistance(mm);
    if (distMm > L3_STOP_MM) {
      farTicks++;
      if (farTicks == 1) { farPos = L3MotorA.position(degrees); }   // first FAR reading = where it released
    } else {
      farTicks = 0;
    }
    needReload = (farTicks >= 3);   // FAR for 3 loops (60 ms) in a row
#endif
    bool armOk = (!L4_busy) && (armDeg >= shootMinPos);
    bool distOk = true;           // no sensor -> the distance condition is skipped
#if USE_DISTANCE_SENSOR
    distOk = (distMm <= L3_STOP_MM);
#endif
    bool shootReady = armOk && distOk;
    if (fUp && !fUpWasPressed && L3_shotTicks == 0 && !L3_firing && shootReady && !maintenance) {
      L3_firing = true;           // L3 spins at once (also cancels a running pre-load)
      L3_skipLearn = L3_preloading;   // FUp pressed before the pre-load finished: this shot says nothing about the pre-load
      L3_preloading = false;
      L3_preHeld = false;
      L3_fireTicks = 0;
      L3_fireStartPos = L3MotorA.position(degrees);
      shootCycle = true;          // STOP 1: FUp clicked -> intake off, LUp locked
      Intake_F = 0.0;
    }
    fUpWasPressed = fUp;

    // FUp shot ends when the sensor says it FIRED (FAR for 3 loops, after the minimum time). L3 is NOT stopped:
    // it keeps spinning forward and the reload branch below takes over = reload starts with no gap.
    if (L3_firing) {
      L3_fireTicks++;
      if (needReload && L3_fireTicks * 20 >= L3_FIRE_MIN_MS) {
        L3_firing = false;
        L3_travel = farPos - L3_fireStartPos;       // degrees from the hold position to the release
#if L3_PRELOAD_AUTO
        if (!L3_skipLearn) {                        // learn by feedback: the travel still left after the hold should be ~ the margin
          double delta = L3_travel - L3_PRELOAD_MARGIN;          // + = can pull more, - = pulled too far
          if (delta > L3_PRELOAD_STEP) { delta = L3_PRELOAD_STEP; }   // grow slowly (a few shots), shrink at once
          if (delta > 0.0 && L3_preloadLocked) { delta = 0.0; }       // after a self-release it may only shrink
          double learned = L3_preloadDeg + delta;
          if (learned < 0.0) { learned = 0.0; }
          if (learned > L3_PRELOAD_MAX) { learned = L3_PRELOAD_MAX; }
          L3_preloadDeg = learned;
        }
#endif
      } else if (L3_fireTicks * 20 >= L3_FIRE_TIMEOUT_MS) {
        L3_firing = false;                          // never fired: falls through to hold below
      }
    }

    // FDown: shoot WITHOUT reload (same gate: arm OK + distance OK; the arm is never touched).
    //   FDown again while in maintenance = re-validate like first start (shooter first, then arm).
    bool fDown = Controller.ButtonFDown.pressing();
    if (fDown && !fDownWasPressed && L3_shotTicks == 0 && !L3_firing) {
      if (maintenance) {
        calibrateAll();           // blocks: distance -> pull shooter back -> then arm -> flags reset (maintenance = false)
        L3_holding = true;
        L3_preloading = false;
        L3_preHeld = false;
        L3_catchPos = 0.0;        // calibrateAll() zeroed the encoders at the sensor catch
      } else if (shootReady) {
        L3_preloading = false;
        L3_preHeld = false;
        L3_shotTicks = L3_MAINT_SHOT_MS / 20;
        maintenance = true;       // no reload from now until re-validated
        shootCycle = true;
        Intake_F = 0.0;
      }
    }
    fDownWasPressed = fDown;

    if (L3_firing) {
      L3.spin(forward);           // FUp shot: spin until the sensor says it fired (handled above)
      L3_holding = false;
    } else if (L3_shotTicks > 0) {
      L3.spin(forward);           // FDown maintenance shot (fixed time)
      L3_holding = false;
      L3_shotTicks--;
      if (L3_shotTicks == 0) {
        if (maintenance) { L3.stop(coast); L3_holding = true; }   // left released, free to maintain
        else             { L3.stop(); }
      }
    } else if (maintenance) {
      if (shootCycle) { shootCycle = false; }   // shot done, NO reload, intake stays off
    } else if (needReload) {
      if (L3_preloading || L3_preHeld) {          // it released BY ITSELF during/after the pre-load: pre-load was too big
        L3_preloadDeg -= 2.0 * L3_PRELOAD_STEP;   // back off two steps (the last shot was still fine one step ago)
        if (L3_preloadDeg < 0.0) { L3_preloadDeg = 0.0; }
        L3_preloadLocked = true;                  // and never grow again until restart
      }
      L3_preloading = false;      // released (shot) or lost the sensor -> normal reload
      L3_preHeld = false;
      L3.spin(forward);
      L3_holding = false;
    } else if (L3_preloading) {
      L3.spin(forward);           // extra pull after the sensor caught it, then HOLD
      L3_preloadTicks--;
      if (L3MotorA.position(degrees) >= L3_preloadTarget || L3_preloadTicks <= 0) {
        L3_preloading = false;
        L3_preHeld = true;
        L3.stop(hold);
      }
    } else {
      if (!L3_holding) {          // the sensor JUST caught it
        L3_catchPos = L3MotorA.position(degrees);
        if (L3_preloadDeg > 0.0) {
          L3_preloading = true;   // keep spinning (no stop) for L3_preloadDeg more, then hold
          L3_preloadTarget = L3_catchPos + L3_preloadDeg;
          L3_preloadTicks = L3_PRELOAD_TIMEOUT_MS / 20;
        } else {
          L3.stop(hold);          // pulled back: HOLD it (once)
        }
        L3_holding = true;
      }
      if (shootCycle) {           // STOP 2 ends: shot done, L3 pulled back and holding -> intake starts again, LUp unlocked
        shootCycle = false;
        Intake_F = 1.0;
      }
    }

    // ---- Show L4 position on the Brain (use it to tune L4_UP_POS) ----
    screenTick++;
    if (screenTick >= 10) {
      screenTick = 0;
      Brain.Screen.setCursor(1, 1);
      Brain.Screen.print("L4 %4.0f/%.0f %s   ", armDeg, shootMinPos, armOk ? "OK" : "LOW");
      Brain.Screen.setCursor(2, 1);
#if USE_DISTANCE_SENSOR
      Brain.Screen.print("Dist %4.0f %s   ", distMm, distOk ? "OK" : "FAR");
#else
      Brain.Screen.print("Dist: no sensor   ");
#endif
      Brain.Screen.setCursor(3, 1);
      if (maintenance) { Brain.Screen.print("MAINT: FDown=revalidate  "); }
      else             { Brain.Screen.print("SHOOT %s   ", shootReady ? "READY" : "BLOCKED"); }
      Brain.Screen.setCursor(4, 1);
      Brain.Screen.print("L3 trv %3.0f pre %3.0f  ", L3_travel, L3_preloadDeg);   // last FUp: degrees from hold to release / current pre-load
    }

    wait(20, msec);
  }
  return 0;
}

// ============================================================
// BUTTON EVENTS (all ignored until calibration is finished)
// ============================================================

// LUp: toggle intake
void onevent_ControllerButtonLUp_pressed_0() {
  if (!controlsEnabled || shootCycle) return;   // can't turn the intake on during a shoot cycle
  Intake_F = (Intake_F == 0.0) ? 1.0 : 0.0;
}

// EUp: go to the up position and hold. Blocked if already up.
void onevent_ControllerButtonEUp_pressed_0() {
  if (!controlsEnabled || L4_busy || L4_up) return;
  L4_busy = true;
  L4_moveTo(L4_UP_POS, L4_UP_SPEED, L4_UP_TIMEOUT_MS);
  L4.stop(hold);
  L4_upReached = armPos();   // remember the real height for the shoot gate
  L4_up = true;
  TouchLED12.setColor(red);         // red = at up position
  L4_busy = false;
}

// EDown: slow down to the ground (position 0). Only works if L4 is up.
void onevent_ControllerButtonEDown_pressed_0() {
  if (!controlsEnabled || L4_busy || !L4_up) return;
  L4_busy = true;
  L4_moveTo(L4_DOWN_POS, L4_DOWN_SPEED, L4_DOWN_TIMEOUT_MS);
  L4.stop(brake);
  L4.setVelocity(MAX_SPEED, percent);
  L4_up = false;
  TouchLED12.setColor(green);       // green = EUp allowed again
  L4_busy = false;
}

int main() {
  vexcodeInit();

  calibrateAll();                   // blocks here; controller is dead until it finishes

  // Buttons are only registered AFTER calibration
  Controller.ButtonLUp.pressed(onevent_ControllerButtonLUp_pressed_0);
  Controller.ButtonEUp.pressed(onevent_ControllerButtonEUp_pressed_0);
  Controller.ButtonEDown.pressed(onevent_ControllerButtonEDown_pressed_0);

  wait(15, msec);
  whenStarted1();
}
