/*
  UNO Q robot: TB6612FNG, two DC motors, Wi-Fi commands via Arduino Bridge.
  This Arduino IDE sketch controls motors; the included App Lab Python program
  provides Wi-Fi on the UNO Q Linux processor.

  TB6612FNG     UNO Q Arduino header
  PWMA          D10 (PWM, MCU PB9)
  AIN1          D11 (MCU PB15)
  AIN2          D12 (MCU PB14)
  PWMB          D6  (PWM, MCU PB1)
  BIN1          D5  (MCU PA11)
  BIN2          D7  (MCU PB2)
  VCC, STBY     3.3V (STBY tied HIGH for this prototype)
  VM            motor battery positive (+6V ONLY if motors are rated for 6V)
  GND           motor battery negative AND UNO Q GND (common ground)
  AO1/AO2       left motor; BO1/BO2 right motor

  The UNO Q itself must be powered separately by a suitable 5V / 3A USB-C supply.
  Never connect the 6V motor supply to UNO Q 3.3V or 5V.
*/

#include <Arduino.h>
#include <Arduino_RouterBridge.h>

const uint8_t PWMA_PIN = 10;
const uint8_t AIN1_PIN = 11;
const uint8_t AIN2_PIN = 12;
const uint8_t PWMB_PIN = 6;
const uint8_t BIN1_PIN = 5;
const uint8_t BIN2_PIN = 7;

// If a wheel spins backward when commanded forward, set its flag to true.
// The Invert checkboxes in the web UI change these at runtime via
// robot_invert(); once you know the right pair, set them here so they survive
// a restart.
// Both false because the left motor's AO1/AO2 leads are now swapped at the
// driver. If you ever unswap them, set leftReversed back to true.
bool leftReversed = false;
bool rightReversed = false;

// Adjust after testing with wheels raised. Valid range: 0..100.
const int DRIVE_PERCENT = 60;

// A spin turn has to scrub both tyres sideways, so it needs noticeably more
// torque than driving straight. Raise this first if turns are sluggish.
const int TURN_PERCENT = 90;

// Breakaway burst at the start of every move: full duty for KICK_MS, then the
// normal duty. Without it short commands only make the motors buzz. The burst
// is capped at a third of the move so short turns do not over-rotate.
const int KICK_PERCENT = 100;
const unsigned long KICK_MS = 120UL;

// Active (short) brake at the end of a move so the robot does not coast past
// its target angle. Set to 0 to go back to coasting.
const unsigned long BRAKE_MS = 90UL;

// Any non-zero duty below this stalls instead of moving; it gets raised to it.
const int MIN_RUN_PERCENT = 35;

// Per-side scaling in percent. If the robot curves while driving straight, or
// one turn direction is wider than the other, trim the stronger side down.
const int LEFT_TRIM = 100;
const int RIGHT_TRIM = 100;

const unsigned long MAX_MOVE_MS = 5000UL;

enum Phase : uint8_t { PHASE_IDLE, PHASE_KICK, PHASE_RUN, PHASE_BRAKE };

Phase phase = PHASE_IDLE;
unsigned long phaseStartedMs = 0;
unsigned long movementStartedMs = 0;
unsigned long movementDurationMs = 0;
unsigned long activeKickMs = 0;
int runLeft = 0;
int runRight = 0;

// Last commanded direction per motor, for H-bridge dead time. 0 = left, 1 = right.
int8_t lastSign[2] = {0, 0};

static int signOf(int value) {
  if (value > 0) return 1;
  if (value < 0) return -1;
  return 0;
}

void setOneMotor(uint8_t index, uint8_t pwmPin, uint8_t in1, uint8_t in2,
                 int percent, bool reversed, int trim) {
  percent = constrain(percent, -100, 100);
  percent = (percent * trim) / 100;
  if (reversed) percent = -percent;

  int sign = signOf(percent);
  int magnitude = abs(percent);
  if (magnitude > 0 && magnitude < MIN_RUN_PERCENT) magnitude = MIN_RUN_PERCENT;

  // Cut PWM before changing the direction pins.
  analogWrite(pwmPin, 0);

  if (sign != 0 && lastSign[index] != 0 && sign != lastSign[index]) {
    // Dead time: a straight-through reversal collapses VM and the motor never
    // breaks away. Both inputs low coasts the bridge while it settles.
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    delayMicroseconds(2000);
  }

  if (sign > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (sign < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  lastSign[index] = (int8_t)sign;
  analogWrite(pwmPin, (magnitude * 255) / 100);
}

void setMotors(int leftPercent, int rightPercent) {
  setOneMotor(0, PWMA_PIN, AIN1_PIN, AIN2_PIN,
              leftPercent, leftReversed, LEFT_TRIM);
  setOneMotor(1, PWMB_PIN, BIN1_PIN, BIN2_PIN,
              rightPercent, rightReversed, RIGHT_TRIM);
}

void coastMotors() {
  analogWrite(PWMA_PIN, 0);
  analogWrite(PWMB_PIN, 0);
  digitalWrite(AIN1_PIN, LOW);
  digitalWrite(AIN2_PIN, LOW);
  digitalWrite(BIN1_PIN, LOW);
  digitalWrite(BIN2_PIN, LOW);
  lastSign[0] = 0;
  lastSign[1] = 0;
  phase = PHASE_IDLE;
}

// TB6612FNG short brake: both inputs high with PWM asserted.
void brakeMotors() {
  analogWrite(PWMA_PIN, 0);
  analogWrite(PWMB_PIN, 0);
  digitalWrite(AIN1_PIN, HIGH);
  digitalWrite(AIN2_PIN, HIGH);
  digitalWrite(BIN1_PIN, HIGH);
  digitalWrite(BIN2_PIN, HIGH);
  analogWrite(PWMA_PIN, 255);
  analogWrite(PWMB_PIN, 255);
  lastSign[0] = 0;
  lastSign[1] = 0;
}

void beginBrake() {
  runLeft = 0;
  runRight = 0;
  if (BRAKE_MS == 0) {
    coastMotors();
    return;
  }
  brakeMotors();
  phase = PHASE_BRAKE;
  phaseStartedMs = millis();
}

void startMovement(int leftPercent, int rightPercent, unsigned long durationMs) {
  runLeft = leftPercent;
  runRight = rightPercent;
  movementStartedMs = millis();
  movementDurationMs = durationMs;
  phaseStartedMs = movementStartedMs;

  // A full-length burst on a short move would dominate it, so cap the kick at
  // a third of the requested time. Small turns stay proportional to big ones.
  activeKickMs = min(KICK_MS, durationMs / 3);

  if (activeKickMs > 0 && KICK_PERCENT > 0) {
    phase = PHASE_KICK;
    setMotors(signOf(leftPercent) * KICK_PERCENT,
              signOf(rightPercent) * KICK_PERCENT);
  } else {
    phase = PHASE_RUN;
    setMotors(runLeft, runRight);
  }
}

// Drives the kick -> run -> brake sequence and the movement timeout.
void updateMotion() {
  unsigned long now = millis();

  switch (phase) {
    case PHASE_KICK:
      if ((unsigned long)(now - movementStartedMs) >= movementDurationMs) {
        beginBrake();
      } else if ((unsigned long)(now - phaseStartedMs) >= activeKickMs) {
        setMotors(runLeft, runRight);
        phase = PHASE_RUN;
        phaseStartedMs = now;
      }
      break;

    case PHASE_RUN:
      if ((unsigned long)(now - movementStartedMs) >= movementDurationMs) {
        beginBrake();
      }
      break;

    case PHASE_BRAKE:
      if ((unsigned long)(now - phaseStartedMs) >= BRAKE_MS) {
        coastMotors();
      }
      break;

    case PHASE_IDLE:
      break;
  }
}

// Called from the UNO Q Linux side via Bridge.call("robot_command", ...).
// action: 0=stop, 1=forward, 2=left, 3=right
// Return: 1=accepted, 0=busy, -1=invalid command.
int robotCommand(int action, int durationMs) {
  if (action == 0) {
    if (phase != PHASE_IDLE) beginBrake();
    return 1;
  }

  if (action < 1 || action > 3 ||
      durationMs < 1 || durationMs > (int)MAX_MOVE_MS) {
    return -1;
  }

  // Braking is not a movement: a new command may interrupt it.
  if (phase == PHASE_KICK || phase == PHASE_RUN) return 0;

  switch (action) {
    case 1: startMovement( DRIVE_PERCENT,  DRIVE_PERCENT, (unsigned long)durationMs); break;
    case 2: startMovement(-TURN_PERCENT,   TURN_PERCENT,  (unsigned long)durationMs); break;
    case 3: startMovement( TURN_PERCENT,  -TURN_PERCENT,  (unsigned long)durationMs); break;
  }
  return 1;
}

// Flip either wheel's polarity at runtime, for sorting out motor wiring.
// Always stops first so the change cannot be applied mid-move.
// Return: 1=applied.
int robotInvert(int left, int right) {
  if (phase != PHASE_IDLE) beginBrake();
  leftReversed = (left != 0);
  rightReversed = (right != 0);
  return 1;
}

// Raw per-wheel control, for wiring checks and timing calibration.
// Percentages are -100..100 (negative = that wheel backwards).
// Return: 1=accepted, 0=busy, -1=invalid command.
int robotDrive(int leftPercent, int rightPercent, int durationMs) {
  if (durationMs < 1 || durationMs > (int)MAX_MOVE_MS) return -1;
  if (leftPercent < -100 || leftPercent > 100) return -1;
  if (rightPercent < -100 || rightPercent > 100) return -1;

  if (phase == PHASE_KICK || phase == PHASE_RUN) return 0;

  startMovement(leftPercent, rightPercent, (unsigned long)durationMs);
  return 1;
}

// 1=still moving or braking; 0=fully stopped. The Linux side polls this for DONE.
int robotStatus() {
  updateMotion();
  return phase != PHASE_IDLE ? 1 : 0;
}

void setup() {
  // Configure safe startup states before enabling PWM.
  pinMode(PWMA_PIN, OUTPUT);
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(PWMB_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);

  analogWriteResolution(8);
  coastMotors();

  Bridge.begin();
  // Hardware API calls execute in the Arduino loop context.
  Bridge.provide_safe("robot_command", robotCommand);
  Bridge.provide_safe("robot_drive", robotDrive);
  Bridge.provide_safe("robot_invert", robotInvert);
  Bridge.provide_safe("robot_status", robotStatus);
}

void loop() {
  // Independent 5-second maximum MCU-side movement timeout.
  updateMotion();
  delay(1);
}
