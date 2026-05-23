#include <Arduino.h>
#include <Servo.h>
#include <stdlib.h>
#include <string.h>

const unsigned long SERIAL_BAUD = 9600;

// Servo pins
const byte PIN_D11_BASE = 11;
const byte PIN_D10_LIFT = 10;
const byte PIN_D9_EXTEND = 9;
const byte PIN_D6_GRIPPER = 6;

// =========================
// Manual calibration angles
// =========================
// Remembered safe defaults:
// D11 Base = 82, D10 Lift = 0, D9 Extend = 0, D6 Gripper/Open = 135.
// Fill PICK/DROP/GRIP_CLOSE with real calibrated angles before full picking.

const int HOME_D11 = 82;
const int HOME_D10 = 0;
const int HOME_D9 = 0;
const int GRIP_OPEN = 135;
const int GRIP_CLOSE = 90;    // TODO: fill calibrated close angle

const int PICK_UP_D10 = 0;    // TODO: fill calibrated angle
const int PICK_UP_D9 = 0;     // TODO: fill calibrated angle

const int PICK_DOWN_D10 = 0;  // TODO: fill calibrated angle
const int PICK_DOWN_D9 = 0;   // TODO: fill calibrated angle

const int DROP_D11 = 82;      // TODO: fill calibrated angle
const int DROP_D10 = 0;       // TODO: fill calibrated angle
const int DROP_D9 = 0;        // TODO: fill calibrated angle

// =========================
// Motion tuning
// =========================

const int SERVO_MIN_ANGLE = 0;
const int SERVO_MAX_ANGLE = 180;
const unsigned int SERVO_STEP_DELAY_MS = 15;
const unsigned int POSE_SETTLE_MS = 150;
const unsigned int GRIP_SETTLE_MS = 350;

Servo baseServo;
Servo liftServo;
Servo extendServo;
Servo gripperServo;

bool servosEnabled = false;

int baseCurrentAngle = HOME_D11;
int liftCurrentAngle = HOME_D10;
int extendCurrentAngle = HOME_D9;
int gripperCurrentAngle = GRIP_OPEN;

char commandBuffer[64];
byte commandLength = 0;

int limitAngle(int angle) {
  return constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
}

void waitSmall(unsigned int durationMs) {
  unsigned long startMs = millis();
  while (millis() - startMs < durationMs) {
    delay(5);
  }
}

void writeServoSafe(Servo &servo, int angle) {
  servo.write(limitAngle(angle));
}

bool parseIntStrict(const char *text, int &value) {
  while (*text == ' ' || *text == '\t') {
    text++;
  }

  if (*text == '\0') {
    return false;
  }

  char *endPtr = NULL;
  long parsed = strtol(text, &endPtr, 10);

  while (*endPtr == ' ' || *endPtr == '\t') {
    endPtr++;
  }

  if (*endPtr != '\0') {
    return false;
  }

  value = limitAngle((int)parsed);
  return true;
}

bool readNextInt(int &value) {
  char *token = strtok(NULL, ",");
  if (token == NULL) {
    return false;
  }
  return parseIntStrict(token, value);
}

bool ensureServosEnabled() {
  if (servosEnabled) {
    return true;
  }

  Serial.println(F("ERR SERVOS_DISABLED PRESS ENABLE FIRST"));
  return false;
}

void attachServosIfNeeded() {
  if (!baseServo.attached()) {
    baseServo.attach(PIN_D11_BASE);
  }
  if (!liftServo.attached()) {
    liftServo.attach(PIN_D10_LIFT);
  }
  if (!extendServo.attached()) {
    extendServo.attach(PIN_D9_EXTEND);
  }
  if (!gripperServo.attached()) {
    gripperServo.attach(PIN_D6_GRIPPER);
  }
}

void enableServosAtAngles(int d11, int d10, int d9, int d6) {
  baseCurrentAngle = limitAngle(d11);
  liftCurrentAngle = limitAngle(d10);
  extendCurrentAngle = limitAngle(d9);
  gripperCurrentAngle = limitAngle(d6);

  attachServosIfNeeded();

  writeServoSafe(baseServo, baseCurrentAngle);
  writeServoSafe(liftServo, liftCurrentAngle);
  writeServoSafe(extendServo, extendCurrentAngle);
  writeServoSafe(gripperServo, gripperCurrentAngle);

  servosEnabled = true;
  waitSmall(200);

  Serial.print(F("SERVOS ENABLED "));
  Serial.print(baseCurrentAngle);
  Serial.print(',');
  Serial.print(liftCurrentAngle);
  Serial.print(',');
  Serial.print(extendCurrentAngle);
  Serial.print(',');
  Serial.println(gripperCurrentAngle);
}

void detachServos() {
  baseServo.detach();
  liftServo.detach();
  extendServo.detach();
  gripperServo.detach();
  servosEnabled = false;
  Serial.println(F("SERVOS DETACHED"));
}

void moveServoSmooth(Servo &servo, int &currentAngle, int targetAngle) {
  if (!ensureServosEnabled()) {
    return;
  }

  targetAngle = limitAngle(targetAngle);
  currentAngle = limitAngle(currentAngle);

  while (currentAngle != targetAngle) {
    if (currentAngle < targetAngle) {
      currentAngle++;
    } else {
      currentAngle--;
    }

    writeServoSafe(servo, currentAngle);
    delay(SERVO_STEP_DELAY_MS);
  }
}

void moveTwoServoSmooth(
  Servo &servoA,
  int &currentA,
  int targetA,
  Servo &servoB,
  int &currentB,
  int targetB
) {
  if (!ensureServosEnabled()) {
    return;
  }

  targetA = limitAngle(targetA);
  targetB = limitAngle(targetB);
  currentA = limitAngle(currentA);
  currentB = limitAngle(currentB);

  int startA = currentA;
  int startB = currentB;
  int deltaA = targetA - startA;
  int deltaB = targetB - startB;
  int steps = max(abs(deltaA), abs(deltaB));

  if (steps == 0) {
    return;
  }

  for (int step = 1; step <= steps; step++) {
    int nextA = startA + (long)deltaA * step / steps;
    int nextB = startB + (long)deltaB * step / steps;

    if (nextA != currentA) {
      currentA = limitAngle(nextA);
      writeServoSafe(servoA, currentA);
    }

    if (nextB != currentB) {
      currentB = limitAngle(nextB);
      writeServoSafe(servoB, currentB);
    }

    delay(SERVO_STEP_DELAY_MS);
  }

  currentA = targetA;
  currentB = targetB;
  writeServoSafe(servoA, currentA);
  writeServoSafe(servoB, currentB);
}

void moveHome() {
  if (!ensureServosEnabled()) {
    return;
  }

  moveTwoServoSmooth(liftServo, liftCurrentAngle, HOME_D10, extendServo, extendCurrentAngle, HOME_D9);
  waitSmall(POSE_SETTLE_MS);
  moveServoSmooth(baseServo, baseCurrentAngle, HOME_D11);
  waitSmall(POSE_SETTLE_MS);
  moveServoSmooth(gripperServo, gripperCurrentAngle, GRIP_OPEN);
  waitSmall(GRIP_SETTLE_MS);

  Serial.println(F("HOME DONE"));
}

void runPickSequence(int baseAngle) {
  Serial.println(F("RECEIVED PICK"));

  if (!ensureServosEnabled()) {
    return;
  }

  baseAngle = limitAngle(baseAngle);

  moveServoSmooth(gripperServo, gripperCurrentAngle, GRIP_OPEN);
  waitSmall(GRIP_SETTLE_MS);

  Serial.println(F("MOVING BASE"));
  moveServoSmooth(baseServo, baseCurrentAngle, baseAngle);
  waitSmall(POSE_SETTLE_MS);

  Serial.println(F("PICK UP"));
  moveTwoServoSmooth(liftServo, liftCurrentAngle, PICK_UP_D10, extendServo, extendCurrentAngle, PICK_UP_D9);
  waitSmall(POSE_SETTLE_MS);

  Serial.println(F("PICK DOWN"));
  moveTwoServoSmooth(liftServo, liftCurrentAngle, PICK_DOWN_D10, extendServo, extendCurrentAngle, PICK_DOWN_D9);
  waitSmall(POSE_SETTLE_MS);

  Serial.println(F("GRIP CLOSE"));
  moveServoSmooth(gripperServo, gripperCurrentAngle, GRIP_CLOSE);
  waitSmall(GRIP_SETTLE_MS);

  Serial.println(F("LIFT"));
  moveTwoServoSmooth(liftServo, liftCurrentAngle, PICK_UP_D10, extendServo, extendCurrentAngle, PICK_UP_D9);
  waitSmall(POSE_SETTLE_MS);

  Serial.println(F("DROP"));
  moveServoSmooth(baseServo, baseCurrentAngle, DROP_D11);
  waitSmall(POSE_SETTLE_MS);
  moveTwoServoSmooth(liftServo, liftCurrentAngle, DROP_D10, extendServo, extendCurrentAngle, DROP_D9);
  waitSmall(POSE_SETTLE_MS);
  moveServoSmooth(gripperServo, gripperCurrentAngle, GRIP_OPEN);
  waitSmall(GRIP_SETTLE_MS);

  moveHome();
}

void setSingleServo(const char *servoId, int angle) {
  if (!ensureServosEnabled()) {
    return;
  }

  angle = limitAngle(angle);

  if (strcmp(servoId, "D11") == 0) {
    moveServoSmooth(baseServo, baseCurrentAngle, angle);
  } else if (strcmp(servoId, "D10") == 0) {
    moveServoSmooth(liftServo, liftCurrentAngle, angle);
  } else if (strcmp(servoId, "D9") == 0) {
    moveServoSmooth(extendServo, extendCurrentAngle, angle);
  } else if (strcmp(servoId, "D6") == 0) {
    moveServoSmooth(gripperServo, gripperCurrentAngle, angle);
  } else {
    Serial.println(F("ERR SERVO_ID USE D11 D10 D9 D6"));
    return;
  }

  Serial.print(F("OK SET "));
  Serial.print(servoId);
  Serial.print(',');
  Serial.println(angle);
}

void setAllServos(int d11, int d10, int d9, int d6) {
  if (!ensureServosEnabled()) {
    return;
  }

  moveServoSmooth(baseServo, baseCurrentAngle, d11);
  moveTwoServoSmooth(liftServo, liftCurrentAngle, d10, extendServo, extendCurrentAngle, d9);
  moveServoSmooth(gripperServo, gripperCurrentAngle, d6);
  Serial.println(F("OK SETALL"));
}

void printStatus() {
  Serial.print(F("STATUS "));
  if (servosEnabled) {
    Serial.print(F("ENABLED"));
  } else {
    Serial.print(F("DISABLED"));
  }
  Serial.print(F(" D11="));
  Serial.print(baseCurrentAngle);
  Serial.print(F(" D10="));
  Serial.print(liftCurrentAngle);
  Serial.print(F(" D9="));
  Serial.print(extendCurrentAngle);
  Serial.print(F(" D6="));
  Serial.println(gripperCurrentAngle);
}

void processCommand(char *command) {
  char *action = strtok(command, ",");
  if (action == NULL) {
    return;
  }

  if (strcmp(action, "ENABLE") == 0) {
    int d11;
    int d10;
    int d9;
    int d6;
    if (!readNextInt(d11) || !readNextInt(d10) || !readNextInt(d9) || !readNextInt(d6)) {
      Serial.println(F("ERR FORMAT USE ENABLE,<D11>,<D10>,<D9>,<D6>"));
      return;
    }
    enableServosAtAngles(d11, d10, d9, d6);
    return;
  }

  if (strcmp(action, "PICK") == 0) {
    int baseAngle;
    if (!readNextInt(baseAngle)) {
      Serial.println(F("ERR FORMAT USE PICK,<base_angle>"));
      return;
    }
    runPickSequence(baseAngle);
    return;
  }

  if (strcmp(action, "DETACH") == 0) {
    detachServos();
    return;
  }

  if (strcmp(action, "HOME") == 0) {
    moveHome();
    return;
  }

  if (strcmp(action, "STATUS") == 0) {
    printStatus();
    return;
  }

  if (strcmp(action, "SET") == 0) {
    char *servoId = strtok(NULL, ",");
    int angle;
    if (servoId == NULL || !readNextInt(angle)) {
      Serial.println(F("ERR FORMAT USE SET,<D11|D10|D9|D6>,<angle>"));
      return;
    }
    setSingleServo(servoId, angle);
    return;
  }

  if (strcmp(action, "SETALL") == 0) {
    int d11;
    int d10;
    int d9;
    int d6;
    if (!readNextInt(d11) || !readNextInt(d10) || !readNextInt(d9) || !readNextInt(d6)) {
      Serial.println(F("ERR FORMAT USE SETALL,<D11>,<D10>,<D9>,<D6>"));
      return;
    }
    setAllServos(d11, d10, d9, d6);
    return;
  }

  Serial.println(F("ERR UNKNOWN COMMAND"));
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    char incoming = (char)Serial.read();

    if (incoming == '\r') {
      continue;
    }

    if (incoming == '\n') {
      commandBuffer[commandLength] = '\0';
      if (commandLength > 0) {
        processCommand(commandBuffer);
      }
      commandLength = 0;
      return;
    }

    if (commandLength < sizeof(commandBuffer) - 1) {
      commandBuffer[commandLength++] = incoming;
    } else {
      commandLength = 0;
      Serial.println(F("ERR CMD_TOO_LONG"));
    }
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);

  // Safety: do not attach or write any servo on boot/reset/upload.
  Serial.println(F("READY ROBOT_4DOF_SAFE_WAIT"));
  Serial.println(F("SERVOS DISABLED"));
  Serial.println(F("CMD ENABLE,<D11>,<D10>,<D9>,<D6>"));
  Serial.println(F("CMD PICK,<base_angle>"));
}

void loop() {
  readSerialCommands();
}
