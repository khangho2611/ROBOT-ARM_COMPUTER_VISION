#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

const byte PCA9685_ADDRESS = 0x40;
const unsigned long SERIAL_BAUD = 115200;
const int SERVO_FREQ_HZ = 60;
const int SERVOMIN = 250;
const int SERVOMAX = 490;
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;
const byte SERVO_COUNT = 3;
const byte PCA9685_OUTPUT_COUNT = 16;
const int SERVO_MIN_ANGLE = 0;
const int SERVO_MAX_ANGLE = 180;
const int HOME_ANGLE = 90;
const byte SERVO_STEP_DEG = 4;
const int SERVO_STEP_DELAY_MS = 4;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDRESS);

int servoAngles[SERVO_COUNT] = {HOME_ANGLE, HOME_ANGLE, HOME_ANGLE};
char commandBuffer[32];
byte commandLength = 0;

uint16_t angleToPulse(int angle) {
  angle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  long pulse = map(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, SERVOMIN, SERVOMAX);
  return constrain(pulse, 0L, 4095L);
}

uint16_t microsecondsToPulse(int microseconds) {
  microseconds = constrain(microseconds, SERVO_MIN_US, SERVO_MAX_US);
  long pulse = ((long)microseconds * SERVO_FREQ_HZ * 4096L) / 1000000L;
  return constrain(pulse, 0L, 4095L);
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

  value = (int)parsed;
  return true;
}

bool parseIntPair(char *text, int &first, int &second) {
  char *comma = strchr(text, ',');

  if (comma == NULL) {
    return false;
  }

  *comma = '\0';
  return parseIntStrict(text, first) && parseIntStrict(comma + 1, second);
}

bool isValidChannel(int channel) {
  return channel >= 0 && channel < SERVO_COUNT;
}

bool isValidAngle(int angle) {
  return angle >= SERVO_MIN_ANGLE && angle <= SERVO_MAX_ANGLE;
}

bool isValidMicroseconds(int microseconds) {
  return microseconds >= SERVO_MIN_US && microseconds <= SERVO_MAX_US;
}

void releaseServo(byte channel) {
  pwm.setPWM(channel, 0, 0);
}

void writeServoNow(byte channel, int angle) {
  uint16_t pulse = angleToPulse(angle);
  pwm.setPWM(channel, 0, pulse);
  servoAngles[channel] = angle;
}

void setServoAngle(byte channel, int angle) {
  angle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  int currentAngle = servoAngles[channel];

  if (currentAngle == angle) {
    writeServoNow(channel, angle);
  } else {
    int direction = angle > currentAngle ? SERVO_STEP_DEG : -SERVO_STEP_DEG;

    while (currentAngle != angle) {
      currentAngle += direction;

      if ((direction > 0 && currentAngle > angle) ||
          (direction < 0 && currentAngle < angle)) {
        currentAngle = angle;
      }

      writeServoNow(channel, currentAngle);
      delay(SERVO_STEP_DELAY_MS);
    }
  }

  Serial.print(F("OK "));
  Serial.print(channel);
  Serial.print(F(","));
  Serial.println(angle);
}

void setServoMicroseconds(byte channel, int microseconds) {
  microseconds = constrain(microseconds, SERVO_MIN_US, SERVO_MAX_US);
  uint16_t pulse = microsecondsToPulse(microseconds);

  pwm.setPWM(channel, 0, pulse);
  servoAngles[channel] = map(
      microseconds,
      SERVO_MIN_US,
      SERVO_MAX_US,
      SERVO_MIN_ANGLE,
      SERVO_MAX_ANGLE);

  Serial.print(F("OK U "));
  Serial.print(channel);
  Serial.print(F(","));
  Serial.print(microseconds);
  Serial.print(F(" PULSE "));
  Serial.println(pulse);
}

void homeAllServos() {
  for (byte channel = 0; channel < SERVO_COUNT; channel++) {
    setServoAngle(channel, HOME_ANGLE);
    delay(50);
  }
}

void releaseAllServos() {
  for (byte channel = 0; channel < PCA9685_OUTPUT_COUNT; channel++) {
    pwm.setPWM(channel, 0, 0);
  }
}

void resetPCA9685ToIdle() {
  pwm.reset();
  delay(10);
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ_HZ);
  delay(10);
  releaseAllServos();

  for (byte channel = 0; channel < SERVO_COUNT; channel++) {
    servoAngles[channel] = HOME_ANGLE;
  }
}

void processCommand(char *command) {
  if (strcmp(command, "H") == 0) {
    homeAllServos();
    return;
  }

  if (strcmp(command, "P") == 0) {
    resetPCA9685ToIdle();
    Serial.println(F("OK PCA_RESET_IDLE"));
    return;
  }

  if (strcmp(command, "R") == 0) {
    releaseAllServos();
    Serial.println(F("OK RELEASE"));
    return;
  }

  if (strncmp(command, "U,", 2) == 0) {
    int channel = 0;
    int microseconds = 0;

    if (!parseIntPair(command + 2, channel, microseconds)) {
      Serial.println(F("ERR U_FORMAT"));
      return;
    }

    if (!isValidChannel(channel)) {
      Serial.println(F("ERR CHANNEL"));
      return;
    }

    if (!isValidMicroseconds(microseconds)) {
      Serial.println(F("ERR US"));
      return;
    }

    setServoMicroseconds((byte)channel, microseconds);
    return;
  }

  int channel = 0;
  int angle = 0;

  if (!parseIntPair(command, channel, angle)) {
    Serial.println(F("ERR FORMAT"));
    return;
  }

  if (!isValidChannel(channel)) {
    Serial.println(F("ERR CHANNEL"));
    return;
  }

  if (!isValidAngle(angle)) {
    Serial.println(F("ERR ANGLE"));
    return;
  }

  setServoAngle((byte)channel, angle);
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    char incoming = (char)Serial.read();

    if (incoming == '\r') {
      continue;
    }

    if (incoming == '\n') {
      commandBuffer[commandLength] = '\0';
      processCommand(commandBuffer);
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
  Wire.begin();

  pwm.begin();
  resetPCA9685ToIdle();
  Serial.println(F("READY PCA9685 0x40 WAITING"));
}

void loop() {
  readSerialCommands();
}
