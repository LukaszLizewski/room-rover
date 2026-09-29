/*
 * Room Rover
 * -----------
 * Small autonomous rover with:
 *   - an IR distance sensor used to detect obstacles,
 *   - two DC motors sharing a single PWM speed line,
 *   - a servo that sweeps the IR sensor left/right to "look around",
 *   - a 4-digit, 7-segment multiplexed display showing the sensor
 *     reading (in volts).
 *
 * Display wiring:
 *   PIN_SEG_ADDR_A/B/C   - 3-bit address fed into a demultiplexer that
 *                          selects which segment (a..g, or the dot) is
 *                          currently being driven.
 *   PIN_DIGIT_1..4       - enable line for each of the 4 digit positions.
 *                          The segment selected above is only visible on
 *                          digit positions where the enable line is HIGH.
 *
 * The display is refreshed by scanning through the 7 segments (plus the
 * decimal point) and, for every segment, enabling only the digit
 * positions whose current value needs that segment lit. This has to be
 * repeated continuously (see loop()) to avoid visible flicker.
 */

#include <Servo.h>

// ---------------------------------------------------------------------
// Pin configuration
// ---------------------------------------------------------------------
const uint8_t PIN_IR_SENSOR   = A0; // analog input - IR distance sensor
const uint8_t PIN_MOTOR_LEFT  = 2;  // digital output - left motor direction
const uint8_t PIN_MOTOR_RIGHT = 4;  // digital output - right motor direction
const uint8_t PIN_MOTOR_SPEED = 3;  // PWM output - shared motor speed
const uint8_t PIN_SERVO       = 5;  // PWM output - IR sensor sweep servo

const uint8_t PIN_SEG_ADDR_A = 13; // segment-select address, bit A
const uint8_t PIN_SEG_ADDR_B = 12; // segment-select address, bit B
const uint8_t PIN_SEG_ADDR_C = 11; // segment-select address, bit C
const uint8_t PIN_DIGIT_1    = 10; // digit position 1 (hundreds) enable
const uint8_t PIN_DIGIT_2    = 9;  // digit position 2 (tens) enable
const uint8_t PIN_DIGIT_3    = 8;  // digit position 3 (units) enable
const uint8_t PIN_DIGIT_4    = 7;  // digit position 4 (tenths) enable

const uint8_t DIGIT_ENABLE_PIN[4] = { PIN_DIGIT_1, PIN_DIGIT_2, PIN_DIGIT_3, PIN_DIGIT_4 };

// ---------------------------------------------------------------------
// Display timing / behaviour constants
// ---------------------------------------------------------------------
const uint8_t SEGMENT_REFRESH_MS = 2;  // how long each segment stays lit while scanning
const uint8_t DISPLAY_REFRESH_PASSES = 10; // number of full scans per loop() iteration
const uint8_t BLANK_DIGIT = 11;   // sentinel value meaning "no digit displayed here"
const uint8_t DECIMAL_POINT_DIGIT_POS = 2; // dot is always shown after the units digit (0-based index)

// 3-bit address sent to the segment demultiplexer, one row per segment a..g.
const uint8_t SEGMENT_ADDRESS[7][3] = {
  { 0, 0, 1 }, // a
  { 1, 1, 1 }, // b
  { 1, 0, 0 }, // c
  { 1, 1, 0 }, // d
  { 1, 0, 1 }, // e
  { 0, 1, 1 }, // f
  { 0, 1, 0 }, // g
};
const uint8_t DECIMAL_POINT_ADDRESS[3] = { 0, 0, 0 };

// Standard 7-segment truth table: for each digit 0-9, which segments (a..g) are lit.
const bool SEGMENT_LIT_FOR_DIGIT[10][7] = {
  /*        a  b  c  d  e  f  g */
  /* 0 */ { 1, 1, 1, 1, 1, 1, 0 },
  /* 1 */ { 0, 1, 1, 0, 0, 0, 0 },
  /* 2 */ { 1, 1, 0, 1, 1, 0, 1 },
  /* 3 */ { 1, 1, 1, 1, 0, 0, 1 },
  /* 4 */ { 0, 1, 1, 0, 0, 1, 1 },
  /* 5 */ { 1, 0, 1, 1, 0, 1, 1 },
  /* 6 */ { 1, 0, 1, 1, 1, 1, 1 },
  /* 7 */ { 1, 1, 1, 0, 0, 0, 0 },
  /* 8 */ { 1, 1, 1, 1, 1, 1, 1 },
  /* 9 */ { 1, 1, 1, 1, 0, 1, 1 },
};

// ---------------------------------------------------------------------
// Motor / servo behaviour constants
// ---------------------------------------------------------------------
const uint8_t MOTOR_SPEED_FULL = 255;
const uint8_t MOTOR_SPEED_SLOW = 150;
const uint8_t MOTOR_SPEED_OFF  = 0;

const uint8_t SERVO_CENTER = 90;
const uint8_t SERVO_RIGHT  = 120;
const uint8_t SERVO_LEFT   = 60;

const uint16_t SERVO_MOVE_DELAY_MS = 1000;
const uint16_t MOTOR_STEP_DELAY_MS = 1000;

const float VOLTAGE_LOW_THRESHOLD  = 0.0;
const float VOLTAGE_HIGH_THRESHOLD = 1.0;
const float ADC_TO_VOLTS_DIVISOR   = 204.6; // converts analogRead() counts to volts

Servo trackerServo; // controls the servo sweeping the IR sensor

// ---------------------------------------------------------------------
// Display driver
// ---------------------------------------------------------------------

// Drives a single segment: sets the segment address lines, then enables
// every digit position that needs this segment lit for its current value.
void driveSegment(uint8_t segmentIndex, const uint8_t address[3], const int digits[4]) {
  digitalWrite(PIN_SEG_ADDR_A, address[0]);
  digitalWrite(PIN_SEG_ADDR_B, address[1]);
  digitalWrite(PIN_SEG_ADDR_C, address[2]);

  for (uint8_t pos = 0; pos < 4; pos++) {
    int digit = digits[pos];
    if (digit >= 0 && digit <= 9 && SEGMENT_LIT_FOR_DIGIT[digit][segmentIndex]) {
      digitalWrite(DIGIT_ENABLE_PIN[pos], HIGH);
    }
  }

  delay(SEGMENT_REFRESH_MS);

  for (uint8_t pos = 0; pos < 4; pos++) {
    digitalWrite(DIGIT_ENABLE_PIN[pos], LOW);
  }
}

// The decimal point is not tied to a digit value - it is always shown at
// a fixed position (after the units digit).
void driveDecimalPoint() {
  digitalWrite(PIN_SEG_ADDR_A, DECIMAL_POINT_ADDRESS[0]);
  digitalWrite(PIN_SEG_ADDR_B, DECIMAL_POINT_ADDRESS[1]);
  digitalWrite(PIN_SEG_ADDR_C, DECIMAL_POINT_ADDRESS[2]);

  digitalWrite(DIGIT_ENABLE_PIN[DECIMAL_POINT_DIGIT_POS], HIGH);
  delay(SEGMENT_REFRESH_MS);
  digitalWrite(DIGIT_ENABLE_PIN[DECIMAL_POINT_DIGIT_POS], LOW);
}

// One full refresh pass: scans through all 7 segments plus the dot.
void displayDigits(int hundreds, int tens, int units, int tenths) {
  for (uint8_t pos = 0; pos < 4; pos++) {
    digitalWrite(DIGIT_ENABLE_PIN[pos], LOW);
  }

  int digits[4] = { hundreds, tens, units, tenths };
  for (uint8_t segment = 0; segment < 7; segment++) {
    driveSegment(segment, SEGMENT_ADDRESS[segment], digits);
  }
  driveDecimalPoint();
}

// Suppresses leading zeros (e.g. shows "1.9" instead of "001.9"), except
// when the whole number is exactly zero - then it is shown as "0.0".
void suppressLeadingZeros(int &hundreds, int &tens, int units, int tenths) {
  bool isExactlyZero = (tens == 0 && units == 0 && tenths == 0);

  if (hundreds == 0 && !isExactlyZero) {
    hundreds = BLANK_DIGIT;
    if (tens == 0) {
      tens = BLANK_DIGIT;
    }
  }
}

// Splits a non-negative floating point value (0.0 - 999.9) into its
// hundreds/tens/units/tenths digits and refreshes the display once.
void updateDisplay(float value) {
  if (value < 0) {
    value = 0; // the display cannot show negative numbers
  }

  int scaledTenths = (int)(value * 10.0f + 0.5f); // round to the nearest 0.1

  int hundreds = (scaledTenths / 1000) % 10;
  int tens     = (scaledTenths / 100) % 10;
  int units    = (scaledTenths / 10) % 10;
  int tenths   = scaledTenths % 10;

  suppressLeadingZeros(hundreds, tens, units, tenths);

  displayDigits(hundreds, tens, units, tenths);
}

// Simple animation used to show a "turning" direction indicator, sweeping
// segment g across all 4 digit positions.
void showTurnAnimation(uint8_t repetitions, bool toTheRight) {
  const uint8_t order[4] = { 0, 1, 2, 3 };

  for (uint8_t rep = 0; rep < repetitions; rep++) {
    digitalWrite(PIN_SEG_ADDR_A, SEGMENT_ADDRESS[6][0]); // segment 'g'
    digitalWrite(PIN_SEG_ADDR_B, SEGMENT_ADDRESS[6][1]);
    digitalWrite(PIN_SEG_ADDR_C, SEGMENT_ADDRESS[6][2]);

    for (uint8_t step = 0; step < 4; step++) {
      uint8_t pos = toTheRight ? order[step] : order[3 - step];
      digitalWrite(DIGIT_ENABLE_PIN[pos], HIGH);
      delay(step == 0 ? 400 : 100);
    }

    for (uint8_t pos = 0; pos < 4; pos++) {
      digitalWrite(DIGIT_ENABLE_PIN[pos], LOW);
    }
  }
}

// Lights every segment/digit/dot in sequence once, used as a power-on
// self-test for the display.
void runDisplaySelfTest() {
  const uint8_t stepDelayMs = 150;

  for (uint8_t pos = 0; pos < 4; pos++) {
    digitalWrite(DIGIT_ENABLE_PIN[pos], HIGH);
  }

  for (uint8_t segment = 0; segment < 7; segment++) {
    digitalWrite(PIN_SEG_ADDR_A, SEGMENT_ADDRESS[segment][0]);
    digitalWrite(PIN_SEG_ADDR_B, SEGMENT_ADDRESS[segment][1]);
    digitalWrite(PIN_SEG_ADDR_C, SEGMENT_ADDRESS[segment][2]);
    delay(stepDelayMs);
  }

  digitalWrite(PIN_SEG_ADDR_A, DECIMAL_POINT_ADDRESS[0]);
  digitalWrite(PIN_SEG_ADDR_B, DECIMAL_POINT_ADDRESS[1]);
  digitalWrite(PIN_SEG_ADDR_C, DECIMAL_POINT_ADDRESS[2]);
  delay(stepDelayMs + 800);

  for (uint8_t pos = 0; pos < 4; pos++) {
    digitalWrite(DIGIT_ENABLE_PIN[pos], LOW);
  }
}

// ---------------------------------------------------------------------
// Motor / servo driver
// ---------------------------------------------------------------------

void setServoAngle(uint8_t angle) {
  trackerServo.write(angle);
  delay(15); // give the servo time to reach the position
}

void setMotorLeft(uint8_t speed, bool enabled) {
  digitalWrite(PIN_MOTOR_LEFT, enabled ? HIGH : LOW);
  analogWrite(PIN_MOTOR_SPEED, speed);
}

void setMotorRight(uint8_t speed, bool enabled) {
  digitalWrite(PIN_MOTOR_RIGHT, enabled ? HIGH : LOW);
  analogWrite(PIN_MOTOR_SPEED, speed);
}

void turnRight() {
  Serial.println("turning right");
  showTurnAnimation(3, true);

  setServoAngle(SERVO_CENTER);
  delay(SERVO_MOVE_DELAY_MS);
  setMotorRight(MOTOR_SPEED_FULL, true);
  delay(MOTOR_STEP_DELAY_MS);
  setMotorRight(MOTOR_SPEED_OFF, false);

  setServoAngle(SERVO_RIGHT);
  delay(SERVO_MOVE_DELAY_MS);
  setMotorLeft(MOTOR_SPEED_FULL, true);
  delay(MOTOR_STEP_DELAY_MS);

  setServoAngle(SERVO_CENTER);
  delay(SERVO_MOVE_DELAY_MS);
}

void turnLeft() {
  Serial.println("turning left");
  showTurnAnimation(3, false);

  setServoAngle(SERVO_CENTER);
  delay(SERVO_MOVE_DELAY_MS);
  setMotorRight(MOTOR_SPEED_FULL, true);
  delay(MOTOR_STEP_DELAY_MS);
  setMotorRight(MOTOR_SPEED_OFF, false);

  setServoAngle(SERVO_LEFT);
  delay(SERVO_MOVE_DELAY_MS);
  setMotorLeft(MOTOR_SPEED_FULL, true);
  delay(MOTOR_STEP_DELAY_MS);

  setServoAngle(SERVO_CENTER);
  delay(SERVO_MOVE_DELAY_MS);
}

// Sweeps the IR sensor left and right, compares the two readings and
// turns towards the direction with more free space (higher reading).
void lookAround() {
  delay(1000);

  Serial.println("backwards");
  setMotorRight(MOTOR_SPEED_FULL, true);
  delay(MOTOR_STEP_DELAY_MS);
  setMotorRight(MOTOR_SPEED_OFF, false);

  Serial.println("looking right");
  setServoAngle(SERVO_RIGHT);
  delay(SERVO_MOVE_DELAY_MS);
  float readingRight = analogRead(PIN_IR_SENSOR);
  Serial.println(readingRight);

  Serial.println("looking left");
  setServoAngle(SERVO_LEFT);
  delay(SERVO_MOVE_DELAY_MS);
  float readingLeft = analogRead(PIN_IR_SENSOR);
  Serial.println(readingLeft);

  if (readingRight > readingLeft) {
    turnRight();
  } else if (readingRight < readingLeft) {
    turnLeft();
  }
}

// ---------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------

void setup() {
  Serial.begin(9600);

  trackerServo.attach(PIN_SERVO);

  pinMode(PIN_MOTOR_LEFT, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT, OUTPUT);
  pinMode(PIN_MOTOR_SPEED, OUTPUT);

  pinMode(PIN_SEG_ADDR_A, OUTPUT);
  pinMode(PIN_SEG_ADDR_B, OUTPUT);
  pinMode(PIN_SEG_ADDR_C, OUTPUT);
  pinMode(PIN_DIGIT_1, OUTPUT);
  pinMode(PIN_DIGIT_2, OUTPUT);
  pinMode(PIN_DIGIT_3, OUTPUT);
  pinMode(PIN_DIGIT_4, OUTPUT);

  runDisplaySelfTest();
}

void loop() {
  int reading = analogRead(PIN_IR_SENSOR);
  float volts = reading / ADC_TO_VOLTS_DIVISOR;
  Serial.println(volts);

  for (uint8_t pass = 0; pass < DISPLAY_REFRESH_PASSES; pass++) {
    updateDisplay(volts);
  }

  if (volts >= VOLTAGE_HIGH_THRESHOLD) {
    Serial.println("POWER");
    setMotorLeft(MOTOR_SPEED_FULL, true);
  } else if (volts >= VOLTAGE_LOW_THRESHOLD) {
    Serial.println("I'm slowing down");
    setMotorLeft(MOTOR_SPEED_SLOW, true);
  } else {
    Serial.println("I'm looking around");
    setMotorLeft(MOTOR_SPEED_OFF, false);
    lookAround();
  }
}
