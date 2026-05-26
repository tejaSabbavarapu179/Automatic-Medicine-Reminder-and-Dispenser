#include <Wire.h>
#include <RTClib.h>
#include <EEPROM.h>

RTC_DS3231 rtc;

// Stepper pins
#define IN1 9
#define IN2 10
#define IN3 11
#define IN4 12

#define BUZZER_PIN 8

// LEDs for compartments 1..4
int ledPins[4] = {7, 6, 5, 13};

// Stepper parameters
const int stepsPerRevolution = 4096;      // set to 4096 (confirmed by your full-rev test)
const int stepsPerCompartment = stepsPerRevolution / 4;
long currentStepPosition = 0;

// Half-step sequence (8-step) for 28BYJ-48
const uint8_t halfSeq[8][4] = {
  {1,0,0,0},
  {1,1,0,0},
  {0,1,0,0},
  {0,1,1,0},
  {0,0,1,0},
  {0,0,1,1},
  {0,0,0,1},
  {1,0,0,1}
};
int halfIndex = 0;

// Alarm structure
struct AlarmTime {
  uint8_t hour;
  uint8_t minute;
};

AlarmTime alarms[4];
int lastTriggeredDay[4];
const int EEPROM_ADDR = 0;     // alarms stored at addr 0..7
const int EEPROM_OFF_ADDR = 8; // offsets stored at addr 8..15 (2 bytes each)

// Calibration offsets (in steps) for each compartment
long compOffsets[4] = {0,0,0,0};

// Smoothing parameters (tweakable)
const int STEP_DELAY_MAX = 18;   // ms (slow at start/stop) — larger = smoother
const int STEP_DELAY_MIN = 9;    // ms (fast cruising) — larger = more torque, slower
const int RAMP_STEPS = 300;      // ramp length (in steps) for accel/decel

// Alarm state
bool alarmActive = false;
int activeAlarmIndex = -1;
unsigned long alarmStartTime = 0;

// RTC presence flag and print timer
bool rtcPresent = false;
unsigned long lastRTCprint = 0;

String inputBuffer = "";

// -------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  // removed blocking while(!Serial) so device will run headless

  Wire.begin();
  rtcPresent = rtc.begin();
  if (! rtcPresent) {
    Serial.println("Warning: RTC not found. Alarm and time features disabled.");
  } else {
    Serial.println("RTC detected.");
  }

  // Output pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  for (int i = 0; i < 4; i++) {
    pinMode(ledPins[i], OUTPUT);
    digitalWrite(ledPins[i], LOW);
  }

  // Default alarms (example times)
  alarms[0] = {22, 42};
  alarms[1] = {22, 43};
  alarms[2] = {22, 44};
  alarms[3] = {22, 45};

  // load saved alarms + offsets (if present)
  loadAlarmsFromEEPROM();
  for (int i = 0; i < 4; i++) lastTriggeredDay[i] = -1;

  // ===========================
  // Minimal reliability edit:
  // energize initial coil state so motor is held at startup
  halfIndex = 0;
  digitalWrite(IN1, halfSeq[halfIndex][0]);
  digitalWrite(IN2, halfSeq[halfIndex][1]);
  digitalWrite(IN3, halfSeq[halfIndex][2]);
  digitalWrite(IN4, halfSeq[halfIndex][3]);
  // ===========================

  moveToStepPosition(0);

  // show current compartment LED (initially comp1)
  for (int i = 0; i < 4; i++) digitalWrite(ledPins[i], LOW);
  digitalWrite(ledPins[0], HIGH);

  Serial.println("System Ready (Smooth movement + per-compartment calibration)");
  printSchedule();
  printOffsets();
}

// -------------------------------------------------------------------
void loop() {

  // Print RTC time every second (only if RTC present)
  if (rtcPresent && (millis() - lastRTCprint >= 1000)) {
    lastRTCprint = millis();
    DateTime now = rtc.now();
    Serial.print("RTC Time: ");
    Serial.print(now.year()); Serial.print("/");
    Serial.print(twoDigits(now.month())); Serial.print("/");
    Serial.print(twoDigits(now.day())); Serial.print("  ");
    Serial.print(twoDigits(now.hour())); Serial.print(":");
    Serial.print(twoDigits(now.minute())); Serial.print(":");
    Serial.println(twoDigits(now.second()));
  }

  processSerialInputs();

  // If RTC present, perform alarm checks; otherwise skip
  if (rtcPresent) {
    DateTime now = rtc.now();
    int today = now.day();

    // Check alarms (hour & minute)
    for (int i = 0; i < 4; i++) {
      if (now.hour() == alarms[i].hour && now.minute() == alarms[i].minute) {
        if (lastTriggeredDay[i] != today) {
          triggerAlarm(i);
          lastTriggeredDay[i] = today;
        }
      }
    }
  }

  delay(50);
}

// ---------------- Serial Processing ----------------
void processSerialInputs() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        handleCommand(inputBuffer);
        inputBuffer = "";
      }
    } else inputBuffer += c;
  }
}

void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  String up = cmd;
  up.toUpperCase();

  if (up == "HELP") { printHelp(); return; }
  if (up == "SHOW") { printSchedule(); return; }
  if (up == "SAVE") { saveAlarmsToEEPROM(); Serial.println("Saved."); return; }
  if (up == "LOAD") { loadAlarmsFromEEPROM(); printSchedule(); printOffsets(); return; }

  if (up == "OFFSETS") { printOffsets(); return; }

  // CAL command: CAL n offset  (n = 1..4, offset = signed steps)
  if (up.startsWith("CAL ")) {
    int idx; long off;
    if (sscanf(cmd.c_str(), "CAL %d %ld", &idx, &off) == 2) {
      if (idx >= 1 && idx <= 4) {
        // ===========================
        // Minimal reliability edit:
        // clamp offset to signed int16 range before saving to EEPROM
        if (off < -32767L) off = -32767L;
        if (off >  32767L) off =  32767L;
        compOffsets[idx-1] = off;
        // ===========================
        saveAlarmsToEEPROM();
        Serial.print("Saved offset for comp ");
        Serial.print(idx);
        Serial.print(" = ");
        Serial.println(off);
      } else Serial.println("CAL: use index 1..4");
    } else Serial.println("CAL format: CAL n offset");
    return;
  }

  if (up == "SETPC") {
    // set RTC to compile-time only if RTC present
    if (rtcPresent) {
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      Serial.println("RTC set to compile time.");
    } else {
      Serial.println("RTC not present — cannot set compile time.");
    }
    return;
  }

  if (up.startsWith("SET ")) {
    int Y, M, D, h, m, s;
    if (sscanf(cmd.c_str(), "SET %d %d %d %d %d %d", &Y, &M, &D, &h, &m, &s) == 6) {
      if (rtcPresent) {
        rtc.adjust(DateTime(Y, M, D, h, m, s));
        Serial.println("RTC updated.");
      } else {
        Serial.println("RTC not present — cannot set time.");
      }
    } else Serial.println("Format: SET YYYY MM DD hh mm ss");
    return;
  }

  if (up.startsWith("GOTO ")) {
    int n = up.substring(5).toInt();
    if (n >= 1 && n <= 4) {
      rotateToCompartmentForward(n - 1);
      // show LED for that compartment
      for (int i = 0; i < 4; i++) digitalWrite(ledPins[i], LOW);
      digitalWrite(ledPins[n-1], HIGH);
      Serial.print("Moved to compartment ");
      Serial.println(n);
    } else Serial.println("GOTO: use 1..4");
    return;
  }

  if (up.startsWith("S ")) {
    int index, hh, mm;
    if (sscanf(cmd.c_str(), "S %d %d %d", &index, &hh, &mm) == 3) {
      if (index >= 1 && index <= 4 && hh >= 0 && hh <= 23 && mm >= 0 && mm <= 59) {
        alarms[index - 1].hour = hh;
        alarms[index - 1].minute = mm;
        Serial.print("Alarm "); Serial.print(index); Serial.print(" set to ");
        Serial.print(twoDigits(hh)); Serial.print(":"); Serial.println(twoDigits(mm));
      } else Serial.println("Invalid values for S command.");
    } else Serial.println("Invalid S format. Use: S n HH MM");
    return;
  }

  if (up == "RESET") {
    rotateToCompartmentForward(0);
    for (int i = 0; i < 4; i++) digitalWrite(ledPins[i], LOW);
    digitalWrite(ledPins[0], HIGH);
    Serial.println("Reset to compartment 1.");
    return;
  }

  Serial.println("Unknown command.");
}

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  HELP");
  Serial.println("  SHOW");
  Serial.println("  S n HH MM     (set alarm n)");
  Serial.println("  SAVE");
  Serial.println("  LOAD");
  Serial.println("  OFFSETS        (print calibration offsets)");
  Serial.println("  CAL n offset   (set offset for compartment n, save to EEPROM)");
  Serial.println("  GOTO n");
  Serial.println("  RESET");
  Serial.println("  SET YYYY MM DD hh mm ss");
  Serial.println("  SETPC");
}

void printSchedule() {
  Serial.println("Current Schedule:");
  for (int i = 0; i < 4; i++) {
    Serial.print(" Compartment "); Serial.print(i + 1); Serial.print(" -> ");
    Serial.print(twoDigits(alarms[i].hour)); Serial.print(":"); Serial.println(twoDigits(alarms[i].minute));
  }
}

void printOffsets() {
  Serial.println("Compartment offsets (steps):");
  for (int i = 0; i < 4; i++) {
    Serial.print(" C"); Serial.print(i+1); Serial.print(" -> "); Serial.println(compOffsets[i]);
  }
}

// ---------------- Alarm handling ----------------
void triggerAlarm(int idx) {
  // rotate forward to the requested compartment (with calibration offset)
  rotateToCompartmentForward(idx);

  alarmActive = true;
  activeAlarmIndex = idx;
  alarmStartTime = millis();

  // LED for the current compartment ON, others OFF
  for (int i = 0; i < 4; i++) digitalWrite(ledPins[i], LOW);
  digitalWrite(ledPins[idx], HIGH);

  // small settle before buzzer
  delay(150);

  // SIMPLE BUZZER PATTERN: 1s ON, 1s OFF (single cycle)
  tone(BUZZER_PIN, 1000);   // ON
  delay(1000);              // 1 second
  noTone(BUZZER_PIN);       // OFF
  delay(1000);              // 1 second

  Serial.print("ALARM for Compartment ");
  Serial.println(idx + 1);

  // after finishing compartment 4 (index 3), rotate forward back to compartment 1 (index 0)
  if (idx == 3) {
    delay(200); // small pause
    rotateToCompartmentForward(0);
    // show LED for comp1
    for (int i = 0; i < 4; i++) digitalWrite(ledPins[i], LOW);
    digitalWrite(ledPins[0], HIGH);
    Serial.println("Completed cycle: returned to Compartment 1.");
  }
}

// Acknowledge (keeps current behavior - no auto-return except after comp4 above)
void acknowledgeAlarm() {
  alarmActive = false;
  activeAlarmIndex = -1;
  noTone(BUZZER_PIN);
  Serial.println("Alarm acknowledged.");
}

// ---------------- Stepper low-level (HALF-STEP) ----------------
void OneStepHalf(bool dir) {
  if (dir) halfIndex = (halfIndex + 1) % 8;
  else halfIndex = (halfIndex + 7) % 8; // -1 mod 8

  digitalWrite(IN1, halfSeq[halfIndex][0]);
  digitalWrite(IN2, halfSeq[halfIndex][1]);
  digitalWrite(IN3, halfSeq[halfIndex][2]);
  digitalWrite(IN4, halfSeq[halfIndex][3]);
}

// ---------------- SMOOTH STEP + CALIBRATION ----------------

// smooth stepping with linear accel/decel (uses OneStepHalf)
void stepN(long steps, int delayMsDummy = STEP_DELAY_MIN) {
  bool dir = steps > 0;
  long count = abs(steps);
  if (count == 0) return;

  // compute ramp length (don't exceed half the move)
  long ramp = RAMP_STEPS;
  if (ramp * 2 > count) ramp = count / 2;

  for (long i = 0; i < count; i++) {
    // compute delay for this step: ramp up, cruise, ramp down (linear)
    int d = STEP_DELAY_MIN;
    if (i < ramp) {
      // accelerating: from max -> min
      float t = (float)(ramp - i) / ramp; // 1..0
      d = (int)(STEP_DELAY_MIN + (STEP_DELAY_MAX - STEP_DELAY_MIN) * t);
    } else if (i >= count - ramp) {
      // decelerating
      float t = (float)(i - (count - ramp)) / ramp; // 0..1
      d = (int)(STEP_DELAY_MIN + (STEP_DELAY_MAX - STEP_DELAY_MIN) * t);
    } else {
      d = STEP_DELAY_MIN;
    }

    OneStepHalf(dir);
    delay(d);

    if (dir) {
      currentStepPosition++;
      if (currentStepPosition >= stepsPerRevolution) currentStepPosition -= stepsPerRevolution;
    } else {
      currentStepPosition--;
      if (currentStepPosition < 0) currentStepPosition += stepsPerRevolution;
    }
  }

  // Hold final coil state for a bit so gearbox can settle (and motor resists backlash)
  delay(150); // hold 100..500 ms as needed
}

// rotate always forward (positive direction) to index (0..3), with offset applied
void rotateToCompartmentForward(int index) {
  long target = (long)index * stepsPerCompartment;
  // apply calibration offset for this compartment
  target += compOffsets[index];

  long diff = target - currentStepPosition;
  diff %= stepsPerRevolution;
  if (diff < 0) diff += stepsPerRevolution;

  if (diff != 0) {
    stepN(diff); // move forward by diff steps (smooth)
    // small settle time to let gears seat (already have hold in stepN but extra safety)
    delay(200);

    // Optional: small seating micro-move to remove backlash (uncomment if needed)
    // stepN(3);    // 3 steps forward
    // stepN(-2);   // 2 steps back -> leaves 1 step forward to seat gears
    // delay(120);
  }
}

// moveToStepPosition uses forward logic (used at startup)
void moveToStepPosition(long pos) {
  pos %= stepsPerRevolution;
  long diff = pos - currentStepPosition;
  if (diff < 0) diff += stepsPerRevolution;
  stepN(diff);
}

// ---------------- EEPROM: save/load alarms + offsets ----------------
void saveAlarmsToEEPROM() {
  int addr = EEPROM_ADDR;
  for (int i = 0; i < 4; i++) {
    EEPROM.update(addr++, alarms[i].hour);
    EEPROM.update(addr++, alarms[i].minute);
  }
  // store offsets as 2 bytes each (signed int16)
  int offAddr = EEPROM_OFF_ADDR;
  for (int i = 0; i < 4; i++) {
    int16_t v = (int16_t)compOffsets[i];
    EEPROM.update(offAddr++, lowByte(v));
    EEPROM.update(offAddr++, highByte(v));
  }
}

void loadAlarmsFromEEPROM() {
  int addr = EEPROM_ADDR;
  bool empty = true;
  for (int i = 0; i < 8; i++) if (EEPROM.read(i) != 0xFF) empty = false;
  if (empty) return;
  for (int i = 0; i < 4; i++) {
    alarms[i].hour = EEPROM.read(addr++);
    alarms[i].minute = EEPROM.read(addr++);
  }
  // read offsets
  int offAddr = EEPROM_OFF_ADDR;
  for (int i = 0; i < 4; i++) {
    byte b0 = EEPROM.read(offAddr++);
    byte b1 = EEPROM.read(offAddr++);
    int16_t v = (int16_t)((b1 << 8) | b0);
    compOffsets[i] = (long)v;
  }
}

String twoDigits(int v) {
  char s[3];
  sprintf(s, "%02d", v);
  return String(s);
} 


