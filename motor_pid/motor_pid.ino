// PID POSITION control: GM25-370CA-12560-47 (12V, 47:1, Hall encoder) + L298 + Uno.
// Serial Monitor @115200, line ending "Newline".
// Zero reference = shaft position at power-on/reset. Every cycle first homes the shaft back
// to 0 deg, then asks for Kp, Ki, Kd and a target angle, moves the output shaft there while
// streaming Set/Pos/P/I/D/PWM for the Serial Plotter, stops once the angle settles and prints a
// step-response report. Then it homes again and asks for the next move. Send x to abort a move.

// ENA jumper cap stays ON (always enabled); PWM goes on IN1 (forward) or IN2 (reverse).
const byte ENC_A = 2, ENC_B = 3, IN1 = 6, IN2 = 5;
const int DIR = -1;           // flipped: +PWM was counting down (runaway). If it runs away again, set 1

const float ENC_PPR = 12;     // measured: 3 ppr read ~275 rpm on a 75 rpm motor -> 12 ppr
const float GEAR    = 47;
const float CPR     = ENC_PPR * 4 * GEAR;           // quadrature, every edge of A and B -> 2256 counts/rev
const unsigned long TS_MS = 20;                     // sample time
const float TS = TS_MS / 1000.0;

const float TOL_DEG = 2;                 // settled = within +-2 deg of target (~12 counts, covers gearbox backlash)
const unsigned long HOLD_MS = 1000;      // ...and stays inside the band this long
const unsigned long TIMEOUT_MS = 30000;  // give up if it never settles
const float MIN_PWM = 40;                // deadband: smallest PWM that turns the shaft (20.6 and 28 didn't). Tune
const float HOME_KP = 3, HOME_KI = 2, HOME_KD = 0.05;   // fixed safe gains for homing, tune if homing hunts

volatile long pos;                       // 0 = zero reference, set at power-on

void onA() { pos += (digitalRead(ENC_A) == digitalRead(ENC_B)) ? 1 : -1; }
void onB() { pos += (digitalRead(ENC_A) != digitalRead(ENC_B)) ? 1 : -1; }

void drive(int pwm) {
  pwm *= DIR;
  if (pwm >= 0) { digitalWrite(IN2, LOW); analogWrite(IN1, pwm); }
  else          { digitalWrite(IN1, LOW); analogWrite(IN2, -pwm); }
}

float deg() {
  noInterrupts();
  long p = pos;
  interrupts();
  return p * 360.0 / CPR;
}

float ask(const __FlashStringHelper *what) {
  Serial.println(what);
  String s;
  while (!s.length()) {                 // skip blank lines / stray CR from the line-ending setting
    while (!Serial.available());
    s = Serial.readStringUntil('\n');
    s.trim();
  }
  Serial.print(F("  got ")); Serial.println(s.toFloat());
  return s.toFloat();
}

void setup() {
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A), onA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), onB, CHANGE);
  Serial.begin(115200);
  drive(0);
}

void loop() {
  Serial.print(F("\n--- Homing to zero reference from ")); Serial.print(deg()); Serial.println(F(" deg ---"));
  if (run(0, HOME_KP, HOME_KI, HOME_KD, false)) Serial.println(F("At zero reference."));
  else Serial.println(F("WARNING: homing did not finish, shaft is NOT at zero."));

  Serial.println(F("\n--- New move ---"));
  float kp = ask(F("Enter Kp (e.g. 3)"));
  float ki = ask(F("Enter Ki (e.g. 2)"));
  float kd = ask(F("Enter Kd (e.g. 0.05)"));
  float target = ask(F("Enter target angle in deg from zero reference (e.g. 180, -90, 720)"));
  Serial.print(F("Moving Kp=")); Serial.print(kp);
  Serial.print(F(" Ki="));       Serial.print(ki);
  Serial.print(F(" Kd="));       Serial.print(kd);
  Serial.print(F(" Target="));   Serial.println(target);
  run(target, kp, ki, kd, true);
}

void plot(float set, float pos, float pT, float iT, float dT, float pwm) {
  Serial.print("Set:");  Serial.print(set);
  Serial.print(",Pos:"); Serial.print(pos);
  Serial.print(",P:");   Serial.print(pT);
  Serial.print(",I:");   Serial.print(iT);
  Serial.print(",D:");   Serial.print(dT);
  Serial.print(",PWM:"); Serial.println(pwm);
}

// Drives to target (deg from zero reference) until settled. Returns false on abort/timeout.
// report=false (homing): no plot, no report.
bool run(float target, float kp, float ki, float kd, bool report) {
  float p0 = deg();                                        // start position, ~0 after homing
  if (report) for (byte i = 0; i < 10; i++) plot(p0, p0, 0, 0, 0, 0);   // flat baseline so the step edge shows
  float integ = 0, lastPos = p0, peak = 0, sumPwm = 0, sumPos = 0, stallPos = p0, kick = MIN_PWM;
  float sgn = target < p0 ? -1 : 1, span = fabs(target - p0);   // metrics measured along the move direction
  long t10 = -1, t90 = -1, tEnter = -1;
  int nIn = 0;
  unsigned long t0 = millis(), next = t0 + TS_MS;

  while (true) {
    if (Serial.available() && Serial.read() == 'x') {
      drive(0);
      Serial.println(F("Aborted."));
      return false;
    }
    if ((long)(millis() - next) < 0) continue;
    long t = next - t0;
    next += TS_MS;

    float p = deg();
    float err = target - p;
    float pT = kp * err, iT = ki * integ, dT = -kd * (p - lastPos) / TS;   // derivative on measurement
    float out = pT + iT + dT;
    if (fabs(out) < 255 || out * err < 0) integ += err * TS;          // anti-windup: no I build-up while saturated
    out = constrain(out, -255, 255);
    kick = (p != lastPos) ? MIN_PWM : min(kick + 2, 120.0f);   // stuck at rest -> ramp boost until it breaks free
    if (fabs(err) > TOL_DEG && fabs(out) < kick) out = out < 0 ? -kick : kick;   // deadband comp
    lastPos = p;
    drive((int)out);

    // stop plotting 0.2 s into the settle hold, so the step response stays in the plotter window
    if (report && (tEnter < 0 || t - tEnter <= 200)) plot(target, p, pT, iT, dT, out);

    if (t % 1000 == 0) {                 // once a second: hard drive but no motion -> stalled / not wired
      if (fabs(out) > 150 && fabs(p - stallPos) < 1) {
        drive(0);
        Serial.println(F("\nSTALL - PWM high but shaft not moving. Motor stopped. Check IN1/IN2 wiring and 12 V."));
        return false;
      }
      stallPos = p;
    }
    if (fabs(err) > span + 360) {        // moved a full turn the wrong way -> direction mismatch
      drive(0);
      Serial.println(F("\nRUNAWAY - motor stopped. Flip DIR (or swap encoder A/B) and reset."));
      return false;
    }

    // step-response metrics
    float prog = (p - p0) * sgn;
    peak = max(peak, prog);
    if (t10 < 0 && prog >= 0.1 * span) t10 = t;
    if (t90 < 0 && prog >= 0.9 * span) t90 = t;
    if (fabs(err) <= TOL_DEG) {
      if (tEnter < 0) { tEnter = t; sumPos = sumPwm = 0; nIn = 0; }
      sumPos += p; sumPwm += out; nIn++;
      if (t - tEnter >= (long)HOLD_MS) break;
    } else {
      tEnter = -1;
    }
    if (t >= (long)TIMEOUT_MS) {
      drive(0);
      Serial.println(F("\nDid NOT settle within 30 s - motor stopped. Try other gains."));
      Serial.print(F("Position now (deg) = ")); Serial.println(deg());
      return false;
    }
  }

  drive(0);
  if (!report) return true;
  float avg = sumPos / nIn;
  Serial.println(F("\n===== Settled - motor stopped ====="));
  Serial.print(F("Kp = "));                   Serial.println(kp);
  Serial.print(F("Ki = "));                   Serial.println(ki);
  Serial.print(F("Kd = "));                   Serial.println(kd);
  Serial.print(F("Target (deg) = "));         Serial.println(target);
  Serial.print(F("Rise time 10-90% (s) = ")); Serial.println(t90 >= 0 && t10 >= 0 ? (t90 - t10) / 1000.0 : -1);
  Serial.print(F("Settling time (s) = "));    Serial.println(tEnter / 1000.0);
  Serial.print(F("Peak (deg) = "));           Serial.println(p0 + peak * sgn);
  Serial.print(F("Overshoot (%) = "));        Serial.println(span > 0 ? max(0.0f, (peak - span) / span * 100) : 0);
  Serial.print(F("Final avg position (deg) = ")); Serial.println(avg);
  Serial.print(F("Steady-state error (deg) = ")); Serial.println(target - avg);
  Serial.print(F("Final avg PWM (-255..255) = ")); Serial.println(sumPwm / nIn);
  Serial.println(F("==================================="));
  return true;
}
