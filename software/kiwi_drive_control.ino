*
 * =====================================================================================
 *  Simple Case-Based Kiwi Drive Controller
 * =====================================================================================
 *  No kinematics formula, no PID — just fixed wheel patterns per command, exactly as
 *  you'd wire it up by hand. Each wheel is either OFF, FORWARD, or REVERSE, driven
 *  full voltage (no PWM speed control).
 *
 *  Wheel layout:
 *      FRONT wheel
 *      LEFT  wheel  (rear-left)
 *      RIGHT wheel  (rear-right)
 *
 *  Serial Monitor @ 115200 baud, send a single character:
 *      w = forward        -> LEFT + RIGHT wheels drive forward, FRONT off
 *      s = backward        -> LEFT + RIGHT wheels reversed, FRONT off
 *      a = move left        -> FRONT + LEFT wheels forward, RIGHT off
 *      d = move right       -> FRONT + RIGHT wheels forward, LEFT off
 *      e = rotate clockwise      -> FRONT forward, LEFT forward, RIGHT reverse
 *      q = rotate anti-clockwise -> FRONT reverse, LEFT reverse, RIGHT forward
 *      x = stop            -> all wheels off
 * =====================================================================================
 */

#include <Arduino.h>

// ---- Pins: TB6612FNG #1 -> FRONT(A)/RIGHT(B), TB6612FNG #2 -> LEFT(A) ----
struct MotorPins { int in1; int in2; int pwm; int stby; };

const MotorPins FRONT = { .in1 = 26, .in2 = 27, .pwm = 25, .stby = 33 };
const MotorPins RIGHT = { .in1 = 12, .in2 = 13, .pwm = 14, .stby = 33 };
const MotorPins LEFT  = { .in1 = 15, .in2 = 4,  .pwm = 32, .stby = 2  };

void motorForward(MotorPins m) {
  digitalWrite(m.in1, HIGH);
  digitalWrite(m.in2, LOW);
  digitalWrite(m.pwm, HIGH);   // full voltage, no speed control
}

void motorReverse(MotorPins m) {
  digitalWrite(m.in1, LOW);
  digitalWrite(m.in2, HIGH);
  digitalWrite(m.pwm, HIGH);
}

void motorOff(MotorPins m) {
  digitalWrite(m.in1, LOW);
  digitalWrite(m.in2, LOW);
  digitalWrite(m.pwm, LOW);
}

void setupPins(MotorPins m) {
  pinMode(m.in1, OUTPUT);
  pinMode(m.in2, OUTPUT);
  pinMode(m.pwm, OUTPUT);
  pinMode(m.stby, OUTPUT);
  digitalWrite(m.stby, HIGH);   // take TB6612 out of standby
  motorOff(m);
}

void stopAll() {
  motorOff(FRONT);
  motorOff(LEFT);
  motorOff(RIGHT);
}

void handleCommand(char c) {
  switch (c) {
    case 'w':   // forward
      motorOff(FRONT);
      motorForward(LEFT);
      motorForward(RIGHT);
      break;

    case 's':   // backward
      motorOff(FRONT);
      motorReverse(LEFT);
      motorReverse(RIGHT);
      break;

    case 'a':   // move left
      motorForward(FRONT);
      motorForward(LEFT);
      motorOff(RIGHT);
      break;

    case 'd':   // move right
      motorForward(FRONT);
      motorForward(RIGHT);
      motorOff(LEFT);
      break;

    case 'e':   // rotate clockwise
      motorForward(FRONT);
      motorForward(LEFT);
      motorReverse(RIGHT);
      break;

    case 'q':   // rotate anti-clockwise
      motorReverse(FRONT);
      motorReverse(LEFT);
      motorForward(RIGHT);
      break;

    case 'x':   // stop
      stopAll();
      break;

    default:
      return;   // ignore newline / unknown chars
  }
}

void setup() {
  Serial.begin(115200);
  setupPins(FRONT);
  setupPins(LEFT);
  setupPins(RIGHT);
  Serial.println("Ready. w=fwd s=back a=left d=right e=rotateCW q=rotateCCW x=stop");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    handleCommand(c);
  }
}

