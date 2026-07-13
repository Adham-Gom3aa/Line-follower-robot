// ======================
//   L298N Motor Pins
// ======================
#define IN1 9
#define IN2 8
#define IN3 7
#define IN4 12
#define ENA 11
#define ENB 10

// ======================
//   IR Sensors Pins
// ======================
#define S1 A1   // Far Left
#define S2 A0   // Mid Left
#define S3 A2   // Mid Right
#define S4 A3   // Far Right

// ======================
//   Constants
// ======================
#define BASE_SPEED 80
#define MAX_PWM 80
#define DEADBAND 5  // small errors ignored

// ======================
//   PID Gains
// ======================
float Kp = 200;
float Ki = 5;
float Kd = 15;

// ======================
//   PID Variables
// ======================
float position = 0;      // current position
float goal = 0;          // desired position
float error = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;
float pid_output = 0;

unsigned long currentTime = 0;

void setup() {
  // Motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Sensor pins
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);

  // Serial for CSV logging
  Serial.begin(19200);
  Serial.println("Time,Position,Goal,Error,PID_Output");
}

void loop() {
  currentTime = millis();

  // ---------- READ SENSORS ----------
  int r1 = analogRead(S1);
  int r2 = analogRead(S2);
  int r3 = analogRead(S3);
  int r4 = analogRead(S4);

  // ---------- COMPUTE POSITION ----------
  // Use mostly middle sensors to reduce wobble
  position = r3 - r2;

  // ---------- APPLY DEAD-BAND ----------
  if (abs(position) < DEADBAND) position = 0;

  // ---------- PID CALCULATION ----------
  error = goal - position;
  integral += error;
  integral = constrain(integral, -100, 100); // prevent windup
  derivative = error - lastError;
  lastError = error;

  pid_output = Kp*error + Ki*integral + Kd*derivative;
  pid_output = constrain(pid_output, -MAX_PWM, MAX_PWM);

  // ---------- MOTOR CONTROL ----------
  int leftSpeed  = BASE_SPEED + pid_output;
  int rightSpeed = BASE_SPEED - pid_output;

  leftSpeed  = constrain(leftSpeed, 0, MAX_PWM);
  rightSpeed = constrain(rightSpeed, 0, MAX_PWM);

  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);

  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);

  // ---------- LOG DATA ----------
  Serial.print(currentTime); Serial.print(",");
  Serial.print(position); Serial.print(",");
  Serial.print(goal); Serial.print(",");
  Serial.print(error); Serial.print(",");
  Serial.println(pid_output);

  delay(20); // ~50 Hz loop
}