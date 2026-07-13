// ======================
//   L298N Motor Pins
// ======================
#define IN1 9    // Left motor
#define IN2 8
#define IN3 7    // Right motor
#define IN4 12

#define ENA 11   // Left motor speed
#define ENB 10   // Right motor speed

// ======================
//   IR Sensors Pins
// ======================
#define S1 A1   // Far Left
#define S2 A0   // Mid Left
#define S3 A2   // Mid Right
#define S4 A3   // Far Right

// ======================
//   Motor Speed
// ======================
#define MOTOR_SPEED 80  // 0–255

// ======================
//   IR Threshold
// ======================
#define THRESHOLD 300

// ======================
//   New Variables for Logging
// ======================
unsigned long currentTimeMs = 0;  
int steering_cmd = 0;       // U input for MATLAB
int error = 0;              // Y output for MATLAB

void setup() {
  // Motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // IR sensor pins
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);

  // Set motor speed
  analogWrite(ENA, MOTOR_SPEED);
  analogWrite(ENB, MOTOR_SPEED);

  // Start Serial
  Serial.begin(19200);

  // Print CSV header for MATLAB
  Serial.println("Time,SteeringCmd,Error");
}

void loop() {
  currentTimeMs = millis();  // Time in milliseconds

  // ------ Read analog sensors ------
  int r1 = analogRead(S1);
  int r2 = analogRead(S2);
  int r3 = analogRead(S3);
  int r4 = analogRead(S4);

  int leftMotorSpeed = 0;
  int rightMotorSpeed = 0;

  // --------- LINE FOLLOWING LOGIC ---------
  if (r4 > THRESHOLD) {                        // Left turn
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    leftMotorSpeed = MOTOR_SPEED / 2;
    rightMotorSpeed = MOTOR_SPEED;
    steering_cmd = -MOTOR_SPEED/2;
    error = r4 - r1;
  }
  else if (r1 > THRESHOLD) {                   // Right turn
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
    leftMotorSpeed = MOTOR_SPEED;
    rightMotorSpeed = MOTOR_SPEED / 2;
    steering_cmd = MOTOR_SPEED/2;
    error = r4 - r1;
  }
  else if (r2 > THRESHOLD || r3 > THRESHOLD) { // Forward
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    leftMotorSpeed = MOTOR_SPEED;
    rightMotorSpeed = MOTOR_SPEED;
    steering_cmd = 0;
    error = r4 - r1;
  }

  // --------- Log CSV row ---------
  Serial.print(currentTimeMs); Serial.print(",");  // Time in ms
  Serial.print(steering_cmd); Serial.print(",");  // U input
  Serial.println(error);                           // Y output
}