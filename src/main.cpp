#include <SBUS.h>
#define RX_PIN 3
#define TX_PIN -1
#define MOTOR1_IN1 32
#define MOTOR1_IN2 33
#define MOTOR1_EN 35
#define MOTOR2_IN1 14
#define MOTOR2_IN2 12
#define MOTOR2_EN 27
// Separate PWM channels for independent speed control
#define MOTOR1_CHANNEL 0
#define MOTOR2_CHANNEL 1
#define MOTOR_FREQ 5000
#define MOTOR_RES 8
#define MAX_SPEED 255
#define MAX_CHANNEL_VALUE 1800
#define MIN_CHANNEL_VALUE 200
#define DIRECTION_CONTROL_DEADZONE 50
#define MOVE_CONTROL_DEADZONE 100

#define SIGNAL_TIMEOUT 500 // ms before considering signal lost

unsigned long lastValidSignal = 0; // Timestamp of last valid signal

void TestMotor();
void PlotSerial();
void ControlMotors();
void ControlWheels(int16_t movingValue, int16_t directionValue);
void MoveBackword(int speed, bool moveLeft, bool moveRight);
void MoveForward(int speed, bool moveLeft, bool moveRight);
int NormalizeRawValue(int16_t raw);
void StopWheels();
bool isSignalLost();
void ShowChannelOutput();

bfs::SbusRx sbus(&Serial2, RX_PIN, TX_PIN, true);

void setup()
{
  Serial.begin(115200);
  delay(1000);

  sbus.Begin();
  // Configure motor direction pins as outputs
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR2_IN1, OUTPUT);
  pinMode(MOTOR2_IN2, OUTPUT);

  // Configure PWM for motor speed control - separate channels
  ledcSetup(MOTOR1_CHANNEL, MOTOR_FREQ, MOTOR_RES);
  ledcSetup(MOTOR2_CHANNEL, MOTOR_FREQ, MOTOR_RES);
  ledcAttachPin(MOTOR1_EN, MOTOR1_CHANNEL);
  ledcAttachPin(MOTOR2_EN, MOTOR2_CHANNEL);

  // Start with motors stopped
  StopWheels();
}

void loop()
{

  ControlMotors();

  // Uncomment for debugging
  // PlotSerial();
}

// Check if signal is lost
bool isSignalLost()
{
  if (sbus.Read())
  {
    bfs::SbusData data = sbus.data();

    // Update timestamp on valid signal
    lastValidSignal = millis();

    // Check failsafe flag from receiver
    if (data.failsafe)
    {
      Serial.println("Receiver failsafe active!");
      return true;
    }
  }

  // Check for signal timeout
  if (millis() - lastValidSignal > SIGNAL_TIMEOUT)
  {
    return true;
  }

  return false;
}

void TestMotor()
{
  // Test motor 1
  Serial.println("Testing Motor 1 - Forward");
  digitalWrite(MOTOR1_IN1, HIGH);
  digitalWrite(MOTOR1_IN2, LOW);
  ledcWrite(MOTOR1_CHANNEL, MAX_SPEED);
  delay(2000);

  // Brake (short pause)
  Serial.println("Motor 1 - Brake");
  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, LOW);
  delay(500);

  // Reverse
  Serial.println("Motor 1 - Reverse");
  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, HIGH);
  ledcWrite(MOTOR1_CHANNEL, MAX_SPEED);
  delay(2000);

  // Brake (short pause)
  Serial.println("Motor 1 - Brake");
  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, LOW);
  delay(500);

  // Test motor 2
  Serial.println("Testing Motor 2 - Forward");
  digitalWrite(MOTOR2_IN1, HIGH);
  digitalWrite(MOTOR2_IN2, LOW);
  ledcWrite(MOTOR2_CHANNEL, MAX_SPEED);
  delay(2000);

  // Brake (short pause)
  Serial.println("Motor 2 - Brake");
  digitalWrite(MOTOR2_IN1, LOW);
  digitalWrite(MOTOR2_IN2, LOW);
  delay(500);

  // Reverse
  Serial.println("Motor 2 - Reverse");
  digitalWrite(MOTOR2_IN1, LOW);
  digitalWrite(MOTOR2_IN2, HIGH);
  ledcWrite(MOTOR2_CHANNEL, MAX_SPEED);
  delay(2000);

  // Brake (short pause)
  Serial.println("Motor 2 - Brake");
  digitalWrite(MOTOR2_IN1, LOW);
  digitalWrite(MOTOR2_IN2, LOW);
  delay(500);
}

void PlotSerial()
{
  if (sbus.Read())
  {
    // Get the decoded SBUS data
    bfs::SbusData data = sbus.data();

    // Print values in a format suitable for Serial Plotter
    // All values on one line for best plotter visualization
    Serial.print("CH1:");
    Serial.print(data.ch[0]);
    Serial.print(" CH2:");
    Serial.print(data.ch[1]);
    Serial.print(" CH3:");
    Serial.print(data.ch[2]);
    Serial.print(" CH4:");
    Serial.print(data.ch[3]);
    Serial.print(" CH5:");
    Serial.print(data.ch[4]);
    Serial.print(" CH6:");
    Serial.print(data.ch[5]);
    Serial.print(" CH7:");
    Serial.print(data.ch[6]);
    Serial.print(" CH8:");
    Serial.println(data.ch[7]);
  }
  delay(50);
}

void ShowChannelOutput()
{
  if (sbus.Read())
  {
    bfs::SbusData data = sbus.data();

    // Clear console and move cursor to home position
    Serial.write("\033[2J\033[H"); // Clear screen and position cursor at top-left

    Serial.println("SBUS Channel Data:");
    Serial.println("-----------------");

    for (int i = 0; i < 8; i++)
    {
      int16_t rawValue = data.ch[i];

      Serial.print("Channel ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(rawValue);

      float normalized = (rawValue - 1500) / 500.0;
      Serial.print(", Normalized: ");
      Serial.println(normalized, 2);
    }
    Serial.print("Failsafe: ");
    Serial.print(data.failsafe ? "Active" : "Inactive");
    Serial.print(", Lost Frame: ");
    Serial.println(data.lost_frame ? "Yes" : "No");

    Serial.println("-----------------");
  }
  delay(100);
}

void ControlMotors()
{
  if (sbus.Read())
  {
    bfs::SbusData data = sbus.data();
    lastValidSignal = millis(); // Update timestamp for signal monitoring
    ControlWheels(data.ch[2], data.ch[0]);
  }
}

void ControlWheels(int16_t movingValue, int16_t directionValue)
{
  int normalized_move_value = NormalizeRawValue(movingValue);
  int normalized_direction_value = NormalizeRawValue(directionValue);
  bool moveRight = (normalized_direction_value > DIRECTION_CONTROL_DEADZONE);
  bool moveLeft = (normalized_direction_value < -DIRECTION_CONTROL_DEADZONE);

  // For debugging
  Serial.print(">Moving:");
  Serial.println(normalized_move_value);
  Serial.print(">Direction:");
  Serial.println(normalized_direction_value);
  Serial.print(">Left:");
  Serial.println((int)moveLeft);
  Serial.print(">Right:");
  Serial.println((int)moveRight);

  if (normalized_move_value > MOVE_CONTROL_DEADZONE)
    MoveForward(normalized_move_value, moveLeft, moveRight);
  else if (normalized_move_value < -MOVE_CONTROL_DEADZONE)
    MoveBackword(normalized_move_value, moveLeft, moveRight);
  else
  {
    StopWheels();
  }
}

int NormalizeRawValue(int16_t raw)
{
  // Constrain raw values first to prevent extreme outputs
  raw = constrain(raw, MIN_CHANNEL_VALUE, MAX_CHANNEL_VALUE);
  return map(raw, MIN_CHANNEL_VALUE, MAX_CHANNEL_VALUE, -MAX_SPEED, MAX_SPEED);
}

void MoveForward(int speed, bool moveLeft, bool moveRight)
{
  speed = constrain(speed, 0, MAX_SPEED);

  // Adjust speeds for turning - differential drive
  if (moveLeft)
  {
    digitalWrite(MOTOR1_IN1, HIGH);
    digitalWrite(MOTOR1_IN2, LOW);
    digitalWrite(MOTOR2_IN1, LOW);
    digitalWrite(MOTOR2_IN2, LOW);
    ledcWrite(MOTOR1_CHANNEL, 255);
    ledcWrite(MOTOR2_CHANNEL, 0);
  }
  else if (moveRight)
  {
    // Set directions for forward movement

    digitalWrite(MOTOR2_IN1, HIGH);
    digitalWrite(MOTOR2_IN2, LOW);
    digitalWrite(MOTOR1_IN1, LOW);
    digitalWrite(MOTOR1_IN2, LOW);
    ledcWrite(MOTOR2_CHANNEL, 255);
    ledcWrite(MOTOR1_CHANNEL, 0);
  }
  else
  {
    digitalWrite(MOTOR1_IN1, HIGH);
    digitalWrite(MOTOR1_IN2, LOW);
    digitalWrite(MOTOR2_IN1, HIGH);
    digitalWrite(MOTOR2_IN2, LOW);
    ledcWrite(MOTOR1_CHANNEL, 255);
    ledcWrite(MOTOR2_CHANNEL, 255);
  }

  // Apply speeds to each motor independently
}

void MoveBackword(int speed, bool moveLeft, bool moveRight)
{
  speed = abs(speed); // Ensure positive value
  speed = constrain(speed, 0, MAX_SPEED);

  // Calculate individual motor speeds for turning
  int leftSpeed = speed;
  int rightSpeed = speed;

  // Adjust speeds for turning - differential drive
  // Note: When going backward, reducing right motor makes it turn left and vice versa
  if (moveLeft)
  {
    rightSpeed = speed / 2; // Reduce right motor speed to turn left while going backward
  }
  else if (moveRight)
  {
    leftSpeed = speed / 2; // Reduce left motor speed to turn right while going backward
  }

  // Set directions for backward movement
  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, HIGH);
  digitalWrite(MOTOR2_IN1, LOW);
  digitalWrite(MOTOR2_IN2, HIGH);

  // Apply speeds to each motor independently
  ledcWrite(MOTOR1_CHANNEL, leftSpeed);
  ledcWrite(MOTOR2_CHANNEL, rightSpeed);
}

void StopWheels()
{
  // Set all direction pins LOW for brake mode
  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, LOW);
  digitalWrite(MOTOR2_IN1, LOW);
  digitalWrite(MOTOR2_IN2, LOW);

  // Set PWM to 0 to ensure motors get no power
  ledcWrite(MOTOR1_CHANNEL, 0);
  ledcWrite(MOTOR2_CHANNEL, 0);
}