#include <SBUS.h>

// Pin definitions
#define RX_PIN 3
#define TX_PIN -1
#define MOTOR1_EN 35
#define MOTOR1_IN1 32
#define MOTOR1_IN2 33
#define MOTOR2_EN 27
#define MOTOR2_IN1 14
#define MOTOR2_IN2 21

// Motor configuration
#define MOTOR1_CHANNEL 0
#define MOTOR2_CHANNEL 1
#define MOTOR_FREQ 490
#define MOTOR_RES 8
#define MAX_SPEED 255

// Control parameters
#define MAX_CHANNEL_VALUE 1800
#define MIN_CHANNEL_VALUE 200
#define DIRECTION_CONTROL_DEADZONE 50
#define MOVE_CONTROL_DEADZONE 50
#define SIGNAL_TIMEOUT 500 // ms before considering signal lost
#define LEFT_MIN_START 40  // Tune experimentally for your hardware
#define RIGHT_MIN_START 36 // Tune experimentally for your hardware

// Weapon Config
#define WEAPON_PIN 26
// Motor direction enum for better readability
enum Direction
{
  FORWARD,
  BACKWARD,
  STOP
};

// Motor class to encapsulate motor functionality
class Motor
{
private:
  uint8_t in1Pin;
  uint8_t in2Pin;
  uint8_t enPin;
  uint8_t pwmChannel;
  String name;

public:
  Motor(uint8_t in1, uint8_t in2, uint8_t en, uint8_t channel, String name)
      : in1Pin(in1), in2Pin(in2), enPin(en), pwmChannel(channel) {}

  void setup()
  {
    pinMode(in1Pin, OUTPUT);
    pinMode(in2Pin, OUTPUT);
    pinMode(enPin, OUTPUT);
    pinMode(WEAPON_PIN, OUTPUT);
    digitalWrite(WEAPON_PIN, LOW);
    ledcSetup(pwmChannel, MOTOR_FREQ, MOTOR_RES);
    ledcAttachPin(enPin, pwmChannel);
    this->name = name;
    stop();
  }

  void setDirection(Direction dir)
  {
    switch (dir)
    {
    case FORWARD:
      digitalWrite(in1Pin, HIGH);
      digitalWrite(in2Pin, LOW);
      break;
    case BACKWARD:
      digitalWrite(in1Pin, LOW);
      digitalWrite(in2Pin, HIGH);
      break;
    case STOP:
    default:
      digitalWrite(in1Pin, LOW);
      digitalWrite(in2Pin, LOW);
      break;
    }
  }

  void setSpeed(int speed)
  {
    Serial.print(">" + this->name + "Speed:");
    Serial.println(speed);

    speed = constrain(speed, 0, MAX_SPEED);
    if (speed > MOVE_CONTROL_DEADZONE && speed < MOVE_CONTROL_DEADZONE + 30)
    {
      Serial.println("BOOOST !!");
      speed = MAX_SPEED;
    }
    else
    {
      Serial.println("HHOOHHAaaa");
    }

    ledcWrite(pwmChannel, speed);
  }

  void stop()
  {
    setDirection(STOP);
    setSpeed(0);
  }

  void run(Direction dir, int speed)
  {
    setDirection(dir);

    setSpeed(speed);
  }
};

// Global variables
unsigned long lastValidSignal = 0; // Timestamp of last valid signal
bfs::SbusRx sbus(&Serial2, RX_PIN, TX_PIN, true);
// Fix motor naming
Motor leftMotor(MOTOR1_IN1, MOTOR1_IN2, MOTOR1_EN, MOTOR1_CHANNEL, "LEFT_MOTOR");
Motor rightMotor(MOTOR2_IN1, MOTOR2_IN2, MOTOR2_EN, MOTOR2_CHANNEL, "RIGHT_MOTOR");

// Function prototypes
void processReceiverInput();
void controlVehicle(int16_t throttleValue, int16_t steeringValue, int16_t weaponValue, int16_t weaponSpeed);
int normalizeChannelValue(int16_t raw, int16_t minVal = MIN_CHANNEL_VALUE, int16_t maxVal = MAX_CHANNEL_VALUE, int minOutput = -MAX_SPEED, int maxOutput = MAX_SPEED);
void PlotSerial();
void processWeaponInput(int weaponValue, int processWeaponInput);

void setup()
{
  Serial.begin(115200);
  delay(1000);

  // Initialize SBUS receiver
  sbus.Begin();

  // Setup motors
  leftMotor.setup();
  rightMotor.setup();
}

void loop()
{
  processReceiverInput();

  // PlotSerial();
}

void processReceiverInput()
{
  if (sbus.Read())
  {
    bfs::SbusData data = sbus.data();

    // Channel 0 is throttle, Channel 2 is steering
    controlVehicle(data.ch[1], data.ch[3], data.ch[5], data.ch[7]);
  }
}

void controlVehicle(int16_t throttleValue, int16_t steeringValue, int16_t weaponValue, int16_t weaponSpeed)
{
  // Normalize raw values to motor speed range
  int throttle = normalizeChannelValue(throttleValue);
  int steering = normalizeChannelValue(steeringValue);
  int weaponActivation = normalizeChannelValue(weaponValue);
  int normalizedWeaponSpeed = normalizeChannelValue(weaponSpeed, 200, 1800, 0, 255);
  // Determine steering direction
  bool turnLeft = (steering < -DIRECTION_CONTROL_DEADZONE);
  bool turnRight = (steering > DIRECTION_CONTROL_DEADZONE);

  // For debugging
  Serial.print(">Throttle:");
  Serial.println(throttle);
  Serial.print(">Steering:");
  Serial.println(steering);
  Serial.print(">Left:");
  Serial.println((int)turnLeft);
  Serial.print(">Right:");
  Serial.println((int)turnRight);
  Serial.print(">Weapon:");
  Serial.println((int)weaponActivation);
  Serial.print(">normalizedWeaponSpeed:");
  Serial.println((int)normalizedWeaponSpeed);
  processWeaponInput(weaponActivation, weaponSpeed);
  // Handle forward movement
  if (throttle > MOVE_CONTROL_DEADZONE)
  {
    if (turnLeft && !turnRight)
    {
      // Turn left while moving forward
      leftMotor.stop();
      rightMotor.run(FORWARD, throttle);
    }
    else if (turnRight && !turnLeft)
    {
      // Turn right while moving forward
      leftMotor.run(FORWARD, throttle);
      rightMotor.stop();
    }
    else
    {
      // Move straight forward
      leftMotor.run(FORWARD, throttle);
      rightMotor.run(FORWARD, throttle);
    }
  }
  // Handle backward movement
  else if (throttle < -MOVE_CONTROL_DEADZONE)
  {
    int speed = abs(throttle);
    if (turnLeft && !turnRight)
    {
      // Turn left while moving backward
      leftMotor.stop();
      rightMotor.run(BACKWARD, speed);
    }
    else if (turnRight && !turnLeft)
    {
      // Turn right while moving backward
      leftMotor.run(BACKWARD, speed);
      rightMotor.stop();
    }
    else
    {
      // Move straight backward
      leftMotor.run(BACKWARD, speed);
      rightMotor.run(BACKWARD, speed);
    }
  }
  // Stop if within deadzone
  else
  {
    leftMotor.stop();
    rightMotor.stop();
  }
}

void processWeaponInput(int weaponValue, int weaponSpeed)
{
  int isActivated = weaponValue <= 0 ? HIGH : LOW;
  digitalWrite(WEAPON_PIN, isActivated);
}

int normalizeChannelValue(int16_t raw, int16_t minVal, int16_t maxVal, int minOutput, int maxOutput)
{
  // Constrain raw values first to prevent extreme outputs
  raw = constrain(raw, minVal, maxVal);
  return map(raw, minVal, maxVal, minOutput, maxOutput);
}

void PlotSerial()
{
  if (sbus.Read())
  {
    // Get the decoded SBUS data
    bfs::SbusData data = sbus.data();
    // Print values in a format suitable for Serial Plotter
    // All values on one line for best plotter visualization
    Serial.print(">CH1:");
    Serial.println(data.ch[0]);
    Serial.print(">CH2:");
    Serial.println(data.ch[1]);
    Serial.print(">CH3:");
    Serial.println(data.ch[2]);
    Serial.print(">CH4:");
    Serial.println(data.ch[3]);
    Serial.print(">CH5:");
    Serial.println(data.ch[4]);
    Serial.print(">CH6:");
    Serial.println(data.ch[5]);
    Serial.print(">CH7:");
    Serial.println(data.ch[6]);
    Serial.print(">CH8:");
    Serial.println(data.ch[7]);
    Serial.print(">CH9:");
    Serial.println(data.ch[8]);
    Serial.print(">CH10:");
    Serial.println(data.ch[9]);
  }
  delay(50);
}
