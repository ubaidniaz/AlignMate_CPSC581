#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Servo.h> // Include the Servo library

/* MPU6050 default I2C address is 0x68 */
MPU6050 mpu;

int const INTERRUPT_PIN = 2; // Define the interrupt pin
int const BUZZER_PIN = 47;   // Define the buzzer pin
int const SERVO_PIN = 40;     // Define the servo pin
bool blinkState;

Servo myServo;               // Create a Servo object

/* MPU6050 Control/Status Variables */
bool DMPReady = false;       // Set true if DMP init was successful
uint8_t MPUIntStatus;        // Interrupt status byte from MPU
uint8_t devStatus;           // Return status after device operation (0 = success, !0 = error)
uint16_t packetSize;         // Expected DMP packet size
uint8_t FIFOBuffer[64];      // FIFO storage buffer

/* Orientation/Motion Variables */
Quaternion q;                // Quaternion container
VectorFloat gravity;         // Gravity vector
float ypr[3];                // Yaw, pitch, and roll angles (only roll will be used)

/* Interrupt detection routine */
volatile bool MPUInterrupt = false; // Indicates whether MPU6050 interrupt pin has gone high
void DMPDataReady() {
  MPUInterrupt = true;
}

void setup() {
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock

  Serial.begin(115200); // Initialize serial communication
  while (!Serial);

  // Initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT); // Set buzzer pin as output

  // Attach servo to the specified pin
  myServo.attach(SERVO_PIN);

  // Verify connection
  Serial.println(F("Testing MPU6050 connection..."));
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (true);
  } else {
    Serial.println("MPU6050 connection successful");
  }

  // Wait for user input to continue
  Serial.println(F("\nSend any character to begin: "));
  while (Serial.available() && Serial.read()); // Empty buffer
  while (!Serial.available());
  while (Serial.available() && Serial.read()); // Empty buffer again

  // Initialize and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // Supply gyro offsets (scaled for min sensitivity)
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

  // Check if DMP initialization was successful
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);  // Generate offsets and calibrate MPU6050
    mpu.CalibrateGyro(6);
    Serial.println("Active offsets:");
    mpu.PrintActiveOffsets();

    // Enable DMP
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // Enable interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
    MPUIntStatus = mpu.getIntStatus();

    // Set DMP ready flag
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  if (!DMPReady) return; // Stop the program if DMP initialization fails

  // Read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
    // Get roll angle
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    float roll = ypr[2] * 180 / M_PI; // Convert roll to degrees
    Serial.print("Roll: ");
    Serial.println(roll);

    // Check roll thresholds and control buzzer and servo
    if (roll >= 3 && roll < 6) {
      Serial.println("Close");
      tone(BUZZER_PIN, 2000, 200); // Play a short 1kHz tone for 200ms
      
      delay(300); // Delay to prevent overlapping sounds
    } else if (roll >= 6) {
      Serial.println("Far");
      tone(BUZZER_PIN, 1000); // Play a continuous 2kHz tone

      // Move servo back and forth
      for (int angle = 0; angle <= 180; angle += 10) {
        myServo.write(angle); // Move servo to the current angle
        delay(20);            // Small delay for smooth motion
      }
      for (int angle = 180; angle >= 0; angle -= 10) {
        myServo.write(angle); // Move servo back to the starting angle
        delay(20);            // Small delay for smooth motion
      }
    } else if (roll < 3){
      noTone(BUZZER_PIN);  // Stop the buzzer when roll is below 10 degrees
      myServo.write(90);  // Stop servo at a neutral position (90°)
    }

    // Blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_BUILTIN, blinkState);
  }
}
