//this code is for reading the MPU6050 and other sensors - IR and rotary switch to manipulate various functions of the teleoperation

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// --- CONFIGURATION ---
#define ENC_CLK D5
#define ENC_DT  D6
#define ENC_SW  D7  // Home Button
#define MODE_PIN D3 // Flash Button (Mode Switch)
#define IR_PIN   D4 // IR Sensor (Trigger Gripper)

Adafruit_MPU6050 mpu;

// --- STATE VARIABLES ---
volatile long encoderValue = 0; 
volatile int lastEncoded = 0;

float pitch = 0, roll = 0;
float alpha = 0.90; 
unsigned long lastTime = 0;

// Debounce helpers
int btnState = HIGH;
int lastBtnState = HIGH;
unsigned long lastDebounceTime = 0;

// Interrupt for Encoder
ICACHE_RAM_ATTR void handleEncoder() {
  int MSB = digitalRead(ENC_CLK); 
  int LSB = digitalRead(ENC_DT);  
  int encoded = (MSB << 1) | LSB; 
  int sum  = (lastEncoded << 2) | encoded; 
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue--;
  lastEncoded = encoded;
}

void setup() {
  Serial.begin(1000000);
  Wire.begin();
  
  if (!mpu.begin()) while (1) yield();
  
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);
  pinMode(MODE_PIN, INPUT_PULLUP);
  
  // IR Sensor Setup
  pinMode(IR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_CLK), handleEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_DT), handleEncoder, CHANGE);
  
  lastTime = millis();
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  float accel_pitch = atan2(a.acceleration.x, sqrt(a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z)) * 180.0 / PI;
  float accel_roll  = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;

  pitch = alpha * (pitch + g.gyro.y * dt) + (1.0 - alpha) * accel_pitch;
  roll  = alpha * (roll + g.gyro.x * dt)  + (1.0 - alpha) * accel_roll;

  // Read Home Button (Debounced)
  int reading = digitalRead(ENC_SW);
  int outputBtn = 0; 
  if (reading != lastBtnState) lastDebounceTime = millis();
  if ((millis() - lastDebounceTime) > 50) {
    if (reading != btnState) {
      btnState = reading;
      if (btnState == LOW) outputBtn = 1;
    }
  }
  lastBtnState = reading;

  // Read Mode (Active LOW)
  int modeStatus = (digitalRead(MODE_PIN) == LOW) ? 1 : 0;

  // Read IR (Active LOW = Object Detected)
  // 1 if Object Detected, 0 if Clear
  int irReading = digitalRead(IR_PIN);
  int irStatus = (irReading == LOW) ? 1 : 0;

  // Output: "S, Pitch, Roll, Enc, BtnHome, Mode, IR, E"
  Serial.print("S,");
  Serial.print(pitch, 1); Serial.print(",");
  Serial.print(roll, 1); Serial.print(",");
  Serial.print(encoderValue); Serial.print(",");
  Serial.print(outputBtn); Serial.print(",");
  Serial.print(modeStatus); Serial.print(",");
  Serial.print(irStatus);
  Serial.println(",E");

  delay(20);
}
