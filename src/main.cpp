#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <TinyGPSPlus.h>

// =================== Pin Config ===================
const int motorLeftPWM = 5;
const int motorLeftDir = 4;
const int motorRightPWM = 6;
const int motorRightDir = 7;

const int batteryPin = 34; // Analog input
const int gpsRxPin = 16;   // GPS TX -> ESP RX
const int gpsTxPin = 17;   // GPS RX -> ESP TX (optional)

// =================== Objects ===================
MPU6050 mpu;
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

// =================== Globals ===================
unsigned long lastTelemetryTime = 0;
int telemetryInterval = 1000;

// =================== Setup ===================
void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, gpsRxPin, gpsTxPin);

  // Motor Pins
  pinMode(motorLeftPWM, OUTPUT);
  pinMode(motorLeftDir, OUTPUT);
  pinMode(motorRightPWM, OUTPUT);
  pinMode(motorRightDir, OUTPUT);

  // MPU6050
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
  }

  Serial.println("System Ready.");
}

// =================== Motor Control ===================
void setMotor(int pwmPin, int dirPin, int speed) {
  digitalWrite(dirPin, speed >= 0 ? HIGH : LOW);
  analogWrite(pwmPin, abs(speed));
}

void drive(int leftSpeed, int rightSpeed) {
  setMotor(motorLeftPWM, motorLeftDir, leftSpeed);
  setMotor(motorRightPWM, motorRightDir, rightSpeed);
}

// =================== Sensor Reading ===================
String getIMUData() {
  mpu.getMotion6();
  int ax = mpu.getAccelerationX();
  int ay = mpu.getAccelerationY();
  int az = mpu.getAccelerationZ();
  int gx = mpu.getRotationX();
  int gy = mpu.getRotationY();
  int gz = mpu.getRotationZ();

  return "IMU:AX=" + String(ax) + " AY=" + String(ay) + " AZ=" + String(az) +
         " GX=" + String(gx) + " GY=" + String(gy) + " GZ=" + String(gz);
}

String getGPSData() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isValid()) {
    return "GPS:LAT=" + String(gps.location.lat(), 6) +
           " LNG=" + String(gps.location.lng(), 6);
  }
  return "GPS:No Fix";
}

float readBatteryVoltage() {
  return analogRead(batteryPin) * (3.3 / 4095.0) * 2; // Adjust if using voltage divider
}

// =================== Loop ===================
void loop() {
  if (millis() - lastTelemetryTime > telemetryInterval) {
    lastTelemetryTime = millis();

    Serial.println(getIMUData());
    Serial.println(getGPSData());
    Serial.println("Battery: " + String(readBatteryVoltage(), 2) + " V");
  }

  // Example drive (test): slow forward
  drive(100, 100);
  delay(1000);
  drive(0, 0);
  delay(1000);
}
