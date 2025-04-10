#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoOTA.h>

// =================== Wi-Fi Setup ===================
const char* ssid = "homesweethome";
const char* password = "johnandamy";

AsyncWebServer server(80);



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
  // Declare variables for sensor data
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  
  // Get the raw sensor data
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Return the data as a formatted string
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
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Initialize OTA
  
    ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });

   // Set OTA password (optional for security)
  ArduinoOTA.setPassword("12345678");

  // Start OTA
  ArduinoOTA.begin();

  // Initialize the web server

  // Start the server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    String html = "<html><body><h1>Telemetry Dashboard</h1>";
    html += "<p>IMU Data: " + getIMUData() + "</p>";
    html += "<p>GPS Data: " + getGPSData() + "</p>";
    html += "<p>Battery Voltage: " + String(readBatteryVoltage(), 2) + " V</p>";
    html += "</body></html>";
    request->send(200, "text/html", html);
  });

  server.begin();


  Serial.println("System Ready.");
}


// =================== Loop ===================
void loop() {
  //This is needed for OTA updates
  ArduinoOTA.handle();


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
