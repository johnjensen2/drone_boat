#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoOTA.h>
#include <WebSerial.h>

// =================== Wi-Fi Setup ===================
const char* ssid = "homesweethome";
const char* password = "johnandamy";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");


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
// Variables for GPS data
double currentLat = 29.676096;
double currentLon = -98.057546;
// Serial data variable
String serialData = "";
bool autoScroll = true;  // Autoscroll toggle

// =================== Motor Control ===================
void setMotor(int pwmPin, int dirPin, int speed) {
  digitalWrite(dirPin, speed >= 0 ? HIGH : LOW);
  analogWrite(pwmPin, abs(speed));
}

void drive(int leftSpeed, int rightSpeed) {
  setMotor(motorLeftPWM, motorLeftDir, leftSpeed);
  setMotor(motorRightPWM, motorRightDir, rightSpeed);
}

// =================== WebSocket event handler ===================
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.println("WebSocket client connected");
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.println("WebSocket client disconnected");
  } else if (type == WS_EVT_DATA) {
    // Handle incoming data (if needed)
    String message = String((char*)data);
    Serial.println("Received message: " + message);
  }
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
// =================== WebSerial Setup ===================

unsigned long last_print_time = millis();
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
 // ArduinoOTA.setPassword("12345678");

  // Start OTA
  ArduinoOTA.begin();

  // Initialize the web server

  // Setup WebSocket
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  // Start the server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    String html = "<html><head>";
    html += "<link rel='stylesheet' href='https://unpkg.com/leaflet/dist/leaflet.css' />";
    html += "<script src='https://unpkg.com/leaflet/dist/leaflet.js'></script>";
    html += "</head><body>";
    html += "<h1>Telemetry Dashboard</h1>";
    
    // Display IMU and GPS data
    html += "<p>IMU Data: " + getIMUData() + "</p>";
    html += "<p>GPS Data: Latitude: " + String(currentLat, 6) + ", Longitude: " + String(currentLon, 6) + "</p>";
    html += "<p>Battery Voltage: " + String(readBatteryVoltage(), 2) + " V</p>";

    // Leaflet Map
    html += "<div id='map' style='width: 100%; height: 400px;'></div>";

    // Serial Data Display Box
    html += "<h3>Serial Data</h3>";
    html += "<textarea id='serialBox' style='width:100%; height:200px;' readonly></textarea><br>";
    html += "<button onclick='toggleAutoscroll()'>Toggle Autoscroll</button>";

    // JavaScript for setting up the map and GPS coordinates
    html += "<script>";
    html += "var map = L.map('map').setView([0, 0], 13);"; // Default center
    html += "L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {";
    html += "    attribution: '© OpenStreetMap contributors'";
    html += "}).addTo(map);";
    html += "var marker = L.marker([0, 0]).addTo(map);"; // Default marker

    // Update GPS coordinates dynamically from ESP32
    html += "function updateLocation(lat, lon) {";
    html += "    marker.setLatLng([lat, lon]);";
    html += "    map.setView([lat, lon], 13);"; // Update map view to new coordinates
    html += "}";

    // Call updateLocation with your GPS data
    html += "updateLocation(" + String(currentLat, 6) + ", " + String(currentLon, 6) + ");"; // Real GPS data

    
    html += "</script>";
    html += "</body></html>";
    request->send(200, "text/html", html);
  });
  // Start the webserial server
  // WebSerial is accessible at "<IP Address>/webserial" in browser
  WebSerial.begin(&server);

  /* Attach Message Callback */
  WebSerial.onMessage([&](uint8_t *data, size_t len) {
    Serial.printf("Received %u bytes from WebSerial: ", len);
    Serial.write(data, len);
    Serial.println();
    WebSerial.println("Received Data...");
    String d = "";
    for(size_t i=0; i < len; i++){
      d += char(data[i]);
    }
    WebSerial.println(d);
  });

  // Start the server
  server.begin();

  Serial.println("System Ready.");
}


// =================== Loop ===================
void loop() {
  //This is needed for OTA updates
  ArduinoOTA.handle();

  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
    if (gps.location.isUpdated()) {
      // Update the GPS coordinates when available
      currentLat = gps.location.lat();
      currentLon = gps.location.lng();
    }
  }
  if (millis() - lastTelemetryTime > telemetryInterval) {
    lastTelemetryTime = millis();

    Serial.println(getIMUData());
    Serial.println(getGPSData());
    Serial.println("Battery: " + String(readBatteryVoltage(), 2) + " V");
  }

// =================== this works but i need to edit the liberay page ===================
// Print every 2 seconds (non-blocking)
/*   if ((unsigned long)(millis() - last_print_time) > 2000) {
    WebSerial.print(F("IP address: "));
    WebSerial.println(WiFi.localIP());
    WebSerial.printf("Uptime: %lums\n", millis());
    #if defined(ESP8266)
      WebSerial.printf("Free heap: %" PRIu32 "\n", ESP.getFreeHeap());
    #elif defined(ESP32)
      WebSerial.printf("Free heap: %" PRIu32 "\n", ESP.getFreeHeap());
    #elif defined(TARGET_RP2040) || defined(TARGET_RP2350) || defined(PICO_RP2040) || defined(PICO_RP2350)
      WebSerial.printf("Free heap: %" PRIu32 "\n", rp2040.getFreeHeap());
    #endif
    last_print_time = millis();
  } */

  WebSerial.loop();
 
  // Web server will run in the background
}
