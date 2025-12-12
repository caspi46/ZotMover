#include <Arduino.h>
#include "SparkFunLSM6DSO.h"
#include "L298NX2.h"
#include "Wire.h"
#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>
#include "math.h"

// pin declarations
#define BAT_ADC 34
#define ENAPin 27
#define motorAPin1 26
#define motorAPin2 25
#define ENBPin 15
#define motorBPin3 12
#define motorBPin4 13


#define Kp  40
#define Kd  0.05
#define Ki  40
#define sampleTime  0.005
#define targetAngle -2.5

// Wi-Fi credentials
#define WIFI_SSID "TC"           // NOTE: Please delete this value before submitting assignment
#define WIFI_PASSWORD "12341234" // NOTE: Please delete this value before submitting assignment
// Azure IoT Hub configuration
#define SAS_TOKEN ""
// Root CA certificate for Azure IoT Hub
const char* root_ca =
    "-----BEGIN CERTIFICATE-----\n"
    "MIIEtjCCA56gAwIBAgIQCv1eRG9c89YADp5Gwibf9jANBgkqhkiG9w0BAQsFADBh\n"
    "MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n"
    "d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH\n"
    "MjAeFw0yMjA0MjgwMDAwMDBaFw0zMjA0MjcyMzU5NTlaMEcxCzAJBgNVBAYTAlVT\n"
    "MR4wHAYDVQQKExVNaWNyb3NvZnQgQ29ycG9yYXRpb24xGDAWBgNVBAMTD01TRlQg\n"
    "UlMyNTYgQ0EtMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAMiJV34o\n"
    "eVNHI0mZGh1Rj9mdde3zSY7IhQNqAmRaTzOeRye8QsfhYFXSiMW25JddlcqaqGJ9\n"
    "GEMcJPWBIBIEdNVYl1bB5KQOl+3m68p59Pu7npC74lJRY8F+p8PLKZAJjSkDD9Ex\n"
    "mjHBlPcRrasgflPom3D0XB++nB1y+WLn+cB7DWLoj6qZSUDyWwnEDkkjfKee6ybx\n"
    "SAXq7oORPe9o2BKfgi7dTKlOd7eKhotw96yIgMx7yigE3Q3ARS8m+BOFZ/mx150g\n"
    "dKFfMcDNvSkCpxjVWnk//icrrmmEsn2xJbEuDCvtoSNvGIuCXxqhTM352HGfO2JK\n"
    "AF/Kjf5OrPn2QpECAwEAAaOCAYIwggF+MBIGA1UdEwEB/wQIMAYBAf8CAQAwHQYD\n"
    "VR0OBBYEFAyBfpQ5X8d3on8XFnk46DWWjn+UMB8GA1UdIwQYMBaAFE4iVCAYlebj\n"
    "buYP+vq5Eu0GF485MA4GA1UdDwEB/wQEAwIBhjAdBgNVHSUEFjAUBggrBgEFBQcD\n"
    "AQYIKwYBBQUHAwIwdgYIKwYBBQUHAQEEajBoMCQGCCsGAQUFBzABhhhodHRwOi8v\n"
    "b2NzcC5kaWdpY2VydC5jb20wQAYIKwYBBQUHMAKGNGh0dHA6Ly9jYWNlcnRzLmRp\n"
    "Z2ljZXJ0LmNvbS9EaWdpQ2VydEdsb2JhbFJvb3RHMi5jcnQwQgYDVR0fBDswOTA3\n"
    "oDWgM4YxaHR0cDovL2NybDMuZGlnaWNlcnQuY29tL0RpZ2lDZXJ0R2xvYmFsUm9v\n"
    "dEcyLmNybDA9BgNVHSAENjA0MAsGCWCGSAGG/WwCATAHBgVngQwBATAIBgZngQwB\n"
    "AgEwCAYGZ4EMAQICMAgGBmeBDAECAzANBgkqhkiG9w0BAQsFAAOCAQEAdYWmf+AB\n"
    "klEQShTbhGPQmH1c9BfnEgUFMJsNpzo9dvRj1Uek+L9WfI3kBQn97oUtf25BQsfc\n"
    "kIIvTlE3WhA2Cg2yWLTVjH0Ny03dGsqoFYIypnuAwhOWUPHAu++vaUMcPUTUpQCb\n"
    "eC1h4YW4CCSTYN37D2Q555wxnni0elPj9O0pymWS8gZnsfoKjvoYi/qDPZw1/TSR\n"
    "penOgI6XjmlmPLBrk4LIw7P7PPg4uXUpCzzeybvARG/NIIkFv1eRYIbDF+bIkZbJ\n"
    "QFdB9BjjlA4ukAg2YkOyCiB8eXTBi2APaceh3+uBLIgLk8ysy52g2U3gP7Q26Jlg\n"
    "q/xKzj3O9hFh/g==\n"
    "-----END CERTIFICATE-----\n";

// Telemetry interval
#define TELEMETRY_INTERVAL 3000 // Send data every 3 seconds


// IMU object
LSM6DSO myIMU;
// Motor driver object
L298NX2 motors(ENAPin, motorAPin1, motorAPin2, ENBPin, motorBPin3, motorBPin4);
// motor direction bool 
bool motorDirection = true; // true: forward, false: backward 

// Cloud variables 
String iothubName = "ZotMoverHub"; // Your hub name (replace if needed)
String deviceName = "147esp32";   // Your device name (replace if needed)
String url = "https://" + iothubName + ".azure-devices.net/devices/" +
             deviceName + "/messages/events?api-version=2021-04-12";


// Gyroscope / Accelerometer variables 
int16_t accX, accY, accZ; // accelerometer values
int16_t gyroX, gyroY, gyroZ; // gyroscope values

float accAngle, gyroAngle; 

volatile float motorPower, gyroRate, currentAngle, prevAngle = 0, error, prevError=0, errorSum = 0;

int speed;


long lastTime = 0;
uint32_t lastTelemetryTime = 0;

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  // wifi/cloud setup
  WiFi.mode(WIFI_STA);
  delay(1000);

  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
    Serial.print(WiFi.status());
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("MAC address: ");
  Serial.println(WiFi.macAddress());
  pinMode(BAT_ADC, INPUT);

  // motor setup
  motors.setSpeed(0); // Set initial speed to 0

  // IMU setup
  Wire.begin();
  delay(10);
  if (myIMU.begin())
    Serial.println("Ready.");
  else
  {
    Serial.println("Could not connect to IMU.");
    Serial.println("Freezing");
  }

  if (myIMU.initialize(BASIC_SETTINGS))
    Serial.println("Loaded Settings.");
  
   
}

void loop()
{


  unsigned long now = micros();

    // Run control loop every 5ms
    if (now - lastTime >= 5000) {
      lastTime = now;

      accAngle = atan2(accY, accZ) * RAD_TO_DEG;
      int gyroRate = map(gyroX, -250, 250, -125, 125); // map gyro rate to degrees per second
      gyroAngle = gyroRate * sampleTime;
      currentAngle = 0.9934 * (prevAngle + gyroAngle) + 
                     0.0066 * accAngle;
      error = currentAngle - targetAngle; 
      errorSum += error * sampleTime; 

      motorPower = Kp * error + Ki * errorSum + Kd * (error - prevError) / sampleTime;

      prevAngle = currentAngle;
    }

    accY = myIMU.readFloatAccelY();
    accZ = myIMU.readFloatAccelZ();
    gyroX = myIMU.readFloatGyroX();

    motorPower = constrain(motorPower, -255, 255);
    setMotors(motorPower, motorPower);


  // cloud data
  // if (millis() - lastTelemetryTime >= TELEMETRY_INTERVAL) { // uncomment to enable telemetry
  if (false) {
    float acceleration = myIMU.readFloatAccelX(); // TODO: revise data to send after robot mechanics done
    float tilt = myIMU.readFloatGyroX();

    ArduinoJson::JsonDocument doc;
    doc["acceleration"] = acceleration;
    doc["tilt"] = tilt;
    char buffer[256];
    serializeJson(doc, buffer, sizeof(buffer));

    // Send telemetry via HTTPS
    WiFiClientSecure client;
    client.setCACert(root_ca); // Set root CA certificate
    HTTPClient http;
    http.begin(client, url);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Authorization", SAS_TOKEN);
    int httpCode = http.POST(buffer);

    if (httpCode == 204) { // IoT Hub returns 204 No Content for successful telemetry
    Serial.println("Telemetry sent: " + String(buffer));
    } else {
    Serial.println("Failed to send telemetry. HTTP code: " + String(httpCode));
    }
    http.end();

    lastTelemetryTime = millis();
  }

  delay(1000);
}

void setMotors(int leftMotorSpeed, int rightMotorSpeed) {
  if(leftMotorSpeed >= 0) {
    motors.setSpeedA(leftMotorSpeed);
    motors.forwardA();

  }
  else {
    motors.setSpeedA(255 + leftMotorSpeed); 
    motors.backwardA();
  }
  if(rightMotorSpeed >= 0) {
    motors.setSpeedB(rightMotorSpeed);
    motors.forwardB();
  }
  else {
    motors.setSpeedB(255 + rightMotorSpeed);
    motors.backwardB();
  }
}

// TODO:
// #1 Set up: cloud (done), motor driver (done), gyroscope (done)
// #2 Test individually (just connection)=> review cloud (done), motor driver (not enough torque), and gyroscope (done) from examples or previous assignments
// TODO: Mix the balancing logic with cloud communication
// #3 discuss the communication protocol between motor driver and gyropscope (in review)
// #4 send data to cloud
// #5 visualize tilt and speed over time