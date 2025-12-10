#include <Arduino.h>
#include "SparkFunLSM6DSO.h"
#include "L298NX2.h"
#include "Wire.h"
#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include "Wire.h"
#include <ArduinoJson.h>

// Wi-Fi credentials
#define WIFI_SSID "TC"           // NOTE: Please delete this value before submitting assignment
#define WIFI_PASSWORD "12341234" // NOTE: Please delete this value before submitting assignment
// Azure IoT Hub configuration
#define SAS_TOKEN "SharedAccessSignature sr=cs147hub39.azure-devices.net%2Fdevices%2F147esp32&sig=YvsUAQIDHyB2BVio%2FewGasVdA7Q5Ivw6pPYZdGPh%2BBM%3D&se=1763775054"
// Root CA certificate for Azure IoT Hub
// const char* root_ca = "Get from Azure IoT Hub";
const char *root_ca =
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

#define BAT_ADC 34
#define motorAPin1 26
#define motorAPin2 25
#define ENAPin 38
#define motorBPin3 33
#define motorBPin4 32
#define ENBPin 37

LSM6DSO myIMU;
L298NX2 motor(motorAPin1, motorAPin2, ENAPin, motorBPin3, motorBPin4, ENBPin);

String iothubName = "cs147hub39"; // Your hub name (replace if needed)
String deviceName = "147esp32";   // Your device name (replace if needed)
String url = "https://" + iothubName + ".azure-devices.net/devices/" +
             deviceName + "/messages/events?api-version=2021-04-12";

void setup()
{
  Wire.begin();

  WiFi.mode(WIFI_STA);
  delay(1000);
  Serial.println();
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
  Serial.begin(9600);
  pinMode(BAT_ADC, INPUT);

  // Set initial speed
  motor.setSpeed(0); // Set initial speed to 0
}

void loop()
{

  delay(1000); // Delay for readability

  // motor driver
  // gyroscope: (x, y, z)
  // accelerometer: (x, y, z)
  // cloud data
}