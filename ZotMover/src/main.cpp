#include <Arduino.h>

#define BAT_ADC 34


void setup() {
  Serial.begin(115200); 
  pinMode(BAT_ADC, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  int rawValue = analogRead(BAT_ADC);
  float voltage = ((float)rawValue / 4095.0) * 3.3 * 2.0; // Assuming a voltage divider with equal resistors
  Serial.print("Raw Value: "); 
  Serial.print(rawValue);
  Serial.print(" | Voltage: ");
  Serial.print(voltage);

  if (voltage < 1) {
    Serial.println(" V - Battery Low!");
  } else {
    Serial.println(" V");
  }

  delay(2000); // Delay for readability
}
