#include <Wire.h>
#include "MAX30100_PulseOximeter.h"

#define BLYNK_PRINT         Serial
#define BLYNK_TEMPLATE_ID   "TMPL6bmtSn3I5"
#define BLYNK_TEMPLATE_NAME "HeartRate"
#define REPORTING_PERIOD_MS 1000
#define BUFFER_SIZE         4

#include <Blynk.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

PulseOximeter pox;

char auth[] = "L00zQf1iHzKtQz3UpgXjEezG327BWudl";             // You should get Auth Token in the Blynk App.
char ssid[] = "pass 12345678";                                     // Your WiFi credentials.
char pass[] = "12345678";

uint32_t tsLastReport = 0; //Time at which the last beat occurred

float beatsPerMinute;

float buffer[BUFFER_SIZE];
byte b_index = 0;

void UpdateBuffer(){
  buffer[b_index++] = pox.getHeartRate();
  if (b_index >= BUFFER_SIZE) {
    b_index = 0;
  }
}

float AverageValue(){
  float sum = 0;
  for (byte i = 0; i < BUFFER_SIZE; i++){
    sum += buffer[i];
  }
  return sum / BUFFER_SIZE;
}

// Callback (registered below) fired when a pulse is detected
void onBeatDetected()
{
    Serial.println("Beat!");
}

void setup()
{
  //Serial display
  Serial.begin(115200);
  Serial.println("Initializing Pulse Oximeter..");

  //Connect to Blynk
  Blynk.begin(auth, ssid, pass);


  // Initialize the PulseOximeter instance
  // Failures are generally due to an improper I2C wiring, missing power supply
  // or wrong target chip
  if (!pox.begin()) {
    Serial.println("FAILED: MAX30100 was not found. Please check wiring/power.");
    while(1);
  } else {
    Serial.println("SUCCESS: Place your index finger on the sensor with steady pressure.");
  }
  // Register a callback for the beat detection
  pox.setOnBeatDetectedCallback(onBeatDetected);
}

void loop()
{
  Blynk.run();
  
  // Make sure to call update as fast as possible
  pox.update();

  // Asynchronously dump heart rate to the serial
  // For both, a value of 0 means "invalid"
  if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
      UpdateBuffer();
      beatsPerMinute = AverageValue();
      Serial.print("Heart rate:");
      Serial.println(beatsPerMinute);
      Blynk.virtualWrite(V0, beatsPerMinute);
      tsLastReport = millis();
  }
}