#include <ESP8266HTTPClient.h>

// For Blynk
#define BLYNK_PRINT Serial
#define BLYNK_TEMPLATE_ID "TMPL6bmtSn3I5"
#define BLYNK_TEMPLATE_NAME "HeartRate"
#include <Blynk.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

#include <Wire.h>
#include "MAX30100_PulseOximeter.h"
#include "arduinoFFT.h"

#define REPORTING_PERIOD_MS                 20
#define FINGER_ON                           20000 // if red signal is lower than this, it indicates your finger is not on the sensor
#define PULSE_SAMPLES                       256
#define SAMPLE_FREQ                         50

#define HB_BUFFER_SIZE                      4

// Sampling is tightly related to the dynamic range of the ADC.
// refer to the datasheet for further info
#define SAMPLING_RATE                       MAX30100_SAMPRATE_100HZ

// The LEDs currents must be set to a level that avoids clipping and maximises the
// dynamic range
#define IR_LED_CURRENT                      MAX30100_LED_CURR_50MA
#define RED_LED_CURRENT                     MAX30100_LED_CURR_27_1MA

// The pulse width of the LEDs driving determines the resolution of
// the ADC (which is a Sigma-Delta).
// set HIGHRES_MODE to true only when setting PULSE_WIDTH to MAX30100_SPC_PW_1600US_16BITS
#define PULSE_WIDTH                         MAX30100_SPC_PW_1600US_16BITS
#define HIGHRES_MODE                        true

#define START_COUNTER                       (1500 / REPORTING_PERIOD_MS)
#define RESET_COUNTER                       (200 / REPORTING_PERIOD_MS)

MAX30100 sensor;
arduinoFFT FFT;

char auth[] = "L00zQf1iHzKtQz3UpgXjEezG327BWudl";  // You should get Auth Token in the Blynk App.
char ssid[] = "pass 12345678";                     // Your WiFi credentials.
char pass[] = "12345678";

uint32_t tsLastReport = 0;
uint32_t curTime = 0;
uint16_t ir, red;

// When the finger isn't on the sensor about 200ms, it will reset.
uint16_t resetCounter = RESET_COUNTER;
// Delay 500ms when the finger just placed on the sensor.
uint16_t startCounter = START_COUNTER;

// These are belong to software filter.
float cur_y = 0, pre_y = 0, cur_x = 0, pre_x = 0;

// Create buffer for FFT
uint8_t i = 0;
double redArray[PULSE_SAMPLES];  // array to store samples from the sensor
double vReal[PULSE_SAMPLES];
double vImag[PULSE_SAMPLES];

// Create buffer to reduce fructuation.
uint8_t idx_HB = HB_BUFFER_SIZE;
double HB_Buffer[HB_BUFFER_SIZE];
uint8_t numHBValue = 0;

int positive_round(double value){
  int res = (int) value;
  if (value - res < 0.5) return res;
  return res + 1;
}

uint8_t start(){
  if (startCounter <= 0) return 1;
  startCounter--;
  if (startCounter <= 0) return 1;
  return 0;
}

void refreshResetCounter(){
  resetCounter = RESET_COUNTER;
}

// Reset 2 buffers above
void resetTimerRun(){
  if (resetCounter <= 0) return;
  resetCounter--;
  if (resetCounter > 0) return;

  i = 0;
  numHBValue = 0;
  HB_Buffer[0] = 0;
  HB_Buffer[1] = 0;
  HB_Buffer[2] = 0;
  HB_Buffer[3] = 0;

  startCounter = START_COUNTER;
  Serial.println(0);
  Blynk.virtualWrite(V0, 0);
}

// Calculate average of 4 latest records.
void update_HB_Buffer(double HB){
  if (++idx_HB >= HB_BUFFER_SIZE){
    idx_HB = 0;
  }
  if (numHBValue < HB_BUFFER_SIZE) numHBValue++;
  HB_Buffer[idx_HB] = HB;

  double sum = 0;
  for(uint8_t j = 0; j < HB_BUFFER_SIZE; j++){
    sum += HB_Buffer[j];
  }
  int heartrate = positive_round((sum / numHBValue) * 60);
  Serial.println(heartrate);
  Blynk.virtualWrite(V0, heartrate);
}

void setup() {
  Serial.begin(115200);
  Blynk.begin(auth, ssid, pass);
  Serial.println("Initializing MAX30100..");
  while (!sensor.begin()) {
    Serial.println("FAILED");
    delay(500);
  }
  // Set up the wanted parameters
  sensor.setMode(MAX30100_MODE_SPO2_HR);
  sensor.setLedsCurrent(IR_LED_CURRENT, RED_LED_CURRENT);
  sensor.setLedsPulseWidth(PULSE_WIDTH);
  sensor.setSamplingRate(SAMPLING_RATE);
  sensor.setHighresModeEnabled(HIGHRES_MODE);
}

void loop() {
  while(1){
    Blynk.run();
    sensor.update();
    curTime = millis();
    sensor.getRawValues(&ir, &red);
    if (curTime - tsLastReport >= REPORTING_PERIOD_MS){
      tsLastReport = curTime;
      if (red >= FINGER_ON){
        refreshResetCounter();
        if (!start()) continue;
        // Filter with passband <= 4 Hz
        pre_y = cur_y;
        pre_x = cur_x;
        cur_x = red;
        cur_y = 0.5983 * pre_y + 0.2008 * cur_x + 0.2008 * pre_x;

        redArray[i++] = (double)cur_y;
        // Use FFT to calculate HeartRate when buffer is full.
        if (i == 0)
        {
          for (int idx = 0; idx < PULSE_SAMPLES; idx++) {
            vReal[idx] = redArray[idx];
            vImag[idx] = 0.0;
          }
          // uint32_t flag0 = millis();
          FFT = arduinoFFT(vReal, vImag, PULSE_SAMPLES, SAMPLE_FREQ); /* Create FFT object */
          FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);            /* Weigh data */
          FFT.Compute(FFT_FORWARD);                                   /* Compute FFT */
          FFT.ComplexToMagnitude();                                   /* Compute magnitudes */

          update_HB_Buffer(FFT.MajorPeak());
        }
      } else {
        resetTimerRun();
      }
    }
  }
}
