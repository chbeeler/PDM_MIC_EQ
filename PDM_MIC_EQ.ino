#include <PDM.h>
#include "battery.h"
#include "ble_control.h"

#define USE_BLE

// ---- PWM configuration ----
const uint8_t PWM_RES_BITS = 12;    // 12-bit: 0..4095 (nRF52840 HW can do up to 15 bits)
uint16_t pwmMax = (1 << PWM_RES_BITS) - 1;

#define LED_CHANNEL 3

// default number of output channels
static const char channels = 1;

// default PCM output frequency
static const int frequency = 16000;

// Buffer to read samples into, each sample is 16-bits
short sampleBuffer[512];

// Number of audio samples read
volatile int samplesRead = 0;

// ---- Beat / loudness detection variables ----

// Low-pass filtered loudness value
float filteredLoudness = 0.0f;

// How fast the filter reacts (0..1, closer to 0 = slower, more smoothing)
const float LP_ALPHA = 0.15f;

// Scale mic loudness into LED brightness (tune this)
const float SENSITIVITY = 8.0f;  // increase if LED is too dark, decrease if always maxed

// Optional: simple noise floor to ignore very quiet sounds
const float NOISE_FLOOR = 10.0f;



unsigned long lastBatSampleMs = 0;
bool lowBattery = false;

int brightness = 600;

void check_vbat();
void check_ble();

void setup() {
  Serial.begin(115200);
  /*while (!Serial) {
    // wait for serial
  }*/

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_CHANNEL, OUTPUT);
  analogWriteResolution(PWM_RES_BITS);

  // Configure the data receive callback
  PDM.onReceive(onPDMdata);

  // Optionally set the gain
  //PDM.setGain(25);

  if (!PDM.begin(channels, frequency)) {
    Serial.println("Failed to start PDM!");
    while (1);
  }

  Serial.println("PDM audio - started");

  pinMode (P0_13, INPUT);
  //digitalWrite(P0_13, HIGH);

  //pinMode(PIN_BAT_EN, INPUT);

  batteryInit();
#ifdef USE_BLE
  bleInit(brightness);     // your existing global brightness
#endif
}

void loop() 
{
  batteryUpdate(bleGetVbatThres());
  if (batteryIsLow()) {
    // TODO: kill LEDs & BLE 
    // and bail out of loop early
    lowBattery = true;

    // Immediately stop LEDs
    analogWrite(LED_CHANNEL, 0); 
    digitalWrite(LED_BUILTIN, HIGH);   // off (active-low)

    Serial.println("Vbat low");

    delay(1000);
  }
  else
  {
#ifdef USE_BLE
    bleSetVBat_mV((uint16_t)(getBatteryVoltage()*1000.0f));
    bleUpdate();
    brightness = bleGetBrightness();
#endif

    // Wait for samples to be read
    if (samplesRead) {
      // ---- Compute average absolute amplitude for this block ----
      long sumAbs = 0;

      for (int i = 0; i < samplesRead; i++) {
        int16_t s = sampleBuffer[i];
        if (s < 0) s = -s;
        sumAbs += s;
      }

      float avgLoudness = 0.0f;
      if (samplesRead > 0) {
        avgLoudness = (float)sumAbs / (float)samplesRead;
      }

      // ---- Low-pass filter over time (simple 1st-order IIR) ----
      // filtered = filtered + alpha * (input - filtered)
      filteredLoudness = filteredLoudness + LP_ALPHA * (avgLoudness - filteredLoudness);

      // ---- Apply noise floor ----
      float effectiveLoudness = filteredLoudness - NOISE_FLOOR;
      if (effectiveLoudness < 0.0f) {
        effectiveLoudness = 0.0f;
      }

      // ---- Map loudness to LED brightness ----
      float ledValueF = effectiveLoudness * SENSITIVITY;

      if (ledValueF > (float)pwmMax)
        ledValueF = (float)pwmMax;

      int ledValue = (int)ledValueF;
      
      if (ledValue > brightness)
        ledValue = brightness;

      analogWrite(LED_CHANNEL, ledValue);
      
      // Plot the filtered loudness (or effectiveLoudness) in Serial Plotter
      Serial.print(filteredLoudness);
      Serial.print(", ");
      Serial.println(ledValue);

      samplesRead = 0;    // Clear the read count
    }
  }
}

/**
 * Callback function to process the data from the PDM microphone.
 * NOTE: This callback is executed as part of an ISR.
 * Therefore using `Serial` to print messages inside this function isn't supported.
 */
void onPDMdata() {
  // Query the number of available bytes
  int bytesAvailable = PDM.available();

  // Read into the sample buffer
  PDM.read(sampleBuffer, bytesAvailable);

  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable / 2;
}
