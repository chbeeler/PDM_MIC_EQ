#include <PDM.h>
#include <Filters.h>
#include <Filters/Butterworth.hpp>
#include "battery.h"
#include "ble_control.h"

#define USE_BLE

// ---- PWM configuration ----
const uint8_t PWM_RES_BITS = 12;    // 12-bit: 0..4095 (nRF52840 HW can do up to 15 bits)
uint16_t pwmMax = (1 << PWM_RES_BITS) - 1;

#define LED_CHANNEL 3

static const char channels = 1;   // default number of output channels
static const int f_s = 16000;   // default PCM output frequency

short sampleBuffer[512];    // Buffer to read samples into, each sample is 16-bits
volatile int samplesRead = 0;   // Number of audio samples read

// --- Butterworth LPF for bass ---
const double f_c = 150.0;      // try 80..200 Hz
const double f_n = 2.0 * f_c / (double)f_s;
auto bassLP = butter<6>(f_n);

// ---- Beat / loudness detection variables ----

// Low-pass filtered loudness value
float filteredLoudness = 0.0f;

// How fast the filter reacts (0..1, closer to 0 = slower, more smoothing)
const float LP_ALPHA = 0.15f;

// Scale mic loudness into LED brightness (tune this)
const float SENSITIVITY = 3.0f;  // increase if LED is too dark, decrease if always maxed

// Optional: simple noise floor to ignore very quiet sounds
const float NOISE_FLOOR = 50.0f;



unsigned long lastBatSampleMs = 0;
bool lowBattery = false;

int brightness = 600;


static const uint16_t ENV_BLOCK_N = 320;   // 20 ms @ 16 kHz
static float    ENV_ALPHA   = 0.1f;

static uint32_t envSumAbs = 0;
static uint16_t envCount  = 0;
static float    envLp     = 0.0f;          // smoothed envelope (your LED driver input)

// call this once per filtered sample:
static inline void envelope_update(int16_t filtSample)
{
  uint16_t a = (filtSample < 0) ? (uint16_t)(-filtSample) : (uint16_t)filtSample;
  envSumAbs += a;
  envCount++;

  if (envCount >= ENV_BLOCK_N) {
    float env = (float)envSumAbs / (float)ENV_BLOCK_N;      // mean(|x|) for the block
    envLp = envLp + ENV_ALPHA * (env - envLp);              // one-pole low-pass

    // reset for next block
    envSumAbs = 0;
    envCount  = 0;
    
    // ---- Apply noise floor ----
    float effectiveLoudness = envLp - NOISE_FLOOR;
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

    switch(getLedMode())
    {
      case 0: ledValue = 0; break;
      case 2: ledValue = brightness; break;
    }

    analogWrite(LED_CHANNEL, ledValue);
    
    // Plot the filtered loudness (or effectiveLoudness) in Serial Plotter
    Serial.print(envLp);
    Serial.print(", ");
    Serial.println(ledValue);
  }
}


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
  PDM.setGain(55);

  if (!PDM.begin(channels, f_s)) {
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
  if (batteryIsLow() == true && vbusConnected() == false) {
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
    if (samplesRead) 
    {
      for (int i = 0; i < samplesRead; i++) 
      {
        int16_t raw  = sampleBuffer[i];
        int16_t filt = (int16_t)bassLP((float)raw);

        envelope_update(filt);    //also updates LED brightness
      }

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
