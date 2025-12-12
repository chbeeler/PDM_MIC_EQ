#include <PDM.h>

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
const float SENSITIVITY = 2.0f;  // increase if LED is too dark, decrease if always maxed

// Optional: simple noise floor to ignore very quiet sounds
const float NOISE_FLOOR = 10.0f;

// Battery measurement pins (XIAO nRF52840 Sense)
#define PIN_BAT_EN   P0_14   // P0.14, enables divider when LOW
const int PIN_BAT_ADC  = P0_31;   // P0.31 / AIN7

// Divider values: 1M (top) and 510k (bottom)
const float VBAT_R1 = 1000000.0f;   // top resistor (to battery +)
const float VBAT_R2 = 510000.0f;    // bottom resistor (to GND)

// ADC configuration (nRF52 core: AR_DEFAULT = 3.6V, 12-bit)
const float ADC_REF_V       = 2.4f;
const int   ADC_RES_BITS    = 12;
const float VBAT_LOW_THRESH = 3.8f;      // V: go to sleep below this

unsigned long lastBatSampleMs = 0;
bool lowBattery = false;


void setup() {
  Serial.begin(115200);
  /*while (!Serial) {
    // wait for serial
  }*/

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_CHANNEL, OUTPUT);

  // Configure the data receive callback
  PDM.onReceive(onPDMdata);

  // Optionally set the gain
  // PDM.setGain(30);

  // Initialize PDM with:
  // - one channel (mono mode)
  // - a 16 kHz sample rate
  if (!PDM.begin(channels, frequency)) {
    Serial.println("Failed to start PDM!");
    while (1);
  }

  Serial.println("PDM audio - started");

  pinMode (P0_13, INPUT);
  //digitalWrite(P0_13, HIGH);

  pinMode(PIN_BAT_EN, INPUT);

  // Configure ADC once
  //analogReference(AR_INTERNAL2V4);
  //analogReadResolution(ADC_RES_BITS);

}

int v=150;
void loop() 
{
  //check_vbat();

  if (Serial.available() > 3) 
  {
    v = Serial.parseInt();        // read number -1..255
    if(v < 0 || v > -255)
      analogWrite(LED_CHANNEL, -v);   // test LED brightness
    else
      digitalWrite(LED_CHANNEL, HIGH);
    Serial.println(v);
  }

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
    // Scale and clamp into [0..255] for analogWrite
    float ledValueF = effectiveLoudness * SENSITIVITY;
    if (ledValueF > 255.0f) ledValueF = 255.0f;

    int ledValue = 0;
    if(v >= 0)
    {
      ledValue = ((int)ledValueF);
      if(ledValue > v)
        ledValue = v;
      analogWrite(LED_BUILTIN, 255-ledValue);
      analogWrite(LED_CHANNEL, ledValue);
    }

    // If analogWrite doesn't compile for your core, uncomment this instead:
    //digitalWrite(LED_BUILTIN, ledValue > 50 ? HIGH : LOW);

    // ---- Debug / plotting ----
    // Plot the filtered loudness (or effectiveLoudness) in Serial Plotter
    Serial.print(filteredLoudness);
    Serial.print(", ");
    Serial.println(ledValue);

    // Clear the read count
    samplesRead = 0;
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

void check_vbat(void)
{
  unsigned long now = millis();
  //if (!lowBattery && (now - lastBatSampleMs >= 300000UL)) {   // every 120 s
  if(0) {
    lastBatSampleMs = now;

    // Enable divider: P0.14 as output LOW
    /*pinMode(PIN_BAT_EN, OUTPUT);
    digitalWrite(PIN_BAT_EN, LOW);
    delay(5);  // let the divider settle a bit

    // Read divided battery voltage on P0.31 / AIN7
    int raw = analogRead(PIN_BAT_ADC);

    // Disable divider again: high-Z
    pinMode(PIN_BAT_EN, INPUT);*/
    int raw = 150;

    // Convert ADC reading to real battery voltage
    float adcMax = (1 << ADC_RES_BITS) - 1;
    float vDiv   = (raw * ADC_REF_V) / adcMax;               // voltage at divider node
    float vBat   = vDiv * ((VBAT_R1 + VBAT_R2) / VBAT_R2);   // undo divider

    Serial.print("Battery: ");
    Serial.print(vBat, 3);
    Serial.println(" V");

    // Stop LED output (active-low LED → 255 = off)
    analogWrite(LED_BUILTIN, 255);
    analogWrite(LED_CHANNEL, 0);

    // Stop microphone / PDM to reduce current
    PDM.end();

    Serial.flush();

    NRF_POWER->SYSTEMOFF = 1;
    while (1) {
      // should not return
    }

    /*if (vBat < VBAT_LOW_THRESH) {
      lowBattery = true;
      Serial.println("Battery low, going to sleep...");

      // Stop LED output (active-low LED → 255 = off)
      analogWrite(LED_BUILTIN, 255);
      analogWrite(LED_CHANNEL, 0);

      // Stop microphone / PDM to reduce current
      PDM.end();

      Serial.flush();

      // Deep sleep / system off (nRF52)
      NRF_POWER->SYSTEMOFF = 1;
      while (1) {
        // should not return
      }
    }*/
  }
}
