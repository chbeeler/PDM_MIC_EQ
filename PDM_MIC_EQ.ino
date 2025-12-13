#include <PDM.h>

#define USE_BLE

#ifdef USE_BLE
#include <ArduinoBLE.h>

BLEService ledService("19B10000-E8F2-537E-4F6C-D104768A1214"); // Bluetooth® Low Energy LED Service

// Bluetooth® Low Energy LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEByteCharacteristic brightnessCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

BLEUnsignedShortCharacteristic vBatCharacteristic("19B10002-E8F2-537E-4F6C-D104768A1214", BLERead);

#endif
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
//const int PIN_VBAT  = P0_31;   // P0.31 / AIN7

// Divider values: 1M (top) and 510k (bottom)
const int VBAT_R1 = 1000;   // top resistor (to battery +)
int VBAT_R2 = 485;    // bottom resistor (to GND) (510?)

// ADC configuration (nRF52 core: AR_DEFAULT = 3.6V, 12-bit)
const float ADC_REF_V       = 2.4f;
const int   ADC_RES_BITS    = 12;
const float VBAT_LOW_THRESH = 3.9f;      // V: go to sleep below this

unsigned long lastBatSampleMs = 0;
bool lowBattery = false;

int brightness = 150;

void check_vbat();
void check_ble();

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

  if (!PDM.begin(channels, frequency)) {
    Serial.println("Failed to start PDM!");
    while (1);
  }

  Serial.println("PDM audio - started");

  pinMode (P0_13, INPUT);
  //digitalWrite(P0_13, HIGH);

  pinMode(PIN_BAT_EN, INPUT);

  // Configure ADC once
  analogReference(AR_INTERNAL2V4);
  analogReadResolution(ADC_RES_BITS);

#ifdef USE_BLE
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");
  }
  else
  {
    // set advertised local name and service UUID:
    BLE.setLocalName("LED Necklace");
    BLE.setAdvertisedService(ledService);

    // add the characteristic to the service
    ledService.addCharacteristic(brightnessCharacteristic);
    ledService.addCharacteristic(vBatCharacteristic);

    // add service
    BLE.addService(ledService);

    // set the initial value for the characeristic:
    brightnessCharacteristic.writeValue(brightness);

    // start advertising
    BLE.advertise();

    Serial.println("BLE LED Peripheral");
  }
#endif
}

void loop() 
{
  check_vbat();
#ifdef USE_BLE
  check_ble();
#endif

  if (Serial.available() > 3) 
  {
    VBAT_R2 = Serial.parseInt();
    /*brightness = Serial.parseInt();        // read number -1..255
    if(brightness < 0 || brightness > -255)
      analogWrite(LED_CHANNEL, -brightness);   // test LED brightness
    else
      digitalWrite(LED_CHANNEL, HIGH);
    Serial.println(brightness);*/
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
    if(brightness >= 0)
    {
      ledValue = ((int)ledValueF);
      if(ledValue > brightness)
        ledValue = brightness;
      analogWrite(LED_BUILTIN, 255-ledValue);
      analogWrite(LED_CHANNEL, ledValue);
    }

    // Plot the filtered loudness (or effectiveLoudness) in Serial Plotter
    /*Serial.print(filteredLoudness);
    Serial.print(", ");
    Serial.println(ledValue);*/

    samplesRead = 0;    // Clear the read count
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
  if (!lowBattery && (now - lastBatSampleMs >= 1000UL)) {   // every 120 s
    lastBatSampleMs = now;

    // Enable divider: P0.14 as output LOW
    pinMode(PIN_BAT_EN, OUTPUT);
    digitalWrite(PIN_BAT_EN, LOW);
    delay(5);  // let the divider settle a bit

    // Read divided battery voltage on P0.31 / AIN7
    int raw = analogRead(PIN_VBAT);

    // Disable divider again: high-Z
    pinMode(PIN_BAT_EN, INPUT);

    // Convert ADC reading to real battery voltage
    float adcMax = (1 << ADC_RES_BITS) - 1;
    float vDiv   = (raw * ADC_REF_V) / adcMax;               // voltage at divider node
    float vBat   = vDiv * ((float)(VBAT_R1 + VBAT_R2) / (float)(VBAT_R2));   // undo divider

    uint16_t vBat_mV = (uint16_t)(vBat * 1000.0f);
    vBatCharacteristic.writeValue(vBat_mV);

    //Serial.print("Battery: ");
    /*Serial.print(vBat, 3);
    Serial.print(",");
    Serial.println(raw);*/

    if (vBat < VBAT_LOW_THRESH) {
      lowBattery = true;

      // Stop LED output (active-low LED → 255 = off)
      digitalWrite(LED_BUILTIN, HIGH);
      digitalWrite(LED_CHANNEL, LOW);

      // Stop microphone / PDM to reduce current
      PDM.end();

      BLE.stopAdvertise();
      BLE.disconnect();
      BLE.end();

      Serial.flush();

      delay(30000);
      // Deep sleep / system off (nRF52)
      NRF_POWER->SYSTEMOFF = 1;
      while (1) {
        // should not return
      }
    }
  }
}

#ifdef USE_BLE
void check_ble()
{
  static int ble_state = 0;
  static BLEDevice central;
  switch(ble_state)
  {
    case 0:   // listen for Bluetooth® Low Energy peripherals to connect:
      central = BLE.central();

      if (central) {    // if a central is connected to peripheral:
        Serial.print("Connected to central: ");      
        Serial.println(central.address());    // print the central's MAC address
        ble_state = 1;
      }
    break;

    case 1:
      // while the central is still connected to peripheral:
      if(central.connected()) 
      {
        if (brightnessCharacteristic.written()) 
        {
          brightness = brightnessCharacteristic.value();

          Serial.print(F("Brightness: "));
          Serial.println(brightness);
        }
      }
      else
      {
        // when the central disconnects, print it out:
        Serial.print(F("Disconnected from central: "));
        Serial.println(central.address());
        ble_state = 0;
      }
    break;
  }
}
#endif