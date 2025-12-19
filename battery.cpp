#include "battery.h"
#include <Arduino.h>

// reuse your existing constants / pins here
// (or move them into this file and keep only API in the header)

// Battery measurement pins (XIAO nRF52840 Sense)
#define PIN_BAT_EN   P0_14   // P0.14, enables divider when LOW
#define PIN_nCHARGING   P0_17

// Divider values: 1M (top) and 510k (bottom)
const int VBAT_R1 = 1000;   // top resistor (to battery +)
int VBAT_R2 = 485;    // bottom resistor (to GND) (510?)

// ADC configuration (nRF52 core: AR_DEFAULT = 3.6V, 12-bit)
const float ADC_REF_V       = 2.4f;
const int   ADC_RES_BITS    = 12;


static unsigned long lastBatSampleMs = 0;
static bool  lowBattery = false;
static float lastVBat   = 0.0f;
static int raw = 0;
static int raw_prev1 = 0;
static int raw_prev2 = 0;

int measureVBatPinRaw()
{
  pinMode(PIN_BAT_EN, OUTPUT);
  digitalWrite(PIN_BAT_EN, LOW);
  delay(5);
  int raw = analogRead(PIN_VBAT);
  pinMode(PIN_BAT_EN, INPUT);
  return raw;
}

void batteryInit()
{
  pinMode(PIN_BAT_EN, INPUT);
  analogReference(AR_INTERNAL2V4);
  analogReadResolution(ADC_RES_BITS);
  raw_prev2 = measureVBatPinRaw();
  raw_prev1 = measureVBatPinRaw();  
  raw = measureVBatPinRaw();
}

bool vbusConnected()
{
  return !digitalRead(PIN_nCHARGING);
}

const float adcMax = (1 << ADC_RES_BITS) - 1;
void batteryUpdate(float vbatThres)
{
  unsigned long now = millis();
  if (lowBattery == true && vbusConnected() == false) return;

  if (now - lastBatSampleMs < 1000UL) return;   // 1 s

  raw_prev2 = raw_prev1;
  raw_prev1 = raw;  
  lastBatSampleMs = now;
  raw = measureVBatPinRaw();

  float vDiv   = (raw+raw_prev1+raw_prev2)/3 * ADC_REF_V / adcMax;
  lastVBat     = vDiv * ((VBAT_R1 + VBAT_R2) / (float)VBAT_R2);

  if (lastVBat < vbatThres && vbusConnected() == false)
    lowBattery = true;
  else
    lowBattery = false;
}

bool batteryIsLow()
{
  return lowBattery;
}

float getBatteryVoltage()
{
  return lastVBat;
}
