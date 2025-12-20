//#ifdef USE_BLE
#include "ble_control.h"

BLEService ledService("19B20050-E8F2-537E-4F6C-D104768A1214"); // Bluetooth® Low Energy LED Service

// Bluetooth® Low Energy LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEUnsignedIntCharacteristic  brightnessCharacteristic("19B20041-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEUnsignedIntCharacteristic  sensitivityCharacteristic("19B20042-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

BLEUnsignedShortCharacteristic vBatCharacteristic("19B20052-E8F2-537E-4F6C-D104768A1214", BLERead);

BLEByteCharacteristic debugCharacteristic("19B20053-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

BLEUnsignedIntCharacteristic  vBatThresCharacteristic("19B20054-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

BLEUnsignedIntCharacteristic  filterCharacteristic("19B20100-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

BLEStringCharacteristic settingsStrCharacteristic(
  "19B20110-E8F2-537E-4F6C-D104768A1214",
  BLERead | BLEWrite,
  64
);


// --- Add these descriptors (UUID 0x2901 = Characteristic User Description)
BLEDescriptor descBrightness("2901", "Brightness (0..255)");
BLEDescriptor descSensitivity("2901", "Sensitivity (0..255)");
BLEDescriptor descVBat("2901", "Battery voltage (mV)");
BLEDescriptor descMode("2901", "Mode (0=Off,1=EQ,2=Solid,3=Pulse)");
BLEDescriptor descVBatThres("2901", "VBat threshold (V*10)");
BLEDescriptor descFilter("2901", "Filter amount (0..255)");
BLEDescriptor descSettingsStr("2901", "Settings (b, s, a)");


//#endif

#define SENSITIVITY_CONV_FACTOR   (0.0005f)

static BLEDevice central;
static uint16_t brightness = 100;
static float sensitivityF = 0.5f;
static uint8_t ledMode = 1;
static float filter_alpha = 0.15f;
static float noise_floor = 10.0f;   // same unit you use in DSP (avg abs, etc.)
uint16_t vbat_mV=0;

static const uint8_t MODE_EQ       = 1;
static const uint8_t MODE_CONSTANT = 2;

static float vbatThres = 3.8f;

static void writeSettingsString()
{
  char s[64];

  // bm encodes mode in sign
  int bm = (int)brightness;
  if (ledMode == MODE_CONSTANT) bm = -bm;

  int sensitivityRaw = (int)(sensitivityF / SENSITIVITY_CONV_FACTOR + 0.5f);
  int alphaRaw       = (int)(filter_alpha * 500.0f + 0.5f);
  int noiseFloorRaw          = (int)(noise_floor + 0.5f);

  // clamp to your intended ranges
  if (bm > 4095) bm = 4095; 
  if (bm < -4095) bm = -4095;
  if (sensitivityRaw < 0) sensitivityRaw = 0;
  if (sensitivityRaw > 4095) sensitivityRaw = 4095;
  if (alphaRaw < 0) alphaRaw = 0;
  if (alphaRaw > 255) alphaRaw = 255;
  if (noiseFloorRaw < 0) noiseFloorRaw = 0;
  if (noiseFloorRaw > 255) noiseFloorRaw = 255; 

  snprintf(s, sizeof(s), "%d,%d,%d,%d,%d", bm, sensitivityRaw, alphaRaw, noiseFloorRaw, vbat_mV);
  settingsStrCharacteristic.writeValue(s);
}

void bleInit(uint16_t initialBrightness)
{
  brightness = initialBrightness;

  if (!BLE.begin()) return;

  BLE.setLocalName("LED Necklace");
  BLE.setAdvertisedService(ledService);

  ledService.addCharacteristic(settingsStrCharacteristic);
  ledService.addCharacteristic(vBatCharacteristic);
  ledService.addCharacteristic(brightnessCharacteristic);
  ledService.addCharacteristic(sensitivityCharacteristic);
  ledService.addCharacteristic(debugCharacteristic);  
  ledService.addCharacteristic(vBatThresCharacteristic);
  ledService.addCharacteristic(filterCharacteristic);
  

  brightnessCharacteristic.addDescriptor(descBrightness);
  sensitivityCharacteristic.addDescriptor(descSensitivity);
  vBatCharacteristic.addDescriptor(descVBat);
  debugCharacteristic.addDescriptor(descMode);
  vBatThresCharacteristic.addDescriptor(descVBatThres);
  filterCharacteristic.addDescriptor(descFilter);
  settingsStrCharacteristic.addDescriptor(descSettingsStr);

  BLE.addService(ledService);

  brightnessCharacteristic.writeValue(brightness);
  sensitivityCharacteristic.writeValue((uint16_t)(sensitivityF / SENSITIVITY_CONV_FACTOR + 0.5f));
  filterCharacteristic.writeValue((uint16_t)(filter_alpha * 500.0f + 0.5f));

  // --- NEW: write initial settings string ---
  writeSettingsString();

  BLE.advertise();
}


void bleUpdate()
{
  if (!central) {
    central = BLE.central();
  } else {
    if (central.connected()) {

      if (brightnessCharacteristic.written()) {
        uint32_t v = brightnessCharacteristic.value();
        if (v > 4095) v = 4095;
        brightness = (uint16_t)v;
        writeSettingsString();
      }

      if (sensitivityCharacteristic.written()) {
        uint32_t raw = sensitivityCharacteristic.value();
        if (raw > 255) raw = 255;
        sensitivityF = (float)raw * SENSITIVITY_CONV_FACTOR;
        writeSettingsString();
      }

      if (filterCharacteristic.written()) {
        uint32_t raw = filterCharacteristic.value();
        if (raw > 255) raw = 255;
        filter_alpha = (float)raw / 500.0f;   // 0..0.255
        writeSettingsString();
      }

      if (vBatThresCharacteristic.written()) {
        vbatThres = (float)vBatThresCharacteristic.value() / 10.0f;
      }

      if (debugCharacteristic.written()) {
        ledMode = debugCharacteristic.value();
      }

      // --- NEW: parse comma-separated settings string: "b,s,a" ---
      if (settingsStrCharacteristic.written()) {
        int bm = 0, b = 0, s = 0, a = 0, nfRaw = 0;

        String str = settingsStrCharacteristic.value();
        int matched = sscanf(str.c_str(), " %d , %d , %d , %d ", &bm, &s, &a, &nfRaw);

        if (matched == 4) {
          // mode from sign of bm
          if (bm < 0) {
            ledMode = MODE_CONSTANT;
            b = -bm;
          } else {
            ledMode = MODE_EQ;
            b = bm;
          }
          if (b < 0) b = 0; if (b > 4095) b = 4095;          
          if (s < 0) s = 0; if (s > 4095) s = 4095;
          if (a < 0) a = 0; if (a > 255) a = 255;
          if (nfRaw < 0) nfRaw = 0; if (nfRaw > 5000) nfRaw = 5000;
          
          brightness   = (uint16_t)b;
          sensitivityF = (float)s * SENSITIVITY_CONV_FACTOR;
          filter_alpha = (float)a / 500.0f;
          noise_floor = (float)nfRaw;

          // keep individual characteristics in sync
          brightnessCharacteristic.writeValue((uint32_t)brightness);
          sensitivityCharacteristic.writeValue((uint32_t)s);
          filterCharacteristic.writeValue((uint32_t)a);
          debugCharacteristic.writeValue(ledMode);

          // rewrite canonical string (normalized formatting)
          writeSettingsString();
        }
      }

    } else {
      central = BLEDevice();
    }
  }
}


uint16_t bleGetBrightness()
{
  return brightness;
}

float bleGetSensitivity()
{
  return sensitivityF;
}

void bleSetVBat_mV(uint16_t mv)
{
  vBatCharacteristic.writeValue(mv);
  vbat_mV = mv;
  writeSettingsString();  //TODO: not efficient
}

float bleGetVbatThres()
{
  return vbatThres;
}

uint8_t getLedMode()
{
  return ledMode;
}

float getFilterAlpha()
{
  return filter_alpha;
}

float bleGetNoiseFloor() { return noise_floor; }