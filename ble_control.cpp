//#ifdef USE_BLE
#include "ble_control.h"

BLEService ledService("19B20050-E8F2-537E-4F6C-D104768A1214"); // Bluetooth® Low Energy LED Service

// Bluetooth® Low Energy LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEUnsignedIntCharacteristic  brightnessCharacteristic("19B20041-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

BLEUnsignedShortCharacteristic vBatCharacteristic("19B20052-E8F2-537E-4F6C-D104768A1214", BLERead);

BLEByteCharacteristic debugCharacteristic("19B20053-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

BLEUnsignedIntCharacteristic  vBatThresCharacteristic("19B20054-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

BLEUnsignedIntCharacteristic  filterCharacteristic("19B20100-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

BLEStringCharacteristic cmdCharacteristic(
  "19B20110-E8F2-537E-4F6C-D104768A1214",
  BLERead | BLEWrite,
  64
);


// --- Add these descriptors (UUID 0x2901 = Characteristic User Description)
BLEDescriptor descBrightness("2901", "Brightness (0..255)");
BLEDescriptor descVBat("2901", "Battery voltage (mV)");
BLEDescriptor descMode("2901", "Mode (0=Off,1=EQ,2=Solid,3=Pulse)");
BLEDescriptor descVBatThres("2901", "VBat threshold (V*10)");
BLEDescriptor descFilter("2901", "Filter amount (0..255)");
BLEDescriptor descStrBrightness("2901", "Brightness String (0..1024)");


//#endif

static BLEDevice central;
static uint16_t brightness = 100;
static uint8_t ledMode = 1;
static float filter_alpha = 0.15f;

static float vbatThres = 3.8f;

void bleInit(uint16_t initialBrightness)
{
  brightness = initialBrightness;

  if (!BLE.begin()) return;

  BLE.setLocalName("LED Necklace");
  BLE.setAdvertisedService(ledService);

  ledService.addCharacteristic(brightnessCharacteristic);
  ledService.addCharacteristic(debugCharacteristic);
  ledService.addCharacteristic(vBatCharacteristic);
  ledService.addCharacteristic(vBatThresCharacteristic);
  ledService.addCharacteristic(filterCharacteristic);
  ledService.addCharacteristic(cmdCharacteristic);

  brightnessCharacteristic.addDescriptor(descBrightness);
  vBatCharacteristic.addDescriptor(descVBat);
  debugCharacteristic.addDescriptor(descMode);
  vBatThresCharacteristic.addDescriptor(descVBatThres);
  filterCharacteristic.addDescriptor(descFilter);
  cmdCharacteristic.addDescriptor(descStrBrightness);

  BLE.addService(ledService);

  brightnessCharacteristic.writeValue(brightness);
  BLE.advertise();
}

void bleUpdate()
{
  if (!central) {
    central = BLE.central();
    if (central) {
      // connected
    }
  } else {
    if (central.connected()) {
      if (brightnessCharacteristic.written()) {
        brightness = brightnessCharacteristic.value();
      }
      if (vBatThresCharacteristic.written()) {
        vbatThres = (float)vBatThresCharacteristic.value() / 10.0f;
      }
      if (debugCharacteristic.written()) {
        ledMode = debugCharacteristic.value();
      }
      if (filterCharacteristic.written()) {
        filter_alpha = 0.15f - ((float)filterCharacteristic.value()) * (0.15f / 255.0f);
      }
    } else {
      central = BLEDevice();   // reset handle
    }
  }
}

uint16_t bleGetBrightness()
{
  return brightness;
}

void bleSetVBat_mV(uint16_t mv)
{
  vBatCharacteristic.writeValue(mv);
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
