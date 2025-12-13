//#ifdef USE_BLE
#include "ble_control.h"

BLEService ledService("19B10050-E8F2-537E-4F6C-D104768A1214"); // Bluetooth® Low Energy LED Service

// Bluetooth® Low Energy LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEUnsignedIntCharacteristic  brightnessCharacteristic("19B10051-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

BLEUnsignedShortCharacteristic vBatCharacteristic("19B10052-E8F2-537E-4F6C-D104768A1214", BLERead);

BLEByteCharacteristic debugCharacteristic("19B10053-E8F2-537E-4F6C-D104768A1214", BLERead);

BLEUnsignedIntCharacteristic  vBatThresCharacteristic("19B10054-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

//#endif

static BLEDevice central;
static uint16_t brightness = 100;

static float vbatThres = 3.8f;

void bleInit(uint16_t initialBrightness)
{
  brightness = initialBrightness;

  if (!BLE.begin()) return;

  BLE.setLocalName("LED Necklace");
  BLE.setAdvertisedService(ledService);

  ledService.addCharacteristic(brightnessCharacteristic);
  ledService.addCharacteristic(vBatCharacteristic);
  ledService.addCharacteristic(vBatThresCharacteristic);
  BLE.addService(ledService);

  brightnessCharacteristic.writeValue(brightness/4);
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
        brightness = brightnessCharacteristic.value()*4;
      }
      if (vBatThresCharacteristic.written()) {
        vbatThres = (float)vBatThresCharacteristic.value() / 10.0f;
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
