#pragma once
#include <Arduino.h>

void batteryInit();
void batteryUpdate(float vbatThres);          // call each loop
bool batteryIsLow();
float getBatteryVoltage();        // last measured voltage
bool vbusConnected();