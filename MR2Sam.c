#include <Arduino.h>
#include <ArduinoBLE.h>

// https://github.com/aollin/racechrono-ble-diy-device
BLEService batteryService("00001ff8-0000-1000-8000-00805f9b34fb");

BLECharacteristic canBusMainCharacteristic("00000001-0000-1000-8000-00805f9b34fb", BLERead | BLENotify, 8, false);
BLEUnsignedLongCharacteristic canBusFilterCharacteristic("00000002-0000-1000-8000-00805f9b34fb", BLEWrite);

uint8_t tempData[20];

void setup(void) {
  analogReadResolution(14);

  Serial.begin(1000000);
  while (!Serial) {}

  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    for (;;) {}
  }

  BLE.setLocalName("RC DIY #0003");
  BLE.setAdvertisedService(batteryService);
  batteryService.addCharacteristic(canBusMainCharacteristic);
  batteryService.addCharacteristic(canBusFilterCharacteristic);
  BLE.addService(batteryService);
  ((uint32_t *)tempData)[0] = 0x0000ff01;
  canBusMainCharacteristic.writeValue(tempData, 5);

  BLE.advertise();

  Serial.println("Bluetooth® device active, waiting for connections...");
}


void loop(void) {
  updateAnalogData();
  sendViaBluetooth();
}


#define BUFFER_SIZE 51

int measurements[BUFFER_SIZE];
int measurementCount = 0;

void clearBuffer() {
  measurementCount = 0;
}

void insertIntoSortedArray(int arr[], int *size, int element) {
  int i = *size - 1;
  while (i >= 0 && arr[i] > element) {
    arr[i + 1] = arr[i];
    i--;
  }
  arr[i + 1] = element;
  (*size)++;
}

void insertMeasurement(int measurement) {
  insertIntoSortedArray(measurements, &measurementCount, measurement);
}

int getMedian() {
  if (measurementCount % 2 == 0) {
    return (measurements[measurementCount / 2] + measurements[measurementCount / 2 - 1]) / 2;
  } else {
    return measurements[measurementCount / 2];
  }
}

const float VCC = 4.67;

long t2 = 0;
float brakePressure = 0;
float oilPressure = 0;
float oilTemperature = 0;
void updateAnalogData() {
  int i;

  clearBuffer();
  for (i = 0; i < BUFFER_SIZE; i++) {
    insertMeasurement(analogRead(A0));
  }

  oilPressure = (getMedian() * VCC / 16383.0 - 0.5) / 0.4;
  tempData[4] = (int(oilPressure * 100) >> 8) & 0xFF;
  tempData[5] = int(oilPressure * 100) & 0xFF;

  clearBuffer();
  for (i = 0; i < BUFFER_SIZE; i++) {
    insertMeasurement(analogRead(A1));
  }

  float V2 = getMedian() * VCC / 16383.0;
  float lnR2 = log(V2 * 330.0 / (VCC - V2));
  oilTemperature = 1.0 / (1.294333144e-3 + 2.602193987e-4 * lnR2 + 1.738527465e-7 * lnR2 * lnR2 * lnR2) - 273.15;
  tempData[6] = (int(oilTemperature * 100) >> 8) & 0xFF;
  tempData[7] = int(oilTemperature * 100) & 0xFF;
}

void sendViaBluetooth() {
  BLEDevice central = BLE.central();

  if (central) {
    while (central.connected()) {
      updateAnalogData();
      ((uint32_t *)tempData)[0] = 0x0000ff01;
      canBusMainCharacteristic.writeValue(tempData, 8, false);
      delayMicroseconds(20000);
    }
  }
}
