#include <ArduinoBLE.h>

// https://github.com/aollin/racechrono-ble-diy-device
BLEService batteryService("00001ff8-0000-1000-8000-00805f9b34fb");

BLECharacteristic canBusMainCharacteristic("00000001-0000-1000-8000-00805f9b34fb", BLERead | BLENotify, 20, false);
BLEUnsignedLongCharacteristic canBusFilterCharacteristic("00000002-0000-1000-8000-00805f9b34fb", BLEWrite);

uint8_t tempData[20];

long previousMillis = 0;


#include <SPI.h>

#define CAN_2515

const int SPI_CS_PIN = 10;
const int CAN_INT_PIN = 2;

#include "mcp2515_can.h"
mcp2515_can CAN(SPI_CS_PIN);
#define MAX_DATA_SIZE 8


void setup() {
  Serial.begin(115200);

  while (!Serial)
    ;

  pinMode(LED_BUILTIN, OUTPUT);

  analogReadResolution(14);

  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");

    while (1)
      ;
  }

  BLE.setLocalName("RC DIY #0000");
  BLE.setAdvertisedService(batteryService);
  batteryService.addCharacteristic(canBusMainCharacteristic);
  batteryService.addCharacteristic(canBusFilterCharacteristic);
  BLE.addService(batteryService);

  canBusMainCharacteristic.writeValue(tempData, 5);

  BLE.advertise();

  Serial.println("Bluetooth® device active, waiting for connections...");


  while (CAN_OK != CAN.begin(CAN_500KBPS)) {
    Serial.println(F("CAN init fail, retry..."));
    delay(100);
  }
  Serial.println(F("CAN init ok!"));
}


uint32_t id;
uint8_t type;
uint8_t len;
byte cdata[MAX_DATA_SIZE] = { 0 };

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    digitalWrite(LED_BUILTIN, HIGH);

    while (central.connected()) {
      long currentMillis = millis();
      if (currentMillis - previousMillis >= 0) {
        previousMillis = currentMillis;
        updateBatteryLevel();
      }
    }

    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}

byte val2;

void updateBatteryLevel() {
  // Serial.print("Battery Level % is now: ");

  int val = (analogRead(A0) * (5.0 / 16383.0) - analogRead(A3) * (5.0 / 16383.0)) * 100;

  // Serial.print(voltage);
  // Serial.print(" ");
  // Serial.println(val);

  // https://github.com/aollin/racechrono-ble-diy-device?tab=readme-ov-file#can-bus-main-characteristic-uuid-0x0001
  // 32-bit packet ID (Notice: this value is a little-endian integer, unlike other values in this API)
  ((uint32_t*)tempData)[0] = 0x0000ff01;
  tempData[4] = (val >> 8) & 0xff;
  tempData[5] = val & 0xff;
  canBusMainCharacteristic.writeValue(tempData, 6, false);

  // delay(10);

  int val2 = (analogRead(A1) * (4.82 / 16383.0)) * 100;
  ((uint32_t*)tempData)[0] = 0x0000ff02;
  tempData[4] = (val2 >> 8) & 0xff;
  tempData[5] = val2 & 0xff;
  canBusMainCharacteristic.writeValue(tempData, 6, false);

  int val3 = (analogRead(A2) * (4.82 / 16383.0)) * 100;
  ((uint32_t*)tempData)[0] = 0x0000ff03;
  tempData[4] = (val3 >> 8) & 0xff;
  tempData[5] = val3 & 0xff;
  canBusMainCharacteristic.writeValue(tempData, 6, false);

  // R2 = V2 * R1 / (Vcc - V2)
  // 1/(1.294333144e-3 + 2.602193987e-4 * ln(244) + 1.738527465e-7* ln(244)^3)-273,15


  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    int i;

    CAN.readMsgBuf(&len, cdata);
    
    ((uint32_t*)tempData)[0] = CAN.getCanId();
    for (i = 0; i < len; i++) {
      tempData[i + 4] = cdata[i];
    }

    canBusMainCharacteristic.writeValue(tempData, len + 4, false);

    // if (id != 0x81) return;
  }
}
