#include <Arduino.h>
#include <U8g2lib.h>
#include <Arduino_CAN.h>
#include <ArduinoBLE.h>
#include <Wire.h>

// https://github.com/aollin/racechrono-ble-diy-device
BLEService batteryService("00001ff8-0000-1000-8000-00805f9b34fb");

BLECharacteristic canBusMainCharacteristic("00000001-0000-1000-8000-00805f9b34fb", BLERead | BLENotify, 20, false);
BLEUnsignedLongCharacteristic canBusFilterCharacteristic("00000002-0000-1000-8000-00805f9b34fb", BLEWrite);

U8G2_SSD1306_128X64_NONAME_F_HW_I2C *u8g2;
U8G2_SSD1306_128X64_NONAME_F_HW_I2C *u8g2_2;

uint8_t tempData[20];
uint8_t tempData2[20];

#define LED_PIN_RED 9
#define LED_PIN_GREEN 6
#define LED_PIN_YELLOW 5

int buttonState = 0;


void setup(void) {
  pinMode(LED_PIN_RED, OUTPUT);
  pinMode(LED_PIN_GREEN, OUTPUT);
  pinMode(LED_PIN_YELLOW, OUTPUT);

  pinMode(4, INPUT);

  analogReadResolution(14);

  Serial.begin(1000000);
  while (!Serial) {}

  if (!CAN.begin(CanBitRate::BR_500k)) {
    Serial.println("CAN.begin(...) failed.");
    for (;;) {}
  }

  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    for (;;) {}
  }

  BLE.setLocalName("RC DIY #0000");
  BLE.setAdvertisedService(batteryService);
  batteryService.addCharacteristic(canBusMainCharacteristic);
  batteryService.addCharacteristic(canBusFilterCharacteristic);
  BLE.addService(batteryService);
  ((uint32_t *)tempData)[0] = 0x0000ff01;
  ((uint32_t *)tempData2)[0] = 0x0000ff02;
  canBusMainCharacteristic.writeValue(tempData, 5);

  BLE.advertise();

  Serial.println("Bluetooth® device active, waiting for connections...");

  u8g2 = new U8G2_SSD1306_128X64_NONAME_F_HW_I2C(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
  u8g2->setI2CAddress(0x7A);
  u8g2->setBusClock(400000);
  u8g2->begin();

  u8g2_2 = new U8G2_SSD1306_128X64_NONAME_F_HW_I2C(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
  u8g2_2->setI2CAddress(0x78);
  u8g2_2->setBusClock(400000);
  u8g2_2->begin();
}

int mode = 0;
int canChangeMode = true;
int checkSeconds = true;

void checkButton(void) {
  buttonState = digitalRead(4);
  if (buttonState == HIGH) {
    if (canChangeMode) {
      mode ^= 1;
      canChangeMode = false;
      checkSeconds = false;
    }
  } else {
    canChangeMode = true;
  }

  if (checkSeconds && millis() > 15000) {
    checkSeconds = false;
    mode = 1;
  }
}


void loop(void) {
  checkButton();
  checkThresholds();

  for (int i = 0; i < 30; i++) {
    checkCAN();
  }

  updateAnalogData();
  sendViaBluetooth();
  drawDisplays();
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
    insertMeasurement(analogRead(A0) - analogRead(A1));
  }

  brakePressure = (getMedian() * VCC / 16383.0 - 0.5) / 0.027;
  if (brakePressure < 0) {
    brakePressure = 0;
  }

  tempData2[14] = (int(brakePressure * 100) >> 8) & 0xFF;
  tempData2[15] = int(brakePressure * 100) & 0xFF;

  clearBuffer();
  for (i = 0; i < BUFFER_SIZE; i++) {
    insertMeasurement(analogRead(A2));
  }

  oilPressure = (getMedian() * VCC / 16383.0 - 0.5) / 0.4;
  tempData2[16] = (int(oilPressure * 100) >> 8) & 0xFF;
  tempData2[17] = int(oilPressure * 100) & 0xFF;

  clearBuffer();
  for (i = 0; i < BUFFER_SIZE; i++) {
    insertMeasurement(analogRead(A3));
  }

  float V2 = getMedian() * VCC / 16383.0;
  float lnR2 = log(V2 * 330.0 / (VCC - V2));
  oilTemperature = 1.0 / (1.294333144e-3 + 2.602193987e-4 * lnR2 + 1.738527465e-7 * lnR2 * lnR2 * lnR2) - 273.15;
  tempData2[18] = (int(oilTemperature * 100) >> 8) & 0xFF;
  tempData2[19] = int(oilTemperature * 100) & 0xFF;
}

uint8_t found[4];
long found_c[4];

long previousMillis = 0;
long val = 0;
int i = 0;
long totalTime = 0;
void sendViaBluetooth() {
  BLEDevice central = BLE.central();

  if (central) {
    while (central.connected()) {
      found[0] = 0;
      found[1] = 0;
      found[2] = 0;
      found[3] = 0;
      long start = micros();
      for (i = 0; i < 50; i++) {
        checkCAN();
        delayMicroseconds(20);
      }
      for (i = 0; i < 4; i++) {
        found_c[i] += found[i];
      }

      if (false && micros() % 10 == 0) {
        Serial.print(micros() - start);
        Serial.print(" ");
        for (i = 0; i < 4; i++) {
          Serial.print(found_c[i]);
          Serial.print(" ");
        }
        Serial.println();
      }

      ((uint32_t *)tempData)[0] = 0x0000ff01;
      canBusMainCharacteristic.writeValue(tempData, 20, false);

      for (i = 0; i < 20; i++) {
        checkCAN();
      }

      updateAnalogData();

      ((uint32_t *)tempData2)[0] = 0x0000ff02;
      canBusMainCharacteristic.writeValue(tempData2, 20, false);

      checkButton();
      checkThresholds();

      drawDisplays();
    }
  }
}

bool needAsk = true;
long askTime = 0;
long startAsk = 0;

long startTime = 0;
long count = 0;

/*

  mafAirFlowRatio = OBD2.pidRead(16);  0x10  ((A * 256.0 + B) / 100.0);
  egt = OBD2.pidRead(60);              0x3c  (((A * 256.0 + B) / 10.0) - 40.0);
  voltage = OBD2.pidRead(66);          0x42  ((A * 256.0 + B) / 1000.0);
  afr = OBD2.pidRead(68) * 14.6412885; 0x44  (2.0 * (A * 256.0 + B) / 65536.0);
  
  intakeAirTemperature = OBD2.pidRead(15); 0xf  (A - 40.0);


0081	2-3 (BE)	0xFFFF	Steering angle (FDE1 to 021E)	    bytesToInt (2, 2)
0201	0-1 (BE)	0xFFFF	RPM * 4	                          bytesToUint(0, 2)/4
0201	4-5 (BE)	0xFFFF	Vehicle speed kph * 100 + 10000	 (bytesToUint(4, 2)-10000)/360
0201	6	0xFF	Throttle % * 2	                            bytesToUint(6, 1)/2
0212	5	0x8	Brake on	                                    bitsToUint (44,1)*100
0420  0 Coolant + 40                                      bytesToUint(0, 1)-40           <---- coolantTemparature
04B0	0-1 (BE)	0xFFFF	LF wheel kph * 100 + 10000	     (bytesToUint(0, 2)-10000)/360
04B0	2-3 (BE)	0xFFFF	RF wheel kph * 100 + 10000	     (bytesToUint(2, 2)-10000)/360
04B0	4-5 (BE)	0xFFFF	LR wheel kph * 100 + 10000	     (bytesToUint(4, 2)-10000)/360
04B0	6-7 (BE)	0xFFFF	RR wheel kph * 100 + 10000	     (bytesToUint(6, 2)-10000)/360


0081 - 100Hz   
0201 - 62Hz
0212 - 50Hz
0420
04B0 - 50Hz
*/

double mafAirFlowRatio = 0.0;
double egt = 0.0;
double voltage = 0.0;
double afr = 0.0;
double intakeAirTemperature = 0.0;
double coolantTemperature = 0.0;


void checkCAN() {
  if (CAN.available()) {
    CanMsg const msg = CAN.read();

    switch (msg.id) {
      case 0x81:
        tempData[4] = msg.data[2];  // 0081	2-3 (BE)	0xFFFF	Steering angle (FDE1 to 021E)	    bytesToInt (2, 2)
        tempData[5] = msg.data[3];
        found[0] = 1;
        break;
      case 0x201:
        tempData[6] = msg.data[0];  // 0201	0-1 (BE)	0xFFFF	RPM * 4	                          bytesToUint(0, 2)/4
        tempData[7] = msg.data[1];
        tempData[8] = msg.data[4];  // 0201	4-5 (BE)	0xFFFF	Vehicle speed kph * 100 + 10000	 (bytesToUint(4, 2)-10000)/360
        tempData[9] = msg.data[5];
        tempData[10] = msg.data[6];  // 0201	6	0xFF	Throttle % * 2	                            bytesToUint(6, 1)/2
        found[1] = 1;
        break;
      case 0x212:
        tempData[11] = msg.data[5];  // 0212	5	0x8	Brake on	  0001000                                  bitsToUint (44,1)*100
        found[2] = 1;
        break;
      case 0x4B0:
        tempData[12] = msg.data[0];  // 04B0	0-1 (BE)	0xFFFF	LF wheel kph * 100 + 10000	     (bytesToUint(0, 2)-10000)/360
        tempData[13] = msg.data[1];
        tempData[14] = msg.data[2];  // 04B0	2-3 (BE)	0xFFFF	RF wheel kph * 100 + 10000	     (bytesToUint(2, 2)-10000)/360
        tempData[15] = msg.data[3];
        tempData[16] = msg.data[4];  // 04B0	4-5 (BE)	0xFFFF	LR wheel kph * 100 + 10000	     (bytesToUint(4, 2)-10000)/360
        tempData[17] = msg.data[5];
        tempData[18] = msg.data[6];  // 04B0	6-7 (BE)	0xFFFF	RR wheel kph * 100 + 10000	     (bytesToUint(6, 2)-10000)/360
        tempData[19] = msg.data[7];
        found[3] = 1;
        break;
      case 0x420:
        tempData2[4] = msg.data[0];  // 0420  0 Coolant + 40                                      bytesToUint(0, 1)-40
        coolantTemperature = msg.data[0] - 40.0;
        break;
      default:
        if (msg.data[1] == 0x41) {
          switch (msg.data[2]) {
            case 0x10:
              tempData2[5] = msg.data[3];
              tempData2[6] = msg.data[4];
              mafAirFlowRatio = ((msg.data[3] * 256.0 + msg.data[4]) / 100.0);
              break;
            case 0x3c:
              tempData2[7] = msg.data[3];
              tempData2[8] = msg.data[4];
              egt = (((msg.data[3] * 256.0 + msg.data[4]) / 10.0) - 40.0);
              break;
            case 0x42:
              tempData2[9] = msg.data[3];
              tempData2[10] = msg.data[4];
              voltage = ((msg.data[3] * 256.0 + msg.data[4]) / 1000.0);
              break;
            case 0x44:
              tempData2[11] = msg.data[3];
              tempData2[12] = msg.data[4];
              afr = (2.0 * (msg.data[3] * 256.0 + msg.data[4]) / 65536.0) * 14.6412885;
              break;
            case 0xf:
              tempData2[13] = msg.data[3];
              intakeAirTemperature = (msg.data[3] - 40.0);
              break;
          }
        }
    }
  }
}

void checkThresholds(void) {
  if (coolantTemperature < 50 && oilTemperature < 110) {
    digitalWrite(LED_PIN_RED, LOW);
    digitalWrite(LED_PIN_YELLOW, HIGH);
    digitalWrite(LED_PIN_GREEN, LOW);
  }

  if (coolantTemperature > 50 && coolantTemperature < 60 && oilTemperature < 110) {
    digitalWrite(LED_PIN_RED, LOW);
    digitalWrite(LED_PIN_YELLOW, LOW);
    digitalWrite(LED_PIN_GREEN, HIGH);
  }

  if (coolantTemperature > 60 && coolantTemperature < 102 && oilTemperature < 102) {
    digitalWrite(LED_PIN_RED, LOW);
    digitalWrite(LED_PIN_YELLOW, LOW);
    digitalWrite(LED_PIN_GREEN, LOW);
  }

  if (coolantTemperature >= 102 && coolantTemperature < 107 && oilTemperature < 107) {
    digitalWrite(LED_PIN_RED, LOW);
    digitalWrite(LED_PIN_YELLOW, HIGH);
    digitalWrite(LED_PIN_GREEN, LOW);
  }

  if (coolantTemperature >= 107 || oilTemperature >= 107) {
    digitalWrite(LED_PIN_RED, HIGH);
    digitalWrite(LED_PIN_YELLOW, HIGH);
    digitalWrite(LED_PIN_GREEN, LOW);
  }

  if (coolantTemperature < 108 && oilTemperature < 108) {
    pinMode(3, INPUT);
  }

  if (coolantTemperature > 110 || oilTemperature > 110) {
    tone(3, 550);
  }
}


long state = 0;

void askCanOBD() {
  uint8_t pid = 0;

  switch (state) {
    case 11:
      pid = 0xf;  // intakeAirTemperature
      break;
    case 15:
      pid = 0x42;  // voltage
      break;
    case 19:
      pid = 0x3c;  // EGT
      break;
    case 23:
      pid = 0x10;  // MAF
      break;
    case 27:
      pid = 0x44;  // AFR
      break;
  }

  if (pid > 0) {
    uint8_t const msg_data[] = { 0x02, 0x01, pid, 0, 0, 0, 0, 0 };
    CanMsg const msg(CanStandardId(0x7DF), sizeof(msg_data), msg_data);
    int const rc = CAN.write(msg);
  }
}

char buffer[10];

void drawDisplays(void) {
  switch (state) {
    case 0:
      u8g2->clearBuffer();

      if (mode == 0) {
        u8g2->setCursor(0, 32);
        u8g2->setFont(u8g2_font_logisoso22_tn);
        dtostrf(oilTemperature, -5, 1, buffer);
        u8g2->print(buffer);

        u8g2->setCursor(0, 7);
        u8g2->setFont(u8g2_font_6x10_tf);
        u8g2->print("OIL T");
      } else {
        u8g2->setCursor(12, 64);
        u8g2->setFont(u8g2_font_logisoso62_tn);
        dtostrf(oilTemperature, 3, 0, buffer);
        u8g2->print(buffer);


        u8g2->setFontDirection(3);
        u8g2->setCursor(14, 61);
        u8g2->setFont(u8g2_font_UnnamedDOSFontIV_tr);  //u8g2_font_6x10_tf
        u8g2->print("OIL");
        u8g2->setFontDirection(0);
      }

      u8g2->updateDisplayArea(0, 0, 8, 1);
      break;
    case 1:
      u8g2->updateDisplayArea(0, 1, 8, 1);
      break;
    case 2:
      u8g2->updateDisplayArea(0, 2, 8, 1);
      break;
    case 3:
      u8g2->updateDisplayArea(0, 3, 8, 1);
      break;

    case 4:
      if (mode == 0) {
        u8g2->setCursor(86, 32);
        u8g2->setFont(u8g2_font_logisoso22_tn);
        dtostrf(coolantTemperature, 3, 0, buffer);
        u8g2->print(buffer);

        u8g2->setCursor(87, 7);
        u8g2->setFont(u8g2_font_6x10_tf);
        u8g2->print("WATER T");
      }

      u8g2->updateDisplayArea(8, 0, 8, 1);
      break;
    case 5:
      u8g2->updateDisplayArea(8, 1, 8, 1);
      break;
    case 6:
      u8g2->updateDisplayArea(8, 2, 8, 1);
      break;
    case 7:
      u8g2->updateDisplayArea(8, 3, 8, 1);
      break;

    case 8:
      if (mode == 0) {
        u8g2->setCursor(0, 56);
        u8g2->setFont(u8g2_font_logisoso22_tn);
        dtostrf(oilPressure, 3, 1, buffer);
        u8g2->print(buffer);

        u8g2->setCursor(0, 64);
        u8g2->setFont(u8g2_font_6x10_tf);
        u8g2->print("OIL P");
      }

      u8g2->updateDisplayArea(0, 4, 8, 1);
      break;
    case 9:
      u8g2->updateDisplayArea(0, 5, 8, 1);
      break;
    case 10:
      u8g2->updateDisplayArea(0, 6, 8, 1);
      break;
    case 11:
      u8g2->updateDisplayArea(0, 7, 8, 1);
      askCanOBD();
      break;

    case 12:
      if (mode == 0) {
        u8g2->setCursor(86, 56);
        u8g2->setFont(u8g2_font_logisoso22_tn);
        dtostrf(intakeAirTemperature, 3, 0, buffer);
        u8g2->print(buffer);

        u8g2->setCursor(94, 64);
        u8g2->setFont(u8g2_font_6x10_tf);
        u8g2->print("INTAKE");
      }

      u8g2->updateDisplayArea(8, 4, 8, 1);
      break;
    case 13:
      u8g2->updateDisplayArea(8, 5, 8, 1);
      break;
    case 14:
      u8g2->updateDisplayArea(8, 6, 8, 1);
      break;
    case 15:
      u8g2->updateDisplayArea(8, 7, 8, 1);
      askCanOBD();
      break;

    case 16:
      u8g2_2->clearBuffer();

      if (mode == 0) {
        u8g2_2->setCursor(0, 32);
        u8g2_2->setFont(u8g2_font_logisoso22_tn);
        u8g2_2->print(voltage, 1);
        u8g2_2->setCursor(0, 7);
        u8g2_2->setFont(u8g2_font_6x10_tf);
        u8g2_2->print("VOLT");
      } else {
        u8g2_2->setCursor(12, 64);
        u8g2_2->setFont(u8g2_font_logisoso62_tn);
        dtostrf(coolantTemperature, 3, 0, buffer);
        u8g2_2->print(buffer);

        u8g2_2->setFontDirection(3);
        u8g2_2->setCursor(14, 61);
        u8g2_2->setFont(u8g2_font_UnnamedDOSFontIV_tr);  //u8g2_font_6x10_tf
        u8g2_2->print("WATER");
        u8g2_2->setFontDirection(0);
      }

      u8g2_2->updateDisplayArea(0, 0, 8, 1);
      break;
    case 17:
      u8g2_2->updateDisplayArea(0, 1, 8, 1);
      break;
    case 18:
      u8g2_2->updateDisplayArea(0, 2, 8, 1);
      break;
    case 19:
      u8g2_2->updateDisplayArea(0, 3, 8, 1);
      askCanOBD();
      break;

    case 20:
      if (mode == 0) {
        u8g2_2->setCursor(86, 32);
        u8g2_2->setFont(u8g2_font_logisoso22_tn);
        dtostrf(egt, 3, 0, buffer);
        u8g2_2->print(buffer);

        u8g2_2->setCursor(110, 7);
        u8g2_2->setFont(u8g2_font_6x10_tf);
        u8g2_2->print("EGT");
      }

      u8g2_2->updateDisplayArea(8, 0, 8, 1);
      break;
    case 21:
      u8g2_2->updateDisplayArea(8, 1, 8, 1);
      break;
    case 22:
      u8g2_2->updateDisplayArea(8, 2, 8, 1);
      break;
    case 23:
      u8g2_2->updateDisplayArea(8, 3, 8, 1);
      askCanOBD();
      break;
    case 24:
      if (mode == 0) {
        u8g2_2->setCursor(0, 56);
        u8g2_2->setFont(u8g2_font_logisoso22_tn);
        u8g2_2->print(mafAirFlowRatio, 2);

        u8g2_2->setCursor(0, 64);
        u8g2_2->setFont(u8g2_font_6x10_tf);
        u8g2_2->print("MAF");
      }

      u8g2_2->updateDisplayArea(0, 4, 8, 1);
      break;
    case 25:
      u8g2_2->updateDisplayArea(0, 5, 8, 1);
      break;
    case 26:
      u8g2_2->updateDisplayArea(0, 6, 8, 1);
      break;
    case 27:
      u8g2_2->updateDisplayArea(0, 7, 8, 1);
      askCanOBD();
      break;

    case 28:
      if (mode == 0) {
        u8g2_2->setCursor(64, 56);
        u8g2_2->setFont(u8g2_font_logisoso22_tn);
        dtostrf(afr, 5, 1, buffer);
        u8g2_2->print(buffer);

        u8g2_2->setCursor(110, 64);
        u8g2_2->setFont(u8g2_font_6x10_tf);
        u8g2_2->print("AFR");
      }

      u8g2_2->updateDisplayArea(8, 4, 8, 1);
      break;
    case 29:
      u8g2_2->updateDisplayArea(8, 5, 8, 1);
      break;
    case 30:
      u8g2_2->updateDisplayArea(8, 6, 8, 1);
      break;
    case 31:
      u8g2_2->updateDisplayArea(8, 7, 8, 1);
      state = -1;
      break;
  }

  state++;
}
