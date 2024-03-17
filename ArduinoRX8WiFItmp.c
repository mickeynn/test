#include <ArduinoBLE.h>

// https://github.com/aollin/racechrono-ble-diy-device
BLEService batteryService("00001ff8-0000-1000-8000-00805f9b34fb");

BLECharacteristic canBusMainCharacteristic("00000001-0000-1000-8000-00805f9b34fb", BLERead | BLENotify, 20, false);
BLEUnsignedLongCharacteristic canBusFilterCharacteristic("00000002-0000-1000-8000-00805f9b34fb", BLEWrite);

uint8_t tempData[20];

#include <SPI.h>

#define CAN_2515

const int SPI_CS_PIN = 10; // 9 for test, 10 for prod
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


  // while (CAN_OK != CAN.begin(CAN_500KBPS)) {
  //   Serial.println(F("CAN init fail, retry..."));
  //   delay(100);
  // }

  // CAN.init_Mask(0, 0, 0xfff);
  // CAN.init_Mask(1, 0, 0xfff);

  // CAN.init_Filt(0, 0, 0x81);
  // CAN.init_Filt(1, 0, 0x201);
  // CAN.init_Filt(2, 0, 0x212);
  // CAN.init_Filt(3, 0, 0x4b0);
  // CAN.init_Filt(4, 0, 0x420);

  Serial.println(F("CAN init ok!"));
}


uint32_t id;
uint8_t type;
uint8_t len;
byte cdata[MAX_DATA_SIZE] = { 0 };

long previousMillisAnalog = 0;


void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    digitalWrite(LED_BUILTIN, HIGH);

    while (central.connected()) {
      long currentMillis = millis();

      if (currentMillis - previousMillisAnalog >= 20) {
        previousMillisAnalog = currentMillis;
        sendAnalogCounters();
      }

      //sendCANData();
    }

    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}

const float VCC = 4.67;

// https://github.com/aollin/racechrono-ble-diy-device?tab=readme-ov-file#can-bus-main-characteristic-uuid-0x0001
// 32-bit packet ID (Notice: this value is a little-endian integer, unlike other values in this API)
void sendAnalogCounters() {
  int brakePressure = ((analogRead(A0) * (VCC / 16383.0) - analogRead(A1) * (VCC / 16383.0) - 0.5) / 0.027) * 100;
  int oilPressure = ((analogRead(A2) * (VCC / 16383.0) - 0.5) / 0.4) * 100;

  // R2 = V2 * R1 / (Vcc - V2)
  // 1/(1.294333144e-3 + 2.602193987e-4 * ln(244) + 1.738527465e-7* ln(244)^3)-273,15
  float V2 = analogRead(A3) * (VCC / 16383.0);
  float lnR2 = log(V2 * 330.0 / (VCC - V2));
  int oilTemperature = (1 / (1.294333144e-3 + 2.602193987e-4 * lnR2 + 1.738527465e-7 * lnR2 * lnR2 * lnR2) - 273.15) * 100;

  ((uint32_t*)tempData)[0] = 0x0000ff01;
  tempData[4] = (brakePressure >> 8) & 0xff;
  tempData[5] = brakePressure & 0xff;
  tempData[6] = (oilPressure >> 8) & 0xff;
  tempData[7] = oilPressure & 0xff;
  tempData[8] = (oilTemperature >> 8) & 0xff;
  tempData[9] = oilTemperature & 0xff;

  canBusMainCharacteristic.writeValue(tempData, 10, false);
}


long previousMillisCANx81 = 0;
long previousMillisCANx201 = 0;
long previousMillisCANx212 = 0;
long previousMillisCANx4b0 = 0;
long previousMillisCANx420 = 0;

/*
0081	2-3 (BE)	0xFFFF	Steering angle (FDE1 to 021E)
0201	6	0xFF	Throttle % * 2
0201	0-1 (BE)	0xFFFF	RPM * 4
0201	4-5 (BE)	0xFFFF	Vehicle speed kph * 100 + 10000
0212	4	0x40	Handbrake on
0212	5	0x8	Brake on
0212	5	0x40	DSC on
0x420  Coolant temperature
04B0	0-1 (BE)	0xFFFF	LF wheel kph * 100 + 10000
04B0	2-3 (BE)	0xFFFF	RF wheel kph * 100 + 10000
04B0	4-5 (BE)	0xFFFF	LR wheel kph * 100 + 10000
04B0	6-7 (BE)	0xFFFF	RR wheel kph * 100 + 10000
*/

void sendCANData() {
  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    int i;

    CAN.readMsgBuf(&len, cdata);

    unsigned long pid = CAN.getCanId();

    if (!(pid == 0x81 || pid == 0x201 || pid == 0x212 || pid == 0x4b0 || pid == 0x420)) return;

    long currentMillis = millis();

    switch (pid) {
      case 0x81:
        if (currentMillis - previousMillisCANx81 < 40) return;
        previousMillisCANx81 = currentMillis;
        break;
      case 0x201:
        if (currentMillis - previousMillisCANx201 < 40) return;
        previousMillisCANx201 = currentMillis;
        break;
      case 0x212:
        if (currentMillis - previousMillisCANx212 < 40) return;
        previousMillisCANx212 = currentMillis;
        break;
      case 0x4b0:
        if (currentMillis - previousMillisCANx4b0 < 40) return;
        previousMillisCANx4b0 = currentMillis;
        break;
      case 0x420:
        if (currentMillis - previousMillisCANx420 < 40) return;
        previousMillisCANx420 = currentMillis;
        break;
      default:
        return;
        break;
    }

    ((uint32_t*)tempData)[0] = pid;
    for (i = 0; i < len; i++) {
      tempData[i + 4] = cdata[i];
    }

    canBusMainCharacteristic.writeValue(tempData, len + 4, false);
  }
}


/*
  WiFi Web Server LED Blink

  A simple web server that lets you blink an LED via the web.
  This sketch will create a new access point (with no password).
  It will then launch a new server and print out the IP address
  to the Serial Monitor. From there, you can open that address in a web browser
  to turn on and off the LED on pin 13.

  If the IP address of your board is yourAddress:
    http://yourAddress/H turns the LED on
    http://yourAddress/L turns it off

  created 25 Nov 2012
  by Tom Igoe
  adapted to WiFi AP by Adafruit

  Find the full UNO R4 WiFi Network documentation here:
  https://docs.arduino.cc/tutorials/uno-r4-wifi/wifi-examples#access-point
 */

#include "WiFiS3.h"

#include "arduino_secrets.h"

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = "ARC";       // your network SSID (name)
char pass[] = "11111111";  // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;          // your network key index number (needed only for WEP)

int led = LED_BUILTIN;
int status = WL_IDLE_STATUS;
WiFiServer server(80);

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(112500);
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Access Point Web Server");

  pinMode(led, OUTPUT);  // set the LED pin mode

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true)
      ;
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // by default the local IP address will be 192.168.4.1
  // you can override it with the following:
  WiFi.config(IPAddress(192, 48, 56, 2));

  // print the network name (SSID);
  Serial.print("Creating access point named: ");
  Serial.println(ssid);

  // Create open network. Change this line if you want to create an WEP network:
  status = WiFi.beginAP(ssid, pass);
  if (status != WL_AP_LISTENING) {
    Serial.println("Creating access point failed");
    // don't continue
    while (true)
      ;
  }

  // wait 10 seconds for connection:
  delay(10000);

  // start the web server on port 80
  server.begin();

  // you're connected now, so print out the status
  printWiFiStatus();
}

static char buf1[1024];

void loop() {

  // compare the previous status to the current status
  if (status != WiFi.status()) {
    // it has changed update the variable
    status = WiFi.status();

    if (status == WL_AP_CONNECTED) {
      // a device has connected to the AP
      Serial.println("Device connected to AP");
    } else {
      // a device has disconnected from the AP, and we are back in listening mode
      Serial.println("Device disconnected from AP");
    }
  }

  WiFiClient client = server.available();  // listen for incoming clients

  if (client) {                    // if you get a client,
    Serial.println("new client");  // print a message out the serial port
    String currentLine = "";       // make a String to hold incoming data from the client
    while (client.connected()) {   // loop while the client's connected
      delay(1);                    // This is required for the Arduino Nano RP2040 Connect - otherwise it will loop so fast that SPI will never be served.               // print it out to the serial monitor

      rc3_sprintf(buf1);
      Serial.print(buf1);
      client.write(buf1);
    }
    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }
}

void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}

uint8_t nmea_checksum(const char *s) {  // s should not include the leading $-sign and the ending *-sign
  int c = 0;
  while (*s) c ^= *s++;
  return c;
}

float tempNTC = 0.0;
unsigned short count = 0;

void rc3_sprintf(char buf[]) {
  tempNTC = tempNTC + 0.1;
  if (tempNTC > 99.9) tempNTC = 0.0;

  // RaceChrono output in $RC3 format ------------------------------------------------------------------------
  /* $RC3,[time],[count],[xacc],[yacc],[zacc],[gyrox],[gyroy],[gyroz],[rpm/d1],[d2],
     [a1],[a2],[a3],[a4],[a5],[a6],[a7],[a8],[a9],[a10],[a11],[a12],[a13],[a14],[a15]*checksum
     - $ is message start character
     - RC2 and RC3 are message identifiers
     - time stamp is not used (empty). (for blended GNS support this should be a GNS synchronized realtime timestamp in NMEA 0183 format).
     - count is an overflowing line counter 0-65535. Can be left empty if GNS timestamp is provided.
     - acc fields: -1.000 = -1G, 1.000 = +1G
     - gyro fields: degrees per second, -1.000 = -1 deg/s, 1.000 = +1 deg/s
     - dx are digital channel fields, range -2000000.000 - 2000000.000
     - ax are analog channel fields, range -2000000.000 - 2000000.000
     - * is message separator character
     - NMEA 0183 type checksum, with two uppercase hexadecimal digits (one byte)
     - each line is terminated with CR plus LF
   */

  sprintf(buf, "$RC3,,%lu,,,,,,,,,%1.1f,,,,,,,,,,,,,,",
          count++, tempNTC);

  sprintf(buf + strlen(buf), "*%02X\r\n", nmea_checksum(buf + 1));
}
