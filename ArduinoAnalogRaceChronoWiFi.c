#include "WiFiS3.h"

char ssid[] = "ARC";
char pass[] = "11111111";
int keyIndex = 0;

int status = WL_IDLE_STATUS;
WiFiServer server(80);

static char buf1[1024];

void setup() {
  Serial.begin(500000);
  while (!Serial)
    ;

  Serial.println("Access Point Web Server");

  analogReadResolution(14);

  pinMode(LED_BUILTIN, OUTPUT);

  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    while (true)
      ;
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  WiFi.config(IPAddress(192, 48, 56, 2));

  Serial.print("Creating access point named: ");
  Serial.println(ssid);

  status = WiFi.beginAP(ssid, pass);
  if (status != WL_AP_LISTENING) {
    Serial.println("Creating access point failed");
    while (true)
      ;
  }

  server.begin();

  printWiFiStatus();
}

void loop() {
  if (status != WiFi.status()) {
    status = WiFi.status();

    if (status == WL_AP_CONNECTED) {
      Serial.println("Device connected to AP");
    } else {
      Serial.println("Device disconnected from AP");
    }
  }

  WiFiClient client = server.available();

  if (client) {
    long nextTime = micros() + 10000;
    Serial.println("new client");
    while (client.connected()) {
      long currentMillis = micros();

      if (nextTime - currentMillis > 10000) {
        delayMicroseconds(5000);
        continue;
      }

      if (nextTime - currentMillis > 0) {
        delayMicroseconds(nextTime - currentMillis);
      }

      nextTime += 50000;

      rc2_sprintf(buf1);
      client.write(buf1);
    }

    client.stop();
    Serial.println("client disconnected");
  }
}

void printWiFiStatus() {
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}

uint8_t nmea_checksum(const char *s) {
  int c = 0;
  while (*s) c ^= *s++;
  return c;
}

const float AVG_COUNT = 50;
const float VCC = 4.67;

unsigned short count = 0;



void rc2_sprintf(char buf[]) {
  int i;
  float brakePressureV = 0;
  float oilPressureV = 0;
  float oilTemperatureV = 0;

  for (i = 0; i < AVG_COUNT; i++) {
    brakePressureV += analogRead(A0) - analogRead(A1);
    oilPressureV += analogRead(A2);
    oilTemperatureV += analogRead(A3);
  }

  float brakePressure = (brakePressureV * VCC / AVG_COUNT / 16383.0 - 0.5) / 0.027;

  float oilPressure = (oilPressureV * VCC / AVG_COUNT / 16383.0 - 0.5) / 0.4;

  float V2 = oilTemperatureV * VCC / AVG_COUNT / 16383.0;
  float lnR2 = log(V2 * 330.0 / (VCC - V2));
  float oilTemperature = 1 / (1.294333144e-3 + 2.602193987e-4 * lnR2 + 1.738527465e-7 * lnR2 * lnR2 * lnR2) - 273.15;

  sprintf(buf, "$RC2,,%lu,,,,,,%1.2f,%1.2f,%1.2f,,,,,", count++, brakePressure, oilPressure, oilTemperature);
  sprintf(buf + strlen(buf), "*%02X\r\n", nmea_checksum(buf + 1));
}
