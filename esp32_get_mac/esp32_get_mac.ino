#include "WiFi.h"


// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_MODE_STA);
  Serial.print("yo,");
  delay(500);
  Serial.println(" here is my MAC address:");
  delay(500);
  String macAddress = WiFi.macAddress();
  Serial.println(macAddress);
  if (macAddress.length() != 17) {
    Serial.print("weird mac address length: ");
    Serial.println(macAddress.length());
  }
  Serial.println("and as array:");
  Serial.print("{ ");
  for (int i = 0; i < 6; i++) {
    if (i != 0) Serial.print(", ");
    Serial.print("0x");
    Serial.print(macAddress.substring(i * 3, i * 3 + 2));
  }
  Serial.println(" },");
}

void loop() {}
