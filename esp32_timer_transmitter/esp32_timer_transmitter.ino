
#include <esp_now.h>
#include <WiFi.h>


#define NUM_REVCIEVERS 4

uint8_t addresses[NUM_REVCIEVERS][6] = {
  // { 0x48, 0xE7, 0x29, 0x8C, 0x2F, 0xC4 }, (transmitter)
  { 0x48, 0xE7, 0x29, 0x8C, 0x31, 0xBC },
  { 0x64, 0xB7, 0x08, 0xCA, 0xC3, 0xA0 },
  { 0xE4, 0x65, 0xB8, 0x0F, 0xD7, 0x54 },
  { 0x10, 0x52, 0x1C, 0x72, 0x9C, 0x58 },
  //{ 0x30, 0xC9, 0x22, 0xD1, 0xC2, 0xA8 },
};

typedef struct metronom_struct {
  uint8_t position;
  int8_t direction;
  bool trigger_click;
} metronom_struct;

metronom_struct metronom;
metronom_struct prev_metronom;

esp_now_peer_info_t peerInfo;

unsigned long send_val;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  delay(1000);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  Serial.println("initialized esp now");

  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  for (int i = 0; i < NUM_REVCIEVERS; i++) {
    Serial.print("initializing peer: ");
    Serial.println(i);
    memcpy(peerInfo.peer_addr, addresses[i], 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add peer");
      return;
    }
  }
  Serial.println("esp-now Setup done.");

  // init metronom
  metronom.direction = 1;
  metronom.position = 0;
  metronom.trigger_click = false;

  memcpy(&prev_metronom, &metronom, sizeof(metronom));

  pinMode(2, OUTPUT);
}

template<typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

int8_t get_direction() {
  if (metronom.direction > 0) {
    if (metronom.position < prev_metronom.position) return -1;
  } else if (metronom.position > prev_metronom.position) {
    return 1;
  }
  return metronom.direction;
}

bool get_trigger() {
  return metronom.position != 0 && prev_metronom.position == 0 || metronom.position != 255 && prev_metronom.position == 255;
}

void loop() {
  // put your main code here, to run repeatedly:
  metronom.position = (uint8_t)((sin(millis() / 2000.0f) + 1.0) / 2.0 * 255.9);
  metronom.direction = get_direction();
  metronom.trigger_click = get_trigger();

  // Serial.print("new: ");
  // Serial.print(metronom.position);
  // Serial.print(", prev:");
  // Serial.println(prev_metronom.position);

  memcpy(&prev_metronom, &metronom, sizeof(metronom));
  // Serial.print("Sending: ");
  // Serial.print(metronom.position);
  // Serial.print(", ");
  // Serial.print(metronom.direction);
  // Serial.print(", ");
  // Serial.println(metronom.trigger_click);
  esp_err_t result = esp_now_send(0, (uint8_t*)&metronom, sizeof(metronom));
  analogWrite(2, metronom.position);
  delay(40);
}
