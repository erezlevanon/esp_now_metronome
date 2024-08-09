
#include <esp_now.h>
#include <WiFi.h>


#define NUM_REVCIEVERS 3


// THIS SHOULD BE THE SAME AS IN THE RECIEVER!!!!!!
const float time_movement_seconds = 21.0f; // PLAYABLE

uint8_t addresses[NUM_REVCIEVERS][6] = {
  // { 0xC8, 0x2E, 0x18, 0xC3, 0xA3, 0x08 }, (transmitter)
  { 0xC8, 0x2E, 0x18, 0xC3, 0xA2, 0xD8 },
  { 0x30, 0xC9, 0x22, 0xD1, 0xC3, 0x50 },
  { 0x30, 0xC9, 0x22, 0xD1, 0xC2, 0xA8 },
};

typedef struct metronom_struct {
  int8_t direction;
  bool trigger_click;
} metronom_struct;

metronom_struct metronom;
metronom_struct prev_metronom;

esp_now_peer_info_t peerInfo;

long timer = 0;


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
  Serial.println();
  Serial.println("--------------------------");
  Serial.println("time_movement_seconds");
  Serial.println(time_movement_seconds);

  // init metronom
  metronom.direction = 1;
  metronom.trigger_click = false;

  memcpy(&prev_metronom, &metronom, sizeof(metronom));

  pinMode(2, OUTPUT);
}

template<typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

int8_t get_direction() {
    const time_now = millis()
    if (time_now - timer > (time_movement_seconds * 1000)) {
        timer = time_now;
        return -1 * prev_metronom.direction;
    }
    return prev_metronom.direction;
}

bool get_trigger() {
    return metronom.direction != prev_metronom.direction;
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void loop() {
  metronom.direction = get_direction();
  metronom.trigger_click = get_trigger();

  // if (metronom.trigger_click) {
  //   Serial.println("click");
  // }
  // Serial.print("new: ");
  // Serial.println(metronom.direction);

  // Serial.print(", prev:");
  // Serial.println(prev_metronom.direction);

  memcpy(&prev_metronom, &metronom, sizeof(metronom));
  // Serial.print("Sending: ");
  // Serial.print(metronom.direction);
  // Serial.print(", ");
  // Serial.println(metronom.trigger_click);
  esp_err_t result = esp_now_send(0, (uint8_t*)&metronom, sizeof(metronom));
  digitalWrite(2, metronom.direction > 0);
  delay(90);
}
