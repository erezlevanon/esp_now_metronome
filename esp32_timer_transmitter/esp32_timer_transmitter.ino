
#include <esp_now.h>
#include <WiFi.h>


#define NUM_REVCIEVERS 3

const float steps_per_revolution = 2038.0f;
const float angle_fraction_of_circle = 0.333f;
const int full_movement_steps = steps_per_revolution * angle_fraction_of_circle;
const int one_side_movement = full_movement_steps / 2;

const float bpm = 60.0f;
const float click_duration_ms = (60.0f / bpm) * 1000.0f;
const float half_circle = PI;
const float denominator = click_duration_ms * half_circle;


uint8_t addresses[NUM_REVCIEVERS][6] = {
  // { 0xC8, 0x2E, 0x18, 0xC3, 0xA3, 0x08 }, (transmitter)
  { 0xC8, 0x2E, 0x18, 0xC3, 0xA2, 0xD8 },
  { 0x30, 0xC9, 0x22, 0xD1, 0xC3, 0x50 },
  { 0x30, 0xC9, 0x22, 0xD1, 0xC2, 0xA8 },
};

typedef struct metronom_struct {
  int position;
  int8_t direction;
  bool trigger_click;
} metronom_struct;

metronom_struct metronom;
metronom_struct prev_metronom;

esp_now_peer_info_t peerInfo;


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
  Serial.println("one_side_movement");
  Serial.println(one_side_movement);
  Serial.println("bpm");
  Serial.println(bpm);
  Serial.println("click_duration_ms");
  Serial.println(click_duration_ms);
  Serial.println("half_circle");
  Serial.println(half_circle);
  Serial.println("denominator");
  Serial.println(denominator);

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

float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int get_position() {
  return (int) fmap(sin(millis() / denominator), -1.0, 1.0, -one_side_movement, one_side_movement);
}

void loop() {
  // put your main code here, to run repeatedly:
  metronom.position = get_position();
  metronom.direction = get_direction();
  metronom.trigger_click = get_trigger();

  // if (metronom.trigger_click) {
  //   Serial.println("click");
  // }
  // Serial.print("new: ");
  // Serial.println(metronom.position);

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
  analogWrite(2, map(metronom.position, -one_side_movement, one_side_movement, 0, 255));
  delay(50);
}
