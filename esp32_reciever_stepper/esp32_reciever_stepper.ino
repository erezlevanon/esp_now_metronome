#include <esp_now.h>
#include <WiFi.h>
#include <AccelStepper.h>


// Define click.
#define CLICK_PIN 21
#define CLICK_DURATION_MS 50

// Stepper definitions
#define STEPPER_IN_1 19
#define STEPPER_IN_2 18
#define STEPPER_IN_3 5
#define STEPPER_IN_4 17

const float steps_per_revolution = 2038.0f;
const float angle_fraction_of_circle = 0.333f;
const int full_movement_steps = steps_per_revolution * angle_fraction_of_circle;
const int one_side_movement = full_movement_steps / 2;
// Should match code in the transmitter.
const uint8_t min_recieved_pos = 0;
const uint8_t max_recieved_pos = 255;

// DO NOT TOUCH - data structure defined also in transmitter.
typedef struct metronom_struct {
  uint8_t position;
  int8_t direction;
  bool trigger_click;
} metronom_struct;

metronom_struct metronom;

// Stepper definition
AccelStepper stepper(AccelStepper::FULL4WIRE, STEPPER_IN_1, STEPPER_IN_3, STEPPER_IN_2, STEPPER_IN_4);

// Variables to manage click, do not touch.
bool click_now = false;
long click_start = 0;

// Callback function that will be executed when data is received.
void OnDataRecv(const esp_now_recv_info *r_info, const unsigned char *incomingData, int len) {
  memcpy(&metronom, incomingData, sizeof(metronom));
  const int target = map(metronom.position, min_recieved_pos, max_recieved_pos, -one_side_movement, one_side_movement);
  stepper.moveTo(target);
  click_now = metronom.trigger_click;
  if (click_now) {
    click_start = millis();
    digitalWrite(CLICK_PIN, HIGH);
  }
  // Serial.println(target);
}

void setup() {
  // Setup serial
  Serial.begin(115200);

  // Setup stepper
  pinMode(STEPPER_IN_1, OUTPUT);
  pinMode(STEPPER_IN_2, OUTPUT);
  pinMode(STEPPER_IN_3, OUTPUT);
  pinMode(STEPPER_IN_4, OUTPUT);
  stepper.setMaxSpeed(500.0);
  stepper.setAcceleration(500.0);

  // Set home position for stepper motor.
  set_home();

  // Set click
  pinMode(CLICK_PIN, OUTPUT);
  digitalWrite(CLICK_PIN, LOW);


  //Init ESP-NOW
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
}

void set_home() {
  // TODO: Put here the code for moving the arm.
  stepper.setCurrentPosition(0);
}

void loop() {
  if ((millis() - click_start) > CLICK_DURATION_MS) {
    digitalWrite(CLICK_PIN, LOW);
    click_now = false;
  }
  if (stepper.distanceToGo() != 0) {
    stepper.run();
  }
}
