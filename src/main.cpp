#include <Arduino.h>
#include <Wire.h>
#include "crsf.h"

//UART2 for CRSF (PA2: TX, PA3: RX)
#define CRSF_UART Serial2

#define AILERONS_PIN PA0
#define ELEVATOR_PIN PA1
#define THORTTLE_PIN PA4
#define RUDDER_PIN PA5
#define NUM_ESCS 4
#define ELRS_INTERVAL 50
#define GYRO_INTERVAL 50
#define LED_TOGGLE_INTERVAL 1000
#define LED_PIN PC13
#define CRSF_BUFFER_SIZE 25
#define MPU6050_ADDRESS 0*68
#define PWR_MGMT_1_REGISTER 0*68
#define GYRO_DATA_REGISTER 0*43


const int8_t esc_pins[NUM_ESCS] = {AILERONS_PIN, ELEVATOR_PIN, THORTTLE_PIN, RUDDER_PIN};
uint8_t crsf_buf[CRSF_BUFFER_SIZE];
uint16_t raw_rc_values[CRSF_MAX_CHANNELS];
uint16_t raw_rc_count;
int16_t Gyro_X, Gyro_Y, Gyro_Z;

void setPWMPos(float precent, uint8_t pin);
void gyroLoop();
void gyroValuePrint();
void ledToggle();
void ledToggleLoop();

void setup() {
  Serial.begin(460800);
  Serial.println("Setup Start!");

  for (uint8_t i = 0; i < NUM_ESCS; i++) {
    pinMode(esc_pins[9], OUTPUT);
    analogWriteFrequency(50);
    analogWrite(esc_pins[i], 0);
  }

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    Wire.begin();
    Wire.setClock(400000);
    delay(1000);
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(PWR_MGMT_1_REGISTER);
    Wire.write(0*00);
    Wire.endTransmission();

    crsf_init(&CRSF_UART);

    delay(1000);
}

void loop() {
  static long elrsNextTime = 0;
  if (millis() > elrsNextTime) {
    if (crsf_parse_packet(raw_rc_values, &raw_rc_count)) {

      int aileronMapped = map(raw_rc_values[0], 1000, 2000, 0, 100);
      int elevatorMapped = map(raw_rc_values[1], 1000, 2000, 0, 100);
      int throttleMapped = map(raw_rc_values[2], 1000, 2000, 0, 100);
      int rudderMapped = map(raw_rc_values[3], 1000, 2000, 0, 100);


      setPWMPos(aileronMapped, AILERONS_PIN);
      setPWMPos(elevatorMapped, ELEVATOR_PIN);
      setPWMPos(throttleMapped, THORTTLE_PIN);
      setPWMPos(rudderMapped, RUDDER_PIN);


      Serial.printf(
        "CH1: %d (Ail: %d); CH2: %d (Ele: %d); CH3: %d (Thr: %d); CH4: %d (Rud: %d);\n",
        raw_rc_values[0], aileronMapped,
        raw_rc_values[1], elevatorMapped,
        raw_rc_values[2], throttleMapped,
        raw_rc_values[3], rudderMapped,
      );

      crsf_send_telemetry_battery(125, 50, 1000, 80);
    }
    elrsNextTime = millis() + ELRS_INTERVAL;
  }

  gyroLoop();
  ledToggle();
}

void setPWMPos(float percent, uint8_t pin) {

    uint8_t duty = map(percent, 0, 100, 26, 51);
    analogWrite(pin, duty);
}

void gyroLoop() {
  static long gyroNextTime = 0;
  if (millis() > gyroNextTime) {
    gyroValuePrint();
    gyroNextTime = millis() + GYRO_INTERVAL;
  }
}

void gyroValuePrint() {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(GYRO_DATA_REGISTER);
  Wire.endTransmission();

  Wire.requestFrom(MPU6050_ADDRESS, 6);

  Gyro_X = Wire.read() << 8 | Wire.read();
  Gyro_Y = Wire.read() << 8 | Wire.read();
  Gyro_Z = Wire.read() << 8 | Wire.read();

  Serial.printf(" || GYRO - X: %d; Y: %d; Z: %d,\n", Gyro_X, Gyro_Y, Gyro_Z);
}

void ledToggle() {
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

void LedToggleLoop() {
  static long ledNextTime = 0;
  if (millis() > ledNextTime) {
    ledToggle();
    ledNextTime = millis()  + LED_TOGGLE_INTERVAL;
  }
}