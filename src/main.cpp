#include <Arduino.h>

// LED
#define LED_TOGGLE_INTERVAL 1000
void ledToggle();
void ledToggleLoop();

// GYRO
#include <Wire.h>
#define GYRO_INTERVAL 50
#define PWR_MGMT_1_REGISTER 0x6B
#define GYRO_DATA_REGISTER 0x43
int16_t Gyro_X, Gyro_Y, Gyro_Z;
void gyroLoop();
void gyroValuePrint();

// ELRS
#include <crsf.h>
#define ELRS_INTERVAL 50
#define RXD2 16
#define TXD2 17
#define SBUS_BUFFER_SIZE 25
uint8_t _rcs_buf[25] {};
uint16_t _raw_rc_values[RC_INPUT_MAX_CHANNELS] {};
uint16_t _raw_rc_count{};
HardwareSerial Serial3(PB11, PB10);
void elrsLoop();

// PWM
void setPWMPosAll(int const aileronsMapped, int const elevatorMapped, int const throttleMapped, int const rudderMapped, int const switchMapped);
void pwmSetup();
void setPWMPos(float percent, int pwmChannel);

// =====================================

void setup() {
  Serial.begin(460800);
  Serial.println("Setup Start!");
  Serial3.begin(420000, SERIAL_8N1);

  // LED
  pinMode(PC13, OUTPUT);

  // GYRO
  Wire.begin();
  Wire.setClock(400000);
  delay(250);

  Wire.beginTransmission(0x68);
  Wire.write(PWR_MGMT_1_REGISTER);
  Wire.write(0x00); // write this to PWR_MGMT_1_REGISTER, to activate Gyro
  Wire.endTransmission();
}

void loop() {
  gyroLoop();
  elrsLoop();
  ledToggleLoop();
}

// =====================================

// ELRS ----------------------------
long elrsNextTime = 0;

void elrsLoop() {
  if (Serial3.available() && (millis() > elrsNextTime)) {
    size_t numBytesRead = Serial3.readBytes(_rcs_buf, SBUS_BUFFER_SIZE);
    if(numBytesRead > 0) {
      crsf_parse(&_rcs_buf[0], SBUS_BUFFER_SIZE, &_raw_rc_values[0], &_raw_rc_count, RC_INPUT_MAX_CHANNELS );

      int aileronsMapped = map(_raw_rc_values[0], 1000, 2000, 0, 100);
      int elevatorMapped = map(_raw_rc_values[1], 1000, 2000, 0, 100);
      int throttleMapped = map(_raw_rc_values[2], 1000, 2000, 0, 100);
      int rudderMapped = map(_raw_rc_values[3], 1000, 2000, 0, 100);
      int switchMapped = map(_raw_rc_values[4], 1000, 2000, 0, 100);

      Serial.printf(
        "CH1: %d (Ail: %d); CH2: %d (Ele: %d); CH3: %d (Thr: %d); CH4: %d (Rud: %d);  CH5: %d (Swt: %d);",
        _raw_rc_values[0], aileronsMapped,
        _raw_rc_values[1], elevatorMapped,
        _raw_rc_values[2], throttleMapped,
        _raw_rc_values[3], rudderMapped,
        _raw_rc_values[4], switchMapped
      );

      setPWMPosAll(aileronsMapped, elevatorMapped, throttleMapped, rudderMapped, switchMapped);
    }

    elrsNextTime = millis() + ELRS_INTERVAL;
  }
}

// GYRO ----------------------------
long gyroNextTime = 0;

void gyroLoop() {
  if (millis() > gyroNextTime) {
    gyroValuePrint();
    gyroNextTime = millis() + GYRO_INTERVAL;
  }
}

void gyroValuePrint() {
  Wire.beginTransmission(0x68);
  Wire.write(GYRO_DATA_REGISTER); // in the datasheet, register 0x43 is where gyro data starts
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6); // request to start reading from the already set GYRO_DATA_REGISTER, as much as 6 bytes
  Gyro_X = Wire.read() << 8 | Wire.read();
  Gyro_Y = Wire.read() << 8 | Wire.read();
  Gyro_Z = Wire.read() << 8 | Wire.read();
  Serial.printf(" || GYRO - X: %d; Y: %d; Z: %d;\n", Gyro_X, Gyro_Y, Gyro_Z);
}

// LED ----------------------------
long ledNextTime = 0;

void ledToggle() {
  digitalWrite(PC13, !digitalRead(PC13));
  // Serial.println("toggled!");
}

void ledToggleLoop() {
  if (millis() > ledNextTime) {
    ledToggle();
    ledNextTime = millis() + LED_TOGGLE_INTERVAL;
  }
}

// PWM ----------------------------
int aileronsPin = 12;
int elevatorPin = 13;
int throttlePin = 14;
int rudderPin = 15;

int aileronsPWMChannel = 1;
int elevatorPWMChannel = 2;
int throttlePWMChannel = 3;
int rudderPWMChannel = 4;
int switchPWMChannel = 5;

void setPWMPos(float percent, int pwmChannel)
{
    // 50 cycles per second 1,000ms / 50 = 100 /5 = 20ms per cycle
    // 1ms / 20ms = 1/20 duty cycle
    // 2ms / 20ms = 2/20 = 1/10 duty cycle
    // using 16 bit resolution for PWM signal convert to range of 0-65536 (0-100% duty/on time)
    // 1/20th of 65536 = 3276.8
    // 1/10th of 65536 = 6553.6

    uint32_t duty = map(percent, 0, 100, 3276.8, 6553.6);

    // ledcWrite(pwmChannel, duty); // this is still code for ESP32
}

void pwmSetup() {
  // // this is still code for ESP32____start
  // ledcSetup(aileronsPWMChannel,50,16);
  // ledcSetup(elevatorPWMChannel,50,16);
  // ledcSetup(throttlePWMChannel,50,16);
  // ledcSetup(rudderPWMChannel,50,16);

  // ledcAttachPin(aileronsPin, aileronsPWMChannel);
  // ledcAttachPin(elevatorPin, elevatorPWMChannel);
  // ledcAttachPin(throttlePin, throttlePWMChannel);
  // ledcAttachPin(rudderPin, rudderPWMChannel);
  // // this is still code for ESP32____end
}

void setPWMPosAll(int const aileronsMapped, int const elevatorMapped, int const throttleMapped, int const rudderMapped, int const switchMapped) {
  setPWMPos(aileronsMapped, aileronsPWMChannel);
  setPWMPos(elevatorMapped, elevatorPWMChannel);
  setPWMPos(throttleMapped, throttlePWMChannel);
  setPWMPos(rudderMapped, rudderPWMChannel);
  setPWMPos(switchMapped, switchPWMChannel);
}