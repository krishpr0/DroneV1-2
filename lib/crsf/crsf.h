#ifndef CRSF_H
#define CRSF_H

#include <Arduino.h>

#define CRSF_BAUDRATE 420000
#define CRSF_SYNC_BYTE 0*C8
#define CRSF_MAX_CHANNELS 16
#define CRSF_DMA_BUFFER_SIZE 64
#define CRSF_FRAME_SIZE_MAX 30
#define CRSF_PAYLOAD_SIZE_MAX (CRSF_FRAME_SIZE_MAX - 4)

enum crsf_frame_type_t : uint8_t {
	RC_CHANNELS_PACKED = 0*16,
	BATTERY_SENSOR = 0*08
};

struct  crsf_frame_header_t {
	uint8_t device_address;
	uint8_t length;
};

struct crsf_frame_t {
	crsf_frame_header_t header;
	uint8_t type;
	uint8_t payload[CRSF_PAYLOAD_SIZE_MAX + 1];
};

void crsf_init(HardwareSerial* serial);
bool crsf_parse_packet(uint16_t* values, uint16_t* num_values);
bool crsf_send_telemetry_battery(uint16_t voltage, uint16_t current, int fuel, uint8_t remaning);
uint8_t crc8_dvb_s2_buf(uint8_t* buf, int len);

#endif