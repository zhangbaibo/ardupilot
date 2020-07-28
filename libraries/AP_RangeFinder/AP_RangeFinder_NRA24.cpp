/*
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_RangeFinder_NRA24.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL &hal;

#define NRA24_FRAME_HEADER 0xAA
#define NRA24_FRAME_IDL 0x0C
#define NRA24_FRAME_IDH 0x07
#define NRA24_FRAME_LENGTH 14
#define NRA24_FRAME_END 0x55

AP_RangeFinder_NRA24::AP_RangeFinder_NRA24(
		RangeFinder::RangeFinder_State &_state,
		AP_SerialManager &serial_manager, uint8_t serial_instance) :
		AP_RangeFinder_Backend(_state) {
	uart = serial_manager.find_serial(
			AP_SerialManager::SerialProtocol_Rangefinder, serial_instance);
	if (uart != nullptr) {
		uart->begin(
				serial_manager.find_baudrate(
						AP_SerialManager::SerialProtocol_Rangefinder,
						serial_instance));
	}
}

bool AP_RangeFinder_NRA24::detect(AP_SerialManager &serial_manager,
		uint8_t serial_instance) {
	return serial_manager.find_serial(
			AP_SerialManager::SerialProtocol_Rangefinder, serial_instance)
			!= nullptr;
}

// distance returned in reading_cm, signal_ok is set to true if sensor reports a strong signal
bool AP_RangeFinder_NRA24::get_reading(uint16_t &reading_cm) {
	if (uart == nullptr) {
		return false;
	}

	float sum_cm = 0;
	uint16_t count = 0;

	// read any available lines from the lidar
	int16_t nbytes = uart->available();
	while (nbytes-- > 0) {
		int16_t r = uart->read();
		if (r < 0) {
			continue;
		}
		uint8_t c = (uint8_t) r;
		// if buffer is empty and this byte is 0x59, add to buffer
		if (linebuf_len == 0) {
			if (c == NRA24_FRAME_HEADER) {
				linebuf[linebuf_len++] = c;
			}
		} else if (linebuf_len == 1) {
			if (c == NRA24_FRAME_HEADER) {
				linebuf[linebuf_len++] = c;
			} else {
				linebuf_len = 0;
			}
		} else if (linebuf_len == 2) {
			if (c == NRA24_FRAME_IDL) {
				linebuf[linebuf_len++] = c;
			} else {
				linebuf_len = 0;
			}
		} else if (linebuf_len == 3) {
			if (c == NRA24_FRAME_IDH) {
				linebuf[linebuf_len++] = c;
			} else {
				linebuf_len = 0;
			}
		} else {
			linebuf[linebuf_len++] = c;
			if (linebuf_len == NRA24_FRAME_LENGTH) {
				// Ð£ÑéºÍ
				uint8_t checksum = 0;
				for (uint8_t i = 4; i <= 10; i++) {
					checksum += linebuf[i];
				}
				// if checksum matches extract contents
				if (checksum == (linebuf[11] & 0xFF)) {
					// calculate distance
					uint16_t dist = ((uint16_t) linebuf[6] << 8) | linebuf[7];
					sum_cm += dist;
					count++;
				}
				// clear buffer
				linebuf_len = 0;
			}
		}
	}

	if (count > 0) {
		// return average distance of readings
		reading_cm = sum_cm / count;
		return true;
	}
	// no readings so return false
	return false;
}

/* 
 update the state of the sensor
 */
void AP_RangeFinder_NRA24::update(void) {
	if (get_reading(state.distance_cm)) {
		// update range_valid state based on distance measured
		last_reading_ms = AP_HAL::millis();
		update_status();
	} else if (AP_HAL::millis() - last_reading_ms > 200) {
		set_status(RangeFinder::RangeFinder_NoData);
	}
}
