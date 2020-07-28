#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

class AP_RangeFinder_NRA24 : public AP_RangeFinder_Backend
{

public:

    AP_RangeFinder_NRA24(RangeFinder::RangeFinder_State &_state,
                            AP_SerialManager &serial_manager,
                            uint8_t serial_instance);

    // static detection function
    static bool detect(AP_SerialManager &serial_manager, uint8_t serial_instance);

    // update state
    void update(void);

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:

    bool get_reading(uint16_t &reading_cm);

    AP_HAL::UARTDriver *uart = nullptr;
    uint32_t last_reading_ms;
    uint8_t linebuf[15];
    uint8_t linebuf_len;
};
