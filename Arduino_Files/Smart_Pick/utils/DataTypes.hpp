#pragma once

//this is a sensor packet that we'll write to flash
struct SensorPacket {
    uint32_t timestamp;
    uint32_t mag_accel;
    uint32_t capacitance;
};
