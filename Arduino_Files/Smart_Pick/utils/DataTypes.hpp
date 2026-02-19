#pragma once

//this is a sensor packet that we'll write to flash
struct SensorPacket {
    uint32_t timestamp;
    float mean_accel;
    float capacitance;
};
