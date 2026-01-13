#ifndef DOUBLE_BUFFER_HPP
#define DOUBLE_BUFFER_HPP

#include <Arduino.h>
//can create double buffer of any size
template <size_t N>
class DoubleBuffer {
public:
  //sample structure containing time acceleration and capacitance
    struct Sample {
        uint32_t t_us;
        float accelZ;
        long capRaw;
    };
    //constructor with its members
    DoubleBuffer() {
        writeBuf = bufferA;
        readBuf  = bufferB;
        writeIndex = 0;
        ready = false;
    }

    // Called from real-time sampling loop
    inline void push(float accelZ, long capRaw) {
        writeBuf[writeIndex++] = {
            micros(),
            accelZ,
            capRaw
        };
        //if index gets to buffer size swap buffers
        if (writeIndex >= N) {
            swapBuffers();
        }
    }

    // Returns true if a full buffer is ready to read
    inline bool available() const {
        return ready;
    }

    // Consumer gets pointer to contiguous block
    inline const Sample* read() {
        ready = false;
        return readBuf;
    }

    constexpr size_t size() const {
        return N;
    }

private:
    Sample bufferA[N];
    Sample bufferB[N];

    Sample* writeBuf;
    Sample* readBuf;

    volatile size_t writeIndex;
    volatile bool ready;

    inline void swapBuffers() {
        Sample* tmp = writeBuf;
        writeBuf = readBuf;
        readBuf = tmp;

        writeIndex = 0;
        ready = true;
    }
};

#endif // DOUBLE_BUFFER_HPP
