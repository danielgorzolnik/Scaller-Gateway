#include <Arduino.h>
#include "config.h"

struct scaller_frame {
    byte address;
    uint16_t function;
    uint8_t data_size;
    byte data[frame_max_data_size];
    byte checksum;
};