#ifndef LOG_FORMAT_H
#define LOG_FORMAT_H

#include "data.h"
#include <stdint.h>

struct log_frame
{
    int64_t log_timestamp;

    struct imu_data imu;
    struct baro_data baro;
};

#endif
