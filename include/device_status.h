#ifndef __DEVICE_STATUS_H__
#define __DEVICE_STATUS_H__
#include <stdint.h>
#ifndef AUTO_VERSION
#warning "AUTO_VERSION was not defined by the generator!"
#define AUTO_VERSION 0xdeadbeef
#endif
#ifndef FW_PROJECT_IS_DIRTY
#warning "FW_PROJECT_IS_DIRTY was not defined by the generator!"
#define FW_PROJECT_IS_DIRTY 1
#endif
#ifndef FW_PROJECT_IS_MAIN_OR_MASTER
#warning "FW_PROJECT_IS_MAIN_OR_MASTER was not defined by the generator!"
#define FW_PROJECT_IS_MAIN_OR_MASTER 0
#endif

// ideally little endian bc teensy
typedef struct device_status_t
{
    const uint32_t firmware_version = AUTO_VERSION;
    uint16_t on_time_seconds = 0;
    const bool project_on_main_or_master = FW_PROJECT_IS_MAIN_OR_MASTER;
    const bool project_is_dirty = FW_PROJECT_IS_DIRTY;
} device_status_t;
#endif