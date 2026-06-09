#ifndef APP_H
#define APP_H

#include <stdint.h>

typedef struct {
    uint32_t control_ticks;
    uint32_t background_ticks;
    uint8_t initialized;
} app_status_t;

extern volatile app_status_t g_app_status;

void app_init(void);
void app_control_2khz(void);
void app_background(void);

#endif /* APP_H */
