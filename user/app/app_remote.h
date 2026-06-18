#ifndef APP_REMOTE_H
#define APP_REMOTE_H

#include <stdint.h>

typedef struct {
    int16_t vx_mm_s;
    int16_t wz_mrad_s;
    uint8_t online;
    uint32_t last_update_ms;
} app_remote_cmd_t;

extern volatile app_remote_cmd_t g_app_remote_cmd;

void app_remote_init(void);
void app_remote_background(void);
int16_t app_remote_get_vx_mm_s(void);
int16_t app_remote_get_wz_mrad_s(void);

#endif /* APP_REMOTE_H */
