#include "app_remote.h"

#include "app_config.h"
#include "remote_esp32.h"

volatile app_remote_cmd_t g_app_remote_cmd;

static int16_t app_remote_clamp_i16(int16_t value, int16_t min_value, int16_t max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

void app_remote_init(void)
{
    g_app_remote_cmd.vx_mm_s = 0;
    g_app_remote_cmd.wz_mrad_s = 0;
    g_app_remote_cmd.online = 0U;
    g_app_remote_cmd.last_update_ms = 0U;
}

void app_remote_background(void)
{
    remote_esp32_background();

    if (remote_esp32_is_online(APP_REMOTE_TIMEOUT_MS) == 0U) {
        g_app_remote_cmd.vx_mm_s = 0;
        g_app_remote_cmd.wz_mrad_s = 0;
        g_app_remote_cmd.online = 0U;
        return;
    }

    g_app_remote_cmd.vx_mm_s =
        app_remote_clamp_i16(g_remote_esp32.vx_mm_s,
                             (int16_t)-APP_REMOTE_MAX_VX_MM_S,
                             (int16_t)APP_REMOTE_MAX_VX_MM_S);
    g_app_remote_cmd.wz_mrad_s =
        app_remote_clamp_i16(g_remote_esp32.wz_mrad_s,
                             (int16_t)-APP_REMOTE_MAX_WZ_MRAD_S,
                             (int16_t)APP_REMOTE_MAX_WZ_MRAD_S);
    g_app_remote_cmd.online = 1U;
    g_app_remote_cmd.last_update_ms = g_remote_esp32.last_update_ms;
}

int16_t app_remote_get_vx_mm_s(void)
{
    return g_app_remote_cmd.vx_mm_s;
}

int16_t app_remote_get_wz_mrad_s(void)
{
    return g_app_remote_cmd.wz_mrad_s;
}
