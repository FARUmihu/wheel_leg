#ifndef APP_H
#define APP_H

#include <stdint.h>

#define APP_DM_COUNT  4
#define APP_FT_COUNT  2
#define APP_LEG_COUNT 2

typedef enum {
    APP_MODE_IDLE = 0,
    APP_MODE_DM_MANUAL,
    APP_MODE_FT_MANUAL,
    APP_MODE_IMU_READ,
} app_mode_t;

typedef enum {
    APP_DM_LEFT_JOINT = 0,
    APP_DM_RIGHT_JOINT,
    APP_DM_LEFT_WHEEL,
    APP_DM_RIGHT_WHEEL,
} app_dm_index_t;

typedef enum {
    APP_FT_LEFT = 0,
    APP_FT_RIGHT,
} app_ft_index_t;

typedef enum {
    APP_LEG_LEFT = 0,
    APP_LEG_RIGHT,
} app_leg_index_t;

typedef struct {
    uint32_t control_ticks;
    uint32_t background_ticks;
    uint8_t initialized;
} app_status_t;

typedef struct {
    float p;
    float v;
    float kp;
    float kd;
    float t;
} app_dm_cmd_t;

typedef struct {
    uint16_t pos;
    uint16_t speed;
    uint8_t acc;
} app_ft_cmd_t;

typedef struct {
    uint8_t mode;
    uint8_t dm_send_mask;
    uint8_t dm_enable_mask;
    uint8_t ft_write_mask;
    uint8_t ft_read_mask;
    uint8_t imu_request_once;
    app_dm_cmd_t dm[APP_DM_COUNT];
    app_ft_cmd_t ft[APP_FT_COUNT];
} app_cmd_t;

typedef struct {
    uint8_t err;
    float pos;
    float vel;
    float vel_filtered;
    float torque;
    int8_t temp_mos;
    int8_t temp_rotor;
} app_dm_obs_t;

typedef struct {
    int16_t pos;
    int16_t speed;
    int16_t load;
    uint8_t voltage;
    uint8_t temp;
} app_ft_obs_t;

typedef struct {
    float pitch;
    float roll;
    float yaw;
    float gyro[3];
    float accel[3];
    uint32_t rx_count;
    uint32_t can_rx_count;
    uint32_t tx_count;
    uint32_t tx_error_count;
    uint32_t last_update_ms;
    uint32_t last_can_id;
    uint8_t online;
    uint8_t last_type;
    uint8_t last_tx_status;
} app_imu_obs_t;

typedef struct {
    uint16_t ft_raw;
    float dm_rad;
    float theta1_deg;
    float theta2_deg;
    float foot_d_x_mm;
    float foot_d_y_mm;
    uint8_t usable;
} app_leg_obs_t;

typedef struct {
    float pitch;
    float roll;
    float yaw;
    float gyro[3];
    float accel[3];
    uint8_t online;
    uint8_t zeroed;
} app_body_attitude_obs_t;

typedef struct {
    app_dm_obs_t dm[APP_DM_COUNT];
    app_ft_obs_t ft[APP_FT_COUNT];
    app_imu_obs_t imu;
    app_body_attitude_obs_t body;
    app_leg_obs_t leg[APP_LEG_COUNT];
} app_obs_t;

extern volatile app_status_t g_app_status;
extern volatile app_cmd_t g_app_cmd;
extern volatile app_obs_t g_app_obs;

void app_init(void);
void app_control_2khz(void);
void app_background(void);
uint8_t app_imu_zero_current(void);
void app_imu_zero_clear(void);

#endif /* APP_H */
