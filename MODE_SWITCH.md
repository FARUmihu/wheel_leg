# 模式切换草案

目标：用 `g_app_cmd` 做最薄的 GDB 调试入口。模式只决定“哪类命令会被执行”，不做保护、不做自动流程、不隐藏调试入口。

## 总原则

- 默认 `APP_MODE_IDLE`，不驱动任何执行器。
- `app_control_2khz()` 只处理需要实时发送的 DM 命令。
- `app_background()` 只处理阻塞或低频动作，例如飞特舵机写入/读取、IMU 单次请求。
- `g_app_obs` 只汇总可读结构体，不参与决策。
- mask/trigger 由调试者手动改；代码只按当前值执行，不擅自切模式。

## 模式定义

| 模式 | 执行位置 | 行为 |
|---|---|---|
| `APP_MODE_IDLE` | 2 kHz / background | 只刷新 `g_app_obs`，不发执行器命令 |
| `APP_MODE_DM_MANUAL` | 2 kHz | 按 `dm_send_mask` 给指定 DM 电机发送 `g_app_cmd.dm[i]` 的 MIT 帧 |
| `APP_MODE_FT_MANUAL` | background | 按 `ft_write_mask` 写飞特位置；按 `ft_read_mask` 读取飞特反馈 |
| `APP_MODE_IMU_READ` | background | 若 `imu_request_once != 0`，请求一次 IMU 数据，然后清零该 trigger |

## DM 手动模式

`APP_MODE_DM_MANUAL` 只做一件事：在 2 kHz 中断里遍历 4 个 DM 实例。

- `dm_send_mask`：bit 为 1 的电机会持续发送 MIT 命令。
- `dm_enable_mask`：建议作为“一次性使能/失能触发”的下一步扩展，不在第一版里自动反复发送。
- `dm[i].p/v/kp/kd/t`：直接对应 `dm_motor_send_mit()` 参数。

bit 位约定：

| bit | 电机 |
|---|---|
| 0 | `APP_DM_LEFT_JOINT` |
| 1 | `APP_DM_RIGHT_JOINT` |
| 2 | `APP_DM_LEFT_WHEEL` |
| 3 | `APP_DM_RIGHT_WHEEL` |

## 飞特手动模式

`APP_MODE_FT_MANUAL` 只在 `app_background()` 里执行，避免阻塞 2 kHz 中断。

- `ft_write_mask`：bit 为 1 时写一次目标位置，写完后代码清掉对应 bit。
- `ft_read_mask`：bit 为 1 时读一次反馈，读完后代码清掉对应 bit。
- `ft[i].pos/speed/acc`：直接对应飞特位置命令参数。

bit 位约定：

| bit | 舵机 |
|---|---|
| 0 | `APP_FT_LEFT` |
| 1 | `APP_FT_RIGHT` |

## IMU 读取模式

IMU 默认已经在 `app_init()` 配成主动推送。`APP_MODE_IMU_READ` 只保留给手动请求：

- `imu_request_once = 1`：请求一次欧拉角和陀螺仪数据。
- 请求发出后代码清零 `imu_request_once`。

## 第一版不做

- 不做安全状态机。
- 不做在线/离线判定。
- 不做自动失能。
- 不做日志系统。
- 不做并联腿解算。

这些后面需要时单独加，不和这个裸调试入口混在一起。
