# wheel_leg 首次开发对话归档

## 项目目标

为一个双足轮腿机器人搭建 STM32F407 裸机控制工程。当前优先级不是做复杂保护、日志或安全状态机，而是先得到一个“裸但五脏俱全”的工程：

- 可以用 GDB 读写全局结构体。
- 可以单独测试模块。
- 可以快速替换控制/运动学模块。
- 代码入口清楚，避免隐藏逻辑和过度融合。

## 机器人硬件拓扑

机器人为双足轮腿结构，每条腿包含：

- 1 个飞特舵机关节。
- 1 个达妙 DM4310 关节。
- 1 个达妙 DM3510 轮毂。

全车共 6 个执行器。

坐标约定见 `mapping.md`：

- 机器人自身视角定义左右。
- +X 为前方。
- +Y 为左侧。
- +Z 为上方。
- 机器人前进为轮子带机器人向 +X 运动。

## ID 与方向约定

当前约定已写入 `mapping.md` 和部分源码常量。

| 逻辑名 | 类型 | 总线 | ID/地址 |
|---|---|---|---|
| `left_joint_ft` | 飞特舵机 | USART1 | 0 |
| `left_joint_dm` | DM4310 | CAN2 | 1 |
| `left_wheel` | DM3510 | CAN2 | 2 |
| `right_joint_ft` | 飞特舵机 | USART1 | 3 |
| `right_joint_dm` | DM4310 | CAN2 | 4 |
| `right_wheel` | DM3510 | CAN2 | 5 |
| `imu` | DM-IMU | CAN1 | `CAN_ID=6`, `MST_ID=0x00` |

方向约定：

- 左飞特 raw 增大时，连杆朝后转；反馈反向。
- 左 DM4310 发 +速度时，电机轴朝后转；反馈反向。
- 右侧与左侧相反，反馈同向。
- 左轮正转时机器人前进，同向。
- 右轮正转时机器人后退，反向。

并联腿是异构并联腿，后续会需要两套解算。当前不急着写解算，关节方向只作为原始执行器方向记录。

## 已建立的文档

- `NEXT_STEPS.md`：简要开发步骤。
- `mapping.md`：硬件映射、ID、方向、广义量约定。
- `MODE_SWITCH.md`：裸调试模式切换草案。
- `first.md`：本次对话归档。

## 当前代码状态

### `user/app`

已新增 `user/app/app.h` 和 `user/app/app.c`。

当前全局变量：

- `volatile app_status_t g_app_status`
  - `control_ticks`
  - `background_ticks`
  - `initialized`

- `volatile app_cmd_t g_app_cmd`
  - `mode`
  - `dm_send_mask`
  - `dm_enable_mask`
  - `ft_write_mask`
  - `ft_read_mask`
  - `imu_request_once`
  - `dm[4]`：每个 DM 的 `p/v/kp/kd/t`
  - `ft[2]`：每个飞特的 `pos/speed/acc`

- `volatile app_obs_t g_app_obs`
  - 汇总 DM、飞特、IMU 的当前快照，方便 GDB 查看。

`g_dm_motors[4]` 已从 `main.c` 移到 `app.c`。

### `main.c`

`main.c` 已变薄：

- 外设初始化后调用 `app_init()`。
- 启动 `HAL_TIM_Base_Start_IT(&htim6)`。
- `while(1)` 只调用 `app_background()`。

### TIM6 入口

`HAL_TIM_PeriodElapsedCallback()` 目前在 `app.c`：

- 只在 `TIM6` 中断时调用 `app_control_2khz()`。
- `app_control_2khz()` 当前会累加 `control_ticks`。

## 模式切换现状

当前已实现 `MODE_SWITCH.md` 中的薄模式接口：

- `APP_MODE_IDLE`
  - 不发执行器命令，只刷新 `g_app_obs`。

- `APP_MODE_DM_MANUAL`
  - 在 2 kHz 中断里按 `dm_send_mask` 持续发送 `dm_motor_send_mit()`。

- `APP_MODE_FT_MANUAL`
  - 在 `app_background()` 里按 `ft_write_mask` 写一次飞特位置，按 `ft_read_mask` 读一次反馈，执行后清 bit。

- `APP_MODE_IMU_READ`
  - 在 `app_background()` 里按 `imu_request_once` 请求一次 IMU 数据，然后清零。

重要备注：用户指出 `APP_MODE_DM_MANUAL` 和 `APP_MODE_FT_MANUAL` 目前更像预留接口。对于装配好的机器人，不能随便一起修改所有电机状态；真正有意义的模式要等运动学解算和实机控制指令建立后再重构。

## 当前设计原则

- 不提前写复杂安全、保护、日志、在线判断。
- 不把保护逻辑和调试入口揉在一起。
- 先保留清楚、可读、可移植的裸接口。
- 后续模块替换应尽量集中在 `user/app`、`user/controller`、`user/algorithm`。
- `Core/` 里只保留必要 CubeMX USER CODE 调用。

## 已验证

使用命令：

```sh
mingw32-make -C Debug all -j8
```

最近一次构建通过。最近一次构建大小：

```text
text=23332 data=12 bss=2500
```

注意：构建会更新 `Debug/` 下的 `.o/.elf/.map/.list` 等产物。

## 下次继续建议

不要急着加保护和日志。建议下一步先重新审视模式接口：

1. 保留 `g_app_cmd/g_app_obs` 作为 GDB 调试入口。
2. 暂时不要扩大 `DM_MANUAL/FT_MANUAL` 的实机使用范围。
3. 下一阶段更适合做：
   - 并联腿运动学接口定义。
   - 左/右腿单腿测试命令结构体。
   - 将“执行器原始命令”升级为“模块级测试命令”，例如单腿、单轮、IMU、飞特标定。

这份归档的核心结论：当前代码已经有裸工程框架和调试全局变量，但真正的实机调试模式应在运动学和控制命令明确后再收敛。
