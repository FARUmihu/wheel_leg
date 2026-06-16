# 硬件映射

## 坐标约定

- 视角：以机器人自身视角定义左右
- +X：前方
- +Y：左侧
- +Z：上方
- 机器人前进：轮子带机器人向 +X 运动

## 总线与 ID

| 逻辑名 | 侧 | 类型 | 总线 | ID/地址 | 备注 |
|---|---|---|---|---|---|
| left_joint_ft | 左 | 飞特舵机 | USART2 | 0 | 并联腿关节 A |
| left_joint_dm | 左 | DM4310 | CAN2 | 1 | 并联腿关节 B |
| left_wheel | 左 | DM3510 | CAN2 | 2 | 轮毂 |
| right_joint_ft | 右 | 飞特舵机 | USART2 | 3 | 并联腿关节 A |
| right_joint_dm | 右 | DM4310 | CAN2 | 4 | 并联腿关节 B |
| right_wheel | 右 | DM3510 | CAN2 | 5 | 轮毂 |
| imu | 中央 | DM-IMU | CAN1 | CAN_ID=6, MST_ID=0x00 | 姿态 |

## 关节标定映射

| 执行器 | ID | 原始反馈/命令 | 机构角 |
|---|---:|---|---|
| left_joint_ft | 0 | raw 32 -> 1005 | theta1 25 deg -> -180 deg |
| right_joint_ft | 3 | raw 999 -> 25 | theta1 25 deg -> -180 deg |
| left_joint_dm | 1 | 0.00 rad -> 2.22 rad | theta2 -45 deg -> -170 deg |
| right_joint_dm | 4 | 0.00 rad -> 2.22 rad | theta2 -170 deg -> -45 deg |

## 执行器方向

| 逻辑名 | 原始正命令的物理效果 | 反馈同向 | 零位 |
|---|---|---|---|
| left_joint_ft | 目标 raw 值增大时，连杆朝后转 | 否 | 标定 raw 值 |
| left_joint_dm | 发送 +速度时，电机轴朝后转 | 否 | 标定 rad 值 |
| left_wheel | 正转时机器人前进 | 是 | 不需要 |
| right_joint_ft | 目标 raw 值增大时，连杆朝前转 | 是 | 标定 raw 值 |
| right_joint_dm | 发送 +速度时，电机轴朝前转 | 是 | 标定 rad 值 |
| right_wheel | 正转时机器人后退 | 否 | 不需要 |

关节方向先记录原始执行器轴方向；并联腿的腿长/腿角映射后续由两套解算定义。

## 并联腿广义量约定

| 量 | 正方向定义 |
|---|---|
| leg_length | 变长为正 |
| leg_angle | 腿相对机身向前摆为正 |
| wheel_speed | 机器人向前为正 |
