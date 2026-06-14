# leg_kinematics 直接看这个

## 看哪里

看全局变量：

```c
g_app_obs.leg[APP_LEG_LEFT]
g_app_obs.leg[APP_LEG_RIGHT]
```

里面就这些：

- `ft_raw`：飞特当前位置 raw。
- `dm_rad`：DM4310 当前反馈 rad。
- `theta1_deg`：舵机角，单位 deg。
- `theta2_deg`：电机角，单位 deg。
- `foot_d_x_mm`：足端 D 的 x，单位 mm。
- `foot_d_y_mm`：足端 D 的 y，单位 mm。
- `usable`：`1` 才能信，`0` 就别拿去发命令。

## 现在怎么跑

- 工程运行后，`app_background()` 会自动刷新 `g_app_obs.leg[]`。
- 你只要让飞特/DM 有反馈，调试器里直接看 `g_app_obs.leg[]`。
- 不需要自己盯一堆解算返回值。

## 正解怎么用

角度转足端 D：

```c
leg_joint_angles_t joint = {
    .theta1_deg = 25.0f,
    .theta2_deg = -170.0f,
};
leg_kinematics_result_t r;

if (leg_kinematics_forward_from_angles(LEG_SIDE_LEFT, &joint, &r) == LEG_KINEMATICS_OK) {
    /* 看 r.foot_d.x_mm / r.foot_d.y_mm */
}
```

飞特 raw + DM rad 转足端 D：

```c
leg_actuator_state_t a = {
    .ft_raw = 0,
    .dm_rad = 0.0f,
};
leg_kinematics_result_t r;

if (leg_kinematics_forward_from_actuator(LEG_SIDE_LEFT, &a, &r) == LEG_KINEMATICS_OK) {
    /* 看 r.foot_d.x_mm / r.foot_d.y_mm */
}
```

飞特固定，只看 DM 怎么影响 D：

```c
leg_kinematics_result_t r;

if (leg_kinematics_fixed_ft_forward(LEG_SIDE_LEFT, 0, 0.0f, &r) == LEG_KINEMATICS_OK) {
    /* 看 r.foot_d.x_mm / r.foot_d.y_mm */
}
```

## 逆解怎么用

足端 D 转 `theta1/theta2`：

```c
leg_point_t d = {
    .x_mm = 5.0f,
    .y_mm = -110.0f,
};
leg_kinematics_result_t r;

if (leg_kinematics_inverse_to_angles(LEG_SIDE_LEFT, &d, &r) == LEG_KINEMATICS_OK) {
    /* 看 r.joint.theta1_deg / r.joint.theta2_deg */
}
```

足端 D 转飞特 raw + DM rad：

```c
leg_point_t d = {
    .x_mm = 5.0f,
    .y_mm = -110.0f,
};
leg_actuator_state_t a;
leg_kinematics_result_t r;

if (leg_kinematics_inverse_to_actuator(LEG_SIDE_LEFT, &d, &a, &r) == LEG_KINEMATICS_OK) {
    /* 看 a.ft_raw / a.dm_rad */
}
```

上面那个 `== LEG_KINEMATICS_OK` 只当闸门用。进不去就别用结果，更别发硬件。

## 映射

- 飞特 3：`raw=0 -> theta1=25 deg`，`raw=938 -> theta1=-180 deg`。
- 飞特 0：现在先按飞特 3 一样算。
- 左腿 DM：`0 deg -> theta2=-170 deg`，`125 deg -> theta2=-45 deg`。
- 右腿 DM：`0 deg -> theta2=-45 deg`，`125 deg -> theta2=-170 deg`。

## 别干这些

- `usable=0` 时，不要拿 `D/theta1/theta2` 去驱动硬件。
- 舵机 0 安全范围没确认前，不要让它按逆解结果跑。
- 不要一上来同时驱动整条腿。
- 先固定飞特，只测 DM。
- 这个模块现在只回填观测量，不会自动控制电机。
