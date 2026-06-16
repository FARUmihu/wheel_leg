# leg_kinematics 坐标系说明

本文只解释当前 `leg_kinematics.c` 里的坐标定义和换算关系。这里的坐标是机构平面内的解算坐标，单位是 mm，不是 IMU 姿态坐标，也不是整车导航坐标。

## 1. 解算平面坐标系

运动学模块使用一个二维平面坐标系：

- `x_mm`：水平轴，向右为正。
- `y_mm`：竖直轴，向上为正。
- 足端点 `D` 往下运动时，`y_mm` 通常是负数。
- 左腿和右腿共用同一套轴向定义，只是固定铰点 `O2` 在 x 方向镜像。

代码里的两个固定铰点是：

```c
O1 = (0.0, 21.0)
O2_left  = (-36.37306696, 0.0)
O2_right = ( 36.37306696, 0.0)
```

粗略示意：

```text
                 y+
                 ^
                 |
              O1 (0, 21)
                 |
                 |
O2_left ---------+--------- O2_right  ---> x+
(-36.37, 0)      |         (36.37, 0)
                 |
                 |
              D 通常在 y < 0 区域
```

所以在 `main.c` 的演示里，左右腿目标点采用 x 镜像：

```c
left:  { -75.0f, -165.0f }
right: {  75.0f, -165.0f }
```

这表示左右腿做对称动作，而不是同一条腿往同一个 x 方向跑。

## 2. 机构点和连杆

模块内部使用这些点：

- `O1`：飞特舵机侧固定铰点。
- `O2`：DM4310 侧固定铰点，左右腿关于 y 轴镜像。
- `A`：由 `O1` 和 `theta1` 决定的舵机侧活动点。
- `B`：由 `O2` 和 `theta2` 决定的电机侧活动点。
- `C`：四连杆中间点，由圆交点求出来。
- `D`：足端点，也就是正解输出/逆解输入的目标点。

连杆长度在代码中定义为：

```c
O1A = 24 mm
O2B = 120 mm
AC  = 120 mm
BC  = 35 mm
CD  = 155 mm
BD  = 120 mm
```

其中 `BD = CD - BC`，因为当前几何模型里 `C -> B -> D` 在同一直线上，`B` 位于 `C` 和 `D` 之间。

## 3. 角度坐标

运动学里的角度有两个：

- `theta1_deg`：飞特舵机侧解耦后的机构角。
- `theta2_deg`：DM4310 侧解耦后的机构角。

二者都是机构角，不是硬件原始反馈值。硬件值会先转换成这两个角，再参与几何解算。

### theta1

`theta1` 以 `O1` 竖直向上方向为参考。正解中：

```c
left:
A.x = O1.x - L1 * sin(theta1)
A.y = O1.y + L1 * cos(theta1)

right:
A.x = O1.x + L1 * sin(theta1)
A.y = O1.y + L1 * cos(theta1)
```

也就是说：

- `theta1 = 0 deg` 时，`A` 在 `O1` 正上方。
- 右腿 `theta1` 增大时，`A` 往 `+x` 方向偏。
- 左腿 `theta1` 增大时，`A` 往 `-x` 方向偏。
- 左右腿在几何上镜像，但用同一个 `theta1` 范围表达。

当前限制：

```c
theta1: -180 deg ~ 25 deg
```

### theta2

`theta2` 以 `O2` 向腿外侧的水平轴为参考。正解中：

```c
left:
B.x = O2.x - L2 * cos(theta2)
B.y = O2.y + L2 * sin(theta2)

right:
B.x = O2.x + L2 * cos(theta2)
B.y = O2.y + L2 * sin(theta2)
```

因此：

- 右腿的外侧方向是 `+x`。
- 左腿的外侧方向是 `-x`。
- `sin(theta2)` 共同决定 `B.y`，所以负角会让 `B` 位于 `O2` 下方。

当前限制：

```c
theta2: -170 deg ~ -45 deg
```

## 4. 硬件值和机构角的关系

外部硬件不直接使用 `theta1/theta2`，而是使用：

- 飞特舵机：`ft_raw`
- DM4310：`dm_rad`

运动学模块把硬件值和机构角分开，是为了让正解/逆解都在统一的机构角坐标里工作。

### 飞特舵机 raw -> theta1

当前标定为线性映射：

```text
id = 0, 左腿飞特:
raw 32   -> theta1  25 deg
raw 1005 -> theta1 -180 deg

id = 3, 右腿飞特:
raw 999 -> theta1  25 deg
raw 25  -> theta1 -180 deg
```

对应到 `leg_kinematics.c`：

- `LEG_SIDE_LEFT` 使用 id 0 的标定。
- `LEG_SIDE_RIGHT` 使用 id 3 的标定。

逆解输出 `ft_raw` 时也使用同一套标定反算，并会夹在对应 raw 范围内。

### DM rad -> theta2

DM 侧当前也做了左右镜像：

```text
id = 1, 左腿 DM4310:
dm_rad 0.00 -> theta2  -45 deg
dm_rad 2.22 -> theta2 -170 deg

id = 4, 右腿 DM4310:
dm_rad 0.00 -> theta2 -170 deg
dm_rad 2.22 -> theta2  -45 deg
```

所以同一个 `theta2` 机构角，左右腿会换算成相反方向的 DM 原始位置变化。

## 5. 正解和逆解的数据流

### 正解

正解是从硬件反馈或机构角算足端点 `D`：

```text
ft_raw + dm_rad
      |
      v
theta1 + theta2
      |
      v
O1/O2 + 连杆长度
      |
      v
foot D = (x_mm, y_mm)
```

对应接口：

```c
leg_kinematics_forward_from_actuator(side, &actuator, &result);
leg_kinematics_forward_from_angles(side, &joint, &result);
```

### 逆解

逆解是从目标足端点 `D` 算硬件目标值：

```text
target D = (x_mm, y_mm)
      |
      v
theta1 + theta2
      |
      v
ft_raw + dm_rad
```

对应接口：

```c
leg_kinematics_inverse_to_actuator(side, &target, &actuator, &result);
leg_kinematics_inverse_to_angles(side, &target, &result);
```

如果返回值不是 `LEG_KINEMATICS_OK`，说明目标点不可达、参数错误或角度越界，不应该把结果继续发给硬件。

## 6. main.c 演示点怎么理解

当前 `main.c` 里左右腿同步演示使用 10 个目标点循环。右腿点在 `x > 0`，左腿点在 `x < 0`，两边 y 相同：

```text
right: ( 75, -165) ... ( 70, -155)
left:  (-75, -165) ... (-70, -155)
```

这符合当前坐标定义：

- 右腿在坐标系右侧，目标点用正 x。
- 左腿在坐标系左侧，目标点用负 x。
- y 越负，足端越向下。
- 周期循环时，两条腿走的是镜像轨迹。

调试时可以看这些变量：

```c
g_left_ik_test_target
g_right_ik_test_target
g_left_ik_test_actuator
g_right_ik_test_actuator
g_left_ik_test_result
g_right_ik_test_result
g_left_ik_test_status
g_right_ik_test_status
```

其中 `target` 是输入的目标足端坐标，`result.foot_d` 是解算后的足端坐标，`actuator` 是最终要发给飞特和 DM 的硬件目标值。
