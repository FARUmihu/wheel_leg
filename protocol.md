# ESP32 和 STM32 最小通讯协议

当前只做一个功能：ESP32 发送运动控制量，STM32 根据 `vx` 和 `wz` 控制小车前进、后退和转弯。

这版协议没有模式、启动平衡、急停、状态回传。先把行动控制跑通，后面需要时再扩展。

## 1. 串口参数

| 项目 | 参数 |
| --- | --- |
| 波特率 | `115200` |
| 数据位 | `8` |
| 校验 | `None` |
| 停止位 | `1` |
| 电平 | TTL 串口 |
| 接线 | ESP32 TX -> STM32 RX，ESP32 RX -> STM32 TX，GND 共地 |

当前 ESP32 引脚：

| ESP32 | 连接 |
| --- | --- |
| GPIO17 | TX，接 STM32 RX |
| GPIO18 | RX，接 STM32 TX |

## 2. 数据帧格式

```text
AA 55 CMD SEQ LEN PAYLOAD... CRC_LO CRC_HI
```

| 字段 | 长度 | 说明 |
| --- | --- | --- |
| `AA 55` | 2 字节 | 帧头 |
| `CMD` | 1 字节 | 命令字 |
| `SEQ` | 1 字节 | 帧序号，每发一帧加 1，溢出后从 0 开始 |
| `LEN` | 1 字节 | `PAYLOAD` 长度 |
| `PAYLOAD` | `LEN` 字节 | 数据内容 |
| `CRC_LO CRC_HI` | 2 字节 | CRC16 低字节在前 |

多字节数据统一小端序，也就是低字节在前。

CRC 使用 `CRC-16/MODBUS`：

| 项目 | 值 |
| --- | --- |
| 初始值 | `0xFFFF` |
| 多项式 | `0xA001` |
| 参与计算字段 | `CMD`、`SEQ`、`LEN`、`PAYLOAD` |
| 不参与计算字段 | 帧头 `AA 55`、CRC 本身 |

## 3. 唯一命令：CONTROL

| 方向 | CMD | LEN | 说明 |
| --- | --- | --- | --- |
| ESP32 -> STM32 | `0x01` | `0x04` | 发送目标线速度 `vx` 和目标角速度 `wz` |

Payload 固定 4 字节：

```text
byte 0-1: int16 vx_mm_s
byte 2-3: int16 wz_mrad_s
```

| 字节 | 字段 | 类型 | 单位 | 说明 |
| --- | --- | --- | --- | --- |
| `0-1` | `vx_mm_s` | `int16_t` | mm/s | 前后线速度 |
| `2-3` | `wz_mrad_s` | `int16_t` | mrad/s | 转向角速度 |

## 4. 方向约定

| 动作 | `vx_mm_s` | `wz_mrad_s` |
| --- | --- | --- |
| 停止 | `0` | `0` |
| 前进 | 正数 | `0` |
| 后退 | 负数 | `0` |
| 左转 | `0` | 正数 |
| 右转 | `0` | 负数 |

当前 ESP32 网页默认值：

| 控制量 | 默认值 | ESP32 限幅 |
| --- | --- | --- |
| 前进/后退速度 | `160 mm/s` | `-1000` 到 `+1000` |
| 左/右转角速度 | `500 mrad/s` | `-3000` 到 `+3000` |

ESP32 会每 `50 ms` 重复发送当前控制量。STM32 不要只靠单帧控制，建议持续接收最新目标值。

## 5. HTTP 调试接口

ESP32 网页按钮最终都会调用这个接口：

```text
POST /api/control?vx=<vx_mm_s>&wz=<wz_mrad_s>
```

也支持 GET，方便浏览器或串口调试时直接访问。

示例：

| 动作 | HTTP |
| --- | --- |
| 停止 | `/api/control?vx=0&wz=0` |
| 前进 | `/api/control?vx=160&wz=0` |
| 后退 | `/api/control?vx=-160&wz=0` |
| 左转 | `/api/control?vx=0&wz=500` |
| 右转 | `/api/control?vx=0&wz=-500` |

## 6. STM32 侧处理建议

STM32 收到合法 `CONTROL` 帧后，只需要更新两个目标量：

```c
target_vx_mm_s = cmd.vx_mm_s;
target_wz_mrad_s = cmd.wz_mrad_s;
```

建议加失联保护：

| 条件 | STM32 建议动作 |
| --- | --- |
| `300-500 ms` 没收到合法 `CONTROL` 帧 | `vx = 0`，`wz = 0` |
| 收到 CRC 错误帧 | 丢弃，不更新目标值 |
| 收到 `LEN` 不是 `4` 的 `CONTROL` 帧 | 丢弃 |

## 7. Payload 示例

下面只列 `PAYLOAD`：

| 动作 | 数值 | Payload |
| --- | --- | --- |
| 前进 | `vx=160, wz=0` | `A0 00 00 00` |
| 后退 | `vx=-160, wz=0` | `60 FF 00 00` |
| 左转 | `vx=0, wz=500` | `00 00 F4 01` |
| 右转 | `vx=0, wz=-500` | `00 00 0C FE` |
| 停止 | `vx=0, wz=0` | `00 00 00 00` |

## 8. 完整帧示例

下面示例使用 `CMD = 0x01`、`LEN = 0x04`，`SEQ` 从 `0x01` 递增。实际运行时 `SEQ` 不固定，CRC 会跟着 `SEQ` 变化。

| 动作 | 完整帧 |
| --- | --- |
| 前进 160 mm/s，`SEQ=01` | `AA 55 01 01 04 A0 00 00 00 D9 D1` |
| 后退 160 mm/s，`SEQ=02` | `AA 55 01 02 04 60 FF 00 00 D5 D2` |
| 左转 500 mrad/s，`SEQ=03` | `AA 55 01 03 04 00 00 F4 01 7D 33` |
| 右转 500 mrad/s，`SEQ=04` | `AA 55 01 04 04 00 00 0C FE 7F 04` |
| 停止，`SEQ=05` | `AA 55 01 05 04 00 00 00 00 FA 55` |

## 9. STM32 示例文件

仓库里的 `stm32_example/` 是最小版 STM32 参考代码：

| 文件 | 说明 |
| --- | --- |
| `robot_protocol_stm32.h` | 协议结构体、解析器声明 |
| `robot_protocol_stm32.c` | 帧解析、CRC 校验、解码 `vx/wz` |
| `main_loop_example.c` | STM32 主循环接入示例 |

队友重点改 `main_loop_example.c` 里的 `apply_motion_command()`，把 `cmd->vx_mm_s` 和 `cmd->wz_mrad_s` 接到实际运动控制即可。
