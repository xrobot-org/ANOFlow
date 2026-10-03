# ANOFlow

ANOTC 光流传感器（UART）驱动模块 / Driver Module for the ANOTC optical-flow sensor over UART

## 1. 模块作用 / Purpose

构造后，ANOFlow 创建高优先级线程 `ano_flow_thread`，从 UART 逐字节读取并解析 ANOTC 帧，把解码结果发布到 Topic。线程每次读取最多等待 20 ms；读取超时时刷新链路状态标志。

模块在 RamFS 中注册命令 `ano_flow`：

- `ano_flow`：打印用法。
- `ano_flow show <time_ms> <interval_ms>`：在 `time_ms` 内每隔 `interval_ms` 打印一次链路与工作状态、质量、速度和高度，`interval_ms` 限制在 10 到 1000 ms。

After construction, ANOFlow creates the high-priority thread `ano_flow_thread`, which reads the UART byte by byte, parses ANOTC frames and publishes the decoded result to a Topic. Each read waits at most 20 ms; when a read times out, the link state flags are refreshed.

The Module registers the command `ano_flow` in RamFS:

- `ano_flow`: print the usage.
- `ano_flow show <time_ms> <interval_ms>`: print the link and work state, quality, velocity and altitude every `interval_ms` for `time_ms`, with `interval_ms` limited to 10 to 1000 ms.

## 2. 帧格式与数据 / Frame Format and Data

帧结构依次为：帧头 `0xAA`、目标 `0x05` 或 `0xFF`、功能 ID、长度、数据、两个校验字节（和校验与附加和校验）。长度或校验不符的帧被丢弃。解码的帧如下：

| 功能 ID | 内容 | `Data` 字段 |
| --- | --- | --- |
| `0x51`，模式 0 | 原始光流 | `of0_sta`、`raw_dx`、`raw_dy`、`quality` |
| `0x51`，模式 1 | 光流速度 | `of1_sta`、`vel_x_cmps`、`vel_y_cmps`、`quality` |
| `0x51`，模式 2 | 融合光流速度与积分 | `of2_sta`、`vel_x_cmps`、`vel_y_cmps`、`vel_x_fix_cmps`、`vel_y_fix_cmps`、`integral_x`、`integral_y`、`quality` |
| `0x34` | 高度 | `alt_cm` |
| `0x01` | IMU 附带数据（原始值） | `acc`、`gyr` |
| `0x04` | 四元数（放大 10000 倍） | `quaternion` |

速度单位为 cm/s，高度单位为 cm。`flow_update_count` 与 `alt_update_count` 分别统计收到的光流帧（模式 1 与 2）和高度帧的数量。

链路状态：最近 500 ms 内收到有效帧、光流帧、高度帧时，`link_alive`、`flow_valid`、`alt_valid` 分别为 `true`；`work_alive` 等于 `flow_valid && alt_valid`。下游控制可据这些标志判断数据是否过期。

每收到一帧有效帧，以及任一状态标志变化时（在 UART 读取超时时检查），模块发布一次 `Data`。

A frame consists of: head `0xAA`, target `0x05` or `0xFF`, function ID, length, payload and two checksum bytes (sum check and add check). Frames with a wrong length or checksum are dropped. The decoded frames are:

| Function ID | Content | `Data` fields |
| --- | --- | --- |
| `0x51`, mode 0 | Raw flow | `of0_sta`, `raw_dx`, `raw_dy`, `quality` |
| `0x51`, mode 1 | Flow velocity | `of1_sta`, `vel_x_cmps`, `vel_y_cmps`, `quality` |
| `0x51`, mode 2 | Fused flow velocity and integral | `of2_sta`, `vel_x_cmps`, `vel_y_cmps`, `vel_x_fix_cmps`, `vel_y_fix_cmps`, `integral_x`, `integral_y`, `quality` |
| `0x34` | Altitude | `alt_cm` |
| `0x01` | IMU sideband (raw values) | `acc`, `gyr` |
| `0x04` | Quaternion (scaled by 10000) | `quaternion` |

Velocities are in cm/s and altitude in cm. `flow_update_count` and `alt_update_count` count the received flow frames (modes 1 and 2) and altitude frames.

Link state: `link_alive`, `flow_valid` and `alt_valid` are `true` when a valid frame, a flow frame or an altitude frame arrived within the last 500 ms; `work_alive` equals `flow_valid && alt_valid`. Downstream control can use these flags to judge whether the data is stale.

The Module publishes `Data` after every valid frame and whenever a state flag changes (checked when a UART read times out).

## 3. 构造接口 / Constructor

```cpp
ANOFlow(LibXR::UART& uart,
        LibXR::RamFS& ramfs,
        const char* data_topic_name = "ano_flow_data",
        size_t task_stack_depth = 1024);
```

依赖：

- `uart`：连接传感器的 `LibXR::UART`，取自 BSP 的硬件注册（`XR_REGISTER`），波特率由 BSP 配置。
- `ramfs`：接收 `ano_flow` 命令的 `LibXR::RamFS`，取自 BSP 的硬件注册。

配置参数：

- `data_topic_name`：发布的 Topic 名称，默认 `"ano_flow_data"`。
- `task_stack_depth`：解析线程的栈深，默认 1024。

Dependencies:

- `uart`: the `LibXR::UART` connected to the sensor, taken from the BSP's Registration (`XR_REGISTER`); the baud rate is configured by the BSP.
- `ramfs`: the `LibXR::RamFS` that receives the `ano_flow` command, taken from the BSP's Registration.

Configuration parameters:

- `data_topic_name`: name of the published Topic, default `"ano_flow_data"`.
- `task_stack_depth`: stack depth of the parser thread, default 1024.

## 4. Topic

| Topic | 方向 | 类型 | 说明 |
| --- | --- | --- | --- |
| `data_topic_name`（默认 `ano_flow_data`） | 发布 | `ANOFlow::Data` | 光流、高度、IMU 附带数据与链路状态 |

| Topic | Direction | Type | Meaning |
| --- | --- | --- | --- |
| `data_topic_name` (default `ano_flow_data`) | Publish | `ANOFlow::Data` | Optical flow, altitude, IMU sideband data and link state |

## 5. 配置示例 / Configuration Example

`xrobot instance add xrobot-org/ANOFlow` 写入的实例，`uart` 与 `ramfs` 填写为 BSP 中注册的名称：

An instance written by `xrobot instance add xrobot-org/ANOFlow`, with `uart` and `ramfs` set to names registered by the BSP:

```yaml
modules:
  - module: xrobot-org/ANOFlow
    id: anoflow_0
    args:
      - uart: uart_flow
      - ramfs: ramfs
      - data_topic_name: "ano_flow_data"
      - task_stack_depth: 1024
```

## 6. 依赖与硬件 / Dependencies and Hardware

依赖：LibXR。

硬件：一个输出 ANOTC 帧的光流传感器，通过 UART 连接，UART 与 RamFS 由 BSP 通过 `XR_REGISTER` 注册。

Dependencies: LibXR.

Hardware: one optical-flow sensor that outputs ANOTC frames, connected over UART; the UART and RamFS are registered by the BSP with `XR_REGISTER`.
