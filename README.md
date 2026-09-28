# ANOFlow

XRobot Module that parses the ANOTC optical-flow sensor binary stream from a UART.

The `ano_flow_thread` thread (high priority) reads the UART byte by byte and parses
ANOTC frames: head `0xAA`, target `0x05` or `0xFF`, function ID, length, payload and
two checksum bytes (sum check and add check). Frames with a wrong length or checksum
are dropped. Decoded frames:

| Function ID | Content | `Data` fields |
| --- | --- | --- |
| `0x51`, mode 0 | Raw flow | `of0_sta`, `raw_dx`, `raw_dy`, `quality` |
| `0x51`, mode 1 | Flow velocity | `of1_sta`, `vel_x_cmps`, `vel_y_cmps`, `quality` |
| `0x51`, mode 2 | Fused flow velocity and integral | `of2_sta`, `vel_x_cmps`, `vel_y_cmps`, `vel_x_fix_cmps`, `vel_y_fix_cmps`, `integral_x`, `integral_y`, `quality` |
| `0x34` | Altitude | `alt_cm` |
| `0x01` | IMU sideband (raw) | `acc`, `gyr` |
| `0x04` | Quaternion (x 0.0001) | `quaternion` |

Velocities are in cm/s and altitude in cm. `flow_update_count` and
`alt_update_count` count received flow (mode 1/2) and altitude frames.

Link health: `link_alive`, `flow_valid` and `alt_valid` are true when a valid
frame, a flow frame or an altitude frame arrived within the last 500 ms;
`work_alive` is `flow_valid && alt_valid`. Downstream control can use these flags
to reject stale data.

## Topics

| Topic | Type | Published |
| --- | --- | --- |
| `data_topic_name` (default `ano_flow_data`) | `ANOFlow::Data` | After every valid frame, and when a health flag changes (checked whenever a UART read times out after 20 ms) |

## RamFS command

The Module registers the `ano_flow` command in RamFS.

- `ano_flow`: print usage.
- `ano_flow show <time_ms> <interval_ms>`: print link and work state, quality,
  velocity and altitude every `interval_ms` (clamped to 10-1000 ms) for `time_ms`.

## Dependencies

No other Modules; LibXR only.

## Constructor

```cpp
ANOFlow(LibXR::UART& uart,
        LibXR::RamFS& ramfs,
        const char* data_topic_name = "ano_flow_data",
        size_t task_stack_depth = 1024);
```

Dependencies:

- `uart`: `LibXR::UART` connected to the sensor. The baud rate is configured by
  the BSP.
- `ramfs`: `LibXR::RamFS` that receives the `ano_flow` command.

Configuration:

- `data_topic_name`: name of the published topic, default `"ano_flow_data"`.
- `task_stack_depth`: stack depth of the parser thread, default 1024.

## Use

```sh
xrobot module add xrobot-org/ANOFlow
xrobot setup
xrobot instance add xrobot-org/ANOFlow
```

`xrobot instance add` writes an instance to `User/xrobot.yaml` with empty
dependencies and the source defaults; set the dependencies to the names of objects
the BSP registers with `XR_REGISTER`:

```yaml
modules:
  - module: xrobot-org/ANOFlow
    id: anoflow_0
    args:
      - uart: uart_flow
      - ramfs: ramfs
      - data_topic_name: '"ano_flow_data"'
      - task_stack_depth: '1024'
```

BSP side:

```cpp
XR_REGISTER(uart_flow, LibXR::UART);
XR_REGISTER(ramfs, LibXR::RamFS);
```

Run `xrobot setup` again to generate `User/xrobot_main.hpp`.

`xrobot module show .` in this repository, or
`xrobot module show Modules/xrobot-org/ANOFlow` in a BSP, prints the manifest and the
current constructor.
