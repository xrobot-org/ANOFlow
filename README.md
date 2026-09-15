# ANOFlow

## Static assembly source line

This source line uses explicit C++ constructor dependencies and ordered instance
arguments. Inspect the current primary header with `xrobot_mod_parser --path .`;
its declarations, not old manifest/config examples, define the interface.
Historical HardwareContainer/ApplicationManager examples below apply only to the
older dynamic source tags. Device/protocol descriptions remain relevant.
See the XRobot [migration guide](https://github.com/xrobot-org/XRobot/blob/dev/MIGRATION.md).
Compilation is not hardware validation; retain version-specific board evidence.


ANOTC optical flow sensor UART parser module for XRobot.

This module reads the ANOTC optical-flow binary stream from `ano_flow_uart`,
parses flow, altitude, IMU sideband, and quaternion frames, publishes a compact
sensor data topic, and exposes a RamFS shell command for link-health and sample
status.

The parser tracks whether the link, flow stream, altitude stream, and working
state are alive so downstream control logic can reject stale optical-flow data.

## Required Hardware

- `ano_flow_uart`
- `ramfs`

## Constructor Arguments

- `data_topic_name`: default `"ano_flow_data"`
- `task_stack_depth`: default `1024`

## Published Topics

- `data_topic_name`: `ANOFlow::Data`, including link state, flow velocity, altitude, IMU sideband data, and quaternion payloads

## Shell Commands

The module registers `ano_flow` in `RamFS`.

- `ano_flow` or `ano_flow status`: print link state, work state, quality, velocity, and altitude

## XRobot Configuration Example

```yaml
- id: flow
  name: ANOFlow
  constructor_args:
    data_topic_name: "ano_flow_data"
    task_stack_depth: 1024
```
