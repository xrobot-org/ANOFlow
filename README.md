# ANOFlow

ANOTC optical flow V4.0 parser module. Refer to [its official website](https://www.anotc.com/wiki/%E5%8C%BF%E5%90%8D%E4%BA%A7%E5%93%81%E8%B5%84%E6%96%99/%E5%8C%BF%E5%90%8D%E5%85%89%E6%B5%81v4.0)


## Required Hardware

- `ano_flow_uart`
- `ramfs`

## Constructor Arguments

- `data_topic_name`: `ano_flow_data`
- `task_stack_depth`: `1024`

## Output

- `ANOFlow::Data`

## Notes

- Parses the ANO optical-flow binary stream.
- Publishes link-health, flow velocity, altitude, IMU sideband data, and
  quaternion payloads from the sensor.
