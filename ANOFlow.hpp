#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: ANOTC 光流传感器（UART）驱动模块 / Driver Module for the ANOTC optical-flow sensor over UART
depends: []
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>

#include "message.hpp"
#include "ramfs.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "uart.hpp"

/**
 * @brief ANOTC 光流传感器驱动模块，解析 UART 数据流并发布光流、高度与链路状态。
 *        Driver Module for the ANOTC optical-flow sensor; it parses the UART stream and
 *        publishes optical flow, altitude and link state.
 */
class ANOFlow
{
 public:
  /**
   * @brief 发布的光流数据。
   *        Published optical-flow data.
   */
  struct Data
  {
    bool link_alive = false;     ///< 500 ms 内收到有效帧
                                 ///< A valid frame arrived within 500 ms
    bool flow_valid = false;     ///< 500 ms 内收到光流帧（模式 1 或 2）
                                 ///< A flow frame (mode 1 or 2) arrived within 500 ms
    bool alt_valid = false;      ///< 500 ms 内收到高度帧
                                 ///< An altitude frame arrived within 500 ms
    bool work_alive = false;     ///< flow_valid 且 alt_valid
                                 ///< flow_valid and alt_valid
    uint8_t quality = 0;         ///< 光流质量
                                 ///< Optical-flow quality
    uint8_t of0_sta = 0;         ///< 模式 0 状态
                                 ///< Mode 0 status
    uint8_t of1_sta = 0;         ///< 模式 1 状态
                                 ///< Mode 1 status
    uint8_t of2_sta = 0;         ///< 模式 2 状态
                                 ///< Mode 2 status
    int16_t raw_dx = 0;          ///< 模式 0 原始 X 位移
                                 ///< Mode 0 raw X displacement
    int16_t raw_dy = 0;          ///< 模式 0 原始 Y 位移
                                 ///< Mode 0 raw Y displacement
    int16_t vel_x_cmps = 0;      ///< X 方向光流速度 (cm/s)
                                 ///< Optical-flow velocity along X (cm/s)
    int16_t vel_y_cmps = 0;      ///< Y 方向光流速度 (cm/s)
                                 ///< Optical-flow velocity along Y (cm/s)
    int16_t vel_x_fix_cmps = 0;  ///< X 方向融合速度 (cm/s)
                                 ///< Fused velocity along X (cm/s)
    int16_t vel_y_fix_cmps = 0;  ///< Y 方向融合速度 (cm/s)
                                 ///< Fused velocity along Y (cm/s)
    int16_t integral_x = 0;      ///< X 方向光流积分
                                 ///< Optical-flow integral along X
    int16_t integral_y = 0;      ///< Y 方向光流积分
                                 ///< Optical-flow integral along Y
    uint32_t alt_cm = 0;         ///< 高度 (cm)
                                 ///< Altitude (cm)
    std::array<int16_t, 3> acc = {};  ///< 传感器附带的加速度原始值 (x, y, z)
                                      ///< Raw sideband acceleration (x, y, z)
    std::array<int16_t, 3> gyr = {};  ///< 传感器附带的角速度原始值 (x, y, z)
                                      ///< Raw sideband angular velocity (x, y, z)
    /// 四元数 (w, x, y, z)
    /// Quaternion (w, x, y, z)
    std::array<float, 4> quaternion = {1.0f, 0.0f, 0.0f, 0.0f};
    uint32_t flow_update_count = 0;  ///< 收到的光流帧数量
                                     ///< Number of received flow frames
    uint32_t alt_update_count = 0;   ///< 收到的高度帧数量
                                     ///< Number of received altitude frames
  };

  /**
   * @brief 构造 ANOFlow，注册 RamFS 命令并创建解析线程。
   *        Construct ANOFlow, register the RamFS command and create the parser thread.
   *
   * @param uart 连接传感器的 UART。
   *             UART connected to the sensor.
   * @param ramfs 接收 `ano_flow` 命令的 RamFS。
   *              RamFS that receives the `ano_flow` command.
   * @param data_topic_name 发布光流数据的 Topic 名称。
   *                        Name of the Topic that publishes the optical-flow data.
   * @param task_stack_depth 解析线程栈深。
   *                         Stack depth of the parser thread.
   */
  ANOFlow(
      LibXR::UART& uart,
      LibXR::RamFS& ramfs,
      const char* data_topic_name = "ano_flow_data",
      size_t task_stack_depth = 1024)
      : topic_(LibXR::Topic::CreateTopic<Data>(data_topic_name, nullptr, true)),
        uart_(std::addressof(uart)),
        cmd_file_(LibXR::RamFS::CreateFile("ano_flow", CommandFunc, this))
  {
    ramfs.Add(cmd_file_);

    thread_.Create(this, ThreadFunc, "ano_flow_thread", task_stack_depth,
                   LibXR::Thread::Priority::HIGH);
  }

 private:
  static constexpr uint8_t ANO_FRAME_HEAD_DEF = 0xAA;
  static constexpr uint8_t ANO_HW_TYPE_DEF = 0x05;
  static constexpr uint8_t ANO_HW_ALL_DEF = 0xFF;

  static int16_t ReadS16LE(const uint8_t* data)
  {
    return static_cast<int16_t>((static_cast<uint16_t>(data[1]) << 8) | data[0]);
  }

  static uint32_t ReadU32LE(const uint8_t* data)
  {
    return static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8) |
           (static_cast<uint32_t>(data[2]) << 16) |
           (static_cast<uint32_t>(data[3]) << 24);
  }

  void UpdateHealth(const bool force_publish = false)
  {
    const uint32_t now_ms = LibXR::Thread::GetTime();

    const bool last_link_alive = data_.link_alive;
    const bool last_flow_valid = data_.flow_valid;
    const bool last_alt_valid = data_.alt_valid;
    const bool last_work_alive = data_.work_alive;

    data_.link_alive = (now_ms - last_link_rx_ms_) < 500U;
    data_.flow_valid = (now_ms - last_flow_update_ms_) < 500U;
    data_.alt_valid = (now_ms - last_alt_update_ms_) < 500U;
    data_.work_alive = data_.flow_valid && data_.alt_valid;

    if (force_publish || last_link_alive != data_.link_alive ||
        last_flow_valid != data_.flow_valid || last_alt_valid != data_.alt_valid ||
        last_work_alive != data_.work_alive)
    {
      topic_.Publish(data_);
    }
  }

  void HandleFrame(const uint8_t* data, const uint8_t len)
  {
    uint8_t check_sum1 = 0;
    uint8_t check_sum2 = 0;

    if (data[3] != (len - 6U))
    {
      return;
    }

    for (uint8_t i = 0; i < len - 2U; ++i)
    {
      check_sum1 = static_cast<uint8_t>(check_sum1 + data[i]);
      check_sum2 = static_cast<uint8_t>(check_sum2 + check_sum1);
    }

    if (check_sum1 != data[len - 2] || check_sum2 != data[len - 1])
    {
      return;
    }

    last_link_rx_ms_ = LibXR::Thread::GetTime();

    switch (data[2])
    {
      case 0x51:
      {
        switch (data[4])
        {
          case 0:
          {
            data_.of0_sta = data[5];
            data_.raw_dx = static_cast<int8_t>(data[6]);
            data_.raw_dy = static_cast<int8_t>(data[7]);
            data_.quality = data[8];
          }
          break;
          case 1:
          {
            data_.of1_sta = data[5];
            data_.vel_x_cmps = ReadS16LE(&data[6]);
            data_.vel_y_cmps = ReadS16LE(&data[8]);
            data_.quality = data[10];
            data_.flow_update_count++;
            last_flow_update_ms_ = last_link_rx_ms_;
          }
          break;
          case 2:
          {
            data_.of2_sta = data[5];
            data_.vel_x_cmps = ReadS16LE(&data[6]);
            data_.vel_y_cmps = ReadS16LE(&data[8]);
            data_.vel_x_fix_cmps = ReadS16LE(&data[10]);
            data_.vel_y_fix_cmps = ReadS16LE(&data[12]);
            data_.integral_x = ReadS16LE(&data[14]);
            data_.integral_y = ReadS16LE(&data[16]);
            data_.quality = data[18];
            data_.flow_update_count++;
            last_flow_update_ms_ = last_link_rx_ms_;
          }
          break;
          default:
            break;
        }
      }
      break;
      case 0x34:
      {
        data_.alt_cm = ReadU32LE(&data[7]);
        data_.alt_update_count++;
        last_alt_update_ms_ = last_link_rx_ms_;
      }
      break;
      case 0x01:
      {
        data_.acc[0] = ReadS16LE(&data[4]);
        data_.acc[1] = ReadS16LE(&data[6]);
        data_.acc[2] = ReadS16LE(&data[8]);
        data_.gyr[0] = ReadS16LE(&data[10]);
        data_.gyr[1] = ReadS16LE(&data[12]);
        data_.gyr[2] = ReadS16LE(&data[14]);
      }
      break;
      case 0x04:
      {
        data_.quaternion[0] = static_cast<float>(ReadS16LE(&data[4])) * 0.0001f;
        data_.quaternion[1] = static_cast<float>(ReadS16LE(&data[6])) * 0.0001f;
        data_.quaternion[2] = static_cast<float>(ReadS16LE(&data[8])) * 0.0001f;
        data_.quaternion[3] = static_cast<float>(ReadS16LE(&data[10])) * 0.0001f;
      }
      break;
      default:
        break;
    }

    UpdateHealth(true);
  }

  void ParseByte(const uint8_t data)
  {
    switch (rx_state_)
    {
      case 0:
        if (data == ANO_FRAME_HEAD_DEF)
        {
          rx_state_ = 1;
          frame_buffer_[0] = data;
        }
        break;
      case 1:
        if (data == ANO_HW_TYPE_DEF || data == ANO_HW_ALL_DEF)
        {
          rx_state_ = 2;
          frame_buffer_[1] = data;
        }
        else
        {
          rx_state_ = 0;
        }
        break;
      case 2:
        rx_state_ = 3;
        frame_buffer_[2] = data;
        break;
      case 3:
        if (data <= frame_buffer_.size() - 6U)
        {
          rx_state_ = 4;
          frame_buffer_[3] = data;
          data_len_ = data;
          data_cnt_ = 0;
        }
        else
        {
          rx_state_ = 0;
        }
        break;
      case 4:
        if (data_len_ > 0U)
        {
          --data_len_;
          frame_buffer_[4 + data_cnt_++] = data;
          if (data_len_ == 0U)
          {
            rx_state_ = 5;
          }
        }
        else
        {
          rx_state_ = 0;
        }
        break;
      case 5:
        rx_state_ = 6;
        frame_buffer_[4 + data_cnt_++] = data;
        break;
      case 6:
        rx_state_ = 0;
        frame_buffer_[4 + data_cnt_] = data;
        HandleFrame(frame_buffer_.data(), static_cast<uint8_t>(data_cnt_ + 5));
        break;
      default:
        rx_state_ = 0;
        break;
    }
  }

  static void ThreadFunc(ANOFlow* ano_flow)
  {
    LibXR::Semaphore read_sem;
    uint8_t byte = 0;

    while (true)
    {
      LibXR::ReadOperation op(read_sem, 20);
      const auto ans = ano_flow->uart_->Read({&byte, 1}, op);
      if (ans == LibXR::ErrorCode::OK)
      {
        ano_flow->ParseByte(byte);
      }
      else
      {
        ano_flow->UpdateHealth();
      }
    }
  }

  static int CommandFunc(ANOFlow* ano_flow, int argc, char** argv)
  {
    if (argc == 1)
    {
      LibXR::STDIO::Printf<"Usage:\r\n">();
      LibXR::STDIO::Printf<
          "  show [time_ms] [interval_ms] - Print optical flow state.\r\n">();
      return 0;
    }

    if (argc == 4 && std::strcmp(argv[1], "show") == 0)
    {
      int time_ms = std::atoi(argv[2]);
      int interval_ms = std::atoi(argv[3]);
      interval_ms = std::clamp(interval_ms, 10, 1000);

      while (time_ms > 0)
      {
        LibXR::STDIO::Printf<
            "ANOFlow: link=%d work=%d quality=%u vel=(%d,%d) alt=%u\r\n">(
            static_cast<int>(ano_flow->data_.link_alive),
            static_cast<int>(ano_flow->data_.work_alive),
            static_cast<unsigned>(ano_flow->data_.quality),
            static_cast<int>(ano_flow->data_.vel_x_cmps),
            static_cast<int>(ano_flow->data_.vel_y_cmps),
            static_cast<unsigned>(ano_flow->data_.alt_cm));
        LibXR::Thread::Sleep(interval_ms);
        time_ms -= interval_ms;
      }
      return 0;
    }

    LibXR::STDIO::Printf<"Error: Invalid arguments.\r\n">();
    return -1;
  }

  uint8_t rx_state_ = 0;
  uint8_t data_len_ = 0;
  uint8_t data_cnt_ = 0;
  uint32_t last_link_rx_ms_ = 0;
  uint32_t last_flow_update_ms_ = 0;
  uint32_t last_alt_update_ms_ = 0;
  std::array<uint8_t, 64> frame_buffer_ = {};
  Data data_;
  LibXR::Topic topic_;
  LibXR::UART* uart_;
  LibXR::RamFS::File cmd_file_;
  LibXR::Thread thread_;
};
