/**
 * @file ros_bridge.cpp
 * @brief Unavailable native micro-ROS adapter placeholder with explicit disconnected state.
 * @author Caspian Wu
 * @date 2026-10-22
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "ros_bridge.h"

// 当前固件默认运行 SerialConsole；该接口保留给后续原生 micro-ROS 适配。
// 当前串口发送由 SerialConsole 和 serial_telemetry 实现，上位机 ROS 转换不属于本固件。
// 串口桥接方案不调用本类；原生 micro-ROS 传输不能与 SerialConsole 共用 UART0。
namespace robot {
bool RosBridge::begin() { return false; }
void RosBridge::spinOnce() {}
bool RosBridge::takeVelocityCommand(Twist2D &) { return false; }
bool RosBridge::takeLiftCommand(LiftPosition &) { return false; }
void RosBridge::publishOdometry(const OdometryState &) {}
void RosBridge::publishImu(const ImuSample &) {}
bool RosBridge::connected() const { return false; }
} // namespace robot
