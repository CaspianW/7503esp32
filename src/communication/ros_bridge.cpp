/**
 * @file ros_bridge.cpp
 * @brief Unavailable native micro-ROS adapter placeholder with explicit disconnected state.
 * @author Caspian Wu
 * @date 2026-10-22
 * @version 0.1.0
 * @copyright Copyright (c) 2026 Caspian Wu.
 */

#include "ros_bridge.h"

// 当前固件通过 SerialLink 发送二进制串口包。本类保留给原生 micro-ROS，未启用。
namespace robot {
bool RosBridge::begin() { return false; }
void RosBridge::spinOnce() {}
bool RosBridge::takeVelocityCommand(Twist2D &) { return false; }
bool RosBridge::takeLiftCommand(LiftPosition &) { return false; }
void RosBridge::publishOdometry(const OdometryState &) {}
void RosBridge::publishImu(const ImuSample &) {}
bool RosBridge::connected() const { return false; }
} // namespace robot
