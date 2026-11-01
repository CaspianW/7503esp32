// MCU telemetry schema and validity checks, independent of Arduino and ROS.
#include "../src/communication/serial_telemetry.h"
#include <cassert>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

static std::vector<std::string> tokens(const std::string &line) {
  std::istringstream input(line);
  std::vector<std::string> result;
  std::string value;
  while (input >> value) result.push_back(value);
  return result;
}
static std::string encode(const robot::SerialTelemetry &frame) {
  char output[robot::serialTelemetryCapacity];
  const size_t size = robot::formatSerialTelemetry(output, sizeof(output), frame, 100);
  assert(size > 0 && size == std::string(output).size());
  return output;
}
static bool has(const std::string &text, const char *tag) {
  return text.find(std::string("\n") + tag + " ") != std::string::npos;
}

int main() {
  robot::SerialTelemetry frame;
  frame.uptimeMs = 1000;
  frame.initialized = true;
  frame.encoderReady = frame.imuReady = frame.speedReady = true;
  frame.motorOutputsEnabled = frame.chassisControlEnabled = true;
  frame.odometry = {1, -2, 0.25f, 0.1f, -0.2f, 0.3f, 990};
  frame.imu = {1, -2, 9.80665f, -0.1f, 0.2f, -0.3f, 25, 995};
  frame.liftOutputsEnabled = frame.liftConfigured = frame.liftReady = frame.liftActive = true;
  frame.liftTarget = 1;
  frame.liftPulseUs = 500;
  const std::string good = encode(frame);
  assert(good.find("S 1000 1 0 0 0 0 1 1 1 0 1 1 0\n") == 0);
  assert(good.find("O 990 1.00000 -2.00000 0.25000 0.10000 -0.20000 0.30000\n") != std::string::npos);
  assert(good.find("I 995 1.00000 -2.00000 9.80665 -0.10000 0.20000 -0.30000 25.000\n") != std::string::npos);
  assert(good.find("L 1 1 1 1 1 500\n") != std::string::npos);
  std::istringstream lines(good);
  std::string line;
  const size_t counts[] = {14, 13, 8, 9, 7};
  const char tags[] = {'S', 'E', 'O', 'I', 'L'};
  for (size_t n = 0; n < 5; ++n) {
    assert(static_cast<bool>(std::getline(lines, line)));
    const auto fields = tokens(line);
    assert(fields[0] == std::string(1, tags[n]) && fields.size() == counts[n]);
    for (size_t i = 1; i < fields.size(); ++i) (void)std::stod(fields[i]);
  }
  assert(!std::getline(lines, line));

  // Encoding does not substitute send time for actual source time.
  frame.uptimeMs = 1020;
  assert(has(encode(frame), "O") && has(encode(frame), "I"));
  assert(encode(frame).find("O 990 ") != std::string::npos);
  assert(encode(frame).find("I 995 ") != std::string::npos);

  // Exactly 100 ms is stale. Health/status and lift target remain available.
  frame.uptimeMs = 1095;
  const std::string stale = encode(frame);
  assert(!has(stale, "O") && !has(stale, "I"));
  assert(stale.find("S 1095 1 0 0 0 0 1 0 1 0 1 1 0\n") == 0);
  assert(has(stale, "L"));

  frame.uptimeMs = 1000;
  frame.odometry.timestamp = frame.imu.timestamp = 0;
  assert(!has(encode(frame), "O") && !has(encode(frame), "I"));
  frame.odometry.timestamp = frame.imu.timestamp = 990;
  frame.initialized = false;
  assert(!has(encode(frame), "O") && !has(encode(frame), "I"));
  frame.initialized = true;
  frame.encoderReady = frame.imuReady = false;
  assert(!has(encode(frame), "O") && !has(encode(frame), "I"));
  frame.encoderReady = frame.imuReady = true;
  frame.speedReady = false;
  assert(!has(encode(frame), "O") && has(encode(frame), "I"));

  frame.speedReady = true;
  frame.odometry.x = std::numeric_limits<float>::quiet_NaN();
  frame.imu.gyroZ = std::numeric_limits<float>::infinity();
  assert(!has(encode(frame), "O") && !has(encode(frame), "I"));
  frame.odometry.x = 1;
  frame.imu.gyroZ = -0.3f;

  // Unsigned subtraction permits millis() rollover but rejects future samples.
  frame.uptimeMs = 20;
  frame.odometry.timestamp = frame.imu.timestamp = UINT32_MAX - 10;
  assert(has(encode(frame), "O") && has(encode(frame), "I"));
  frame.odometry.timestamp = frame.imu.timestamp = 30;
  assert(!has(encode(frame), "O") && !has(encode(frame), "I"));

  // A short buffer never exposes a truncated telemetry block as a valid packet.
  char small[10] = "sentinel";
  assert(robot::formatSerialTelemetry(small, sizeof(small), frame, 100) == 0);
  assert(small[0] == '\0');
  assert(robot::formatSerialTelemetry(nullptr, 10, frame, 100) == 0);
  assert(robot::formatSerialTelemetry(small, 0, frame, 100) == 0);
  char exact[robot::serialTelemetryCapacity];
  const size_t length = robot::formatSerialTelemetry(exact, sizeof(exact), frame, 100);
  assert(robot::formatSerialTelemetry(exact, length + 1, frame, 100) == length);
  assert(robot::formatSerialTelemetry(exact, length, frame, 100) == 0);
}
