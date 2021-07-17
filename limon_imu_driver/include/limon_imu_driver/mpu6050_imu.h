
#include <async_port/async_serial.hpp>

using namespace westonrobot;

#define MPU6050_FRAME_LEN 11
#define TTL_FRAME_START 0x55

namespace agx {

enum TTLFrameType {
  TIME = 0x50,
  ACCELERATION = 0x51,
  ANGULAR_VELOCITY = 0x52,
  ANGLE = 0x53,
};

class SerialRead {
 public:
  SerialRead();
  ~SerialRead();
  using SerialFrameCallback = AsyncSerial::ReceiveCallback;
  void Connect(std::string dev_name, uint32_t bouad_rate,
               SerialFrameCallback cb);
  void Disconnect();
  void SendData(uint8_t* data, size_t len);
  void ReadRawSerialData(uint8_t* data, const size_t bufsize, size_t len);
  int ParseRawData(const std::vector<uint8_t>& data);
  void ParseSingleFrame(const std::vector<uint8_t>& data, TTLFrameType type);
  void ParseAcceleration(const std::vector<uint8_t>& data);
  void ParseAngularVelocity(const std::vector<uint8_t>& data);
  void ParseAngle(const std::vector<uint8_t>& data);

 private:
  std::shared_ptr<AsyncSerial> serial_;
  std::mutex state_mutex_;
  bool enable_timeout_{true};
  bool serial_connected_{false};
  uint32_t timeout_ms_{500};
  uint32_t watchdog_counter_{0};
  std::vector<uint8_t> binarystream_;
  char* bytestream_{nullptr};
};
};  // namespace agx