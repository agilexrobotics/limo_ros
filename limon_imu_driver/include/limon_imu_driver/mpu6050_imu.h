
#include <async_port/async_serial.hpp>

using namespace westonrobot;

namespace agx {
class SerialRead {
 public:
  SerialRead();
  ~SerialRead();
  using SerialFrameCallback = AsyncSerial::ReceiveCallback;
  void Connect(std::string dev_name, uint32_t bouad_rate,
               SerialFrameCallback cb);
  void Disconnect();
  void SendData(uint8_t *data, size_t len);
  void ReadRawSerialData(uint8_t* data, const size_t bufsize, size_t len);

 private:
  std::shared_ptr<AsyncSerial> serial_;
  std::mutex state_mutex_;
  bool enable_timeout_{true};
  bool serial_connected_{false};
  uint32_t timeout_ms_{500};
  uint32_t watchdog_counter_{0};
  std::vector<uint8_t> binarystream_;
  char *bytestream_{nullptr};
};
};  // namespace agx