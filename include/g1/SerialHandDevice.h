#ifndef G1_SERIAL_HAND_DEVICE_H
#define G1_SERIAL_HAND_DEVICE_H

#include <memory>
#include <string>

#include "DeviceSetup.h"
#include "inspire.h"

namespace g1
{

class SerialHandDevice final : public IHandDevice
{
public:
  SerialHandDevice(const std::string& port, uint8_t id = 1)
  : serial_(std::make_shared<SerialPort>(port, B115200)),
    hand_(std::make_shared<inspire::InspireHand>(serial_, id))
  {}

  bool SetPosition(const HandPositionCommand& command) override
  {
    return hand_->SetPositionRaw(command.raw) == 0;
  }

  bool GetPosition(HandPosition& position) override
  {
    return hand_->GetPosition(position) == 0;
  }

private:
  SerialPort::SharedPtr serial_;
  std::shared_ptr<inspire::InspireHand> hand_;
};

class DefaultHandDeviceFactory final : public IHandDeviceFactory
{
public:
  IHandDevice::SharedPtr CreateSerial(HandSide, const std::string& port) override
  {
    return std::make_shared<SerialHandDevice>(port, 1);
  }

  IHandDevice::SharedPtr CreateMock(HandSide) override
  {
    return std::make_shared<MockHandDevice>();
  }
};

} // namespace g1

#endif // G1_SERIAL_HAND_DEVICE_H
