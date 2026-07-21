#ifndef G1_DEVICE_SETUP_H
#define G1_DEVICE_SETUP_H

#include <string>

#include "Configuration.h"
#include "IHandDevice.h"

namespace g1
{

class IHandDeviceFactory
{
public:
  virtual ~IHandDeviceFactory() = default;
  virtual IHandDevice::SharedPtr CreateSerial(HandSide side, const std::string& port) = 0;
  virtual IHandDevice::SharedPtr CreateMock(HandSide side) = 0;
};

struct Devices
{
  IHandDevice::SharedPtr right;
  IHandDevice::SharedPtr left;
};

inline Devices CreateDevices(const Configuration& config, IHandDeviceFactory& factory)
{
  Devices devices;

  // Keep this branch above every serial factory call: dry-run can only reach
  // CreateMock(), so it cannot construct SerialPort or invoke open/read/write.
  if(!config.left_only)
  {
    devices.right = config.dry_run
      ? factory.CreateMock(HandSide::Right)
      : factory.CreateSerial(HandSide::Right, config.right_port);
  }

  devices.left = config.dry_run
    ? factory.CreateMock(HandSide::Left)
    : factory.CreateSerial(HandSide::Left, config.left_port);

  return devices;
}

} // namespace g1

#endif // G1_DEVICE_SETUP_H
