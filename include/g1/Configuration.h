#ifndef G1_CONFIGURATION_H
#define G1_CONFIGURATION_H

#include <string>

namespace g1
{

enum class HandSide
{
  Right,
  Left,
};

struct Configuration
{
  std::string right_port{"/dev/ttyUSB1"};
  std::string left_port{"/dev/ttyUSB2"};
  bool left_only{false};
  bool dry_run{false};
};

} // namespace g1

#endif // G1_CONFIGURATION_H
