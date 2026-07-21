#ifndef G1_CONTROL_CORE_H
#define G1_CONTROL_CORE_H

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>

#include "IHandDevice.h"

namespace g1
{

enum class CommandResult
{
  Applied,
  TimedOut,
  TooShort,
  NonFinite,
  DeviceError,
};

class G1ControlCore
{
public:
  G1ControlCore(IHandDevice::SharedPtr right, IHandDevice::SharedPtr left)
  : right_(std::move(right)), left_(std::move(left))
  {
    qstate_.fill(0.0);
    lost_.fill(0);
  }

  CommandResult ProcessCommand(const double* command, std::size_t size, bool timed_out)
  {
    if(timed_out)
    {
      // Intentionally do not call SetPosition: no implicit close/open/zero command.
      return CommandResult::TimedOut;
    }
    if(size < kTotalJoints)
    {
      return CommandResult::TooShort;
    }

    // Reject a malformed 12-joint DDS command as a whole, including fields
    // for a disabled hand. No enabled device may be written for NaN or Inf.
    for(std::size_t i = 0; i < kTotalJoints; ++i)
    {
      if(!std::isfinite(command[i]))
      {
        return CommandResult::NonFinite;
      }
    }

    HandPositionCommand right_command;
    HandPositionCommand left_command;

    // Validate all enabled hands before writing either device, preventing a
    // partially applied two-hand command when one side contains NaN or Inf.
    if(right_ && !MakeHandPositionCommand(command + kRightStartIndex, right_command))
    {
      return CommandResult::NonFinite;
    }
    if(!left_ || !MakeHandPositionCommand(command + kLeftStartIndex, left_command))
    {
      return left_ ? CommandResult::NonFinite : CommandResult::DeviceError;
    }

    if(right_ && !right_->SetPosition(right_command))
    {
      return CommandResult::DeviceError;
    }
    if(!left_->SetPosition(left_command))
    {
      return CommandResult::DeviceError;
    }
    return CommandResult::Applied;
  }

  void PollState()
  {
    HandPosition position{HandPosition::Zero()};
    if(right_)
    {
      if(right_->GetPosition(position))
      {
        for(std::size_t i = 0; i < kJointsPerHand; ++i)
        {
          qstate_[kRightStartIndex + i] = position(static_cast<Eigen::Index>(i));
        }
      }
      else
      {
        IncrementLost(kRightStartIndex);
      }
    }

    if(left_->GetPosition(position))
    {
      for(std::size_t i = 0; i < kJointsPerHand; ++i)
      {
        qstate_[kLeftStartIndex + i] = position(static_cast<Eigen::Index>(i));
      }
    }
    else
    {
      IncrementLost(kLeftStartIndex);
    }
  }

  const std::array<double, kTotalJoints>& qstate() const { return qstate_; }
  const std::array<uint32_t, kTotalJoints>& lost() const { return lost_; }

private:
  void IncrementLost(std::size_t start)
  {
    for(std::size_t i = 0; i < kJointsPerHand; ++i)
    {
      ++lost_[start + i];
    }
  }

  IHandDevice::SharedPtr right_;
  IHandDevice::SharedPtr left_;
  std::array<double, kTotalJoints> qstate_{};
  std::array<uint32_t, kTotalJoints> lost_{};
};

} // namespace g1

#endif // G1_CONTROL_CORE_H
