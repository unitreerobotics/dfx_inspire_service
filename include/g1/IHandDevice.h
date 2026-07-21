#ifndef G1_IHAND_DEVICE_H
#define G1_IHAND_DEVICE_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>

#include <eigen3/Eigen/Dense>

namespace g1
{

constexpr std::size_t kJointsPerHand = 6;
constexpr std::size_t kTotalJoints = 12;
constexpr std::size_t kRightStartIndex = 0;
constexpr std::size_t kLeftStartIndex = 6;

enum class HandJointIndex : std::size_t
{
  Pinky = 0,
  Ring = 1,
  Middle = 2,
  Index = 3,
  ThumbBend = 4,
  ThumbRotation = 5,
};

static_assert(static_cast<std::size_t>(HandJointIndex::Pinky) == 0);
static_assert(static_cast<std::size_t>(HandJointIndex::Ring) == 1);
static_assert(static_cast<std::size_t>(HandJointIndex::Middle) == 2);
static_assert(static_cast<std::size_t>(HandJointIndex::Index) == 3);
static_assert(static_cast<std::size_t>(HandJointIndex::ThumbBend) == 4);
static_assert(static_cast<std::size_t>(HandJointIndex::ThumbRotation) == 5);

using HandPosition = Eigen::Matrix<double, 6, 1>;
using HandPositionRaw = Eigen::Matrix<int16_t, 6, 1>;

struct HandPositionCommand
{
  HandPosition normalized{HandPosition::Zero()};
  HandPositionRaw raw{HandPositionRaw::Zero()};
};

// Convert normalized positions to the hand's [0, 1000] wire representation.
// Non-finite input rejects the complete hand command before any device call.
inline bool MakeHandPositionCommand(const double* input, HandPositionCommand& output)
{
  HandPositionCommand converted;
  for(std::size_t i = 0; i < kJointsPerHand; ++i)
  {
    if(!std::isfinite(input[i]))
    {
      return false;
    }

    const double normalized = std::clamp(input[i], 0.0, 1.0);
    converted.normalized(static_cast<Eigen::Index>(i)) = normalized;
    converted.raw(static_cast<Eigen::Index>(i)) =
      static_cast<int16_t>(std::lround(normalized * 1000.0));
  }

  output = converted;
  return true;
}

class IHandDevice
{
public:
  using SharedPtr = std::shared_ptr<IHandDevice>;

  virtual ~IHandDevice() = default;
  virtual bool SetPosition(const HandPositionCommand& command) = 0;
  virtual bool GetPosition(HandPosition& position) = 0;
};

// A deterministic in-memory hand used by --dry-run and offline tests.
class MockHandDevice final : public IHandDevice
{
public:
  bool SetPosition(const HandPositionCommand& command) override
  {
    ++set_position_count_;
    last_command_ = command;
    position_ = command.normalized;
    return set_position_succeeds_;
  }

  bool GetPosition(HandPosition& position) override
  {
    ++get_position_count_;
    if(!get_position_succeeds_)
    {
      return false;
    }
    position = position_;
    return true;
  }

  std::size_t set_position_count() const { return set_position_count_; }
  std::size_t get_position_count() const { return get_position_count_; }
  const HandPositionCommand& last_command() const { return last_command_; }

  void set_position_succeeds(bool value) { set_position_succeeds_ = value; }
  void get_position_succeeds(bool value) { get_position_succeeds_ = value; }

private:
  HandPosition position_{HandPosition::Zero()};
  HandPositionCommand last_command_{};
  std::size_t set_position_count_{0};
  std::size_t get_position_count_{0};
  bool set_position_succeeds_{true};
  bool get_position_succeeds_{true};
};

} // namespace g1

#endif // G1_IHAND_DEVICE_H
