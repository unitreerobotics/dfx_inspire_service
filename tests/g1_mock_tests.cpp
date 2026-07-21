#include "g1/DeviceSetup.h"
#include "g1/G1ControlCore.h"

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <memory>
#include <string>

namespace
{

int failures = 0;

#define CHECK_TRUE(expression) CheckTrue((expression), #expression, __FILE__, __LINE__)

void CheckTrue(bool value, const char* expression, const char* file, int line)
{
  if(!value)
  {
    ++failures;
    std::cerr << file << ':' << line << ": check failed: " << expression << '\n';
  }
}

bool Near(double lhs, double rhs)
{
  return std::abs(lhs - rhs) < 1e-12;
}

class CountingFakeFactory final : public g1::IHandDeviceFactory
{
public:
  g1::IHandDevice::SharedPtr CreateSerial(g1::HandSide side, const std::string&) override
  {
    ++serial_create_count;
    if(side == g1::HandSide::Right)
    {
      ++right_create_count;
    }
    else
    {
      ++left_create_count;
    }
    // Even this fake serial branch returns an in-memory mock. Tests never
    // include or construct SerialHandDevice/SerialPort.
    return std::make_shared<g1::MockHandDevice>();
  }

  g1::IHandDevice::SharedPtr CreateMock(g1::HandSide side) override
  {
    ++mock_create_count;
    if(side == g1::HandSide::Right)
    {
      ++right_create_count;
    }
    else
    {
      ++left_create_count;
    }
    return std::make_shared<g1::MockHandDevice>();
  }

  std::size_t serial_create_count{0};
  std::size_t mock_create_count{0};
  std::size_t right_create_count{0};
  std::size_t left_create_count{0};
};

void ExpectConversion(const std::array<double, 6>& input,
                      const std::array<int16_t, 6>& expected)
{
  g1::HandPositionCommand command;
  CHECK_TRUE(g1::MakeHandPositionCommand(input.data(), command));
  for(std::size_t i = 0; i < input.size(); ++i)
  {
    CHECK_TRUE(command.raw(static_cast<Eigen::Index>(i)) == expected[i]);
  }
}

void TestSyntheticConversions()
{
  ExpectConversion({1, 1, 1, 1, 1, 1}, {1000, 1000, 1000, 1000, 1000, 1000});
  ExpectConversion({0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0});
  ExpectConversion({0.1, 0.2, 0.3, 0.4, 0.5, 0.6}, {100, 200, 300, 400, 500, 600});
}

void TestLeftIndicesSixThroughEleven()
{
  CHECK_TRUE(g1::kLeftStartIndex + static_cast<std::size_t>(g1::HandJointIndex::Pinky) == 6);
  CHECK_TRUE(g1::kLeftStartIndex + static_cast<std::size_t>(g1::HandJointIndex::Ring) == 7);
  CHECK_TRUE(g1::kLeftStartIndex + static_cast<std::size_t>(g1::HandJointIndex::Middle) == 8);
  CHECK_TRUE(g1::kLeftStartIndex + static_cast<std::size_t>(g1::HandJointIndex::Index) == 9);
  CHECK_TRUE(g1::kLeftStartIndex + static_cast<std::size_t>(g1::HandJointIndex::ThumbBend) == 10);
  CHECK_TRUE(g1::kLeftStartIndex + static_cast<std::size_t>(g1::HandJointIndex::ThumbRotation) == 11);

  auto left = std::make_shared<g1::MockHandDevice>();
  g1::G1ControlCore core(nullptr, left);
  std::array<double, 12> command{0.91, 0.92, 0.93, 0.94, 0.95, 0.96,
                                 0.1, 0.2, 0.3, 0.4, 0.5, 0.6};

  CHECK_TRUE(core.ProcessCommand(command.data(), command.size(), false) ==
             g1::CommandResult::Applied);
  CHECK_TRUE(left->set_position_count() == 1);
  for(std::size_t i = 0; i < 6; ++i)
  {
    CHECK_TRUE(Near(left->last_command().normalized(static_cast<Eigen::Index>(i)),
                    command[6 + i]));
  }
}

void TestDryRunCreatesNoSerialDevice()
{
  g1::Configuration config;
  config.dry_run = true;
  CountingFakeFactory factory;

  const auto devices = g1::CreateDevices(config, factory);
  CHECK_TRUE(factory.serial_create_count == 0);
  CHECK_TRUE(factory.mock_create_count == 2);
  CHECK_TRUE(devices.right != nullptr);
  CHECK_TRUE(devices.left != nullptr);
}

void TestConfigurationDefaults()
{
  const g1::Configuration config;
  CHECK_TRUE(config.right_port == "/dev/ttyUSB1");
  CHECK_TRUE(config.left_port == "/dev/ttyUSB2");
  CHECK_TRUE(!config.left_only);
  CHECK_TRUE(!config.dry_run);
}

void TestLeftOnlyNeverCreatesOrPollsRight()
{
  g1::Configuration config;
  config.left_only = true;
  config.dry_run = true;
  CountingFakeFactory factory;

  auto devices = g1::CreateDevices(config, factory);
  CHECK_TRUE(factory.serial_create_count == 0);
  CHECK_TRUE(factory.right_create_count == 0);
  CHECK_TRUE(factory.left_create_count == 1);
  CHECK_TRUE(devices.right == nullptr);

  auto left = std::dynamic_pointer_cast<g1::MockHandDevice>(devices.left);
  g1::G1ControlCore core(devices.right, devices.left);
  core.PollState();
  CHECK_TRUE(left->get_position_count() == 1);
}

void TestClampBeforeIntegerConversion()
{
  const std::array<double, 6> input{-0.5, 0.0, 0.25, 1.0, 1.5, 2.0};
  const std::array<int16_t, 6> raw{0, 0, 250, 1000, 1000, 1000};
  ExpectConversion(input, raw);

  g1::HandPositionCommand command;
  CHECK_TRUE(g1::MakeHandPositionCommand(input.data(), command));
  CHECK_TRUE(Near(command.normalized(0), 0.0));
  CHECK_TRUE(Near(command.normalized(4), 1.0));
  CHECK_TRUE(Near(command.normalized(5), 1.0));
}

void TestNaNAndInfinityRejectWithoutDeviceWrite()
{
  auto right = std::make_shared<g1::MockHandDevice>();
  auto left = std::make_shared<g1::MockHandDevice>();
  g1::G1ControlCore core(right, left);
  std::array<double, 12> command{};
  command.fill(0.5);

  command[6] = std::numeric_limits<double>::quiet_NaN();
  CHECK_TRUE(core.ProcessCommand(command.data(), command.size(), false) ==
             g1::CommandResult::NonFinite);
  CHECK_TRUE(right->set_position_count() == 0);
  CHECK_TRUE(left->set_position_count() == 0);

  command[6] = 0.5;
  command[0] = std::numeric_limits<double>::infinity();
  CHECK_TRUE(core.ProcessCommand(command.data(), command.size(), false) ==
             g1::CommandResult::NonFinite);
  CHECK_TRUE(right->set_position_count() == 0);
  CHECK_TRUE(left->set_position_count() == 0);
}

void TestLeftOnlyAlsoRejectsNonFiniteDDSCommand()
{
  auto left = std::make_shared<g1::MockHandDevice>();
  g1::G1ControlCore core(nullptr, left);
  std::array<double, 12> command{};
  command.fill(0.5);
  command[0] = -std::numeric_limits<double>::infinity();

  CHECK_TRUE(core.ProcessCommand(command.data(), command.size(), false) ==
             g1::CommandResult::NonFinite);
  CHECK_TRUE(left->set_position_count() == 0);
}

void TestShortCommandRejectsWithoutDeviceWrite()
{
  auto right = std::make_shared<g1::MockHandDevice>();
  auto left = std::make_shared<g1::MockHandDevice>();
  g1::G1ControlCore core(right, left);
  std::array<double, 12> command{};

  CHECK_TRUE(core.ProcessCommand(command.data(), 11, false) ==
             g1::CommandResult::TooShort);
  CHECK_TRUE(right->set_position_count() == 0);
  CHECK_TRUE(left->set_position_count() == 0);
}

void TestTimeoutDoesNotSendImplicitPosition()
{
  auto right = std::make_shared<g1::MockHandDevice>();
  auto left = std::make_shared<g1::MockHandDevice>();
  g1::G1ControlCore core(right, left);
  std::array<double, 12> open_command{};
  open_command.fill(1.0);

  CHECK_TRUE(core.ProcessCommand(open_command.data(), open_command.size(), false) ==
             g1::CommandResult::Applied);
  CHECK_TRUE(right->set_position_count() == 1);
  CHECK_TRUE(left->set_position_count() == 1);

  std::array<double, 12> implicit_zero_command{};
  CHECK_TRUE(core.ProcessCommand(implicit_zero_command.data(), implicit_zero_command.size(), true) ==
             g1::CommandResult::TimedOut);
  CHECK_TRUE(right->set_position_count() == 1);
  CHECK_TRUE(left->set_position_count() == 1);
  CHECK_TRUE(right->last_command().raw(0) == 1000);
  CHECK_TRUE(left->last_command().raw(0) == 1000);
}

void TestStateStartsInitialized()
{
  auto left = std::make_shared<g1::MockHandDevice>();
  g1::G1ControlCore core(nullptr, left);
  for(std::size_t i = 0; i < g1::kTotalJoints; ++i)
  {
    CHECK_TRUE(Near(core.qstate()[i], 0.0));
    CHECK_TRUE(core.lost()[i] == 0);
  }
}

} // namespace

int main()
{
  TestSyntheticConversions();
  TestLeftIndicesSixThroughEleven();
  TestConfigurationDefaults();
  TestDryRunCreatesNoSerialDevice();
  TestLeftOnlyNeverCreatesOrPollsRight();
  TestClampBeforeIntegerConversion();
  TestNaNAndInfinityRejectWithoutDeviceWrite();
  TestLeftOnlyAlsoRejectsNonFiniteDDSCommand();
  TestShortCommandRejectsWithoutDeviceWrite();
  TestTimeoutDoesNotSendImplicitPosition();
  TestStateStartsInitialized();

  if(failures != 0)
  {
    std::cerr << failures << " mock test(s) failed\n";
    return EXIT_FAILURE;
  }
  std::cout << "All G1 mock tests passed\n";
  return EXIT_SUCCESS;
}
