#include "param.h"

#include "g1/DeviceSetup.h"
#include "g1/G1ControlCore.h"
#include "g1/SerialHandDevice.h"

#include "dds/Publisher.h"
#include "dds/Subscription.h"
#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>
#include <unitree/common/thread/recurrent_thread.hpp>

#include <algorithm>
#include <array>

class InspireRunner
{
public:
  explicit InspireRunner(const g1::Configuration& config)
  {
    g1::DefaultHandDeviceFactory factory;
    auto devices = g1::CreateDevices(config, factory);
    control = std::make_unique<g1::G1ControlCore>(devices.right, devices.left);

    // dds
    handcmd = std::make_shared<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorCmds_>>(
        "rt/" + param::ns + "/cmd");
    handcmd->msg_.cmds().resize(12);
    handstate = std::make_unique<unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorStates_>>(
        "rt/" + param::ns + "/state");
    handstate->msg_.states().resize(12);

    // Start running
    thread = std::make_shared<unitree::common::RecurrentThread>(
      10000, std::bind(&InspireRunner::run, this)
    );
  }

  void run()
  {
    // Copy a coherent DDS sample. A missing/expired sample deliberately results
    // in no SetPosition call and no implicit close/open/zero command.
    unitree_go::msg::dds_::MotorCmds_ command_message;
    std::array<double, g1::kTotalJoints> command{};
    if(handcmd->getLatest(command_message))
    {
      const std::size_t command_size = command_message.cmds().size();
      const std::size_t copy_size = std::min(command_size, g1::kTotalJoints);
      for(std::size_t i = 0; i < copy_size; ++i)
      {
        command[i] = command_message.cmds()[i].q();
      }
      control->ProcessCommand(command.data(), command_size, false);
    }
    else
    {
      control->ProcessCommand(nullptr, 0, true);
    }

    control->PollState();
    if(handstate->trylock())
    {
      const auto& qstate = control->qstate();
      const auto& lost = control->lost();
      for(std::size_t i = 0; i < g1::kTotalJoints; ++i)
      {
        handstate->msg_.states()[i].q() = qstate[i];
        handstate->msg_.states()[i].lost() = lost[i];
      }
      handstate->unlockAndPublish();
    }
  }

  unitree::common::ThreadPtr thread;

  std::unique_ptr<g1::G1ControlCore> control;

  // dds
  std::unique_ptr<unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorStates_>> handstate;
  std::shared_ptr<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorCmds_>> handcmd;
};

int main(int argc, char ** argv)
{
  g1::Configuration config;
  auto vm = param::g1_helper(argc, argv, config);
  unitree::robot::ChannelFactory::Instance()->Init(0, param::network);

  std::cout << " --- Unitree Robotics --- " << std::endl;
  std::cout << "  Inspire Hand Controller  " << std::endl;

  InspireRunner runner(config);
 
  while (true)
  {
    sleep(1);
  }
  return 0;
}
