#include "inspire.h"
#include "param.h"

#include "dds/Publisher.h"
#include "dds/Subscription.h"
#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>
#include <unitree/common/thread/recurrent_thread.hpp>

class InspireRunner
{
public:
  InspireRunner()
  {
    try
    {
      serial1 = std::make_shared<SerialPort>("/dev/ttyUSB1", B115200);
      is_serial1 = true;
    }
    catch(const std::runtime_error& e)
    {
      if(param::skip_non_hand)
      {
        std::cout << e.what() << ", so this hand has been skipped" << std::endl;
        is_serial1 = false;
      }
      else
      {
        std::cout << e.what() << std::endl;
        exit(-1);
      }
    }
    try
    {
      serial2 = std::make_shared<SerialPort>("/dev/ttyUSB2", B115200);
      is_serial2 = true;
    }
    catch(const std::runtime_error& e)
    {
      if(param::skip_non_hand)
      {
        std::cout << e.what() << ", so this hand has been skipped" << std::endl;
        is_serial2 = false;
      }
      else
      {
        std::cout << e.what() << std::endl;
        exit(-1);
      }
    }

    // If your left and right hand controls are reversed, you can swap the positions of `serial1` and `serial2` below.
    if(is_serial1)
    {
      righthand = std::make_shared<inspire::InspireHand>(serial1, 1);
    }
    if(is_serial2)
    {
      lefthand = std::make_shared<inspire::InspireHand>(serial2, 1);
    }

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
    // Set command
    if(!handcmd->isTimeout())
    {
      for(int i(0); i<12; i++)
      {
        qcmd(i) = handcmd->msg_.cmds()[i].q();
      }
      if(is_serial1)
      {
        righthand->SetPosition(qcmd.block<6, 1>(0, 0));
      }
      if(is_serial2)
      {
        lefthand->SetPosition(qcmd.block<6, 1>(6, 0));
      }
    }

    // Recv state
    Eigen::Matrix<double, 6, 1> qtemp;
    if(is_serial1)
    {
      if(righthand->GetPosition(qtemp) == 0)
      {
        qstate.block<6, 1>(0, 0) = qtemp;
      }
      else
      {
        for(int i(0); i<6; i++)
        {
          handstate->msg_.states()[i].lost()++;
        }
        // spdlog::debug("Failed to get right hand state");
      }
    }
    else
    {
      for(int i(0); i<6; i++)
      {
        qstate(i) = 1;
      }
    }
    if(is_serial2)
    {
      if(lefthand->GetPosition(qtemp) == 0)
      {
        qstate.block<6, 1>(6, 0) = qtemp;
      }
      else
      {
        for(int i(0); i<6; i++)
        {
          handstate->msg_.states()[i+6].lost()++;
        }
        // spdlog::debug("Failed to get left hand state");
      }
    }
    else
    {
      for(int i(0); i<6; i++)
      {
        qstate(i+6) = 1;
      }
    }
    if(handstate->trylock())
    {
        for(int i(0); i<12; i++)
        {
            handstate->msg_.states()[i].q() = qstate(i);
        }
        handstate->unlockAndPublish();
    }
  }

  unitree::common::ThreadPtr thread;

  // inspire
  SerialPort::SharedPtr serial1;
  SerialPort::SharedPtr serial2;
  bool is_serial1;
  bool is_serial2;
  std::shared_ptr<inspire::InspireHand> lefthand;
  std::shared_ptr<inspire::InspireHand> righthand;
  Eigen::Matrix<double, 12, 1> qcmd, qstate;

  // dds
  std::unique_ptr<unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorStates_>> handstate;
  std::shared_ptr<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorCmds_>> handcmd;
};

int main(int argc, char ** argv)
{
  auto vm = param::helper(argc, argv);
  unitree::robot::ChannelFactory::Instance()->Init(0, param::network);

  std::cout << " --- Unitree Robotics --- " << std::endl;
  std::cout << "  Inspire Hand Controller  " << std::endl;

  InspireRunner runner;
 
  while (true)
  {
    sleep(1);
  }
  return 0;
}