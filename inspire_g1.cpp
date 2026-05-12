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
    if(param::use_uni_dev_n)
    {
      serial1 = std::make_shared<SerialPort>("/dev/inspire_right", B115200);
      serial2 = std::make_shared<SerialPort>("/dev/inspire_left", B115200);
    }
    else
    {
      serial1 = std::make_shared<SerialPort>("/dev/ttyUSB1", B115200);
      serial2 = std::make_shared<SerialPort>("/dev/ttyUSB2", B115200);
    }

    // If your left and right hand controls are reversed, you can swap the positions of `serial1` and `serial2` below.
    righthand = std::make_shared<inspire::InspireHand>(serial1, 1);
    lefthand = std::make_shared<inspire::InspireHand>(serial2, 1);

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
      righthand->SetPosition(qcmd.block<6, 1>(0, 0));
      lefthand->SetPosition(qcmd.block<6, 1>(6, 0));
    }

    // Recv state
    Eigen::Matrix<double, 6, 1> qtemp;
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