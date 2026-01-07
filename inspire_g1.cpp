#include "inspire.h"
#include "param.h"

#include "dds/Publisher.h"
#include "dds/Subscription.h"
#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>
#include <unitree/common/thread/recurrent_thread.hpp>

#include <thread>
#include <atomic>
#include <termios.h>
#include <unistd.h>

// Get 2 available ports
std::pair<std::string, std::string> getTwoSerialPorts() {
    std::pair<std::string, std::string> ports;
    int count = 0;
    
    FILE* pipe = popen("ls /dev/ttyUSB* 2>/dev/null", "r");
    if (!pipe) {
        std::cout << "Command error" << std::endl;
        return ports;
    }
    
    char buffer[128];
    while (fgets(buffer, sizeof(buffer), pipe) && count < 2) {
        std::string port = buffer;
        // Remove newline
        if (!port.empty() && port.back() == '\n') port.pop_back();
        
        if (count == 0) ports.first = port;
        else ports.second = port;
        
        count++;
    }
    pclose(pipe);
    
    if (count < 2) {
        std::cout << "Only " << count << " port(s) are available" << std::endl;
        exit(1);
        return {"", ""};
    }
    std::cout << "Working Ports are: " << ports.first <<" and " << ports.second << std::endl;
    return ports;
}


class InspireRunner
{
public:
  InspireRunner()
  {    
    auto ports = getTwoSerialPorts();
    serial1 = std::make_shared<SerialPort>(ports.first, B115200);
    serial2 = std::make_shared<SerialPort>(ports.second, B115200);

    hand_1 = std::make_shared<inspire::InspireHand>(serial1, 1);
    hand_2 = std::make_shared<inspire::InspireHand>(serial2, 1);

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

    // Keyboard listener thread
    keyboard_thread = std::thread(&InspireRunner::keyboardListener, this);
    keyboard_thread.detach();
  }

  void run()
  {
    if(!handcmd->isTimeout())
    {
      for(int i(0); i<12; i++)
      {
        qcmd(i) = handcmd->msg_.cmds()[i].q();
      }
      hand_1->SetPosition(qcmd.block<6, 1>(0, 0));
      hand_2->SetPosition(qcmd.block<6, 1>(6, 0));
    }

    Eigen::Matrix<double, 6, 1> qtemp;
    if(hand_1->GetPosition(qtemp) == 0)
    {
      qstate.block<6, 1>(0, 0) = qtemp;
    }
    else
    {
      for(int i(0); i<6; i++)
      {
        handstate->msg_.states()[i].lost()++;
      }
    }

    if(hand_2->GetPosition(qtemp) == 0)
    {
      qstate.block<6, 1>(6, 0) = qtemp;
    }
    else
    {
      for(int i(0); i<6; i++)
      {
        handstate->msg_.states()[i+6].lost()++;
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

private:
  void keyboardListener()
  {
    termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    while(true)
    {
      char c = getchar();
      if(c == 's')
      {
        std::swap(hand_1, hand_2);
        std::cout << "hands swapped" << std::endl;
      }
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  }

public:
  unitree::common::ThreadPtr thread;

  // inspire
  SerialPort::SharedPtr serial1;
  SerialPort::SharedPtr serial2;
  std::shared_ptr<inspire::InspireHand> hand_1;
  std::shared_ptr<inspire::InspireHand> hand_2;
  Eigen::Matrix<double, 12, 1> qcmd, qstate;

  // dds
  std::unique_ptr<unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorStates_>> handstate;
  std::shared_ptr<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorCmds_>> handcmd;

  std::thread keyboard_thread;
};

int main(int argc, char ** argv)
{
  auto vm = param::helper(argc, argv);
  unitree::robot::ChannelFactory::Instance()->Init(0, param::network);

  std::cout << " --- Unitree Robotics --- " << std::endl;
  std::cout << "  Inspire Hand Controller  " << std::endl;
  std::cout << "  Press 's' to swap hands  " << std::endl;

  InspireRunner runner;

  while (true)
  {
    sleep(1);
  }
  return 0;
}

