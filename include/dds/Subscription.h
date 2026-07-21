#ifndef _UT_ROBOT_SUBSCRIPTION_H_
#define _UT_ROBOT_SUBSCRIPTION_H_

#include <unitree/robot/channel/channel_subscriber.hpp>
#include <mutex>

namespace unitree
{
namespace robot
{

template <typename MessageType>
class SubscriptionBase
{
public:
  using MsgType = MessageType;
  using SharedPtr = std::shared_ptr<SubscriptionBase<MsgType>>;
  
  SubscriptionBase(const std::string& topic)
  {
    last_update_time_ = std::chrono::steady_clock::now() - std::chrono::milliseconds(timeout_ms_);
    sub_ = std::make_shared<unitree::robot::ChannelSubscriber<MessageType>>(topic);
    sub_->InitChannel([this](const void *msg){
      std::lock_guard<std::mutex> lock(mutex_);
      last_update_time_ = std::chrono::steady_clock::now();
      msg_ = *(const MessageType*)msg;
      post_communication();
    });
  }

  void set_timeout_ms(uint32_t timeout_ms) { timeout_ms_ = timeout_ms; }

  bool isTimeout() {
    std::lock_guard<std::mutex> lock(mutex_);
    auto now = std::chrono::steady_clock::now();
    auto elasped_time = now - last_update_time_;
    return elasped_time > std::chrono::milliseconds(timeout_ms_);
  }

  // Atomically copy a complete, non-expired sample. G1 uses this instead of
  // reading msg_ concurrently with the DDS callback.
  bool getLatest(MessageType& message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = std::chrono::steady_clock::now();
    if(now - last_update_time_ > std::chrono::milliseconds(timeout_ms_))
    {
      return false;
    }
    message = msg_;
    return true;
  }


  MessageType msg_;
protected:
  virtual void post_communication() {} // something after receiving message

  uint32_t timeout_ms_{1000};
  std::mutex mutex_;
  unitree::robot::ChannelSubscriberPtr<MessageType> sub_;
  std::chrono::steady_clock::time_point last_update_time_;
};


}; // namespace robot
}; // namespace unitree

#endif // _UT_ROBOT_SUBSCRIPTION_H_
