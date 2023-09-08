#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

template <typename MessageT, typename AllocT = std::allocator<void>>
class OrchestratedPublisher
{
public:
  using PublisherPtr = rclcpp_lifecycle::LifecyclePublisher<MessageT, AllocT>::SharedPtr;
  using SharedPtr = std::shared_ptr<OrchestratedPublisher<MessageT, AllocT>>;

  explicit OrchestratedPublisher(PublisherPtr publisher)
  {
    publisher_ = publisher;
    reset_publish_cnt();
  }

  template <typename M>
  void publish(M msg)
  {
    publisher_->publish(msg);
    publish_cnt_++;
  }

  void reset_publish_cnt()
  {
    publish_cnt_ = 0;
  }

  std::size_t get_publish_cnt()
  {
    return publish_cnt_;
  }

  auto get_topic_name()
  {
    return publisher_->get_topic_name();
  }

  void on_activate()
  {
    publisher_->on_activate();
  }

  void on_deactivate()
  {
    publisher_->on_deactivate();
  }

private:
  std::size_t publish_cnt_;
  PublisherPtr publisher_;
};
