#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

/**
 * Wrapper class for a LifecyclePublisher. Counts the number of \ref publish calls made since the last counter reset.
 * @tparam MessageT The type of the message that the wrapped publisher can publish.
 * @tparam AllocT Allocator
 */
template <typename MessageT, typename AllocT = std::allocator<void>>
class OrchestratedPublisher
{
public:
  /**
   * Type alias for a SharedPtr to a MessageT LifecyclePublisher publisher.
   */
  using PublisherPtr = rclcpp_lifecycle::LifecyclePublisher<MessageT, AllocT>::SharedPtr;
  /**
   * Type alias for an std::shared_ptr to an instance of OrchestratedPublisher.
   */
  using SharedPtr = std::shared_ptr<OrchestratedPublisher<MessageT, AllocT>>;

  /**
   * Constructor. Initializes the internal publish counter to zero.
   * @param publisher Pointer to a LifecyclePublisher instance that performs the actual publishing.
   */
  explicit OrchestratedPublisher(PublisherPtr publisher)
  {
    publisher_ = publisher;
    reset_publish_cnt();
  }

  /**
   * Publish a message via the wrapped LifecyclePublisher instance, and increments the internal counter.
   * @param msg The message to publish
   */
  template <typename M>
  void publish(M msg)
  {
    publisher_->publish(msg);
    publish_cnt_++;
  }

  /**
   * Resets the internal publish counter to zero.
   */
  void reset_publish_cnt()
  {
    publish_cnt_ = 0;
  }

  /**
   * Obtain the value of the internal publish counter.
   */
  [[nodiscard]] std::size_t get_publish_cnt()
  {
    return publish_cnt_;
  }

  /**
   * Returns the name of the topic that the wrapped publisher is publishing on.
   */
  [[nodiscard]] auto get_topic_name()
  {
    return publisher_->get_topic_name();
  }

  /**
   * Calls on_activate on the wrapped LifecyclePublisher.
   */
  void on_activate()
  {
    publisher_->on_activate();
  }

  /**
   * Calls on_deactivate on the wrapped LifecyclePublisher.
   */
  void on_deactivate()
  {
    publisher_->on_deactivate();
  }

private:
  std::size_t publish_cnt_;
  PublisherPtr publisher_;
};
