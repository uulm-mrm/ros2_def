#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <orchestrator_interfaces/msg/status.hpp>
#include <orchestrator_helper/orchestrated_publisher.hpp>

/***
 * Helper class for easier publishing of status messages on ROS callbacks.
 */
class OrchestratorHelper
{
private:
  using StatusMsg = orchestrator_interfaces::msg::Status;
  using LifecycleNodePtr = rclcpp_lifecycle::LifecycleNode*;
  template <typename MessageT, typename AllocT>
  using LifecyclePublisherPtr = rclcpp_lifecycle::LifecyclePublisher<MessageT, AllocT>::SharedPtr;

public:
  /**
   * Type alias for an std::shared_ptr to an instance of OrchestratorHelper.
   */
  using SharedPtr = std::shared_ptr<OrchestratorHelper>;

  /**
   * Constructor.
   * @param node Pointer to a LifecycleNode which will be used to create publishers.
   */
  explicit OrchestratorHelper(LifecycleNodePtr node) : node_(node)
  {
  }

  /**
   * Creates a LifecyclePublisher wrapped in an OrchestratedPublisher instance.
   * @param params Any params that shall be forwarded to the create_publisher method of the node.
   * @return The wrapped OrchestratedPublisher instance.
   * @tparam MessageT The type of the message that the wrapped publisher can publish.
   * @tparam AllocT Allocator
   */
  template <typename MessageT, typename... Params, typename AllocT = std::allocator<void>>
  auto create_publisher(Params&&... params)
  {
    auto publisher = node_->create_publisher<MessageT, AllocT>(std::forward<Params>(params)...);
    return wrap_publisher<MessageT, AllocT>(std::move(publisher));
  }

  /**
   * Wraps an existing LifecyclePublisher in an OrchestratedPublisher instance.
   * @param publisher Pointer to the LifecyclePublisher instance that shall be wrapped.
   * @tparam MessageT The type of the message that the wrapped publisher can publish.
   * @tparam AllocT Allocator
   */
  template <typename MessageT, typename AllocT>
  auto wrap_publisher(LifecyclePublisherPtr<MessageT, AllocT> publisher)
  {
    return std::make_shared<OrchestratedPublisher<MessageT, AllocT>>(std::move(publisher));
  }

  /**
   * Wraps a ROS callback. Will publish status calls signaling to the orchestrator that a callback has completed if it
   * did not produce any outputs or if any of the passed publishers did not publish during the callback.
   * This is a variant in which the callback takes no parameters (e.g., a timer callback).
   * @param publishers OrchestratedPublisher instances that can publish messages during the callback
   * @param f ROS callback function which may call publish() on the passed publishers.
   * @return The wrapped callback function
   * @tparam CallbackT Type of the callback
   * @tparam P Publisher types
   */
  template <typename CallbackT, typename... P>
  std::function<void(void)> wrap_callback(CallbackT&& f, P&&... publishers)
  {
    return [&, f, this]() {
      reset_publishers(std::forward<P>(publishers)...);
      f();
      std::vector<std::string> omitted_outputs;
      get_omitted_outputs(omitted_outputs, std::forward<P>(publishers)...);
      const auto n = sizeof...(publishers);
      if (!omitted_outputs.empty() || n == 0)
      {
        publish_status(omitted_outputs);
      }
    };
  }

  /**
   * Wraps a ROS callback. Will publish status calls signaling to the orchestrator that a callback has completed if it
   * did not produce any outputs or if any of the passed publishers did not publish during the callback.
   * This is a variant in which the callback takes a message as a parameter (e.g., a subscription callback).
   * @param publishers OrchestratedPublisher instances that can publish messages during the callback
   * @param f ROS callback function which may call publish() on the passed publishers.
   * @return The wrapped callback function
   * @tparam CallbackT Type of the callback
   * @tparam P Publisher types
   */
  template <typename MessageT, typename CallbackT, typename... P>
  std::function<void(typename MessageT::ConstSharedPtr)> wrap_callback(CallbackT&& f, P&&... publishers)
  {
    return [&, f, this](MessageT::ConstSharedPtr message) {
      reset_publishers(std::forward<P>(publishers)...);
      f(message);
      std::vector<std::string> omitted_outputs;
      get_omitted_outputs(omitted_outputs, std::forward<P>(publishers)...);
      const auto n = sizeof...(publishers);
      if (!omitted_outputs.empty() || n == 0)
      {
        publish_status(omitted_outputs);
      }
    };
  }

  /**
   * Publishes a status message for the orchestrator.
   * @param omitted_outputs List of omitted outputs (topics of publishers that did not publish although it is expected by the orchestrator).
   * @param debug_id An ID for the status message to help debugging the orchestrator. Values less than 0 produce auto-incrementing debug IDs.
   */
  void publish_status(const std::vector<std::string>& omitted_outputs = {}, int debug_id = -1)
  {
    static int debug_id_ = 0;
    StatusMsg status_msg;
    status_msg.node_name = std::string(node_->get_namespace()) + "/" + node_->get_name();
    status_msg.debug_id = debug_id >= 0 ? debug_id : debug_id_++;
    status_msg.omitted_outputs = omitted_outputs;
    status_pub_->publish(std::move(status_msg));
  }

  /**
   * Advertises topics.
   */
  void advertise()
  {
    status_pub_ = node_->create_publisher<StatusMsg>("/status", 10);
  }

  /**
   * Activates internal LifecyclePublishers.
   */
  void on_activate()
  {
    status_pub_->on_activate();
  }

  /**
   * Deactivates internal LifecyclePublishers.
   */
  void on_deactivate()
  {
    status_pub_->on_deactivate();
  }

  /**
   * Destroys internal LifecyclePublishers.
   */
  void on_cleanup()
  {
    status_pub_.reset();
  }

private:
  template <class T>
  using PublisherPtr = typename rclcpp_lifecycle::LifecyclePublisher<T>::SharedPtr;
  LifecycleNodePtr node_;
  PublisherPtr<StatusMsg> status_pub_{ nullptr };

  // base case
  void reset_publishers()
  {
  }
  template <typename PT, typename... P>
  void reset_publishers(OrchestratedPublisher<PT>& publisher, P&&... publishers)
  {
    publisher.reset_publish_cnt();
    reset_publishers(std::forward<P>(publishers)...);
  }

  // base case
  void get_omitted_outputs(std::vector<std::string>& omitted_outputs)
  {
  }
  template <typename PT, typename... P>
  void get_omitted_outputs(std::vector<std::string>& omitted_outputs,
                           OrchestratedPublisher<PT>& publisher,
                           P&&... publishers)
  {
    auto cnt = publisher.get_publish_cnt();
    if (cnt == 0)
    {
      omitted_outputs.push_back(publisher.get_topic_name());
    }
    if (cnt > 1)
    {
      throw std::runtime_error(std::string("Orchestrator expects that each topic is only published to once per "
                                           "callback, but ") +
                               publisher.get_topic_name() + " published " + std::to_string(cnt) +
                               " times during a callback !");
    }
    get_omitted_outputs(omitted_outputs, std::forward<P>(publishers)...);
  }
};

