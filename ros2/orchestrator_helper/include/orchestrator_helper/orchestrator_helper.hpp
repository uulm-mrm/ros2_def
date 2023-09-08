#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <orchestrator_interfaces/msg/status.hpp>
#include <orchestrator_helper/orchestrated_publisher.hpp>

class OrchestratorHelper
{
private:
  using StatusMsg = orchestrator_interfaces::msg::Status;
  using LifecycleNodePtr = rclcpp_lifecycle::LifecycleNode*;
  template <typename MessageT, typename AllocT>
  using LifecyclePublisherPtr = rclcpp_lifecycle::LifecyclePublisher<MessageT, AllocT>::SharedPtr;

public:
  using SharedPtr = std::shared_ptr<OrchestratorHelper>;

  explicit OrchestratorHelper(LifecycleNodePtr node) : node_(node)
  {
  }

  template <typename MessageT, typename... Params>
  auto create_publisher(Params&&... params)
  {
    auto publisher = node_->create_publisher<MessageT>(std::forward<Params>(params)...);
    // decltype(*publisher)::ROSMessageTypeAllocatorTraits::allocator_type
    return wrap_publisher<MessageT, std::allocator<void>>(std::move(publisher));
  }

  template <typename MessageT, typename AllocT>
  auto wrap_publisher(LifecyclePublisherPtr<MessageT, AllocT> publisher)
  {
    return std::make_shared<OrchestratedPublisher<MessageT, AllocT>>(std::move(publisher));
  }

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

  void publish_status(const std::vector<std::string>& omitted_outputs = {}, int debug_id = -1)
  {
    static int debug_id_ = 0;
    StatusMsg status_msg;
    status_msg.node_name = node_->get_name();
    status_msg.debug_id = debug_id >= 0 ? debug_id : debug_id_++;
    status_msg.omitted_outputs = omitted_outputs;
    status_pub_->publish(std::move(status_msg));
  }

  void advertise()
  {
    status_pub_ = node_->create_publisher<StatusMsg>("/status", 10);
  }

  void on_activate()
  {
    status_pub_->on_activate();
  }

  void on_deactivate()
  {
    status_pub_->on_deactivate();
  }

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

