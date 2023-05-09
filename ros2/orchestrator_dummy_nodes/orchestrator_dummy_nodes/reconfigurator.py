#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from orchestrator_interfaces.msg import Status, SampleMessage
from orchestrator_interfaces.srv import ReconfigurationAnnouncement, ReconfigurationRequest, SetConfigurationService

from orchestrator.orchestrator_lib.name_utils import intercepted_name


class ReconfiguratorNode(Node):

    def __init__(self) -> None:
        super().__init__('ReconfiguratorNode')  # type: ignore
        self.status_publisher = self.create_publisher(Status, "status", 10)

        self.timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.reconfig_timer = self.create_timer(1.0, self.timer_callback, callback_group=self.timer_cb_group)

        self.announce_cb_group = MutuallyExclusiveCallbackGroup()
        self.reconfig_announce_client = self.create_client(
            ReconfigurationAnnouncement,
            "announce_reconfig",
            callback_group=self.announce_cb_group)

        self.reconfig_request_cb_group = MutuallyExclusiveCallbackGroup()
        self.reconfig_request_server = self.create_service(
            ReconfigurationRequest, "request_reconfig", self.reconfig_request_callback,
            callback_group=self.reconfig_request_cb_group
        )

        self.reconfig_cb_group = MutuallyExclusiveCallbackGroup()
        self.configure_a_client = self.create_client(SetConfigurationService, "A/set_output_topic",
                                                     callback_group=self.reconfig_cb_group)
        self.configure_b_client = self.create_client(SetConfigurationService, "B/set_input_topic",
                                                     callback_group=self.reconfig_cb_group)

        while not self.reconfig_announce_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        while not self.configure_a_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        while not self.configure_b_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def publish_status(self):
        status_msg = Status()
        status_msg.node_name = self.get_name()
        self.get_logger().info("Sending Status")
        self.status_publisher.publish(status_msg)
        self.get_logger().info("Done sending Status")

    def timer_callback(self):
        self.get_logger().info("Timer callback")
        (s, ns) = self.get_clock().now().seconds_nanoseconds()
        if s == 5 and ns == 0:
            self.get_logger().info("Reconfiguring!")
            req = ReconfigurationAnnouncement.Request()
            res = self.reconfig_announce_client.call(req)
            self.get_logger().info(f"Got response: {res}")
        else:
            self.get_logger().info("Not reconfiguring!")

        self.publish_status()
        self.get_logger().info("Timer callback done")

    def reconfig_request_callback(self, request, response):
        self.get_logger().info("Got request for reconfiguration!")

        self.get_logger().info("Reconfiguring A...")
        req = SetConfigurationService.Request()
        req.config_value = "T2"
        resp_a = self.configure_a_client.call(req)
        assert resp_a.success

        self.get_logger().info("Reconfiguring B...")
        req_b = SetConfigurationService.Request()
        req_b.config_value = intercepted_name("B", "T2")
        resp_b = self.configure_b_client.call(req_b)
        assert resp_b.success

        self.get_logger().info("Reconfiguration done. Replying to orchestrator.")
        response.new_launch_config_package = "orchestrator_dummy_nodes"
        response.new_launch_config_filename = "reconfiguration_test_after_launch_config.json"
        return response


def main(args=None):
    rclpy.init(args=args)
    sub_node = ReconfiguratorNode()
    executor = MultiThreadedExecutor(4)
    executor.add_node(sub_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    print("Destroying node!")
    sub_node.destroy_node()
    print("Shutting down...")
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
