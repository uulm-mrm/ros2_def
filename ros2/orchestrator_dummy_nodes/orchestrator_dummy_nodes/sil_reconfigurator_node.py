#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from orchestrator_interfaces.msg import Status
from orchestrator_interfaces.srv import ReconfigurationAnnouncement, ReconfigurationRequest, SetConfigurationService


class ReconfiguratorNode(Node):

    def __init__(self) -> None:
        super().__init__('ReconfiguratorNode')  # type: ignore
        self.status_publisher = self.create_publisher(Status, "status", 10)

        self.timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.reconfig_timer = self.create_timer(
            1.0, self.timer_callback, callback_group=self.timer_cb_group)
        self.i = 0

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
        self.sil_config_client = self.create_client(SetConfigurationService, "sil_reconfigure",
                                                    callback_group=self.reconfig_cb_group)

        while not self.sil_config_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def publish_status(self):
        status_msg = Status()
        status_msg.node_name = self.get_name()
        self.get_logger().info("Sending Status")
        self.status_publisher.publish(status_msg)
        self.get_logger().info("Done sending Status")

    def timer_callback(self):
        self.get_logger().info("Timer callback")
        if self.i == 8:
            self.get_logger().info("Reconfiguring!")

            if self.reconfig_announce_client.service_is_ready():
                req = ReconfigurationAnnouncement.Request()
                res = self.reconfig_announce_client.call(req)
                self.get_logger().info(f"Got response: {res}")
            else:
                self.get_logger().warning("Reconfiguring without waiting!!!")
                self.reconfig_request_callback(
                    ReconfigurationRequest.Request(), ReconfigurationRequest.Response())
        else:
            self.get_logger().info("Not reconfiguring!")
        self.i += 1
        self.publish_status()
        self.get_logger().info("Timer callback done")

    def reconfig_request_callback(self, request, response):
        self.get_logger().info("Got request for reconfiguration!")

        self.get_logger().info("Reconfiguring SIL...")
        req = SetConfigurationService.Request()
        resp_a = self.sil_config_client.call(req)
        assert resp_a.success

        self.get_logger().info("Reconfiguration done. Replying to orchestrator.")
        response.new_launch_config_package = "platform_sil"
        response.new_launch_config_filename = "sil_reconfig_launch_config.json"
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

    sub_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
