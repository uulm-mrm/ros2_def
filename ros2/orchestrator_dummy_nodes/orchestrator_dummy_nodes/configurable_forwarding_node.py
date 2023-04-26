#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from orchestrator_interfaces.msg import SampleMessage
from orchestrator_interfaces.srv import SetConfigurationService


class ConfigurableForwardingNode(Node):
    """
    Mock node which forwards a message from "input" to "output".
    Additionally supports changing input and output topic at runtime via services "~/set_input_topic" and
    "~/set_output_topic".
    """

    def __init__(self):
        super().__init__('ForwardingNode')  # type: ignore
        self.output_pub = self.create_publisher(SampleMessage, "output", 10)
        self.input_sub = self.create_subscription(SampleMessage, "input", self.callback, 10)
        self.input_config_srv = self.create_service(SetConfigurationService, '~/set_input_topic', self.set_input_topic)
        self.output_config_srv = self.create_service(SetConfigurationService, '~/set_output_topic',
                                                     self.set_output_topic)

    def set_input_topic(self, request: SetConfigurationService.Request, response):
        self.get_logger().info(f"Setting input topic to \"{request.config_value}\"")
        self.destroy_subscription(self.input_sub)
        self.input_sub = self.create_subscription(SampleMessage, request.config_value, self.callback, 10)
        response.success = True
        return response

    def set_output_topic(self, request: SetConfigurationService.Request, response):
        self.get_logger().info(f"Setting output topic to \"{request.config_value}\"")
        self.destroy_publisher(self.output_pub)
        self.output_pub = self.create_publisher(SampleMessage, request.config_value, 10)
        response.success = True
        return response

    def callback(self, msg: SampleMessage):
        self.get_logger().info(
            f"Forwarding message from \"{self.input_sub.topic_name}\" to \"{self.output_pub.topic_name}\"")
        self.output_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    test_node = ConfigurableForwardingNode()
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass

    test_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
