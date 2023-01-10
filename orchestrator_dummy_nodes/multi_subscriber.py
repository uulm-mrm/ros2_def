import rclpy
from rclpy.node import Node
from enum import Enum
from std_msgs.msg import String
from dataclasses import dataclass

from orchestrator_interfaces.msg import Status

class State(Enum):
    IN_ORDER = 1
    OUT_OF_ORDER = 2


@dataclass
class ReceiveBlockState:
    identifier: str
    state: State
    received_in_order: int


class MultiSubscriber(Node):
    """
    Node with multiple subscribers to topic_N, expecting to receive the same message on each topic in order
    """

    def __init__(self):
        super().__init__('minimal_subscriber')

        self.declare_parameter('nr_publishers', 6)
        nr_publishers = self.get_parameter('nr_publishers').get_parameter_value().integer_value

        self.declare_parameter('intercepted', False)

        self.state = ReceiveBlockState("", State.OUT_OF_ORDER, 0)
        self.block_done = True

        self.all_subscriptions = [self.create_subscription(
            String, f"intercepted__sub__{self.get_name()}__topic_{i}" if self.get_parameter("intercepted").get_parameter_value().bool_value else f"topic_{i}",
            lambda msg, i=i: self.listener_callback(i, msg), 10)
            for i in range(nr_publishers)]

        self.status_publisher = self.create_publisher(Status, "status", 10)

    def listener_callback(self, i: int, msg: String):
        #self.get_logger().info(f"Sub {i}: {msg.data}")
        if msg.data == self.state.identifier:
            if self.block_done:
                #self.get_logger().error(f"Received additional message for completed block? \"{msg.data}\"")
                pass
            else:
                if self.state.state == State.IN_ORDER:
                    # Block was in order so far
                    if i == self.state.received_in_order:
                        # Block is still in order
                        #self.get_logger().info("Received in order message")
                        self.state.received_in_order += 1
                        if (self.state.received_in_order == len(self.all_subscriptions)):
                            self.block_done = True
                            self.get_logger().info("Received entire block in order")
                    else:
                        # Block is now out of order
                        self.get_logger().warn("Received OOO message, this block is now OOO!")
                        self.state.state = State.OUT_OF_ORDER
                else:
                    #self.get_logger().info("Received additional message for already OOO block.")
                    pass
        else:
            #self.get_logger().info(f"New block \"{msg.data}\":")
            self.block_done = False
            if i == 0:
                #self.get_logger().info("Starting in-order")
                self.state = ReceiveBlockState(msg.data, State.IN_ORDER, 1)
            else:
                self.get_logger().warn("Starting out of order")
                self.state = ReceiveBlockState(msg.data, State.OUT_OF_ORDER, 0)
        status_msg = Status()
        status_msg.node_name = self.get_name()
        self.status_publisher.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MultiSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
