from typing import Optional
from rclpy.task import Future
import rclpy
import networkx as nx
from rclpy.node import Node
import matplotlib.pyplot as plt
from std_msgs.msg import String
from dataclasses import dataclass

skip_topics = ["/rosout", "/parameter_events"]


@dataclass(frozen=True)
class Topic:
    name: str
    ros_type: str


@dataclass(frozen=True)
class Node:
    name: str


def create_test_graph() -> nx.DiGraph:
    DG = nx.DiGraph()
    input_node = Node("Sensor")
    sensor_topic = Topic("T1", "sensor_sample")
    DG.add_node(input_node)
    DG.add_node(sensor_topic)
    DG.add_edge(input_node, sensor_topic)
    perception_1 = Node("Perception 1")
    detection_1 = Topic("T2", "detection")
    DG.add_node(perception_1)
    DG.add_edge(sensor_topic, perception_1)
    DG.add_node(detection_1)
    DG.add_edge(perception_1, detection_1)
    perception_2 = Node("Perception 2")
    detection_2 = Topic("T3", "detection")
    DG.add_node(perception_2)
    DG.add_edge(sensor_topic, perception_2)
    DG.add_node(detection_2)
    DG.add_edge(perception_2, detection_2)
    tracking_node = Node("Tracking")
    tracking_result = Topic("T4", "tracks")
    DG.add_node(tracking_node)
    DG.add_edge(detection_1, tracking_node)
    DG.add_edge(detection_2, tracking_node)
    DG.add_node(tracking_result)
    DG.add_edge(tracking_node, tracking_result)
    perception_3 = Node("Perception 3")
    detection_3 = Topic("T5", "detection")
    DG.add_node(perception_3)
    DG.add_node(detection_3)
    DG.add_edge(perception_3, detection_3)
    DG.add_edge(detection_3, tracking_node)

    return DG


def analyze(DG: nx.DiGraph):
    event_list = []
    sensor_nodes = []
    for n in DG.nodes:
        if isinstance(n, Node) and DG.in_degree(n) == 0:
            sensor_nodes.append(n)
    print("Sensor nodes:", sensor_nodes)

    for sn in sensor_nodes:
        print("Analyzing input from sensor node", sn)
        for n in DG.successors(sn):
            print(n)


class Analyzer(Node):
    def __init__(self):
        super().__init__('analyzer')
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        DG = nx.DiGraph()

        for topic, types in self.get_topic_names_and_types():
            if topic in skip_topics:
                continue
            DG.add_node(topic)

        for node, namespace in self.get_node_names_and_namespaces():
            DG.add_node(node)

        subscriptions = {}
        for node, namespace in self.get_node_names_and_namespaces():
            subscriptions[node] = []
            for topic, types in self.get_subscriber_names_and_types_by_node(node, namespace):
                if topic in skip_topics:
                    continue
                subscriptions[node].append(topic)
                DG.add_edge(topic, node)

        publishers = {}
        for node, namespace in self.get_node_names_and_namespaces():
            publishers[node] = []
            for topic, types in self.get_publisher_names_and_types_by_node(node, namespace):
                if topic in skip_topics:
                    continue
                publishers[node].append(topic)
                DG.add_edge(node, topic)

        pos = nx.nx_agraph.graphviz_layout(DG)
        nx.draw(DG, pos=pos, with_labels=True)
        plt.show()


def main(args=None):

    DG = create_test_graph()
    pos = nx.nx_agraph.graphviz_layout(DG, prog = "sfdp")
    nx.draw(DG, pos=pos, with_labels=True)
    analyze(DG)
    plt.show()
    exit(0)

    rclpy.init(args=args)

    analyzer = Analyzer()

    rclpy.spin(analyzer)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    analyzer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
