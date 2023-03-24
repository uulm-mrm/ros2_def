from typing import Optional, Tuple
from rclpy.task import Future
import rclpy
import networkx as nx
import rclpy.node
import matplotlib.pyplot as plt
from std_msgs.msg import String
from dataclasses import dataclass
import dataclasses

skip_topics = ["/rosout", "/parameter_events"]


def all_equal(iterator):
    iterator = iter(iterator)
    try:
        first = next(iterator)
    except StopIteration:
        return True
    return all(first == x for x in iterator)


@dataclass(frozen=True)
class Topic:
    name: str
    ros_type: str

    def __str__(self) -> str:
        return f"Topic {self.name}"


@dataclass(frozen=True)
class TopicWithFrequency:
    topic_name: str
    frequency_hz: float

    def __str__(self) -> str:
        return f"Topic {self.topic_name} at {self.frequency_hz} Hz"


@dataclass(frozen=True)
class TopicPerOutput:
    topic_name: str

    def __str__(self) -> str:
        return f"Topic {self.topic_name} for every output"

# TODO: Define expected inputs always per output?


@dataclass(frozen=True)
class Node:
    name: str
    # TODO: May not be fixed frequency?
    inputs: tuple[TopicWithFrequency | TopicPerOutput, ...] = dataclasses.field(default_factory=tuple)
    independent_outputs: tuple[TopicWithFrequency, ...] = dataclasses.field(default_factory=tuple)

    def __str__(self) -> str:
        return f"Node {self.name}"


def create_test_graph() -> nx.DiGraph:
    DG = nx.DiGraph()
    input_node = Node("Sensor", independent_outputs=(TopicWithFrequency("T1", 1),))
    sensor_topic = Topic("T1", "sensor_sample")
    DG.add_node(input_node)
    DG.add_node(sensor_topic)
    DG.add_edge(input_node, sensor_topic)
    perception_1 = Node("Perception 1", inputs=(TopicPerOutput("T1"),))
    detection_1 = Topic("T2", "detection")
    DG.add_node(perception_1)
    DG.add_edge(sensor_topic, perception_1)
    DG.add_node(detection_1)
    DG.add_edge(perception_1, detection_1)
    perception_2 = Node("Perception 2", inputs=(TopicPerOutput("T1"),))
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
    perception_3 = Node("Perception 3",  independent_outputs=(TopicWithFrequency("T5", 10),))
    detection_3 = Topic("T5", "detection")
    DG.add_node(perception_3)
    DG.add_node(detection_3)
    DG.add_edge(perception_3, detection_3)
    DG.add_edge(detection_3, tracking_node)

    return DG


def analyze(DG: nx.DiGraph):
    if len(list(nx.simple_cycles(DG))) > 1:
        raise RuntimeError("Graph has cycle!", nx.find_cycle(DG))

    known_sensors = ["Sensor", "publisher"]
    sensor_nodes = []
    for n in DG.nodes:
        if isinstance(n, Node):
            print(n.name)
        if isinstance(n, Node) and n.name in known_sensors:
            sensor_nodes.append(n)
    print("Sensor nodes:", sensor_nodes)

    merge_nodes = []

    for sn in sensor_nodes:
        print("Analyzing input from sensor node", sn)
        descendants = list(nx.descendants(DG, sn))
        descendants.sort(key=lambda n: nx.shortest_path_length(DG, sn, n)) # type: ignore
        for target in descendants:
            paths = list(nx.all_simple_paths(DG, sn, target))
            if len(paths) > 1:
                inputs = []
                for path in paths:
                    inputs.append(path[-2])
                if all_equal(inputs):
                    # TODO: Multiple inputs on same topic from different sources
                    continue
                print("Warning: Multiple paths to", target, ":", inputs)
                merge_nodes.append(target)


def draw_nodegraph(DG: nx.DiGraph):
    pos = nx.nx_agraph.graphviz_layout(DG, prog="sfdp")
    colors = ["lightcoral" if isinstance(n, Topic) else "cornflowerblue" for n in DG]
    nx.draw(DG, pos=pos, with_labels=True, node_color=colors)


class Analyzer(rclpy.node.Node):
    def __init__(self):
        super().__init__('analyzer')  # type: ignore
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        DG = nx.DiGraph()

        for topic, types in self.get_topic_names_and_types():
            if topic in skip_topics:
                continue
            for type in types:
                DG.add_node(Topic(topic, type))

        for node, namespace in self.get_node_names_and_namespaces():
            # TODO: Get input specification from file or something like that
            DG.add_node(Node(node, inputs=()))  # type: ignore

        subscriptions = {}
        for node, namespace in self.get_node_names_and_namespaces():
            subscriptions[node] = []
            for topic, types in self.get_subscriber_names_and_types_by_node(node, namespace):
                if topic in skip_topics:
                    continue
                subscriptions[node].append(topic)
                for type in types:
                    DG.add_edge(Topic(topic, type), Node(node, inputs=()))  # type: ignore

        publishers = {}
        for node, namespace in self.get_node_names_and_namespaces():
            publishers[node] = []
            for topic, types in self.get_publisher_names_and_types_by_node(node, namespace):
                if topic in skip_topics:
                    continue
                publishers[node].append(topic)
                for type in types:
                    DG.add_edge(Node(node, inputs=()), Topic(topic, type))  # type: ignore

        analyze(DG)
        draw_nodegraph(DG)
        plt.show()


def main(args=None):

    #DG = create_test_graph()
    # analyze(DG)
    # draw_nodegraph(DG)
    # plt.show()

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
