from dataclasses import dataclass
import networkx as nx

@dataclass
class NodeData:
    name: str

DG = nx.DiGraph()
DG.add_node(1, data=NodeData("one"))
DG.add_node(2, data=NodeData("two"))
DG.add_node(3, data=NodeData("three"))

for node, data in DG.nodes(data=True):
    print(node, data["data"])