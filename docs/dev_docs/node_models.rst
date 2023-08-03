***********
Node Models
***********

Node models specify behavior of ROS nodes.
Behavior is specified by providing a list of possible inputs, and the resulting actions in response to each input.
The implementation allows multiple node model classes inheriting from the (TODO: node model abstract class) class.
The only node model class currently in use is (TODO: file node model class) which allows specifying behavior in a JSON file
(refer to :ref:`sec-impl-node_system_description` for examples).
