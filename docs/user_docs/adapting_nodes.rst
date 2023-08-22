**************
Adapting Nodes
**************

Node Requirements
=================

Ensure the node meets the required execution behavior:

* All execution must happen in ROS callbacks.
  Those can be subscription-, timer- and service-callbacks.
  Execution not inside ROS callbacks might include things such as a separate main-loop possibly in a separate thread, this is not allowed as it can not be controlled by the executor.
  It is permissible to perform processing in a separate thread, if that processing is only ever triggered by a ROS callback, and as such could be modeled or implemented as a single ROS callback.
* Observable node behavior must be deterministic.
  Specifically, the sequence of (ROS-)outputs (outgoing service calls and published messages) must remain exactly the same if the same sequence of inputs is applied again.
  This implicitly forbids reading data from sensors or devices which may change at any point.
  Stochastic processes depending on randomness might be used if the pseudo-RNG is deterministically seeded.
* Inputs and outputs must be causally related, in a way that is describeable by the config file, see below.
  Callbacks may omit one or multiple outputs in some invocations, see the status message.

.. _user_docs-status-outputs:

Status Outputs
==============

.. warning::
  TODO: Status outputs

.. _user_docs-omitted-outputs:

Omitted Outputs
---------------

Status messages may list ``omitted_outputs``.
This is a list of topic outputs which would usually occur during the current callback, but will not occur in this specific callback invocation.
It must contain remapped topic names, not internal names.
The actual topic name is available from ROS publishers in ``rclcpp`` (:external+rclcpp:cpp:func:`rclcpp::PublisherBase::get_topic_name <rclcpp::PublisherBase::get_topic_name>`) and ``rclpy`` (:external+rclpy:py:attr:`rclpy.publisher.Publisher.topic_name`):

.. code-block:: cpp

  status_msg.omitted_outputs
    .push_back(this->publisher->get_topic_name());

.. code-block:: python

  status_msg.omitted_outputs = [self.publisher.topic_name]

JSON Node Behavior Description
==============================

Node behavior needs to be described statically in a JSON configuration file.
The file must adhere to the following schema (which is available at `node_config_schema.json <https://github.com/uulm-mrm/ros2_def/blob/develop/ros2/orchestrator/schemas/node_config_schema.json>`_ for IDE integration):

.. jsonschema:: ../../ros2/orchestrator/schemas/node_config_schema.json
