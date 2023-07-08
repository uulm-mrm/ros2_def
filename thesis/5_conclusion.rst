.. _sec-conclusion:

Conclusion
==========

In this thesis, a method for repeatable execution of system tests for software stacks built using ROS was developed, implemented, and tested.
The orchestrator achieves this without modifying lower levels of the ROS client library stack or middleware, by controlling callback invocations at the ROS topic level which are the source of observed nondeterminism in the execution of ROS software stacks.
This is done while setting minimum requirements for the message-passing implementation, allowing in particular arbitrary transmission delay and reordering of messages.
An ordering for callbacks at each node which also synchronizes service calls and concurrent access to output topics between multiple nodes is ensured using incrementally constructed callback graphs.
The functionality of the implementation has been demonstrated using test cases for the distinct sources of nondeterminism in callback execution as well as with a real use case of running performance evaluation of a multi-object tracking module.
This example represents the intended use case of post-processing evaluation, using the same software stack as when conducting real-world tests (which is explicitly not an area of application for the orchestrator).

This use case has also been utilized to evaluate the continued additional effort in integrating the orchestrator with new and existing ROS components.
The integration of the orchestrator into an existing simulator and the implementation of a ROS bag player supporting the orchestrator has successfully been performed within this thesis.
The ongoing effort of modifying new nodes for use with the orchestrator has been found to be strongly dependent on the complexity of the input/output behavior of the node.
It ranged from no changes at all for simple nodes to larger modifications of the node's callback behavior such as changing from entirely ROS-independent execution to utilizing ROS timers.

Some limitations of the implemented approach have been identified.
The orchestrator may currently execute callbacks in a deterministic, but unexpected order, as seen in :ref:`sec:eval:verification:parallel_inputs`.
This is an effect of not requiring detailed information on the expected timing of callbacks and service calls, resulting in the implicit assumption that every callback has the same duration and makes service calls at the same point.
In :ref:`sec:eval:real_use_case:rosbag`, a node was not able to be fully utilized with the orchestrator due to the specific input/output behavior.
In particular, the behavior of publishing more than one message on a specified output topic in some callback invocations is currently not supported.
:ref:`sec:eval:execution_time` showed a significant increase in execution time when using the orchestrator, resulting in part from non-optimal callback serialization.
Both concurrent service calls (see :ref:`sec:eval:verification:service_calls`) and concurrent callbacks which publish to the same topic (:ref:`sec:eval:verification:multiple_publishers_on_topic`) currently serialize the entire originating callback, even if the concurrent access occurs only during a short fraction of the callback or while publishing an output.

\paragraph{Outlook}
Although the orchestrator is already useful in its current form, and using it to ensure the repeatability of automated testing of ROS components is planned, improvements in multiple areas are proposed.

As became apparent during the integration of the multi-object tracking module, it would be desirable to allow the configuration of a more complex input/output behavior than initially anticipated.
The implementation for combined callbacks using the message filters package supports one complex, stateful model for callback execution, but a more general solution might exist, which would allow the integration of more nodes with fewer modifications.
A possible solution for the specific problem encountered was proposed in :ref:`sec:eval:real_use_case:discussion`.
To further improve usability and ease the integration and maintenance of ROS nodes, automating some aspects of node and launch configuration files would be desirable.

Using static inspection or dynamic observation of a node during runtime could, for example, provide an initial version of a node description, or could detect divergence between an existing description and the observed behavior.
Such analysis is possible within ROS.
For instance, a method for inferring causal links between node inputs and outputs was recently proposed by \citeauthor*{Bedard23} in [Bedard23]_.
Launch configurations and existing ROS launch files currently duplicate a lot of information, with unexpected behavior if configurations such as topic remappings differ between both.
Reducing this redundancy would not only simplify the creation of the configuration file but also significantly reduce the potential for error while maintaining and changing both files.

A possible improvement to align the system behavior while using the orchestrator better to the behavior without the orchestrator could be to allow specifying an expected callback duration (see :ref:`sec:eval:verification:discussion`).
This would allow the orchestrator to order the callbacks not only deterministically, but also in the order one would generally expect without the orchestrator.

Reducing the execution time impact of using the orchestrator is considered to be important for adoption.
Approaches for improving the orchestrator's performance such as by multithreaded execution of orchestrator callbacks have been proposed in :ref:`sec:eval:execution_time:discussion`.
In :ref:`sec:eval:verification:discussion`, methods for improving parallel callback execution by explicitly intercepting service calls and node outputs have been identified.

