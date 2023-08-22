*********
Debugging
*********

State Sequences
===============

This is a debugging tool which first records all inputs which were sent to a specific node
and then compares inputs for the node with the expected inputs during a second run.

:py:meth:`orchestrator.lib.Orchestrator.dump_state_sequence`

Debug Service
=============

It might happen that the orchestrator appears "stuck" at some point during execution.
This typically happens when information about the completion of some callback has not reached the orchestrator,
and it thus assumes that the corresponding node is still running.
When the only remaining actions all depend on this apparently still running callback, the orchestrator deadlocks.
To verify that this is the case, identify the running action in the callback graph, by increasing the orchestrator log
level to ``debug``, which prints the appropriate information to the console, or dump the entire callback graph using
the ROS service once execution is stalled (the latter method is generally preferred):

.. code-block:: console

    $ ros2 service call /<o_node>/get_debug std_srvs/srv/Trigger {}

The service name is ``~/get_debug`` relative to the orchestrator node, so the actual name might differ.
If there exists one or multiple actions in the ``RUNNING`` state, check if those nodes are actually still processing an input
(such as by inspecting the nodes own logs).

If this is the case, a missmatch exists between the node's behavior and the node description.
The typical cause is an early exit of the callback, meaning the required :ref:`status message <user_docs-status-outputs>` is not published, or is missing :ref:`omitted outputs <user_docs-omitted-outputs>`.
