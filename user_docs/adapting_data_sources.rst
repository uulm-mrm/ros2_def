*********************
Adapting Data Sources
*********************

The data source integrates the orchestrator, and must instantiate it and provide a ROS node for it.
The ROS node must not be shared with functionality of the data source,
which usually results in 2 ROS nodes being instantiated in the data source.
Both nodes may be executed by the same executor, which must also be provided to the orchestrator
to enable it to process inputs while waiting for events.

The following changes need to be made:

* Call :py:meth:`.Orchestrator.initialize_ros_communication` once the data source's own publishers and subscribers are set up.
* Call :py:meth:`.Orchestrator.wait_until_publish_allowed` before publishing any message, and wait until the returned Future is ready before publishing. This does not apply to messages on the ``/clock`` topic.
* Call :py:meth:`.Orchestrator.wait_until_time_publish_allowed` with the specific message before publishing it on ``/clock``, and wait until the returned Future is ready before publishing.
* Call :py:meth:`.Orchestrator.wait_until_dataprovider_state_update_allowed` and wait until the returned Future is ready before changing internal state, such as by executing a simulation timestep which may depend on external inputs.
* Call :py:meth:`.Orchestrator.wait_until_pending_actions_complete` before exiting.


Orchestrator Class Reference
============================

.. autoclass:: orchestrator.lib.Orchestrator
    :members:
