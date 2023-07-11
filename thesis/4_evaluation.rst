.. _sec-eval:

**********
Evaluation
**********

.. contents::
   :local:

In this chapter, the functionality and applicability of the proposed framework will be evaluated.
In :ref:`sec-eval-verification`, the behavior with and without the orchestrator in the minimal examples presented in :ref:`sec:impl:nondet_sources` is verified.
:ref:`sec-eval-system_setup` then introduces the experimental setup used for further evaluation, which represents a real use case utilizing an existing autonomous-driving software stack.
The process of integrating the existing components with the orchestrator is covered in :ref:`sec-eval-system_integration`, followed by an evaluation and discussion of using the orchestrator in the presented use case in :ref:`sec-eval-real_use_case`.
In :ref:`sec-eval-execution_time`, the impact of using the orchestrator on execution time is explicitly assessed, and approaches for improvement are discussed.

.. _sec-eval-verification:

Verification of Functionality
=============================

In order to verify the functionality of the orchestrator, without depending on existing ROS node implementations,
individual test cases for specific sources of nondeterminism as well as a combined mockup of an autonomous
driving software stack were developed.
In the following, each of the examples containing sources of nondeterministic callback sequences identified in :ref:`sec:impl:nondet_sources`
is individually evaluated.

Lost or Reordered Messages
--------------------------

To verify that the problem of lost messages due to overflowing subscriber queues, as introduced in :ref:`sec:impl:nondet_sources:reordering`, is solved by the orchestrator, a test case was set up:
A data source :math:`S` publishes messages at a fixed frequency.
The messages are received by the node under test :math:`P`, which has a fixed queue size (of three messages in this example), and a varying processing time that on average is significantly slower than the period of message publishing.
After processing, :math:`P` publishes the result on a different topic.
This behavior might correspond to a simulator running at a fixed frequency, and a computationally expensive processing component such as a perception module, running on a resource-constrained system.


.. figure:: tikz_figures/eval-reordering-timeline.png
    :name: fig-eval-reordering-timeline

    Sequence diagram showing dropped messages due to subscriber queue overflow, with a subscriber queue size of 3 at :math:`P`. The corresponding ROS graph is shown in :numref:`fig-nodegraph-example_reordering`.

:numref:`fig-eval-reordering-timeline` shows the sequence of events when running this test:
The first timeline shows the periodic publishing of input messages by :math:`S`.
The second timeline shows the callback duration of node :math:`P`.
It can be seen that once the processing of the first message finishes, processing immediately continues for message 5, which is the third-recent message published at that point in time, skipping messages 2, 3, and 4 which were published during processing.
During the processing of message 5, four further messages are discarded.
The exact number of skipped messages depends on the callback duration, which in this case is deliberately randomized but is usually highly dependent on external factors such as system load.

.. figure:: tikz_figures/eval-reordering-timeline_orchestrator.png
    :name: fig-eval-reordering-timeline_orchestrator

    Sequence diagram showing a slowdown of the data source to prevent dropping messages by overflowing the subscriber queue.

When using the orchestrator, the message publisher is still configured to the same publishing rate, but waits for the orchestrator before publishing each message.
:numref:`fig-eval-reordering-timeline_orchestrator` shows that each message is now processed, regardless of callback duration.
This necessarily slows down the data source, which can not be avoided without risking dropping messages from the subscription queue at the receiving node.

By only sending messages to a node once the processing of the previous message is completed, reordering of messages by the middleware is also prevented.
This is not explicitly demonstrated here but follows immediately from the fact that only one message per topic is being transmitted at any point in time.

.. _sec-eval-verification-parallel_inputs:

Inputs From Parallel Processing Chains
--------------------------------------

.. figure:: tikz_figures/eval-parallel_inputs-sequence.png
    :name: fig-eval-parallel_inputs-sequence

    Sequence diagram showing the execution of two parallel processing nodes :math:`P1` and :math:`P2` with nondeterministic processing time.
    This results in a nondeterministic callback order at :math:`T`, which subscribes to the outputs of both chains.
    The corresponding ROS graph is shown in :numref:`fig-nodegraph-example_parallel_nodes`.

To verify deterministic callback execution at a node with multiple parallel inputs, the example introduced in :ref:`sec-impl-nondet_sources-parallel` with the ROS graph shown in :numref:`fig-nodegraph-example_parallel_nodes` is realized.
:numref:`fig-eval-parallel_inputs-sequence` shows all callback invocations resulting from
two inputs from :math:`S`.
Without the orchestrator, the combination of nondeterministic transmission latency and variable duration of callback execution at :math:`P1` and :math:`P2` results in a nondeterministic execution order of both callbacks at :math:`T` resulting from one input from :math:`S`.

For input 1, :math:`P1` finishes processing before :math:`P2`, and no significant transmission
latency occurs, which causes :math:`T` to process the message on :math:`D1` before :math:`D2`.
Following input 2, :math:`P2` is slightly faster than :math:`P1` resulting in a different callback order
compared to the first input.


\def\xshift{2.8}
\def\xscale{3.5}
\begin{figure}
    \centering
    \begin{tikzpicture}
        % Timelines
        \timeline{S}{0}{11.5};
        \timeline{P1}{1}{11.5};
        \timeline{P2}{2}{11.5};
        \timeline{T}{3}{11.5};

        % Message connections S -> P1
        \foreach \s / \t in {2.989739063/2.995100707, 3.990143856/3.994808526, 4.990577550/4.996000168} {
            \connectingarrow{0}{\s}{1}{\t}
        }

        % Message connections S -> P2
        \foreach \s / \t in {2.989739063/2.995939516, 3.990143856/3.995178942, 4.990577550/4.997096392} {
            \connectingarrow{0}{\s}{2}{\t}
        }

        % Message connections P1 -> T
        \foreach \s / \t in {3.284122584/3.287048997, 4.275910257/4.279247917, 5.433419724/5.436461745} {
            \connectingarrow{1}{\s}{3}{\t}
        }

        % Message connections P2 -> T
        \foreach \s / \t in {3.266444216/3.491431238, 4.285967486/4.484018241, 5.252872086/5.641027683} {
            \connectingarrow{2}{\s}{3}{\t}
        }

        % P1
        \foreach \s / \e in {2.995100707/3.284122584, 3.994808526/4.275910257, 4.996000168/5.433419724} {
            \callbackinvocation{\s}{\e}{1}{uulm_blue}
        }

        % P2
        \foreach \s / \e in {2.995939516/3.266444216, 3.995178942/4.285967486, 4.997096392/5.252872086} {
            \callbackinvocation{\s}{\e}{2}{uulm_orange}
        }

        % T callbacks A
        \foreach \s / \e in {3.287048997/3.488516435, 4.279247917/4.480807130, 5.436461745/5.637960715} {
            \callbackinvocation{\s}{\e}{3}{uulm_blue}
        }

        % T callbacks B
        \foreach \s / \e in {3.491431238/3.693045765, 4.484018241/4.685469140, 5.641027683/5.842429041} {
            \callbackinvocation{\s}{\e}{3}{uulm_orange}
        }

        % Publish events
        \foreach \x [count=\i] in {2.989739063, 3.990143856, 4.990577550} {
            \datainput{\x};
            \messageid{\x}{\i};
        }
    \end{tikzpicture}
    \caption[Sequence diagram showing a deterministic callback order at :math:`T` despite nondeterministic callback durations at :math:`P1` and :math:`P2`.]{Sequence diagram showing a deterministic callback order at :math:`T` despite nondeterministic callback durations at :math:`P1` and :math:`P2` as an effect of the orchestrator on the behavior shown in :numref:`fig-eval-parallel_inputs:sequence`.}
    \label{fig-eval-parallel_inputs:sequence_orchestrator}
\end{figure}

Using the orchestrator, the callback order changes, as visualized in :numref:`fig-eval-parallel_inputs:sequence_orchestrator`.
For the first and third data input, :math:`P1` requires more processing time than :math:`P2`.
This would ordinarily allow the :math:`D2` callback at :math:`T` to execute before the :math:`D1` callback.
The orchestrator however ensures a deterministic callback order at :math:`T` for every data input from :math:`S`, by buffering the :math:`D2` message until :math:`T` finishes processing :math:`D1`.
Note that the orchestrator does not implement a specific callback order defined by the node or externally.
It only ensures that the order is consistent over multiple executions.
The actual order results from the order in which nodes and callbacks are listed in configuration files, but this is not intended to be adjusted by the user.
If a node requires a distinct receive order, it must implement appropriate ordering internally, to ensure correct operation without the orchestrator.
From the point of the orchestrator, consistently ordering :math:`P2` before :math:`P1` would have also been a valid solution.

.. _sec-eval-verification-multiple_publishers_on_topic:

Multiple Publishers on the Same Topic
-------------------------------------

\def\xshift{9.0}
\def\xscale{3.5}
\begin{figure}[h]
    \centering
    \begin{tikzpicture}
        % Timelines
        \timeline{S}{0}{11.5};
        \timeline{P1}{1}{11.5};
        \timeline{P2}{2}{11.5};
        \timeline{T}{3}{11.5};

        % S -> P1
        \foreach \s / \e in {9.083803676/9.089759276, 10.084325316/10.088845419, 11.084700822/11.089589537} {
            \connectingarrow{0}{\s}{1}{\e}
        }

        % P1 -> T
        \foreach \s / \e in {9.342956161/9.346333677, 10.546876890/10.550012168, 11.415952459/11.419019166} {
            \connectingarrow{1}{\s}{3}{\e}
        }

        % P2 -> T
        \foreach \s / \e in {9.628776904/9.631653622, 11.009939621/11.013125831, 11.691255863/11.694560206} {
            \connectingarrow{2}{\s}{3}{\e}
        }

        % S -> P2
        \foreach \s / \e in {9.083803676/9.346628742, 10.084325316/10.550319789, 11.084700822/11.419394500} {
            \connectingarrow{0}{\s}{2}{\e}
        }

        % P1
        \foreach \s / \e in {9.089759276/9.342956161, 10.088845419/10.546876890, 11.089589537/11.415952459} {
            \callbackinvocation{\s}{\e}{1}{uulm_blue}
        }

        % P2
        \foreach \s / \e in {9.346628742/9.628776904, 10.550319789/11.009939621, 11.419394500/11.691255863} {
            \callbackinvocation{\s}{\e}{2}{uulm_orange}
        }

        % T callbacks A
        \foreach \s / \e in {9.346333677/9.547935381, 10.550012168/10.751362743, 11.419019166/11.620475027} {
            \callbackinvocation{\s}{\e}{3}{uulm_blue}
        }

        % T callbacks B
        \foreach \s / \e in {9.631653622/9.833220960, 11.013125831/11.214068479, 11.694560206/11.896025151} {
            \callbackinvocation{\s}{\e}{3}{uulm_orange}
        }

        % Publish events
        \foreach \x [count=\i] in {9.083803676, 10.084325316, 11.084700822} {
            \datainput{\x};
            \messageid{\x}{\i};
        }
    \end{tikzpicture}
    \caption[Sequence diagram showing serialized callback executions of nodes :math:`P1` and :math:`P2`, which is required to achieve a deterministic callback order.]{Sequence diagram showing serialized callback executions of nodes :math:`P1` and :math:`P2`, which is required to achieve a deterministic callback order at :math:`T` in this example, since :math:`P1` and :math:`P2` use the same output topic.
    The corresponding ROS graph is shown in :numref:`fig-nodegraph-example_multiple_publishers`.}
    \label{fig-eval-same_output:sequence_orchestrator}
\end{figure}

This example extends the previous scenario from :ref:`sec-eval-verification:parallel_inputs` such that both processing nodes publish their result on the same topic, corresponding to the example introduced in :ref:`sec:impl:nondet_sources:multiple_publishers`, with the ROS graph shown in :numref:`fig-nodegraph-example_multiple_publishers`.
Again, this results in nondeterministic callback order at :math:`T`, with a callback order identical to the previous case shown in :numref:`fig-eval-parallel_inputs:sequence`.
In this case, both callback executions at :math:`T` are of the same callback, while previously two distinct callbacks were executed once each.

Because only node *inputs* are intercepted, this scenario requires serializing the callbacks at :math:`P1` and :math:`P2`.
:numref:`fig-eval-same_output:sequence_orchestrator` shows the resulting callback sequence when using the orchestrator.
By ensuring that processing at :math:`P2` only starts after the output from :math:`P1` is received, reordering of the messages on :math:`D` is prevented.
Note that while the different colors of the callbacks at :math:`T` correspond to the sources of the corresponding input, both inputs cause the same subscription callback to be executed at the node.
Generally, the node would not be able to determine the source of the input message.

Since the processing time of :math:`P2` is longer than the processing time of the first callback at :math:`T` in this example, the orchestrator causes a larger overhead for this node graph compared to the previous one.
:math:`P2` starts processing simultaneously to the first :math:`T` callback, causing :math:`T` to be idle between the completion of the first callback and the completion of processing at :math:`P2`.
It should be noted, however, that even though the total processing time exceeds the input frequency of :math:`S` for input 2, the data source was not required to slow down.
:numref:`fig-eval-same_output:sequence_orchestrator` shows that :math:`T` is still running while :math:`P1` processes input 3.
This kind of "pipelining" happens implicitly because the callback execution at :math:`P1` has no dependency on the callback at :math:`T`, and by eagerly allowing inputs from :math:`S`.
In the current implementation, the orchestrator requests the publishing of the next message by the data provider as soon as the processing of the last input on the same topic has started.
In the case of a time input, the input is requested as soon as no actions remain which are still waiting on an input of a previous time update.
Both kinds of input may additionally be delayed if the system is pending dynamic reconfiguration, or if a callback is still running that may cause a reconfiguration at the end of the current timestep.

.. _sec-eval-verification-service_calls:

Parallel Service Calls
--------------------

.. figure:: tikz_figures/eval-service-sequence_before.png
   :name: fig-eval-service-sequence_before

   Sequence diagram showing the parallel execution of callbacks at :math:`N1` and :math:`N2`.
   The hatched area within the callback shows the duration of service calls, which are made to a service provided by :math:`SP`, upwards arrows represent responses to service calls.
   The variable timing of the service calls results in a nondeterministic callback order at :math:`SP`.
   The corresponding ROS graph is shown in :numref:`fig-nodegraph-example_service_calls`.

:numref:`fig-nodegraph-example_service_calls` shows the node setup for this example, which has been identified in :ref:`sec:impl:nondet_sources:service_calls`.
A single message triggers a callback at three nodes, one of which (:math:`SP`) also provides a ROS service.
The two other nodes :math:`N1` and :math:`N2` call the provided service during callback execution.
The resulting order of all three callbacks at :math:`SP` in response to a single message input is nondeterministic, as shown in :numref:`fig-eval-service-sequence_before`.
Since the orchestrator only controls service calls by controlling the callback they originate from, it is necessary to serialize all callbacks interacting with the service, which in this case are the message callbacks at :math:`N1`, :math:`N2`, and :math:`SP`.

.. figure:: tikz_figures/eval-service-sequence_orchestrator.png
   :name: fig-eval-service-sequence_orchestrator

   Sequence diagram showing the serialized callbacks from :numref:`fig-eval-service-sequence_before`. Serialization of the callbacks at :math:`N1` and :math:`N2` leads to a deterministic callback order at :math:`SP`.

The resulting callback sequence is shown in :numref:`fig-eval-service-sequence_orchestrator`.
By serializing the callbacks at :math:`N1` and :math:`N2`, the order of service callbacks at :math:`SP` is now fixed.
In this example, it is again apparent that parallel execution of the :math:`N1` and :math:`N2` callbacks might be possible while still maintaining a deterministic callback order at :math:`SP`.
This limitation is discussed in detail in :ref:`sec-eval-verification-discussion`.

.. _sec-eval-verification-discussion:

Discussion
----------

The ability of the orchestrator to ensure a deterministic callback sequence at all nodes has been shown for the minimal nondeterministic examples which were identified in :ref:`sec:impl:nondet_sources`.
While all examples show successful deterministic execution, some limitations and possible improvements in parallel callback execution and thereby execution time are apparent and will be discussed in the following.

In the case of concurrent callbacks which publish on the same topic, parallelism could further be improved by extending the topic interception strategy.
Currently, only the input topics of each node are intercepted by the orchestrator, the output topics are not changed.
If the output topics of nodes were also remapped to individual topics, all ``SAME_TOPIC`` dependencies would be eliminated.
In the example from :numref:`fig-eval-parallel_inputs:sequence_orchestrator`, this would again allow the concurrent callbacks :math:`P1` and :math:`P2` to execute in parallel, with each output being individually buffered at the orchestrator.
The individually and uniquely buffered outputs could then be forwarded to :math:`T` in a deterministic order, effectively resulting in a callback execution behavior as in :ref:`sec-eval-verification:parallel_inputs`.

The last example of concurrent service calls (:ref:`sec-eval-verification:service_calls`) also shows how this method of ensuring deterministic execution comes with a significant runtime penalty.
Here, the orchestrator now requires all callbacks to execute sequentially, while previously all callbacks started executing in parallel, with the only point of synchronization being the service provider, depending on available parallel callback execution within the node.
An important factor determining the impact of this is the proportion of service-call duration to total callback duration for the calling nodes.
If the service call is expected to take only a small fraction of the entire callback duration, a large improvement in execution time could be gained by allowing parallel execution of the callbacks :math:`N1` and :math:`N2`, which both call the service.
This might be possible by explicitly controlling service calls directly instead of controlling the entire callback executing that call.
In the example shown in :numref:`fig-eval-service-sequence_orchestrator`, serializing only the service calls would allow the portion of the :math:`N2` callback before the service call to execute concurrently to :math:`N1`, and the portion after the service call to overlap with the message callback at :math:`SP`.

Another possible extension to improve parallelism in scenarios involving service calls is to allow specifying that some actions might interact with the service provider without modifying its state.
Currently, all actions interacting with the service (by running at the same node, or calling the service) are assumed to modify the service provider state.
To ensure deterministic execution, synchronization between non-modifying actions is however not required.
If an action only inspects the service providers' state without modifying it, the order with respect to other such actions would not influence its result.
Thus, it would suffice to synchronize non-modifying actions with previous modifying actions,
instead of all previous actions.

In :ref:`sec-eval-verification:parallel_inputs`, it was identified that although the callback order at each node is not deterministic, a different order of callbacks in response to a single input might be expected during normal operation.
This does not reduce the applicability of the orchestrator, since nodes that explicitly require a specific callback order must implement measures to ensure that anyways.
It is however still desirable to keep the system behavior when using the orchestrator as close as possible to the expected or usual system behavior without the orchestrator.
One proposed future addition is thus allowing nodes to optionally specify an expected callback duration in the corresponding configuration file.
This information may then be used by the orchestrator to establish a more realistic callback ordering.

.. _sec-eval-system_setup:

System Setup
============

In the following, the integration of the orchestrator with parts of an already existing autonomous driving software stack is evaluated.
This section introduces the system setup and example use case, which will be utilized in :ref:`sec-eval-system_integration,sec-eval-real_use_case`.

\begin{figure}
    \centering
    \begin{tikzpicture}[
        % https://tex.stackexchange.com/a/125468/143051
        buswidth1/.style={decoration={
            markings,
            mark= at position 0.85 with {\node[font=\normalsize] {/};\node[below=1pt,xshift=2pt] {\scriptsize #1};}
        }, postaction={decorate}},
        buswidth2/.style={decoration={
            markings,
            mark= at position 0.5 with {\node[font=\normalsize] {/};\node[below=1pt,xshift=3pt] {\scriptsize #1};}
        }, postaction={decorate}},
        align=center,
        font={\small}
    ]
        % \draw[step=1cm,gray,very thin] (-5,-5) grid (5,1);
        \node (sim) at (0,0) [rosnode] {Simulator};
        \node (tracking_local) at (3.5,-5) [rosnode] {Vehicle\\Tracking};
        \node (planning) at (3.5,0) [rosnode] {Trajectory\\Planning};
        \node (egomotion) at (3.5,-2.5) [rosnode] {Egomotion};
        \node (tracking_external) at (-3.5,0) [rosnode] {External\\Tracking};
        \node (recorder_tracking) at (-7,0) [rosnode] {Tracking\\Recorder};
        \node (recorder_gt) at (-3.5,-2.5) [rosnode] {Ground Truth\\Recorder};

        \draw [arrow, buswidth2={12}] (sim) -- (tracking_external);
        \draw [arrow] (tracking_external) -- (recorder_tracking);
        \draw [arrow] (sim.240) |- (recorder_gt);

        \draw [arrow, buswidth1={5}] (sim) |- (tracking_local);
        \draw [arrow] (sim.300) |- (egomotion);
        \draw [arrow] (sim) -- (planning);
        \draw [arrow, dashed] (planning) -- (egomotion);
        \draw [arrow, dashed] (tracking_local) -- (egomotion);
        \draw [arrow] (planning) -- (3.5,1) -| (sim);
    \end{tikzpicture}
    \caption[Node graph of the system setup used within :ref:`sec:eval`.]{Node graph of the system setup used within this chapter. The connections between the simulator and both tracking nodes represent multiple parallel ROS topics. Dashed arrows show potential service calls.}
    \label{fig-eval-sil_nodegraph}
\end{figure}

In this use case, the aim is to calculate metrics on the performance of a multi-object tracking module, which tracks vehicles that pass an intersection using infrastructure-mounted sensors.
The ROS graph of the setup is shown in :numref:`fig-eval-sil_nodegraph`.
The software stack consists of this tracking module, as well as components required to autonomously control one of the vehicles passing the intersection in the test scenario.
A simulator provides measurements in the form of (possibly incomplete) bounding boxes and object class estimations, simulating both the sensor itself as well as an object detection algorithm.
Alternatively, the same measurements are played back from a ROS bag.
The tracking module receives measurements on a total of 12 individual topics for each sensor.
Outputs from the tracking module, as well as ground truth object states provided by the simulator, are recorded by dedicated recorder nodes.
This allows later post-processing and evaluation.

The part of the software stack controlling the autonomous vehicle consists of a second instance of the tracking module, a component estimating the vehicle's ego-motion as well as a trajectory planning and control module.
The vehicle-local tracking module receives measurements from five simulated on-vehicle sensors similar to the infrastructure tracking module.
The planning module receives information about the vehicle state from the simulator and produces acceleration and steering angle commands which are fed back to the simulator.
Both the planning and local tracking modules may call the ego-motion service provided by the corresponding node while executing any callback.
The other vehicles present in the scenario are fully controlled by the simulator.

\begin{minipage}{\linewidth}
The simulation is run until the controlled vehicle reaches a predefined area.
When using recorded measurement data from a ROS bag, the scenario ends once every recorded measurement has been processed.
The recorded results of the tracking module and the recorded ground truth data are then used to calculate application-specific metrics to assess the performance of the multi-object tracking algorithm.
\end{minipage}

.. _sec-eval-system_integration:

System Integration
==================

To determine the feasibility of integrating the proposed framework into existing software,
the framework was applied to the scenario for testing a multi-object tracking module introduced in :ref:`sec-eval-system_setup`.
In this section, the necessary modifications to each existing component are discussed.
:ref:`sec-eval-system_integration:simulator,sec-eval-system_integration:bag_player` will cover the integration of both "data provider" components, a simulator, and the ROS bag player, which will contain the orchestrator.
:ref:`sec-eval-system_integration:ros_nodes` covers the integration of the ROS nodes present in
the test scenario.

.. _sec-eval-system_integration-simulator:

Simulator
---------

The orchestrator represents an individual component (see :ref:`sec:impl:controlling_callbacks`),
but is located within the same process as the data provider,
which in this case is the simulator.

The orchestrator component is instantiated within the simulator and then provides an API that the simulator must call at specific points to ensure deterministic execution.
To instantiate and start the orchestrator, the simulator must also provide the orchestrator with the appropriate launch configuration.
All API calls are of the form ``wait_until_<condition>`` and usually return a ``Future`` object that must be awaited before executing the corresponding actions.
The ``wait_until_publish_allowed`` function must be inserted before publishing any ROS message on any topic.
Before publishing a ``/clock`` message, the new time must be provided to the orchestrator using the dedicated ``wait_until_time_publish_allowed`` API call, which is required for the orchestrator to prepare for eventual timer callbacks.
Before changing the internal simulation state, the ``wait_until_dataprovider_state_update_allowed`` method must be called.
This usually happens by performing a simulation timestep, and this method ensures synchronizing this timestep with expected inputs present in a closed-loop simulation, such as vehicle control inputs.
The ``wait_until_pending_actions_complete`` method is used to ensure all callbacks finish cleanly once the simulation is done.

To enable closed-loop simulation, the simulator must accept some input from the software under test, such as a control signal for an autonomous vehicle in this case.
This implies a subscription callback, which must be described in a node configuration file.
If this callback does not publish any further messages, a status message must be published instead.

.. _sec-eval-system_integration-bag_player:

ROS Bag Player
--------------

ROS already provides a ROS bag player, which could be modified to include the orchestrator.
Modifying the official ROS bag player would have the advantage of keeping access to the large set of features already implemented, and preserving the known user interface.
Some aspects of the official player increase the integration effort considerably, however.
Specifically, publishing of the ``/clock`` topic is asynchronous to message playback and at a fixed rate.
While this has some advantages for interactive use, it interferes with deterministic execution and would require a significant change in design to accommodate the orchestrator.
Furthermore, as with the initial architecture considerations of the orchestrator, it is undesirable to fork existing ROS components and maintain alternative versions, as this creates an additional maintenance burden and might prevent the easy adoption of new upstream features.

Thus, a dedicated ROS bag player is implemented for use with the orchestrator instead of modifying the existing player.
This does not have the same feature set as the official player but allows for evaluation of this use case with a reasonable implementation effort.
To integrate the orchestrator, the ROS bag player requires the same adaptation as the simulator, except for the ``wait_until_dataprovider_state_update_allowed`` call which is not applicable without closed-loop execution.
Besides deterministic execution, a new feature is reliable faster-than-realtime execution, details of which are discussed in :ref:`sec-eval-execution_time`.

.. _sec-eval-system_integration-ros_nodes:

ROS Nodes
---------

The individual ROS nodes of the software stack under test are the primary concern regarding implementation effort, as there is usually a large number of ROS nodes, and new ROS nodes may be created or integrated regularly.

The integration effort of a ROS node depends on how well the node already matches the assumptions made and required by the orchestrator:
The orchestrator assumes that all processing in a node happens in a subscription or timer callback, and that each callback publishes at most one message on each configured output topic.
For callbacks without any outputs or callbacks that sporadically omit outputs, a status message must be published instead (see :ref:`sec:impl:controlling_callbacks:outputs`).


\subsubsection{Planning Module}
The integration effort of the trajectory planning and control module is significant because the module violates the assumption that all processing happens in timer and subscription callbacks.

The planning module contains two planning loops:
A high-level planning step runs in a dedicated thread as often as possible.
A low-level planner runs separately at a fixed frequency.
Handling incoming ROS messages happens asynchronously with the planning steps in a third thread.

While this architecture may have some advantages for runtime performance, it prevents external control via the orchestrator.
This represents an inherent limitation for the orchestrator.
Publishing of messages from outside a ROS callback is not able to be supported in any way, since it can not be anticipated in advance, making it impossible to integrate into the callback graph and synchronize it with other callbacks (see :ref:`sec:impl:callback_graphs`).
In order to ensure compatibility with the orchestrator, an optional mode has been introduced in which both planning loops are replaced with ROS timers.

This does make the planning module compatible with the orchestrator, but introduces a problem that should have explicitly been avoided by the specific software architecture chosen:
It runs the planning module in a completely different mode when using the orchestrator than without using the orchestrator.
This reduces the relevance of testing inside the orchestrator framework since specific problems and behaviors might only occur with the manual planning loop.

It might be possible in some cases to change the node in a way such that the usual mode of execution is compatible with the orchestrator, and thus avoids the problem of two discrete modes, but this is not possible in general.
In the case of the trajectory planning module, for example, this is not desirable due to the integration of the planning loop with a graphical user interface that is used to interactively change planner parameters and to introspect the current planner state.

\subsubsection{Tracking Module}\label{sec-eval-system_integration:ros_nodes:tracking}
While the tracking module does only process data within ROS subscription callbacks, the input-output behavior is still not straightforward:
The tracking module employs a sophisticated queueing system, which aims to form batches of inputs from both synchronized and unsynchronized sensors,
while also supporting dynamic addition and removal of sensors.
Additionally, while processing is always triggered by an incoming message, the processing itself happens in a dedicated thread in order to allow the simultaneous processing of ROS messages.

The input-output behavior itself is configurable such that only the reception of specific sensor inputs cause the processing and publishing of a "``tracks``" output message.
This is done to limit the output rate and reduce processing requirements.
Due to the queueing, this does however not imply that reception of the configured input immediately causes an output to appear.
It may be the case that additional inputs are required to produce the expected output.

This behavior can however still be handled by the node configuration without requiring major modification to the tracking module:
The node configuration was modified such that any input may cause an output to be published.
Then, the processing method was adapted such that a status message is published that explicitly excludes the ``tracks`` output using the ``omitted_outputs`` field when no tracks will be published.
In some circumstances, specifically following dropped messages, the queueing  additionally results in multiple outputs in a single callback.
This behavior is described in detail in :ref:`sec-eval-real_use_case:rosbag` and is not currently supported by the orchestrator.

While this is a pragmatic solution for describing the otherwise hard to statically describe input-output behavior of the tracking module, declaring more output topics than necessary for a callback is usually undesired:
Subsequent callbacks which actually publish a message on the specified topic need to wait for this callback to complete due to a false ``SAME_TOPIC`` dependency.
Additionally, the callback graph will contain possibly many actions resulting from the anticipated output.
Those actions are then again false dependencies for subsequent actions, not only as ``SAME_TOPIC`` dependencies but also ``SAME_NODE`` and ``SERVICE_GROUP`` edges.
These false dependencies might reduce the number of callbacks able to execute in parallel and might force callback executions to be delayed more than necessary to ensure deterministic execution.
Once a status message is received which specifies that the output message will not be published, the additional actions are removed, which then allows the execution of dependent actions.

\subsubsection{Recorder Node and Ego-Motion Estimation}
Both the nodes for recording the output of the tracking module and the ego-motion estimation match the assumptions made by the orchestrator and require very little integration effort, although some modification was necessary.
Both nodes only have topic input callbacks that would usually not cause any message to be published, requiring the publishing of a status message to inform the orchestrator of callback completion.

The ego-motion module is the only node in the experimental setup offering a service used during the evaluation.
This does however not require any modification within the node, as service calls are controlled by controlling the originating callbacks.
It is required however to list the service in the node configuration, to ensure a deterministic order between service calls and topic-input callbacks at the node.

Discussion
----------

In :ref:`sec:impl:design_goals`, the design goals towards the integration of existing nodes were established as minimizing the required modification to nodes, maintaining functionality without the orchestrator, and allowing for external nodes to be integrated without modifying their source code.

The implemented approach meets these goals to varying degrees.
The integration of existing components with the orchestrator requires a varying amount of effort, depending primarily on how well the component matches assumptions made by the orchestrator.
ROS nodes that fully comply with the assumptions made by the orchestrator and always publish every configured output require only a configuration file describing the node's behavior, which also works for external nodes without access to or modification of their source code.
Nodes that have callbacks without any output and nodes that may omit some or all configured outputs in some callback executions require publishing a status output as described in :ref:`sec:impl:controlling_callbacks:outputs` after a callback is complete.
Since this only entails publishing an additional message, this modification does not impede the node's functionality in any way when not using the orchestrator.
Nodes that fully deviate from the assumed callback behavior require appropriate modification before being suitable for use with the orchestrator, as was illustrated with the tracking and planning modules in :ref:`sec-eval-system_integration:ros_nodes`.

Creating the node configuration file does not present a significant effort for initial integration, but maintaining the configuration to match the actual node behavior is essential.
Although the orchestrator can detect some mismatches between node behavior and description,
omitted outputs and services can not be controlled by the orchestrator and might lead to nondeterministic system behavior.

While the model of ROS nodes that only execute ROS callbacks, which then publish at most one message on each configured output topic, is clearly not sufficient for all existing ROS nodes, it does apply to a wide class of nodes in use.
Nodes such as detection modules and control algorithms often operate in a simple "one output for each input" way or are completely time triggered, executing the same callback at a fixed frequency.
Such nodes are not part of this experimental setup, since the specific simulator in use already integrates the detection modules.

.. _sec-eval-real_use_case:

Application to Existing Scenario
================================

In this section, the effect of using the orchestrator in the use case introduced in :ref:`sec-eval-system_setup` is evaluated.
In the following, the ability of the orchestrator to ensure deterministic execution up to the metric-calculation step is demonstrated using both the simulator and recorded input data from a ROS bag, as well as combined with dynamic reconfiguration during test execution.

.. _sec-eval-real_use_case-sim:

Simulator
---------

When evaluating the tracking module in the previously introduced scenario, the MOTA and MOTP metrics introduced in :ref:`sec:bg:metrics` are calculated.
To calculate these metrics, the tracking outputs are recorded together with ground truth data from the simulator during a simulation run.
Those recordings are then loaded and processed offline.
When running the evaluation procedure multiple times, it can be observed that the resulting values differ for each run, as shown in :numref:`fig-eval-sim:nondet_metrics`.
This is due to nondeterministic callback execution during evaluation:
Both the simulator and the trajectory planning module run independently of each other, and the callback sequence of the multiple inputs to the tracking module is not fixed.

\begin{filecontents*}{data.csv}
name,num_frames,mota,motp
nd_3,175,0.7714285714285715,0.3296371941675045
nd_4,176,0.7693181818181818,0.3273043101111033
nd_5,178,0.7705286839145107,0.3209015937590458
nd_6,175,0.7700228832951945,0.3298583555342147
nd_7,176,0.770193401592719,0.328963843118783
nd_8,180,0.7708565072302558,0.33951099153421244
\end{filecontents*}

\begin{filecontents*}{data_orchestrator.csv}
name,num_frames,mota,motp
o_1,165,0.757282,0.335777
o_2,165,0.757282,0.335777
o_3,165,0.757282,0.335777
o_4,165,0.757282,0.335777
o_5,165,0.757282,0.335777
o_6,165,0.757282,0.335777
\end{filecontents*}

\begin{figure}
    \centering
    \begin{tikzpicture}
        \begin{axis}[
            axis y line*=left,
            xlabel={Simulation run},
            ymin=0.754,
            ymax=0.781,
            ytick distance=0.005,
            ylabel={\ref{plot_mota} MOTA},
            yticklabel style={/pgf/number format/.cd,fixed,fixed zerofill,precision=3},
        ]
            \addplot[uulm_blue_1,mark=*,dashed] table [x expr=\coordindex+1, y=mota, col sep=comma] {data.csv};
            \addplot[uulm_blue_1,mark=*] table [x expr=\coordindex+1, y=mota, col sep=comma] {data_orchestrator.csv};
            \label{plot_mota}
        \end{axis}

        \begin{axis}[
            axis y line*=right,
            axis x line=none,
            ytick distance=0.005,
            ylabel={\ref{plot_motp} MOTP},
            ymin=0.3175,
            ymax=0.3425,
            yticklabel style={/pgf/number format/.cd,fixed,fixed zerofill,precision=3},
            legend pos=north west,
            legend entries={With Orchestrator,Without Orchestrator}
        ]
            \addlegendimage{solid,black}
            \addlegendimage{dashed,black}
            \addplot[uulm_orange_1,mark=*,dashed] table [x expr=\coordindex+1, y=motp, col sep=comma] {data.csv};
            \addplot[uulm_orange_1,mark=*] table [x expr=\coordindex+1, y=motp, col sep=comma] {data_orchestrator.csv};
            \label{plot_motp}
        \end{axis}
    \end{tikzpicture}
    \caption[Evaluation of the MOTA and MOTP metrics using the experimental setup.]{Evaluation of the MOTA and MOTP metrics in the scenario introduced in :ref:`sec-eval-system_setup` over multiple simulation runs, both with and without the orchestrator.}
    \label{fig-eval-sim:nondet_metrics}
\end{figure}

When running the simulation using the orchestrator, the variance in the calculated metrics is eliminated.
This shows that in this example the orchestrator successfully enabled the use case of repeatable execution of test cases for evaluating a software module inside a more complex system.

Not only are the calculated metrics consistent, the deterministic execution as ensured by the orchestrator results in bit-identical outputs of the tracking module for every simulation run, and thus exact equality of the recordings generated.
This enables additional use cases for testing such as easily comparing the output of the module before and after presumably non-functional changes are made to the source code.
Previously, such a comparison would require parsing the recorded results, calculating some similarity measure or distance between the expected and actual results, and applying some threshold to determine equality.
Now, simply comparing the files without any semantic understanding of the contents is possible.

.. _sec-eval-real_use_case-rosbag:

ROS Bag
-------

In order to test the use case of ROS bag replay, the player implemented in :ref:`sec-eval-system_integration:bag_player` is used.
Although the ROS bag player provides inputs in deterministic order, the characteristics of the input data are different from the simulator.
During the recording of the ROS bag, the sensor input topics and pre-processing nodes are subject to nondeterministic ROS communication and callback behavior.
This results in a ROS bag with missing sensor samples (due to dropped messages as well as unexpected behavior of real sensors) and reordered messages (due to nondeterministic transmission of the messages to the ROS bag recorder).
All those effects would usually not be expected from a simulator, which produces predictable and periodic inputs.

This does not present a problem for the orchestrator:
Since the callback graph construction is incremental for each input, the only a priori knowledge the orchestrator requires is the API call from the data provider informing the orchestrator of the next input, and the node and launch configurations to determine the resulting callbacks.
Specifically, the orchestrator does not require information such as expected publishing frequencies or periodically repeating inputs at all.

In order to reuse the existing test setup, a ROS bag was recorded from the outputs of the simulator.
To simulate the effects described above, the ROS bag is manually modified by randomly dropping messages and randomly reordering recorded messages.

Using the multi-object tracking module was not possible, however, since the high rate of dropped messages causes a callback behavior that can not be modeled by the node configuration as introduced in :ref:`sec:impl:configuration`.
In addition to the behavior described in :ref:`sec-eval-system_integration:ros_nodes:tracking` of zero or one output for each measurement input, certain combinations of inputs may cause multiple outputs from one input callback.
This is due to a sophisticated input queueing approach, that forms batches of inputs with small deviations in measurement time, that only get processed once a batch contains measurements of all sensors.
In case of missing measurements, a newer batch might be complete while older, incomplete batches still exist.
The queueing algorithm assumes in that case that the missing measurements of the old batches will not arrive anymore (ruling out message reordering, but allowing dropping messages), and processes the old batches, producing multiple outputs in one callback.
Handling more outputs than expected is not possible for the orchestrator since the orchestrator must determine when a callback is completed to allow the next input for the corresponding node.
If a callback publishes additional outputs after it is assumed to have been completed already, the orchestrator can not identify the source of the additional output or wrongly assigns the output to the next callback expected to publish on the corresponding topic.

This queueing also makes the tracking module robust against any message reordering between the ROS bag player and the module itself, resulting in deterministic execution even without the orchestrator and at high playback speed.
When using a ROS bag with reordered, but without dropped messages, the experimental setup can be verified and performs as expected with a ROS bag as the data source instead of a simulator, which also shows that the orchestrator can successfully be used in combination with existing node-specific measures to ensure deterministic input ordering.
The further behavior of the orchestrator remains unchanged, meaning nondeterminism in larger systems under test such as the cases demonstrated in :ref:`sec-eval-verification` is prevented.

Furthermore, when using ROS bags as the data source it may be possible to easily maximize the playback speed without manually choosing a rate that does not overwhelm the processing components causing dropped messages.
More details on this specific use case will be given in :ref:`sec-eval-execution_time`.

.. _sec-eval-real_use_case-reconfig:

Dynamic Reconfiguration
-----------------------

To test the orchestrator in a scenario including dynamic reconfiguration, the previous setup was extended by such a component.
Since a module for dynamic reconfiguration of components or the communication structure was not readily available, a minimal functional mockup was created:
A "reconfigurator" component with a periodic timer callback decides within this callback if the system needs to be reconfigured, and then executes that reconfiguration.
The node description for the reconfiguration node is given in \cref{listing:eval:reconfig:node_config}.
In this example, the reconfiguration reduces simulated measurement noise, which could simulate switching to a more accurate, but also more computationally demanding perception module.
The mock reconfigurator always chooses to reconfigure after a set time.
A real working counterpart would require additional inputs such as the current vehicle environment, which are omitted here.

\begin{listing}
    \begin{minted}{json}
{
  "name": "sil_reconfigurator",
  "callbacks": [
    {
      "trigger": {
        "type": "timer",
        "period": 1000000000
      },
      "outputs": [],
      "may_cause_reconfiguration": true
    }
  ]
}
    \end{minted}
    \caption{Node configuration for the reconfiguration node mockup.}
    \label{listing:eval:reconfig:node_config}
\end{listing}

\pgfplotstableread[col sep = comma]{data/_reconfig_nd_1.json.csv}{\tablenda}
\pgfplotstableread[col sep = comma]{data/_reconfig_nd_2.json.csv}{\tablendb}
\pgfplotstableread[col sep = comma]{data/_reconfig_nd_3.json.csv}{\tablendc}
\pgfplotstableread[col sep = comma]{data/_reconfig_nd_4.json.csv}{\tablendd}

\pgfplotstablecreatecol[
  copy column from table={\tablenda}{[index] 1},
  ]{data1}{\tablenda}
\pgfplotstablecreatecol[
  copy column from table={\tablendb}{[index] 1},
  ]{data2}{\tablenda}
\pgfplotstablecreatecol[
  copy column from table={\tablendc}{[index] 1},
  ]{data3}{\tablenda}
\pgfplotstablecreatecol[
  copy column from table={\tablendd}{[index] 1},
  ]{data4}{\tablenda}

\begin{figure}
    \centering
    \begin{tikzpicture}
        \begin{axis}[
            %title=OSPA Distance,
            cycle list name=uulm,
            xlabel={$t [s]$},
            ylabel={OSPA Distance $[m]$},
            ymin=0.25,
            ymax=1.9,
            no markers
            ]
            \addlegendimage{empty legend};
            \addplot table[col sep=comma, header=false, x index=0, y index=1]{data/_reconfig_nd_1.json.csv};
            \addplot table[col sep=comma, header=false, x index=0, y index=1]{data/_reconfig_nd_2.json.csv};
            \addplot table[col sep=comma, header=false, x index=0, y index=1]{data/_reconfig_nd_3.json.csv};
            \addplot table[col sep=comma, header=false, x index=0, y index=1]{data/_reconfig_nd_4.json.csv};

            \addlegendentry{\hspace{-.6cm}\textbf{Run ID}}
            \addlegendentry{$\#1$}
            \addlegendentry{$\#2$}
            \addlegendentry{$\#3$}
            \addlegendentry{$\#4$}

            % Vertical line
            \addplot[thick, samples=50, smooth, dashed] coordinates {(7,0)(7,3)};
        \end{axis}
    \end{tikzpicture}
    \caption[OSPA distance of tracks versus ground truth during multiple simulation runs.]{\Gls{ospa} distance of tracks versus ground truth during multiple simulation runs. The dashed vertical line marks the timestep in which the runtime reconfiguration occurs.}
    \label{fig-eval-config:ospa}
\end{figure}

\begin{figure}
    \centering
    \begin{tikzpicture}
        \begin{axis}[
            cycle list name=uulm,
            xlabel={$t [s]$},
            ylabel={OSPA Distance $[m]$},
            no markers,
            ymin=-0.025,
            ymax=0.23,
            yticklabel style={
                /pgf/number format/fixed
                %/pgf/number format/precision=5,
                %/pgf/number format/fixed zerofill
            },
            ]
            \addlegendimage{empty legend};
            \addplot table[x index=0, y expr=abs(\thisrow{data1}-\thisrow{data2})]{\tablenda};
            \addplot table[x index=0, y expr=abs(\thisrow{data1}-\thisrow{data3})]{\tablenda};
            \addplot table[x index=0, y expr=abs(\thisrow{data1}-\thisrow{data4})]{\tablenda};

            \addlegendentry{\hspace{-.6cm}\textbf{Run ID}}
            \addlegendentry{$|\#1-\#2|$}
            \addlegendentry{$|\#1-\#3|$}
            \addlegendentry{$|\#1-\#4|$}

            \addplot[thick, samples=50, smooth, dashed] coordinates {(7,-1)(7,1)};
        \end{axis}
    \end{tikzpicture}
    \caption[Absolute difference in OSPA distances between the simulation runs.]{Absolute difference in OSPA distances between the simulation runs. The dashed vertical line marks the timestep in which the runtime reconfiguration occurs.}
    \label{fig-eval-config:ospa_diff}
\end{figure}

:numref:`fig-eval-config:ospa` shows the OSPA distance (see :ref:`sec:bg:metrics`) between the tracking result and the ground truth object data from the simulator over multiple simulation runs.
The OSPA distance was chosen as a metric in this case since it is calculated for every time step instead of as an average over the entire simulation run, as is the case with the MOTA and MOTP metrics used above.
This allows evaluation of how the metric changes during the simulation run and clearly shows the reconfiguration step.
It is apparent that the reconfiguration module successfully switched to a lower measurement noise at :math:`t=7s`.
Importantly, however, the evaluation results of the multiple runs do not completely overlap.
This is again due to nondeterministic callback execution within the tracking, planning, and simulator modules.
The differences between the runs, plotted in :numref:`fig-eval-config:ospa_diff`, show that all runs deviate from the first run, with two runs showing the largest difference at the exact time of reconfiguration.

\begin{figure}
    \centering
    \begin{tikzpicture}
        \begin{axis}[
            %title=OSPA Distance,
            cycle list name=uulm,
            xlabel={$t [s]$},
            ylabel={OSPA Distance $[m]$},
            ymin=0.25,
            ymax=1.9,
            no markers,
            legend entries={{Without Orchestrator},{With Orchestrator}}
            ]
            \addplot table[col sep=comma, header=false, x index=0, y index=1]{data/_reconfig_nd_1.json.csv};
            \addplot table[col sep=comma, header=false, x index=0, y index=1]{data/_reconfig_o_1.json.csv};
            \addplot[thick, samples=50, smooth, dashed] coordinates {(7,0)(7,3)};
        \end{axis}
    \end{tikzpicture}
    \caption[OSPA distance of tracks versus ground truth over time, comparison between simulation run with and without the orchestrator.]{\Gls{ospa} distance of tracks versus ground truth over time, comparison between initial simulation run and simulation while using the orchestrator.}
    \label{fig-eval-config:ospa_orchestrator}
\end{figure}

Using the orchestrator, the measured tracking result does differ from the previous simulation runs, as shown in :numref:`fig-eval-config:ospa_orchestrator`.
The output is however deterministic and repeatable, even if a reconfiguration occurs during the simulation.
Again, this demonstrates the successful application of the orchestrator framework, even in the presence of dynamic reconfiguration at runtime.

.. _sec-eval-real_use_case-discussion:

Discussion
----------

In :ref:`sec-eval-real_use_case`, the successful implementation of two design goals was verified:
First, :ref:`sec-eval-real_use_case-sim` and :ref:`sec-eval-real_use_case-rosbag` demonstrate successful use of the orchestrator with both a simulator and ROS bag as data sources.
Notably, no additional requirements are placed on the specific ROS bag used, allowing the use of the orchestrator with already existing recorded data.
Secondly, :ref:`sec-eval-real_use_case-reconfig` shows that the guarantees of the orchestrator hold when the system is dynamically reconfigured at runtime.
These tests represent exactly the use case of evaluation of a component within a larger software stack that motivated this work, that is able to run repeatedly and deterministically using the orchestrator.

In :ref:`sec-eval-real_use_case-rosbag`, a limitation of the orchestrator in terms of modeling a node's output behavior was reached.
In order to use such nodes with the orchestrator in the future, an extension to the current callback handling might be required and is proposed here:
A solution to this problem might be to allow the node to publish a status message after every callback, which specifies the number of outputs that have actually been published in this specific callback invocation.
This would allow the orchestrator to ensure the reception of every callback output, and prevent wrong associations of outputs to callbacks.
As additional messages on the corresponding topics would also cause additional downstream callbacks for subscribers of those topics, this approach might however introduce additional points of synchronization across the callback graph.

.. _sec-eval-execution_time:

Execution-Time Impact
=====================

Due to the required serialization of callbacks and buffering of messages, a general increase in execution time is to be expected when using the orchestrator.
In the following, this impact is measured for a simulation use case and the individual sources of increased execution time, as well as possible future improvements, are discussed.

.. _sec-eval-execution_time-analysis:

Analysis
--------

To measure the impact of topic interception, the induced delay of forwarding a message via a ROS node is measured.
In order to compensate for latency in the measuring node, the difference in latency for directly sending and receiving a message in the same node versus the latency of sending a message and receiving a forwarded message is measured.
When using a measuring and forwarding node implemented in Python and using the "eProsima Fast DDS" middleware, the latency from publishing to receiving increases from a mean of :math:`0.64` ms to :math:`0.99` ms.
This induced latency of :math:`0.35` ms on average is considered acceptable and justifies the design choice of controlling callbacks by intercepting the corresponding message inputs.

\begin{figure}[t]
    \centering
    \begin{tikzpicture}
        \begin{axis}[
            xbar,
            xmin=0,
            enlarge y limits={abs=0.5},
            enlarge x limits={0.15,upper},
            height=5cm,
            width=12cm,
            yticklabels={{\texttt{real\_time},\\orchestrator},{\texttt{fast},\\orchestrator},\texttt{real\_time}},
            yticklabel style={align=right},
            ytick=data,
            nodes near coords, nodes near coords align={horizontal},
            xlabel={Execution time $[s]$}
        ]
            \addplot[uulm_blue_1,fill=uulm_blue_4] coordinates {
                                (63.5866667,0)
                                (57.0966667,1)
                                (32.94,2)};
        \end{axis}
    \end{tikzpicture}
    \caption[Comparison of execution time for one simulation run.]{Comparison of execution time for one simulation run between not using the orchestrator, using the orchestrator with faster than real-time execution, and using the orchestrator with real-time execution.}
    \label{fig-eval-execution_time:sim_comparison_barchart}
\end{figure}

:numref:`fig-eval-execution_time:sim_comparison_barchart` shows a comparison of execution time for one simulation run of the scenario introduced in :ref:`sec-eval-system_setup`.
The first bar shows the runtime without using the orchestrator, the bottom two bars show the time when using the orchestrator.

The simulator currently offers two modes of execution:
``fast`` executes the simulation as fast as possible, while ``real_time`` slows down the simulation to run at real-time speed if the simulation itself would be able to run faster than real-time.
Using the ``fast`` mode is only appropriate combined with the orchestrator or some other method of synchronization between the simulator and software under test.
If the simulator is not able to run in real-time, deliberate delays to ensure real-time execution should already be zero.
Since :numref:`fig-eval-execution_time:sim_comparison_barchart` still shows an increase in runtime for using the ``real_time`` mode compared to the ``fast`` mode, the orchestrator is considered with the ``fast`` execution mode in the following.
% time factor in test: 1.43063584
Nonetheless, it is apparent that the orchestrator causes a significant runtime impact as the execution time is increased by about 73\% in the ``fast`` case.

Evaluating the orchestrator itself for execution time, it can be found that during a simulation run, the callback for intercepted message inputs runs on average :math:`0.6` ms, and the callback for status messages runs :math:`0.9` ms.
The API functions for waiting until publishing a time or data input execute in :math:`0.9` ms and :math:`0.5` ms.
This sums up to more than :math:`12.3` seconds spent executing interception and status callbacks, which in this scenario happens within the simulator.
The simulator furthermore spends about :math:`5` seconds executing orchestrator API calls.

The remaining increase in execution time is explained by serializing the execution of dependent callbacks.
The vehicle tracking and planning components may both call the ego-motion service, which prevents parallel execution.
The speed of publishing inputs by the simulator is greatly reduced especially for nodes like the tracking module, which has a relatively large number of inputs (12, in the evaluated examples) that are published sequentially.
This would usually happen without waiting, but the orchestrator requires confirmation from the tracking module that an input has been processed before forwarding the next input to ensure a deterministic processing order.

Finally, the orchestrator requires the simulator to receive and process the output from the planning module before advancing the simulation.
This is realized by the ``changes_dataprovider_state`` flag for the corresponding callback in the node configuration file, which causes the ``wait_until_dataprovider_state_update_allowed`` API call to block until the callback has finished.
For any simulator, the "dataprovider state update" corresponds to executing a simulation timestep, which results in an effective slowdown of each simulation timestep to the execution time of the longest path resulting in some input to the simulator.

The other available flag for callbacks, ``may_cause_reconfiguration``, presents a similar point of global synchronization:
This flag is applied to callbacks of a component that may decide dynamically reconfigure the ROS system, as described in :ref:`sec:bg:reconfig`, based on the current system state (such as vehicle environment, in the autonomous driving use case).
To ensure that the reconfiguration always occurs at the same point in time with respect to other callback executions at each node, any subsequent data inputs and dataprovider state updates must wait until either the reconfiguration is complete or the callback has finished without requesting reconfiguration.
This presents an even more severe point of synchronization, since it immediately blocks the next data inputs from the simulator, and not only the start of the next timestep, while still allowing to publish the remaining inputs from the current timestep.

.. _sec-eval-execution_time:discussion:

Discussion
----------

Using the orchestrator significantly increased execution time in the simulation scenario.
To reduce the runtime overhead caused by the orchestrator, multiple approaches are viable.
As significant time is spent executing orchestrator callbacks and API calls, improving the performance of the orchestrator itself would be beneficial.
A possible approach worth investigating could be parallelizing the execution of orchestrator callbacks.
Both parallelizing multiple orchestrator callbacks and running those callbacks in parallel to the host node (the simulator or ROS bag player) could be viable.
In addition to a more efficient implementation of the orchestrator itself, the overhead of serializing callback executions is significant.
While some of that overhead is inherently required by the serialization to ensure deterministic execution, it has already been shown in :ref:`sec-eval-verification:multiple_publishers_on_topic,sec-eval-verification:service_calls` that parallelism of callback executions can be improved with more granular control over callbacks, their outputs, and service calls made from within those callbacks.

When using a ROS bag instead of a simulator as the data source, some of the identified problems are less concerning.
Since a ROS bag player does not have to perform any computation and reading recorded data is not usually a bottleneck for performance, the overhead of the orchestrator API calls is less problematic.
Furthermore, without closed-loop simulation, the ``wait_until_dataprovider_state_update_allowed`` API call is not necessary which has been identified as a factor that reduces the potential for parallel callback execution.
In some scenarios, the use of the orchestrator is even able to improve execution time:
When replaying a ROS bag, the speed of playback is often adjusted.
Use cases for playing back a recording at equal to or slower than real-time occur when the developer intends to use interactive tools for introspection and visualization such as for debugging the behavior of a software component in a specific scenario.
Often, however, the user is just interested in processing all messages in the bag, preferably as fast as possible.
The playback speed is thus adjusted to be as fast as possible while the software under test is still able to perform all processing without dropping messages from subscriber queue overflow.
This overflow however is usually not apparent immediately, and processing speed may depend on external factors such as system load, which makes this process difficult.
When using the orchestrator, however, the processing of all messages is guaranteed and queue overflow is not possible.
This allows the ROS bag player to publish messages as soon as the orchestrator allows, without specifying any constant playback rate.
Playing a ROS bag is necessarily an open-loop configuration without any synchronization for dataprovider state update, and the player itself is expected to have a fast execution time when compared to the ROS nodes under test.
If a speedup is achieved in the end depends on if the remaining overhead from serializing callback invocations outweighs the increased playback rate or not.

The design goal of minimizing the execution time impact is thus only partially achieved.
As measured in this section and detailed in :ref:`sec-eval-verification:discussion`, the serialization of callbacks and thus the induced latency of executing callbacks is not minimal.
The runtime of the orchestrator component itself has been shown to be significant as well, although this was not the bottleneck in this test scenario.
