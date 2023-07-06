Introduction
============

The growing complexity of robotics software stacks necessitates the use of frameworks that allow the separation of individual components and explicit communication between them.
In the research community, in particular, open-source frameworks have prevailed over proprietary middleware or custom full-stack implementations.
Their ease of use and the ease of sharing and re-using loosely coupled components often outweigh potentially stronger guarantees about reliability and determinism or performance benefits of competing options.
The Robot Operating System is such an open-source framework that has gained popularity in both research and industry over the last decade ([Macenski2022]_).
It provides abstractions for building modular robotics software and serves as a communication middleware between components.
The event-triggered execution model of ROS combined with its weak guarantees for the message-passing implementation can however not guarantee the deterministic execution of an entire software stack.
This lack of repeatability presents a problem for the testing and evaluation of software components, which has become an integral part of the development cycle of modern robotics applications.
Especially in domains such as automated driving, continued verification of the expected system performance is critical for assuring the safety of the entire system.
Without deterministic execution of test cases, observed changes in resulting metrics can not be traced back to specific software changes with certainty, since random variations may manifest from nondeterministic execution.
This is especially critical when dynamically changing parameters and functionality by runtime reconfiguration of the software stack, which is desirable for example for adaptive, situation-aware environment perception methods as proposed in [Henning2023]_.

In this thesis, a method is presented to enable repeatable execution of ROS software stacks by ensuring a deterministic callback sequence at each individual ROS node.
This is enabled by iteratively building a callback graph from pre-defined specifications of node behavior.
Using the callback graph, data flow within the software stack can be specifically inhibited to ensure deterministic callback ordering at each node.

:ref:`sec-bg` provides the necessary background on ROS and introduces the relevant types of software testing and evaluation.
In :ref:`sec-impl`, first, the nondeterministic communication and callback behavior and its consequences are described.
Then, four fundamental sources of nondeterministic callback ordering are identified.
The rest of :ref:`sec-impl` describes in detail the method of ensuring deterministic callback execution and the design and implementation of the orchestrator, which is the newly developed component implementing the proposed method.
In :ref:`sec-eval`, the functionality of the implementation is verified and discussed using the previously identified minimal examples, and usability is assessed by integrating existing components of an existing autonomous driving stack.
:ref:`sec-conclusion` summarizes the findings and gives an outlook on future enhancements.
