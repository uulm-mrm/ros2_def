******************
Topic Interception
******************

For the general idea, refer to :ref:`sec-impl-controlling_callbacks`.

The interception is done by remapping input topics using globally applied node-specific remapping rules.
Node-specific rules are of the form ``nodename:before:=after`` (refer to the `ROS 2 Design article on remapping <https://design.ros2.org/articles/static_remapping.html#match-part-of-a-rule>`_).
Note that those node-specific rules do not accept nodes in namespaces, meaning ``nodename`` containing forward slashes.
An `Issue about allowing node-FQNs for node-specific remapping rules <https://github.com/ros2/rcl/issues/296>`_ exists since 2018.
A proposed `change to the design doc allowing this <https://github.com/ros2/design/pull/299>`_ was rejected in 2020.

Node-specific remapping rules were chosen since that allows applying all remappings only once,
before including the original launch file, like that:

.. code-block:: python
    :caption: Example launch file

    def generate_launch_description():
        return LaunchDescription([
            *generate_remappings_from_config_file(
                "orchestrator",
                "sil_reconfig_launch_config.json"
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('platform_sil'),
                        'launch/sil.py'
                    ])
                ])
            )
        ])


``generate_remappings_from_config`` expands to a list of node-specific remapping rules.

