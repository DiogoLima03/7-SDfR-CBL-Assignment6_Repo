Package relbot_sequence_controller
-----------------------------------------------
#### Description: 
This package that constains the calculation to set a new velocity according to desired trajectories of the reelbot simulater.


#### Output:
`/input/left_motor/setpoint_vel`
- Type: example_interfaces/msg/Float64

`/input/left_motor/setpoint_vel`
- Type: example_interfaces/msg/Float64

#### Run:
To start the node run the following command:

`ros2 run relbot_sequence_controller relbot_sequence_controller`

You can also launch this node together with turtlesim, the relbot2turtlesim node and the RELbot simulator. Make sure that all nodes are build and sourced in your terminal.

To launch the node, together with the RELbot simulator, relbot2turtlesim and turtlesim run the following command:

`ros2 launch relbot_launch relbot_sequence_controller.launch.py`

#### Parameters:
-int qos_overrides./parameter_events.publisher.depth : QoS policy for depth of publisher /parameter_events.
-string qos_overrides./parameter_events.publisher.durability : QoS policy for durability of publisher /parameter_events.
-string qos_overrides./parameter_events.publisher.history : QoS policy for history setting of publisher /parameter_events.
-string qos_overrides./parameter_events.publisher.reliability : QoS policy for reliability of publisher /parameter_events.
-bool start_type_description_service : Starts the ~/get_type_description service. Default = false.
-float trajectoryType : Sets the actual relbot trajectory. DEFAULT = "still", which makes the relbot stand still
-bool use_sim_time : Use simulation time (from /clock topic) instead of system time. Default = false.

#### Core components:
- calculate_velocity(): The function that calculates the velocity accordingly to the trajectory setted
- timer_callback(): publishes the velocity to both wheel 30 times per second.