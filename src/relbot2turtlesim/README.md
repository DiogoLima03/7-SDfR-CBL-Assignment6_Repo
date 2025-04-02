Package relbot2turtlesim
-----------------------------------------------
#### Description: 
This package implements the conversion from the relbot_simulator package to turtlesim.

#### Inputs:
/output/robot_pose
- Type: geometry_msgs/msg/pose_stamped


#### Output:
/turtle1/cmd_vel
- Type: geometry_msgs/msg/twist_stamped

#### Run:
To start the node run the following command:

`ros2 run relbot2turtlesim relbot2turtlesim`

You can also launch this node together with turtlesim and the RELbot simulator. Make sure that all nodes are build and sourced in your terminal.

To launch the node, together with the RELbot simulator and turtlesim run the following command:

```ros2 launch relbot_launch relbot_to_turtle.launch.py```

#### Core components:
- poseCallback(): Saves the new robot pose, calculates the new velocity for turtlesim and publishes that to the corresponding topic.
- poseToVel(): Calculates the new velocity to be send, based on the previous and new robot position.