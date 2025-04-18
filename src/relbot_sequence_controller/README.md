Package relbot_sequence_controller
-----------------------------------------------
#### Description: 
This package that constains the calculation to set a new velocity according to desired trajectories of the reelbot simulater.

### Input:
"/ball_pose_and_status"
- Type: costum_messages::msg::PoseAndStatus

#### Output:
For runing in the relbot
`/Ros2Xeno`
- Type: xrf2_msgs::msg::Ros2Xeno

For the running in the relbot simulator 
`/input/left_motor/setpoint_vel`
- Type: example_interfaces/msg/Float64

`/input/left_motor/setpoint_vel`
- Type: example_interfaces/msg/Float64

#### Run:
To start the node run the following command:

`ros2 run relbot_sequence_controller relbot_sequence_controller`

`ros2 launch relbot_launch sequence_controller_relbot_launch.launch.py` # for the realbot
`ros2 launch sequence_controller_simulation_launch`# for the relbor simulator

#### Parameters:


#### Core components:
SteerRelbot(): Initializes the node, declares parameters, sets up subscription and publishers depending on simulation or real robot  
pose_callback(): Callback that processes the incoming pose and status message; calculates and publishes wheel velocities  
calculate_velocity(): Computes the linear and angular velocity using a proportional controller and converts it to left and right wheel commands  