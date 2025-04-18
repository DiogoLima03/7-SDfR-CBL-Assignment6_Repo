# Package `costum_messages`

-----------------------------------------------

## Description  
This package creates a costum message type for comunicating between the image_to_pose and the relbot_sequence_controller. 

---

## Inputs  

### `/image`  
**Type:** `sensor_msgs/msg/Image`  

### `/input/left_motor/setpoint_vel`  
### `/input/right_motor/setpoint_vel`  
**Type:** `example_interfaces/msg/Float64`  

---

## Messages  

### `costum_messages::msg::PoseAndStatus` 
That comunicates
**Type:** - float32 position
**Type:** - float32 orientation 
**Type:** - bool status
