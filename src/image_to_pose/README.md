# Package image_to_pose

-----------------------------------------------

## Description  
This package processes images to estimate the pose of an object in the scene.

## Inputs  
### `/image`  
**Type:** `sensor_msgs/msg/Image`  

## Outputs  
### `/ball_pose_and_status`  
**Type:** `costum_messages::msg::PoseAndStatus`  

## Run  
In a terminal, run either of the following commands:  
```bash  
ros2 run image_to_pose image_to_pose  
ros2 launch image_to_pose image_to_pose_simulator_launch.py  # for working in the pc
```  

## Parameters  
- `is_relbot`: Defines if the node is ment to be used in the relbot or not. In the relbot the ball positions in the edges of the screen are not considered for the relbot not to uncontrollably speed up.  

## Core components  
ImageToPose(): Initializes the node, declares parameters, and sets up subscriber and publisher  
image_callback(): Main callback triggered when a new image is received  
imageProcessing(): Converts the image to HSV and generates a green mask  
detect_ball(): Detects the green ball using contour detection and finds its center and radius  
estimate_position(): Estimates the 2D (x, z) position of the ball using pinhole camera model  
calculate_pose(): Calculates pose (distance and orientation) and publishes the PoseAndStatus message  
comunicateBallNotFound(): Publishes a zeroed pose and status=false when no ball is detected  
displayDetection(): Draws circle, center dot, and pose text overlay on the image  
