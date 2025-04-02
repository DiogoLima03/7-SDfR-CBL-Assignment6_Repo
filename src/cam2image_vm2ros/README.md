Package cam2image_vm2ros
-----------------------------------------------
Description: This package aims to set up a connection with cam2image_host2vm to send webcam images of UDP.
   This package is modified from the image_tools package from ROS2.

Output:
/image 
        Type: sensor_msgs/msg/Image
        Published the images received from the UDP stream created by cam2image_host2vm.

Run:
        In a terminal run either of the following commands:
        ros2 run cam2image_vm2ros cam2image --ros-args --params-file src/cam2image_vm2ros/config/cam2image.yaml

Parameters:
        double image_stream_FPS : Sets the output rate of image stream. Default = 30 FPS, which is most webcams

