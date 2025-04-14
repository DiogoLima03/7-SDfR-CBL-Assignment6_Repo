# RELbot demo : Following a Green Ball 

This repository is responsible for running a demo on the RELbot, in which the RELbot will follow a green ball. This repository makes use of the ROS2-Xenomai4 framework.

## Requirements
To run this setup you will need the `cam2image_vm2ros` ROS2 package available from the RAM Gitlab, otherwise the launch file described below will throw an error.

## Instructions

For a more in-depth explanation about how to use XRF2, see appendix I of the Software-Tools Manual. To run the demo, follow the following steps:

1. Create a workspace and a `src` folder:
    ```bash
    mkdir -p ~/workspace/src
    ```
2. Download the framework and this project into the `src` folder.
3. Set the ROS2 messages **Ros2Xeno** and **Xeno2Ros** to:
    ```plaintext
    float64 right_wheel_vel
    float64 left_wheel_vel
    ```
4. Build the project from the workspace folder:
    ```bash
    colcon build
    ```
5. Source the setup script:
    ```bash
    source install/setup.bash
    ```
6. Run the demo:
    ```bash
    sudo ./build/demo/demo
    ```
7. In a separate terminal, source the setup script again:
    ```bash
    source install/setup.bash
    ```
8. Launch the sequence controller:
    ```bash
    ros2 launch sequence_controller sequence_controller.launch.py
    ```
