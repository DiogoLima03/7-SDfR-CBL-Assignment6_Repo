Assignment 6.2.2 (image to pose - position of the ball)

## IN WINDOWS CMD:

1. `cd "C:\Users\Diogo Lima\OneDrive\Ambiente de Trabalho\Documentos com Estrela\Faculty\Master\2_Year\3Q__A2\Software Development for Robotics\Starter Kits\cam2image_host2vm"     `
2. `python videoserver.py`
ALL IN ONE:
1. `cd "C:\Users\Diogo Lima\OneDrive\Ambiente de Trabalho\Documentos com Estrela\Faculty\Master\2_Year\3Q__A2\Software Development for Robotics\Starter Kits\cam2image_host2vm" && python videoserver.py    `

## IN WSL (LINUX):
1. `colcon build --packages-skip ros_xeno_bridge   `
2. `source install/setup.bash  `
3. `ros2 launch launch_relbot image_to_pose_launch.py   `
ALL IN ONE:
1. `colcon build --packages-skip ros_xeno_bridge && source install/setup.bash && ros2 launch launch_relbot image_to_pose_launch.py   `
----------------------------------------------------------------------------------------------------------------------------------------

Assignment 6.3 (sequence controller)

## IN WINDOWS CMD:

1. `cd "C:\Users\Diogo Lima\OneDrive\Ambiente de Trabalho\Documentos com Estrela\Faculty\Master\2_Year\3Q__A2\Software Development for Robotics\Starter Kits\cam2image_host2vm"     `
2. `python videoserver.py`
ALL IN ONE:
1. `cd "C:\Users\Diogo Lima\OneDrive\Ambiente de Trabalho\Documentos com Estrela\Faculty\Master\2_Year\3Q__A2\Software Development for Robotics\Starter Kits\cam2image_host2vm" && python videoserver.py    `

# IN WSL (LINUX):
1. `colcon build --packages-skip ros_xeno_bridge   `
2. `source install/setup.bash  `
3. `ros2 launch launch_relbot sequence_controller__simulate_launch.py   `
ALL IN ONE:
1. `colcon build --packages-skip XRF2 RELbot-loopC costum_messages && source install/setup.bash && ros2 launch launch_relbot sequence_controller_simulation_launch.py              `
----------------------------------------------------------------------------------------------------------------------------------------

Assignment 6.4.1 (Runing nodes (up until the sequence controller with turtlesim) in the relbot without connecting to the motors (Xenomai))

## IN WINDOWS CMD:

To connect to the rasperipy with -X:
1. ` ssh -X <user>@<RELbot-IP>   `

# IN LINUX (Relbot):

1. `colcon build --packages-skip ros_xeno_bridge   `
2. `source install/setup.bash  `
3. `ros2 launch launch_relbot sequence_controller_relbot   `
ALL IN ONE:
1. `colcon build --packages-skip XRF2 RELbot-loopC costum_messages && source install/setup.bash && ros2 launch launch_relbot sequence_controller_relbot.py    `
----------------------------------------------------------------------------------------------------------------------------------------

Assignment 6.4.??? (running the relbot)

## IN WINDOWS CMD:

To connect to the rasperipy with -X:
1. ` ssh -X <user>@<RELbot-IP>   `

# IN LINUX (Relbot):

In one terminal run:
1. `colcon build --packages-skip costum_messages   `
2. `source install/setup.bash   `
3. `sudo ./build/demo/demo   `
ALL IN ONE:
1. `colcon build --packages-skip costum_messages && source install/setup.bash && sudo ./build/demo/demo`

In another terminal run:
1. `source install/setup.bash   `
2. `ros2 launch launch_relbot relbot_launch.py   `
ALL IN ONE:
1. `source install/setup.bash && ros2 launch launch_relbot relbot_launch.py `
