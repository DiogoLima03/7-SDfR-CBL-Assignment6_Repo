Assignment 6.2.2 

## IN WINDOWS CMD:

1. `cd "C:\Users\Diogo Lima\OneDrive\Ambiente de Trabalho\Documentos com Estrela\Faculty\Master\2_Year\3Q__A2\Software Development for Robotics\Starter Kits\cam2image_host2vm"     `

2. `python videoserver.py`

ALL IN ONE:
`cd "C:\Users\Diogo Lima\OneDrive\Ambiente de Trabalho\Documentos com Estrela\Faculty\Master\2_Year\3Q__A2\Software Development for Robotics\Starter Kits\cam2image_host2vm" && python videoserver.py    `

## IN WSL (LINUX):
`colcon build --packages-skip ros_xeno_bridge   `
`source install/setup.bash  `
`ros2 launch launch_relbot image_to_pose_launch.py   `

ALL IN ONE:
`colcon build --packages-skip ros_xeno_bridge && source install/setup.bash && ros2 launch launch image_to_pose_launch.py   `
----------------------------------------------------------------------------------------------------------------------------------------

