- To visualize the urdf robot model in rviz, run the below command
'''
cd /Tinkering-Project-More-Tea-New/ros2_robot_ws
colcon build
source install/setup.bash
ros2 launch moretea_description display.launch.py
'''
- To run the robot simulation in gazebo, run the below command
'''
cd /Tinkering-Project-More-Tea-New/ros2_robot_ws
colcon build
source install/setup.bash
ros2 launch moretea_description gazebo.launch.py
'''