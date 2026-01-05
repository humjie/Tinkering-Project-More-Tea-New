- Here is the explanation of controller launch file
- CONTROLLER_EXPLANATION.md

- To test the controller, first you need to run the robot simulation in gazebo first:
'''
cd /Tinkering-Project-More-Tea-New/ros2_robot_ws
colcon build
source install/setup.bash
ros2 launch moretea_description gazebo.launch.py
'''

- Then, run the controller:
'''
cd /Tinkering-Project-More-Tea-New/ros2_robot_ws
colcon build
source install/setup.bash
ros2 launch moretea_controller controller.launch.py
'''

- You have now activate the controllers!
- To interact with the controller, you may check the commands provided by controller manager: (press one space and twice tab behind!)
'''
ros2 control    
'''

- To show active controllers:
'''
ros2 control list_controllers
'''

- To control the robot in gazebo, you may use the built in controller
'''
ros2 topic pub /simple_velocity_controller/commands std_msgs/msg/Float64MultiArray "layout:
  dim: []
  data_offset: 0
data: [1, 1, 1, 1]"  # [left 1, left 2, right 1, right 2], 1 is forward, -1 is backward
'''
or using custom controller (for mecanum wheel)
'''
ros2 topic pub --once /moretea_controller/cmd_vel geometry_msgs/msg/TwistStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, twist: {linear: {x: 0.0, y: 1.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
'''
- Move Backward: linear.x: -1.0
- Slide Left (Strafe): linear.y: 1.0
- Slide Right (Strafe): linear.y: -1.0
- Spin in Place: angular.z: 1.0