Here is the mechanics of what happens when you run controller.launch.py while Gazebo is already running.

1. The "Server" is already waiting (Inside Gazebo)
- When you first launched Gazebo (using gazebo.launch.py), you likely loaded a plugin called libgazebo_ros2_control.so in your URDF.

    - This plugin started a Controller Manager inside Gazebo.

    - This Manager is currently sitting idle, listening for "service calls" (requests) on specific channels like /controller_manager/load_controller.

2. The "Client" connects (Your new launch file)
- When you run ros2 launch ... controller.launch.py, you are starting Spawner Nodes. These are not "robot drivers" themselves; they are just messengers.

- Here is the sequence of events:

    - Launch: The spawner node starts up.

    - Knock Knock: It looks for the service /controller_manager/load_controller.

    - Handshake: It finds the Controller Manager (running inside the Gazebo window).

    - Request: It sends a message: "Hey, please load the configuration for diff_drive_controller and start it."

    - Action: The Controller Manager inside Gazebo reads the YAML params (which you loaded earlier or passed now), allocates memory for that controller, and starts looping it.

    - Success: The Manager replies "OK". The spawner node usually stays alive to monitor it or exits (depending on configuration).

3. Why is this useful?
- This "separation of concerns" is a huge feature of ROS 2.

    - Restarting Controllers: If your robot acts crazy, you can kill the controller.launch.py (stopping the controllers) without closing Gazebo. You can then tweak your YAML params and relaunch just the controllers. You don't have to wait for the heavy simulation to reload.

    - Modular Loading: You can have one launch file for the base wheels, another for the arm, and another for the gripper. You can load/unload them as needed.