from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    
    use_python_arg = DeclareLaunchArgument(
        'use_python',
        default_value='False',
    )

    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.029315',
    )

    wheel_separation_width_arg = DeclareLaunchArgument(
        'wheel_separation_width',
        default_value='0.16',
    )

    wheel_separation_length_arg = DeclareLaunchArgument(
        'wheel_separation_length',
        default_value='0.21',
    )

    use_python = LaunchConfiguration('use_python')
    wheel_radius = LaunchConfiguration('wheel_radius')
    wheel_separation_width = LaunchConfiguration('wheel_separation_width')
    wheel_separation_length = LaunchConfiguration('wheel_separation_length')

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    mecanum_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "mecanum_velocity_controller",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    mecanum_controller_py = Node(
        package="moretea_controller",
        executable="mecanum_controller.py",
        parameters=[{"wheel_radius": wheel_radius,
                     "wheel_separation_width": wheel_separation_width,
                     "wheel_separation_length": wheel_separation_length}],
        condition=IfCondition(use_python)
    )

    mecanum_controller_cpp = Node(
        package="moretea_controller",
        executable="mecanum_controller",
        parameters=[{"wheel_radius": wheel_radius,
                     "wheel_separation_width": wheel_separation_width,
                     "wheel_separation_length": wheel_separation_length}],
        condition=UnlessCondition(use_python)
    )

    return LaunchDescription([
        use_python_arg,
        wheel_radius_arg,
        wheel_separation_width_arg,
        wheel_separation_length_arg,
        joint_state_broadcaster_spawner,
        mecanum_controller,
        mecanum_controller_py,
        mecanum_controller_cpp
    ])