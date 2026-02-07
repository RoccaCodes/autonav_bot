import launch
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import os
import xacro
from ament_index_python.packages import get_package_share_directory

packageName = 'autonav_bot'

xacroRelPath = 'model/robot_core.xacro'
rvizRelPath = 'parameters/robot_control.rviz'
ros2ControlRelPath = 'parameters/robot_controller.yaml'

def generate_launch_description():

    # Paths for your own package files
    pkgPath = launch_ros.substitutions.FindPackageShare(package=packageName).find(packageName)
    xacroPath = os.path.join(pkgPath, xacroRelPath)
    rvizPath = os.path.join(pkgPath, rvizRelPath)
    ros2ControlPath = os.path.join(pkgPath, ros2ControlRelPath)

    print(f"Xacro path: {xacroPath}")

    robot_desc = xacro.process_file(xacroPath).toxml()
    robot_description = {'robot_description': robot_desc}

    # Launch arguments
    declare_arguments = [
        launch.actions.DeclareLaunchArgument(
            name="gui", default_value="true",
            description="Start the Rviz2 GUI."
        )
    ]
    gui = LaunchConfiguration("gui")

    # Full path to gz_sim.launch.py as a literal string
    gz_sim_launch_path = os.path.join(
        get_package_share_directory("ros_gz_sim"),
        "launch",
        "gz_sim.launch.py"
    )

    # Gazebo GUI
    gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch_path),
        launch_arguments={"gz_args": "-r -v 3 empty.sdf"}.items(),
        condition=launch.conditions.IfCondition(gui)
    )

    # Gazebo headless
    gazebo_headless = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch_path),
        launch_arguments={"gz_args": "--headless-rendering -s -r -v 3 empty.sdf"}.items(),
        condition=launch.conditions.UnlessCondition(gui)
    )

    # Other nodes
    gazebo_bridge = launch_ros.actions.Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen"
    )

    gz_spawn_entity = launch_ros.actions.Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            "robot_system_position",
            "-allow_renaming",
            "true"
        ]
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description]
    )

    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rvizPath]
    )

    control_node = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2ControlPath],
        output="both"
    )

    joint_state_broadcaster_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"]
    )

    robot_controller_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--param-file", ros2ControlPath]
    )

    # Node list
    nodeList = [
        gazebo,
        gazebo_headless,
        gazebo_bridge,
        gz_spawn_entity,
        robot_state_publisher_node,
        rviz_node,
        control_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner
    ]

    return launch.LaunchDescription(declare_arguments + nodeList)
