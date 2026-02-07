import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro

def generate_launch_description():

    robotXacroName = 'robot'

    namePackage = 'autonav_bot'

    modelFilePath = 'model/robot_urdf.xacro'

    pathModelFile = os.path.join(get_package_share_directory(namePackage), modelFilePath)

    robotDescription = xacro.process_file(pathModelFile).toxml()

    gazebo_rosPackageLaunch=PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'),
                                                                        'launch', 'gz_sim.launch.py'))
    
    gazeboLaunch=IncludeLaunchDescription(gazebo_rosPackageLaunch, launch_arguments={'gz_args': ['-r -v -v4 empty.sdf'], 'on_exit_shutdown': 'true'}.items())

    # Gazebo Node
    spawnModelNodeGazebo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robotXacroName,
            '-topic', 'robot_description',
        ],
        output='screen'
    )

    # Robot State Publisher Node
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robotDescription,
        'use_sim_time': True}]
    )

    bridge_params= os.path.join(
    get_package_share_directory(namePackage),
    'parameters',
    'bridge_parameters.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
        '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
    ],
        output='screen'
    )

    # Launch Description
    launchDescriptionObject=LaunchDescription()

    launchDescriptionObject.add_action(gazeboLaunch)
    launchDescriptionObject.add_action(spawnModelNodeGazebo)
    launchDescriptionObject.add_action(nodeRobotStatePublisher)
    launchDescriptionObject.add_action(start_gazebo_ros_bridge_cmd)

    return launchDescriptionObject