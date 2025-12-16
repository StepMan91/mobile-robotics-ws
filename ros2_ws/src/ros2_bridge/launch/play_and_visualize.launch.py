from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('ros2_bridge')
    g1_description_share = FindPackageShare('g1_description')
    
    # Paths
    rviz_config = PathJoinSubstitution([pkg_share, 'launch', 'visualize.rviz']) # Assuming default rviz config exists or use empty
    urdf_file = PathJoinSubstitution([g1_description_share, 'urdf', 'g1_fixed.urdf'])
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', urdf_file])}] 
        # Note: using xacro command even if it's .urdf just in case, or 'cat' if pure urdf.
        # But commonly urdfs in ROS2 might need xacro processing or just file content.
        # If it is pure urdf without xacro macros, Command(['cat ', urdf_file]) works too.
        # Let's assume standard xacro usage as it's safer for urdfs.
    )

    # Human Bridge Node
    human_bridge_node = Node(
        package='ros2_bridge',
        executable='human_bridge_node',
        name='human_bridge_node',
        output='screen'
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    # Rosbag Play
    # Bag Path: /home/wsluser/ros2_ws/recordings/Input_RosBag_3_Fixed
    play_bag_cmd = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', '/home/wsluser/ros2_ws/recordings/Input_RosBag_3_Fixed', '--loop'],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        human_bridge_node,
        rviz_node,
        play_bag_cmd
    ])
