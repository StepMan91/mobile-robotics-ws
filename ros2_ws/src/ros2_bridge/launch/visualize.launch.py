from launch import LaunchDescription
from launch_ros.actions import Node
import os
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Process G1 URDF
    g1_pkg_path = get_package_share_directory('g1_description')
    urdf_file = os.path.join(g1_pkg_path, 'urdf', 'g1.urdf')
    doc = xacro.process_file(urdf_file)
    robot_description = {'robot_description': doc.toxml()}

    return LaunchDescription([
        # 1. Start the UDP Bridge Node
        Node(
            package='ros2_bridge',
            executable='human_bridge_node',
            name='human_bridge_node',
            output='screen',
            parameters=[{'udp_port': 8888}]
        ),

        # 2. Publish a static transform for the camera
        # Map -> Camera Link (0,0,1m up)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '1', '0', '0', '0', 'map', 'camera_link']
        ),
        
        # 2b. Publish a static transform for the robot root (pelvis)
        # REMOVED: Now handled by human_bridge_node for dynamic tracking
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments=['0', '0', '0.78', '0', '0', '0', 'map', 'pelvis']
        # ),
        
        # 3. Robot State Publisher (G1)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),
        
        # 4. Joint State Publisher (Fake for now, to hold robot together)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[robot_description]
        ),

        # 5. Start RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log'
        )
    ])
