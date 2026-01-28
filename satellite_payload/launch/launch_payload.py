from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
 return LaunchDescription([Node(package='satellite_payload', executable='imu_node', 
  output='screen'), Node(package='satellite_payload', executable='env_node', output='screen'), 
  Node(package='satellite_payload', executable='light_node', output='screen'), 
  Node(package='satellite_payload', executable='cam_node', output='screen'), 
  Node(package='satellite_payload', executable='fusion_node', output='screen'), 
  Node(package='satellite_payload', executable='scheduler_node', output='screen'), 
  Node(package='satellite_payload', executable='telemetry_uploader',output='screen')
 ])
