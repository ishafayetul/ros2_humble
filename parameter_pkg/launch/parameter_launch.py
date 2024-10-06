from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    param_path='/home/sib/colcon_ws/src/parameter_pkg/params/client_params.yaml'
    #param_path=os.path.join(get_package_share_directory('parameter_pkg'),'params','client_params.yaml')
    return LaunchDescription([
        Node(
            package='parameter_pkg',
            executable='client',
            name='Client_node',
            output='screen',
            emulate_tty=True,
            parameters=[param_path]
        )
    ])