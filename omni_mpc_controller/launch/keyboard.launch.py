from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import os

def generate_launch_description():
    ld = LaunchDescription()
    pkg_dir = get_package_share_directory('omni_mpc_controller')
    # 指定参数文件路径
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    keyboard_control_node = Node(
        package = 'omni_mpc_controller',
        executable = 'keyboard_control',
        output = 'screen',
        parameters = [params_file]
    )
    ld.add_action(keyboard_control_node)
    
    return ld