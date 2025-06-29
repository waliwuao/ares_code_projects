from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import os

decision_mpc = False


def generate_launch_description():
    ld = LaunchDescription()
     
    pkg_dir = get_package_share_directory('omni_mpc_controller')
    # 指定参数文件路径
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')
    #单点激光定位模块
    
    # pkg_dir_2 = get_package_share_directory('laser')
    # params_file_2 = os.path.join(pkg_dir_2, 'config', 'params.yaml')
    # myRreal_node = Node(
    #     package='laser',
    #     executable='Real',
    #     # name='myRreal_node',
    #     output='screen',
    #     parameters=[params_file_2] # 加载参数文件
    # )    
    # ld.add_action(myRreal_node)

    if decision_mpc:
        mpc_node = Node(
            package = 'omni_mpc_controller',
            executable = 'MPC',
            remappings=[
                ('/mpc_decision','/cmd_vell')
            ],
            output = 'screen',
            parameters = [params_file]
        )
        remote = Node(
            package ='remote_rc_node',
            executable = 'udp_joy_node',
            remappings = [
                ('/cmd_vel','/cmd_vel_remote')
            ],
            output = 'screen'
        )
    else:
        mpc_node = Node(
            package = 'omni_mpc_controller',
            executable = 'MPC',
            output = 'screen',
            parameters = [params_file]
        )
        remote = Node(
            package ='remote_rc_node',
            executable = 'udp_joy_node',
            output = 'screen'
        )
    ld.add_action(mpc_node)
    ld.add_action(remote)

    return ld