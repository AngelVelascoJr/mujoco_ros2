from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_path

from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit



def generate_launch_description():

    #Ruta del archivo URDF
    urdf_path = os.path.join(get_package_share_path('mujoco_ros2_pkg'),
                             'FullURDF', 'full_urdf.xacro')
    
    #Ruta del archvo RVIZ
    rviz_config_path = os.path.join(get_package_share_path('mujoco_ros2_pkg'),
                                    'rviz', 'dofbot_trajectory_rviz.rviz')
    
    #Definicion del parametro de la ruta del archivo URDF
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    
    #Ejecucion del nodo robot_state_publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )

    controller_config_file = os.path.join(get_package_share_path('mujoco_ros2_pkg'), 'config', 'dofbot_controler.yaml')


    #Ejecucion del nodo de RVIZ
    config_arg = DeclareLaunchArgument(name = 'rvizconfig', default_value = rviz_config_path)
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )

    node_mujoco_ros2_control = Node(
        package='mujoco_ros2_control',
        executable='mujoco_ros2_control',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            controller_config_file,
            {'mujoco_model_path':os.path.join(urdf_path)}
        ]
    )

    #Retorno de la funcion del archivo launch
    return LaunchDescription([
        #robot_state_publisher_node,
        node_mujoco_ros2_control,
        #config_arg,
        #rviz2_node,
    ])