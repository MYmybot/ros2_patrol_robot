import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取默认路径
    urdf_pachage_path = get_package_share_directory('mybot_description')
    default_urdf_path = os.path.join(urdf_pachage_path,'urdf','first_robot.urdf')
    default_rviz_config_path = os.path.join(urdf_pachage_path,'config','display_robot_model.rviz')
    # 声明一个urdf的参数，方便修改
    action_declare_arg_node_path = launch.actions.DeclareLaunchArgument(
        name = 'model',default_value = str(default_urdf_path),description='加载的模型文件目录'
    )

    # 通过文件路径获取内容，并转换成参数值对象，以供传入 robot_state_publisher
    substitution_command_result = launch.substitutions.Command(['xacro ',launch.substitutions.LaunchConfiguration('model')])
    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(substitution_command_result,value_type=str)

    # 状态发布节点
    action_robot_state_publisher = launch_ros.actions.Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        parameters = [{'robot_description':robot_description_value}]
    )

    # 关节状态发布节点
    action_joint_state_publisher = launch_ros.actions.Node(
        package = 'joint_state_publisher',
        executable = 'joint_state_publisher',
    )

    # RViz节点
    action_rviz_node = launch_ros.actions.Node(
        package = 'rviz2',
        executable = 'rviz2',
        arguments = ['-d',default_rviz_config_path]
    )

    return launch.LaunchDescription([
        action_declare_arg_node_path,
        action_robot_state_publisher,
        action_joint_state_publisher,
        action_rviz_node
    ])