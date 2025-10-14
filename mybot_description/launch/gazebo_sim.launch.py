import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os

import launch_ros.parameter_descriptions

def generate_launch_description():
    # 获取功能包的share路径
    urdf_pachage_path = get_package_share_directory('mybot_description')
    default_xacro_path = os.path.join(urdf_pachage_path,'urdf','mybot/mybot.urdf.xacro')
    # default_rviz_config_path = os.path.join(urdf_pachage_path,'config','display_robot_model.rviz')
    default_gazebo_world_path = os.path.join(urdf_pachage_path,'world','maze_room.world')
    # 声明一个urdf的参数，方便修改
    action_declare_arg_node_path = launch.actions.DeclareLaunchArgument(
        name = 'model',default_value = str(default_xacro_path),description='加载的模型文件目录'
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

    action_launch_gazebo = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [get_package_share_directory('gazebo_ros'),'/launch','/gazebo.launch.py']
        ),
        launch_arguments=[('world',default_gazebo_world_path),('verbose','true')]
    )

    action_spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic','/robot_description','-entity','mybot']
    )

    action_load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd='ros2 control load_controller mybot_joint_state_broadcaster --set-state active'.split(' '),
        output='screen'
    )

    action_load_effort_controller = launch.actions.ExecuteProcess(
        cmd='ros2 control load_controller mybot_effort_controller --set-state active'.split(' '),
        output='screen'
    )

    action_load_diff_driver_controller = launch.actions.ExecuteProcess(
        cmd='ros2 control load_controller mybot_diff_drive_controller --set-state active'.split(' '),
        output='screen'
    )

    # 关节状态发布节点
    # action_joint_state_publisher = launch_ros.actions.Node(
    #     package = 'joint_state_publisher',
    #     executable = 'joint_state_publisher',
    # )

    # RViz节点
    # action_rviz_node = launch_ros.actions.Node(
    #     package = 'rviz2',
    #     executable = 'rviz2',
    #     arguments = ['-d',default_rviz_config_path]
    # )

    return launch.LaunchDescription([
        action_declare_arg_node_path,
        action_robot_state_publisher,
        # action_joint_state_publisher,
        # action_rviz_node
        action_launch_gazebo,
        action_spawn_entity,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=action_spawn_entity,
                on_exit=[action_load_joint_state_controller],
            )
        ),
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit(
        #         target_action=action_load_joint_state_controller,
        #         on_exit=[action_load_effort_controller],
        #     )
        # ), # 用力控制器控制小车移动
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=action_load_joint_state_controller,
                on_exit=[action_load_diff_driver_controller],
            )
        ) # 用两轮差速控制器控制小车移动
    ])