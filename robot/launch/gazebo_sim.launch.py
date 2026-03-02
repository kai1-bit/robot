import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取默认urdf路径 用于找到文件内容传给robot_state_publiser
    urdf_package_path = get_package_share_directory("robot")
    default_xacro_path = os.path.join(urdf_package_path,"urdf","project_robot","robot.urdf.xacro")
    # gazebo 配置文件路径
    default_world_path = os.path.join(urdf_package_path,"gazebo","new_world.world")
    # 声明参数 方便launch传入其他模型参数调试
    action_declare_arg_mode_value = launch.actions.DeclareLaunchArgument(
        name='model',default_value=str(default_xacro_path),description='加载的urdf模型'
    )
    # 通过文件路径获取内容并转化为参数对象
    substitutions_command_result = launch.substitutions.Command(['xacro ',launch.substitutions.LaunchConfiguration('model')])
    robot_description_urdf_content = launch_ros.parameter_descriptions.ParameterValue(substitutions_command_result,value_type=str)
    # 启动robot_state_publisher节点
    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        # 该参数传递给节点 使用健值对
        parameters=[{'robot_description':robot_description_urdf_content}]
    )

    # 直接包含功能包的launch文件
    action_launch_gazebol = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [get_package_share_directory('gazebo_ros'),'/launch','/gazebo.launch.py']
        ),
        launch_arguments=[('world',default_world_path),('verbose','true')]
    )

    action_spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic','/robot_description','-entity','robot']
    )

    # 用于启动插件的控制器 读取位置接口数据并发布
    action_load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd='ros2 control load_controller robot_joint_state_broadcaster --set-state active'.split(' '),
        output='screen'
    )

    # 加载并激活 robot_effort_controller 控制器
    # action_load_effort_controller = launch.actions.ExecuteProcess(
    #     cmd='ros2 control load_controller robot_effort_controller --set-state active'.split(' '),
    #     output='screen'
    # )    

    # 差速控制器最好不要与力控制器一起使用（命令接口不要暴露给多个控制器）
    action_load_diff_drive_controller = launch.actions.ExecuteProcess(
        cmd='ros2 control load_controller robot_diff_drive_controller --set-state active'.split(' '),
        output='screen'
    )  

    return launch.LaunchDescription([
        action_declare_arg_mode_value,
        action_robot_state_publisher,
        action_launch_gazebol,
        action_spawn_entity,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=action_spawn_entity,
                on_exit=[action_load_joint_state_controller],
            )
        ),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=action_load_joint_state_controller,
                on_exit=[action_load_diff_drive_controller],
            )
        )
    ])