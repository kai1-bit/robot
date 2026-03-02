import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取默认urdf路径 用于找到文件内容传给robot_state_publiser
    urdf_package_path = get_package_share_directory("robot")
    default_xacro_path = os.path.join(urdf_package_path,"urdf","project_robot","robot.urdf.xacro")
    # rviz2 配置文件路径
    default_config_path = os.path.join(urdf_package_path,"config","display_robot_model.rviz")
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

    action_joint_state_publisher = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )

    # 两种方式启动 1：由于rviz2同样存在于功能包 所以可以通过launch_ros启动 2：通过命令行启动
    action_rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        # 该参数直接作用于命令行 即在命令后直接接上
        arguments=['-d', default_config_path]
    )

    return launch.LaunchDescription([
        action_declare_arg_mode_value,
        action_robot_state_publisher,
        action_joint_state_publisher,
        action_rviz_node
    ])