from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    # base_pkg = get_package_share_directory('wifi_logger_visualizer')
    
    enable_interpolation = LaunchConfiguration('enable_interpolation')
    enable_interpolation_arg = DeclareLaunchArgument(
        'enable_interpolation',
        default_value='True',
        description='Enable interpolation for the wifi visualizer node'
    )
    ld.add_action(enable_interpolation_arg)
    
    generate_new_data = LaunchConfiguration('generate_new_data')
    generate_new_data_arg = DeclareLaunchArgument(
        'generate_new_data',
        default_value='False',
        description='Generate new data for the wifi visualizer node'
    )
    ld.add_action(generate_new_data_arg)

    run_logger = LaunchConfiguration('run_logger')
    run_logger_arg = DeclareLaunchArgument(
        'run_logger',
        default_value='False',
        description='Run the wifi logger node'
    )
    ld.add_action(run_logger_arg)
    
    run_visualizer = LaunchConfiguration('run_visualizer')
    run_visualizer_arg = DeclareLaunchArgument(
        'run_visualizer',
        default_value='True',
        description='Run the wifi visualizer node'
    )
    ld.add_action(run_visualizer_arg)
    
    ld.add_action(LogInfo(msg=["[wifi_logger_visualizer_launch.py] enable_interpolation: ", enable_interpolation,
                               ", generate_new_data: ", generate_new_data,
                               ", run_logger: ", run_logger,
                               ", run_visualizer: ", run_visualizer]))
    
    wifi_loger_node = Node(
        condition=IfCondition(run_logger),
        package='wifi_logger_visualizer',
        executable='wifi_logger_node',
        name='wifi_logger_node',
        output='screen',
    )
    ld.add_action(wifi_loger_node)
    
    wifi_visualizer_node = Node(
        condition=IfCondition(run_visualizer),
        package='wifi_logger_visualizer',
        executable='wifi_visualizer_node',
        name='wifi_visualizer_node',
        output='screen',
        parameters=[
            {'enable_interpolation': enable_interpolation, 'generate_new_data': generate_new_data}
                    ]
    )
    ld.add_action(wifi_visualizer_node)
    
    return ld
