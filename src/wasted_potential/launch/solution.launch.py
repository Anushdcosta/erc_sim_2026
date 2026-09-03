from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    shelf_column_number_arg = DeclareLaunchArgument(
        'shelf_column_number',
        default_value='1',
        description='The shelf column number to navigate to (1-5)'
    )
    
    book_colour_arg = DeclareLaunchArgument(
        'book_colour',
        default_value='red',
        description='The color of the book to find'
    )
    
    prepare_perception_node = Node(
        package='wasted_potential',
        executable='prepare_perception.py',
        name='prepare_perception',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    visual_coordinator_node = Node(
        package='wasted_potential',
        executable='visual_coordinator.py',
        name='visual_coordinator',
        output='screen',
        arguments=[LaunchConfiguration('shelf_column_number')],
        parameters=[{'use_sim_time': True}]
    )
    
    perception_node = Node(
        package='wasted_potential',
        executable='perception_node.py',
        name='perception_node',
        output='screen',
        parameters=[{'target_colour': LaunchConfiguration('book_colour'), 'use_sim_time': True}]
    )
    
    grasp_coordinator_node = Node(
        package='wasted_potential',
        executable='grasp_coordinator.py',
        name='grasp_coordinator',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        shelf_column_number_arg,
        book_colour_arg,
        prepare_perception_node,
        visual_coordinator_node,
        perception_node,
        grasp_coordinator_node
    ])
