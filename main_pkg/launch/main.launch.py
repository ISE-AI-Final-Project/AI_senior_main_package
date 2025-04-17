from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def color_log_node(package, executable, name, color_code):
    return Node(
        package=package,
        executable=executable,
        name=name,
        output='screen',
        parameters=[],
        emulate_tty=True,  # Required for colors to show
        additional_env={
            'RCUTILS_COLORIZED_OUTPUT': '1',
            'RCUTILS_CONSOLE_OUTPUT_FORMAT': f'{color_code}[{{severity}}] [{{name}}]: {{message}}\033[0m'
        }
    )

def generate_launch_description():
    return LaunchDescription([
<<<<<<< Updated upstream
        color_log_node('main_pkg', 'main', 'main_node', '\033[38;5;40m'),           # Green
        color_log_node('all_grasp_pkg', 'grasp_server', 'all_grasp_server', '\033[36m'),  # Cyan
        color_log_node('best_grasp_pkg', 'best_grasp_server', 'best_grasp_server', '\033[38;5;87m'),  # Blue
        # color_log_node('obj_collision', 'obj_collision', 'obj_collision_server', '\033[38;5;213m'),  # Pink
        color_log_node('collision_maker_pkg', 'collision_server', 'collision_maker_server', '\033[38;5;208m'),  # Orange
        color_log_node('colliding_links_checker', 'ur_colliding_boxes_pub', 'ur_colliding_boxes_node', '\033[38;5;208m'),  # Orange
        color_log_node('senior_gripper_control', 'gripper_control', 'gripper_control_server', '\033[38;5;93m'),  # Purple
        color_log_node('goal_pub_pkg', 'robot_server', 'robot_server', '\033[38;5;220'),  # Gold
=======
        color_log_node('main_pkg', 'main', 'main_node', '\033[38;5;40m'),  # Green
        color_log_node('all_grasp_pkg', 'grasp_server', 'grasp_server_node', '\033[36m'),  # Cyan
        color_log_node('best_grasp_pkg', 'best_grasp_server', 'best_grasp_node', '\033[33m'),  # Yellow
        color_log_node('collision_maker_pkg', 'collision_server', 'collision_server_node', '\033[38;5;208m'),  # Orange
        color_log_node('goal_pub_pkg', 'robot_server', 'robot_server_node', '\033[38;5;154m'),  # Yellow-Green
        color_log_node('senior_gripper_control', 'gripper_control', 'gripper_control_node', '\033[38;5;141m'),  # Purple
        color_log_node('collision_stl', 'stl_collision_loader', 'stl_collision_node', '\033[38;5;81m'),  # Light Blue
>>>>>>> Stashed changes
    ])
