from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 构建 MoveIt 配置
    moveit_config = MoveItConfigsBuilder(
        "piper",
        package_name="piper_with_gripper_moveit"
    ).moveit_cpp(file_path="config/moveit_cpp.yaml").to_moveit_configs()

    # 2. 获取 demo launch（包含 move_group, Rviz2, fake controller 等）
    ld = generate_demo_launch(moveit_config)

    # 3. 定义你的 MoveItPy 控制节点
    moveit_py_node = Node(
        package="piper_with_gripper_moveit",
        executable="pyctrl.py",          # 你的 Python 脚本名
        name="moveit_py_controller",     # 节点名（可自定义）
        output="screen",
        parameters=[moveit_config.to_dict()],  # 传递 MoveIt 参数（可选，但推荐）
    )


    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": False},
        ],
    )
    # 4. 将你的节点添加到 launch 描述中
    ld.add_action(moveit_py_node)
    ld.add_action(move_group_node)
    return ld