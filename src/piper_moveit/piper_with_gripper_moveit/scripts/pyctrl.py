#!/usr/bin/env python3
"""
A script to outline the fundamentals of the moveit_py motion planning API,
with subscription to a /my_pose_cmd topic for dynamic pose goals.
"""

import time

# generic ros libraries
import rclpy
from rclpy.logging import get_logger
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy

# message type
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64

def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time=0.0,
):
    """Helper function to plan and execute a motion."""
    logger.info("Planning trajectory")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    if plan_result and plan_result.trajectory:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, ["arm_controller"])
        time.sleep(sleep_time)
    else:
        logger.error("Planning failed or no trajectory returned")


def main():
    rclpy.init()
    logger = get_logger("moveit_py.pose_goal")

    # Create a dedicated node for subscription
    node = rclpy.create_node("pose_goal_subscriber")
    callback_group = MutuallyExclusiveCallbackGroup()

    # Instantiate MoveItPy instance (its own internal node)
    piper = MoveItPy(node_name="moveit_py")
    arm = piper.get_planning_component("arm")

    def pose_cmd_callback(msg: PoseStamped):
        logger.info(f"Received new goal pose: {msg}")
        try:
            arm.set_start_state_to_current_state()
            arm.set_goal_state(pose_stamped_msg=msg, pose_link="gripper_base")
            plan_and_execute(piper, arm, logger, sleep_time=1.0)
        except Exception as e:
            logger.error(f"Failed to plan/execute: {e}")

    # Subscribe using the dedicated node
    node.create_subscription(
        PoseStamped,
        "my_pose_cmd",
        pose_cmd_callback,
        10,
        callback_group=callback_group,
    )
    logger.info("Subscribed to /my_pose_cmd")

    # gripper control
    def set_gripper(position: float) -> bool:
        position = max(0.0, min(0.07, position))  # 限幅到有效范围

        try:
            # 获取 RobotModel（用于构建 RobotState）
            robot_model = piper.get_robot_model()
            goal_state = RobotState(robot_model)

            # 方法 1：通过 joint group 名称设置（推荐）
            goal_state.set_joint_group_positions("gripper", [position])

            # 方法 2：通过关节名设置（也可行）
            # goal_state.set_joint_positions(["joint7"], [position])

            # 设置目标状态
            gripper = piper.get_planning_component("gripper")
            gripper.set_start_state_to_current_state()
            gripper.set_goal_state(robot_state=goal_state)

            # 规划并执行
            plan_result = gripper.plan()
            if plan_result and plan_result.trajectory:
                logger.info(f"Gripper moving to: {position:.4f} m")
                piper.execute(plan_result.trajectory, ["gripper_controller"])
                return True
            else:
                logger.error(f"Gripper planning failed for position {position}")
                return False

        except Exception as e:
            logger.error(f"Exception in set_gripper_position: {e}")
            return False

    def gripper_cmd_callback(msg: Float64):
        set_gripper(msg.data)

    # Subscribe
    node.create_subscription(Float64, "my_gripper_cmd", gripper_cmd_callback, 10, callback_group=callback_group)
    logger.info("Subscribed to /my_gripper_cmd")

    # Use MultiThreadedExecutor for both nodes
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    # MoveItPy's internal node is already active; no need to add it explicitly

    try:
        logger.info("Spinning to listen for pose commands...")
        executor.spin()
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    finally:
        node.destroy_node()  # ✅ Clean up your node
        piper.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()