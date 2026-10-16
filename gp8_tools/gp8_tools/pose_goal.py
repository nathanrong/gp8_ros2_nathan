
from pathlib import Path
import time
import math
import rclpy
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from launch_param_builder import load_yaml
from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_msgs.msg import DisplayTrajectory
from moveit.core.robot_state import RobotState

# Runs in VSC for now:
# source /opt/ros/jazzy/setup.bash
# source ~/ros2_ws/install/setup.bash
# python3 ~/ros2_ws/src/gp8_ros2/gp8_tools/gp8_tools/pose_goal.py

def make_pose_goal(target):
    # Make tool0 pose goal for a given target
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "base_link"

    x, y, z = target["position"]
    qx, qy, qz, qw = target["orientation"]

    pose_goal.pose.position.x = x
    pose_goal.pose.position.y = y
    pose_goal.pose.position.z = z
    pose_goal.pose.orientation.x = qx
    pose_goal.pose.orientation.y = qy
    pose_goal.pose.orientation.z = qz
    pose_goal.pose.orientation.w = qw

    return pose_goal

def publish_trajectory(display_publisher, plan_result):
    # Publish trajectory to Rviz/MoveIt for visualization
    trajectory_msg = plan_result.trajectory.get_robot_trajectory_msg()
    display_trajectory = DisplayTrajectory()
    display_trajectory.trajectory.append(trajectory_msg)
    display_publisher.publish(display_trajectory)

def command_gripper(node, gripper_client, position, max_effort=10.0):
    # Command the gripper and wait for action result. 
    # Gripper takes two floats: position and max effort
    # Position: 0.000 = open, 0.031 = closed
    if not gripper_client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error("Gripper action server not availiable...")
        return False

    goal = GripperCommand.Goal()
    goal.command.position = position
    goal.command.max_effort = max_effort

    node.get_logger().info(f"Commanding gripper to position {position:.3f} m")

    goal_future = gripper_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, goal_future)
    goal_handle = goal_future.result()

    if not goal_handle.accepted:
        node.get_logger().error("Gripper command was rejected...")
        return False

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    result = result_future.result().result

    node.get_logger().info(
        f"Gripper result: position = {result.position:.3f} m, "
        f"effort = {result.effort:.3f} N, "
        f"reached_goal = {result.reached_goal}, "
        f"stalled = {result.stalled}"
    )
    return result.reached_goal

def main():
    ## GLOBALS ##

    interactive_mode = False

    # PTP similar to MOVJ
    # LIN imilar to MOVL
    move_mode = "PTP"

    velocity_scale = 0.5 # Multiple of speed
    accel_scale = 0.5
    state_update_wait = 1
    rviz_display_wait = 1
    waypoint_pause = 0.5

    # Gripper positions
    gripper_open = 0.000
    gripper_close = 0.031
    gripper_max_effort = 10.0

    ## GLOBALS ##

    move_mode = move_mode.upper()
    if move_mode not in {"PTP", "LIN"}:
        raise ValueError("Incorrect mode selected...")

    # Define target pose (single or multi)
    targets = [
        # {
        #     "position": (0.3055, 0.0966, 0.8535),
        #     "orientation": (0.707, 0.000, 0.707, 0.000),
        # },
        {
            "position": (0.450, 0.000, 0.750),
            "orientation": (0.707, 0.000, 0.707, 0.000),
        },
        {
            "position": (0.300, -0.150, 0.850),
            "orientation": (0.707, 0.000, 0.707, 0.000),
        },
        {
            "position": (0.400, 0.150, 0.700),
            "orientation": (0.707, 0.000, 0.707, 0.000),
        },
    ]

    # Joint positions in radians
    safe_state = {
        "joint_1_s": 0.0,
        "joint_2_l": 0.0,
        "joint_3_u": 0.0,
        "joint_4_r": 0.0,
        "joint_5_b": -math.pi/2,
        "joint_6_t": 0.0,
    }

    home_state = {
            "joint_1_s": 0.0,
            "joint_2_l": 0.0,
            "joint_3_u": 0.0,
            "joint_4_r": 0.0,
            "joint_5_b": 0.0,
            "joint_6_t": 0.0,
        }
    
    # Build the complete MoveIt configuration.
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="motoman_gp8",
            package_name="gp8_moveit_config",
        )
        .robot_description_kinematics(
            file_path="config/gp8/kinematics.yaml"
        )
        .joint_limits(
            file_path="config/gp8/joint_limits.yaml"
        )
        .trajectory_execution(
            file_path="config/gp8/moveit_controllers.yaml"
        )
        .planning_pipelines(
            pipelines=[
                "ompl",
                "pilz_industrial_motion_planner",
                "stomp",
            ],
            default_planning_pipeline="ompl",
            load_all=True,
        )
        .pilz_cartesian_limits(
            file_path="config/gp8/pilz_cartesian_limits.yaml"
        )
        .to_moveit_configs()
    )

    config_dict = moveit_config.to_dict()

    # Explicitly load the Pilz pipeline configuration. This preserves all
    # three planning pipelines while fixing the missing Pilz dictionary.
    package_path = Path(
        get_package_share_directory("gp8_moveit_config")
    )

    pilz_path = (
        package_path
        / "config"
        / "pilz_industrial_motion_planner_planning.yaml"
    )

    config_dict["pilz_industrial_motion_planner"] = load_yaml(pilz_path)

    pipeline_names = config_dict["planning_pipelines"]
    config_dict["planning_pipelines"] = {
        "pipeline_names": pipeline_names,
    }

    config_dict["plan_request_params"] = {
        "planner_id": "RRTConnectkConfigDefault",
        "planning_pipeline": "ompl",
        "planning_time": 5.0,
        "planning_attempts": 10,
        "max_velocity_scaling_factor": velocity_scale,
        "max_acceleration_scaling_factor": accel_scale,
    }

    config_dict["use_sim_time"] = True
    config_dict.update({
        "qos_overrides./clock.subscription.durability": "volatile",
        "qos_overrides./clock.subscription.reliability": "best_effort",
        "qos_overrides./clock.subscription.history": "keep_last",
        "qos_overrides./clock.subscription.depth": 1,
    })

    print(f"Motion Mode: {move_mode}")
    print("Planning pipelines:", config_dict["planning_pipelines"])
    print(
        "Pilz configuration loaded:",
        config_dict["pilz_industrial_motion_planner"] is not None,
    )

    rclpy.init()
    display_node = rclpy.create_node("gp8_trajectory_display")
    display_publisher = display_node.create_publisher(
        DisplayTrajectory,
        "/move_group/display_planned_path",
        10,
    )

    # Create action client for gripper
    gripper_client = ActionClient(display_node, GripperCommand, 
                                  "/gripper_action_controller/gripper_cmd")

    try:
        moveit = MoveItPy(
            node_name="gp8_pose_goal",
            config_dict=config_dict,
        )

        arm = moveit.get_planning_component("arm")

        if not targets:
            raise ValueError("No pose targets were provided.")

        for target_index, target in enumerate(targets, start=1):
            print(f"Planning target {target_index}/{len(targets)}")

            # Allow the current joint state to update after the previous motion.
            time.sleep(state_update_wait)
            arm.set_start_state_to_current_state()
            arm.set_goal_state(
                pose_stamped_msg = make_pose_goal(target),
                pose_link = "tool0",
            )

            plan_result = arm.plan()

            if not plan_result:
                print(f"Planning failed for target {target_index}.")
                return

            print(f"Planning succeeded for target {target_index}.")

            publish_trajectory(display_publisher, plan_result)
            print("Published trajectory to RViz.")
            time.sleep(rviz_display_wait)

            if interactive_mode:
                input(
                    f"Press Enter to execute target {target_index}, "
                    "or Ctrl+C to stop..."
                )

            execution_result = moveit.execute(
                plan_result.trajectory,
                controllers=[],
            )

            if not execution_result:
                print(f"Execution failed for target {target_index}.")
                return

            print(f"Execution succeeded for target {target_index}.")

            command = input(
                "\nEnter command:\n"
                "  o = open gripper\n"
                "  c = close gripper\n"
                "  s = safe position\n"
                "  r = rest position\n"
                "  Enter = finish\n"
                "Command: "
            ).strip().lower()

            if command == "o":
                command_gripper(
                    display_node,
                    gripper_client,
                    gripper_open,
                    gripper_max_effort,
                )

            elif command == "c":
                command_gripper(
                    display_node,
                    gripper_client,
                    gripper_close,
                    gripper_max_effort,
                )

            elif command in {"s", "r"}:
                if command == "r":
                    target_name = "rest position"
                    target_joints = home_state
                else:
                    target_name = "safe position"
                    target_joints = safe_state

                print(f"Returning to {target_name}...")
                time.sleep(state_update_wait)

                arm.set_start_state_to_current_state()

                target_state = RobotState(moveit.get_robot_model())
                target_state.joint_positions = target_joints
                arm.set_goal_state(robot_state=target_state)

                target_plan = arm.plan()

                if not target_plan:
                    print(f"Planning failed for {target_name}.")
                    return

                print(f"Planning to {target_name} succeeded.")
                publish_trajectory(display_publisher, target_plan)
                time.sleep(waypoint_pause)

                target_execution_result = moveit.execute(
                    target_plan.trajectory,
                    controllers=[],
                )

                if target_execution_result:
                    print(f"Execution to {target_name} succeeded.")
                else:
                    print(f"Execution to {target_name} failed.")

            elif command == "":
                print("Finished.")

            else:
                print("Unknown command.")

    finally:
        display_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
