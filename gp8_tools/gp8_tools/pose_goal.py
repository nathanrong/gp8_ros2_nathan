
from pathlib import Path
import time
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from launch_param_builder import load_yaml
from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_msgs.msg import DisplayTrajectory

# Runs in VSC for now:
# source /opt/ros/jazzy/setup.bash
# source ~/ros2_ws/install/setup.bash
# python3 ~/ros2_ws/src/gp8_ros2/gp8_tools/gp8_tools/pose_goal.py

def main():
    # Define target pose (single or multi)
    targets = [
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

    config_dict["pilz_industrial_motion_planner"] = load_yaml(
        pilz_path
    )

    pipeline_names = config_dict["planning_pipelines"]
    config_dict["planning_pipelines"] = {
        "pipeline_names": pipeline_names,
    }

    config_dict["plan_request_params"] = {
        "planner_id": "RRTConnectkConfigDefault",
        "planning_pipeline": "ompl",
        "planning_time": 5.0,
        "planning_attempts": 10,
        "max_velocity_scaling_factor": 0.5,
        "max_acceleration_scaling_factor": 0.5,
    }

    config_dict["use_sim_time"] = True
    config_dict.update({
        "qos_overrides./clock.subscription.durability": "volatile",
        "qos_overrides./clock.subscription.reliability": "best_effort",
        "qos_overrides./clock.subscription.history": "keep_last",
        "qos_overrides./clock.subscription.depth": 1,
    })


    print("Planning pipelines:", config_dict["planning_pipelines"])
    print(
        "Pilz configuration loaded:",
        config_dict["pilz_industrial_motion_planner"] is not None,
    )

    rclpy.init()

    display_node = rclpy.create_node(
        "gp8_trajectory_display"
    )

    display_publisher = display_node.create_publisher(
        DisplayTrajectory,
        "/move_group/display_planned_path",
        10,
    )


    try:
        moveit = MoveItPy(
            node_name="gp8_pose_goal",
            config_dict=config_dict,
        )

        arm = moveit.get_planning_component("arm")
        time.sleep(1.0)
        if not targets:
            raise ValueError("No pose targets were provided.")

        for target_index, target in enumerate(targets, start=1):
            print(f"Planning target {target_index}/{len(targets)}")

            # Allow the current joint state to update after the previous motion.
            time.sleep(1.0)
            arm.set_start_state_to_current_state()

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

            arm.set_goal_state(
                pose_stamped_msg=pose_goal,
                pose_link="tool0",
            )

            plan_result = arm.plan()

            if not plan_result:
                print(f"Planning failed for target {target_index}.")
                return

            print(f"Planning succeeded for target {target_index}.")

            trajectory_msg = (
                plan_result.trajectory.get_robot_trajectory_msg()
            )

            display_trajectory = DisplayTrajectory()
            display_trajectory.trajectory.append(trajectory_msg)

            display_publisher.publish(display_trajectory)

            print("Published trajectory to RViz.")
            time.sleep(1.0)

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



    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
