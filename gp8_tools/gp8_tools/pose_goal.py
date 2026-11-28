from pathlib import Path
import subprocess
import time
import math
import csv
from tf2_ros import Buffer, TransformListener
import rclpy
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseStamped
from launch_param_builder import load_yaml
from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_msgs.msg import DisplayTrajectory, CollisionObject, AttachedCollisionObject, Constraints, PositionConstraint
from moveit.core.robot_state import RobotState
from shape_msgs.msg import SolidPrimitive

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

def make_position_only_goal(target, pose_link="tool0", tolerance=0.001):
    # MoveIt will constrain the selected link to the requested XYZ position
    position_goal = PositionConstraint()
    position_goal.header.frame_id = "base_link"
    position_goal.link_name = pose_link
    position_goal.weight = 1.0

    x, y, z = target["position"]

    # Allowed position region: sphere with radius = tolerance
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [tolerance]

    sphere_pose = Pose()
    sphere_pose.position.x = x
    sphere_pose.position.y = y
    sphere_pose.position.z = z
    sphere_pose.orientation.w = 1.0

    position_goal.constraint_region.primitives.append(sphere)
    position_goal.constraint_region.primitive_poses.append(sphere_pose)

    goal_constraints = Constraints()
    goal_constraints.position_constraints.append(position_goal)

    return goal_constraints

def load_position_targets(csv_path):
    # Load position-only targets from a CSV file
    # Expected columns: target_id, x_m, y_m, z_m
    targets = []

    with open(csv_path, newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            targets.append({
                "target_id": row["target_id"],
                "position": (
                    float(row["x_m"]),
                    float(row["y_m"]),
                    float(row["z_m"]),
                ),
            })
    return targets

def poll_final_tool_position(
    tf_buffer,
    node,
    tool_link="tool0",
    poll_duration=0.5,
    poll_period=0.01,
):
    # Continuously poll TF and return the newest tool position.

    latest_position = None
    end_time = time.monotonic() + poll_duration

    while time.monotonic() < end_time:
        rclpy.spin_once(node, timeout_sec=poll_period)

        try:
            transform = tf_buffer.lookup_transform(
                "base_link",
                tool_link,
                rclpy.time.Time(),
            )

            translation = transform.transform.translation

            latest_position = (
                translation.x,
                translation.y,
                translation.z,
            )

        except Exception:
            pass

    return latest_position

def run_csv_position_sequence(
    moveit,
    arm,
    display_publisher,
    display_node,
    tf_buffer,
    csv_targets,
    home_state,
    output_csv_path,
    state_update_wait,
    waypoint_pause,
    rviz_display_wait,
    interactive_mode,
):
    # Execute every CSV target using position-only planning.
    # Sequence: home -> target 1 -> home -> target 2 -> home -> ...

    with open(output_csv_path, "w", newline="") as csv_file:
        writer = csv.writer(csv_file)

        writer.writerow([
            "target_id",
            "target_x_m",
            "target_y_m",
            "target_z_m",
            "result_x_m",
            "result_y_m",
            "result_z_m",
        ])

        print("Moving to home position before CSV sequence...")

        home_success = move_to_joint_state(
            moveit=moveit,
            arm=arm,
            display_publisher=display_publisher,
            joint_positions=home_state,
            state_name="home position",
            state_update_wait=state_update_wait,
            waypoint_pause=waypoint_pause,
        )

        if not home_success:
            print("Could not reach home position. Aborting.")
            return False

        for target_index, target in enumerate(csv_targets, start=1):
            target_id = target["target_id"]
            target_position = target["position"]

            print(
                f"\nExecuting target "
                f"{target_index}/{len(csv_targets)}: {target_id}"
            )
            print(f"Target position: {target_position}")

            target_success = execute_pose_targets(
                moveit=moveit,
                arm=arm,
                display_publisher=display_publisher,
                targets=[target],
                move_mode="PTP",
                state_update_wait=state_update_wait,
                rviz_display_wait=rviz_display_wait,
                interactive_mode=interactive_mode,
                pose_link="tool0",
                position_only=True,
            )

            # Poll continuously after reaching the target
            result_position = poll_final_tool_position(
                tf_buffer=tf_buffer,
                node=display_node,
                tool_link="tool0",
                poll_duration=0.5,
                poll_period=0.01,
            )

            if result_position is None:
                result_position = ("", "", "")

            # Write the result before returning home
            writer.writerow([
                target["target_id"],
                target["position"][0],
                target["position"][1],
                target["position"][2],
                result_position[0],
                result_position[1],
                result_position[2],
            ])

            csv_file.flush()

            # Only now return home
            home_success = move_to_joint_state(
                moveit=moveit,
                arm=arm,
                display_publisher=display_publisher,
                joint_positions=home_state,
                state_name="home position",
                state_update_wait=state_update_wait,
                waypoint_pause=waypoint_pause,
            )
            if not home_success:
                print("Could not return home. Aborting CSV sequence.")
                return False

    print(f"\nSaved results to:\n{output_csv_path}")
    return True

def execute_pose_targets(
    moveit,
    arm,
    display_publisher,
    targets,
    move_mode,
    state_update_wait,
    rviz_display_wait,
    interactive_mode,
    pose_link = "tool0",
    position_only = False,
):
    # Plan and execute the tool0 target sequence
    # tool0 default pose goal, can replace with other joints/gripper

    if not targets:
        print("No pose targets were provided.")
        return False

    print(f"Executing {len(targets)} targets using {pose_link}...")

    for target_index, target in enumerate(targets, start=1):
        print(f"Planning target {target_index}/{len(targets)}")

        time.sleep(state_update_wait)

        arm.set_start_state_to_current_state()

        if position_only:
            arm.set_goal_state(
                motion_plan_constraints=[
                    make_position_only_goal(
                        target,
                        pose_link=pose_link,
                        tolerance=0.001,
                    )
                ]
            )
        else:
            arm.set_goal_state(
                pose_stamped_msg=make_pose_goal(target),
                pose_link=pose_link,
            )

        plan_result = arm.plan()

        if not plan_result:
            print(f"Planning failed for target {target_index}.")
            return False

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
            return False

        print(f"Execution succeeded for target {target_index}.")

    print("Tool0 target sequence complete.")
    return True

def move_to_joint_state(
    moveit,
    arm,
    display_publisher,
    joint_positions,
    state_name,
    state_update_wait,
    waypoint_pause,
):
    # Plan and execute a joint trajectory

    print(f"Planning to {state_name}...")

    time.sleep(state_update_wait)
    arm.set_start_state_to_current_state()

    target_state = RobotState(moveit.get_robot_model())
    target_state.joint_positions = joint_positions

    arm.set_goal_state(robot_state=target_state)

    plan_result = arm.plan()

    if not plan_result:
        print(f"Planning failed for {state_name}.")
        return False

    print(f"Planning to {state_name} succeeded.")

    publish_trajectory(display_publisher, plan_result)
    time.sleep(waypoint_pause)

    execution_result = moveit.execute(
        plan_result.trajectory,
        controllers=[],
    )

    if execution_result:
        print(f"Execution to {state_name} succeeded.")
        return True

    print(f"Execution to {state_name} failed.")
    return False

def publish_trajectory(display_publisher, plan_result):
    # Publish trajectory to Rviz/MoveIt for visualization
    # Note: Rviz is NOT required for the sim to work
    trajectory_msg = plan_result.trajectory.get_robot_trajectory_msg()
    display_trajectory = DisplayTrajectory()
    display_trajectory.trajectory.append(trajectory_msg)
    display_publisher.publish(display_trajectory)

def command_gripper(node, gripper_client, position, max_effort=10.0, accept_stall=False):
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

    if result.reached_goal:
        return True
    if result.stalled and accept_stall:
        node.get_logger().info(
            "Gripper stalled against the object. Treat as contact..."
        )
        return True
    return False

def add_box_to_planning_scene(moveit, box_name, box_size, box_position):
    # Add box to MoveIt planning scene
    planning_scene_monitor = moveit.get_planning_scene_monitor()

    collision_object = CollisionObject()
    collision_object.header.frame_id = "base_link"
    collision_object.id = box_name

    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = list(box_size)

    box_pose = Pose()
    box_pose.orientation.w = 1.0

    # box Gazebo position
    box_pose.position.x = box_position[0]
    box_pose.position.y = box_position[1]
    box_pose.position.z = box_position[2]

    collision_object.primitives.append(box)
    collision_object.primitive_poses.append(box_pose)
    collision_object.operation = CollisionObject.ADD

    planning_scene_monitor.process_collision_object(collision_object)

    print("Added pick_box to the MoveIt planning scene.")

def attach_box_to_robot(
    # Temporary idea: attach box to robot
    # This allows moveit to ignore collision
    # preventing OMPL from rejecting plan
    moveit,
    box_name,
    box_size, box_position, robot_link="tool0"):
    planning_scene_monitor = moveit.get_planning_scene_monitor()

    attached_object = AttachedCollisionObject()
    attached_object.link_name = robot_link

    attached_object.object.header.frame_id = "base_link"
    attached_object.object.id = box_name
    attached_object.object.operation = CollisionObject.ADD

    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = list(box_size)

    box_pose = Pose()
    box_pose.orientation.w = 1.0
    box_pose.position.x = box_position[0]
    box_pose.position.y = box_position[1]
    box_pose.position.z = box_position[2]

    attached_object.object.primitives.append(box)
    attached_object.object.primitive_poses.append(box_pose)

    attached_object.touch_links = [
        "tool0",
        "gripper_base",
        "gripper_right",
        "gripper_left",
    ]

    planning_scene_monitor.process_attached_collision_object(attached_object)

    print(f"Attached {box_name} to {robot_link} in MoveIt.")

def allow_box_collision(moveit, box_name):
    planning_scene_monitor = moveit.get_planning_scene_monitor()

    finger_links = [
        "gripper_right",
        "gripper_left",
    ]

    with planning_scene_monitor.read_write() as scene:
        acm = scene.allowed_collision_matrix

        for finger_link in finger_links:
            acm.set_entry(
                box_name,
                finger_link,
                True,
            )

        scene.allowed_collision_matrix = acm
        scene.current_state.update()

    print(f"Allowed {box_name} contact with gripper fingers.")

def spawn_box_in_gazebo(node, box_name, box_sdf_path, box_position):
    # Spawn the box into Gazebo
    x, y, z = box_position
    
    command = [
        "ros2", "run", "ros_gz_sim", "create", "--world", "default",
        "--file", str(box_sdf_path), "--name", box_name,
        "-x", str(x), "-y", str(y), "-z", str(z),
    ]

    node.get_logger().info("Spawning box in Gazebo...")

    # Eventually replace subprocess with ROS service client
    try:
        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=10.0,
        )
    except subprocess.TimeoutExpired:
        node.get_logger().error("Timed out while spawning the box.")
        return False

    if result.returncode != 0:
        node.get_logger().error(f"Gazebo failed to spawn the box:\n{result.stderr}")
        return False

    node.get_logger().info(f"Gazebo box spawned successfully:\n{result.stdout.strip()}")
    return True

def pick_object(moveit, arm, display_publisher, display_node, gripper_client,
    object_data, state_update_wait, rviz_display_wait, interactive_mode, gripper_max_effort):
    # Define box pick up
    
    if not command_gripper(
        display_node,
        gripper_client,
        0.000,
        gripper_max_effort,
    ):
        return False

    print("Planning approach pose...")
    if not execute_pose_targets(
        moveit,
        arm,
        display_publisher,
        [object_data["approach_pose"]],
        "PTP",
        state_update_wait,
        rviz_display_wait,
        interactive_mode,
        pose_link = "gripper_base",
    ):
        return False

    allow_box_collision(moveit = moveit, box_name = object_data["box_name"])

    print("Planning grasp pose...")
    if not execute_pose_targets(
        moveit,
        arm,
        display_publisher,
        [object_data["grasp_pose"]],
        "PTP",
        state_update_wait,
        rviz_display_wait,
        interactive_mode,
        pose_link = "gripper_base",
    ):
        return False

    print("Planning gripper close...")
    if not command_gripper(
        display_node,
        gripper_client,
        object_data["gripper_position"],
        gripper_max_effort,
        accept_stall=True,
    ):
        return False

    print("Gripper closed successfully.")

    # Allow joint states and the gripper action to update.
    time.sleep(0.5)

    # Tell MoveIt that the box is now intentionally inside the gripper.
    attach_box_to_robot(
        moveit = moveit,
        box_name = object_data["box_name"],
        box_size = object_data["box_size"],
        box_position = object_data["box_position"],
        robot_link="gripper_base",
    )

    print("Planning lift pose...")
    # Plan and execute the lift.
    if not execute_pose_targets(
        moveit,
        arm,
        display_publisher,
        [object_data["lift_pose"]],
        "PTP",
        state_update_wait,
        rviz_display_wait,
        interactive_mode,
        pose_link = "gripper_base",
    ):
        print("Lift planning failed.")
        return False

    print("Grasp and lift sequence complete.")
    return True

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

    # Box dimensions
    box_name = "pick_box"
    box_sdf_path = (
        Path.home()
        / "ros2_ws"
        / "src"
        / "gp8_ros2"
        / "gp8_gazebo"
        / "models"
        / "box"
        / "box.sdf"
    )
    box_size = (0.02, 0.02, 0.02)
    box_position_world = (0.40, 0.00, 0.01)
    box_height = box_size[2]
    box_bottom_world = box_position_world[2] - 0.5 * box_height

    # Box Transformation
    # since robot spawns at z = 0.05 
    robot_bottom = 0.05
    box_position_base = (box_position_world[0], box_position_world[1], box_position_world[2] - robot_bottom)
    floor_clearance = 0.025
    gripper_box_2cm = 0.025
    gripper_mesh_bot = 0.065
    grasp_2cm_world = box_bottom_world + gripper_mesh_bot + floor_clearance
    grasp_2cm_base = grasp_2cm_world - robot_bottom
    approach_offset = 0.10
    approach_2cm = grasp_2cm_base + approach_offset

    ## GLOBALS ##

    move_mode = move_mode.upper()
    if move_mode not in {"PTP", "LIN"}:
        raise ValueError("Incorrect mode selected...")
    
    ## TEMPORARY VALUES
    # Define target pose (single or multi)
    targets = [
        # {
        #     "position": (0.3055, 0.0966, 0.8535),
        #     "orientation": (0.707, 0.000, 0.707, 0.000),
        # },
        {
            "position": (0.46, 0.000, 0.715),
            "orientation": (0.707107, 0.000, 0.707107, 0.000),
        },
        {
            "position": (0.39265, 0.065584, 0.788641),
            "orientation": (0.637909, 0.261493, 0.700105, 0.185866),
        },
        {
            "position": (0.305469, 0.096623, 0.853482),
            "orientation": (0.497165, 0.434162, 0.643534, 0.387549),
        },
    ]

    box_2cm = {
        "box_name": box_name,
        "box_size": box_size,
        "box_position": box_position_base,
        "approach_pose": {
            "position": (0.40, 0.00, approach_2cm),
            "orientation": (-0.500, 0.500, -0.500, 0.500),
        },
        "grasp_pose": {
            "position": (0.40, 0.00, grasp_2cm_base),
            "orientation": (-0.500, 0.500, -0.500, 0.500),
        },
        "lift_pose": {
            "position": (0.40, 0.00, approach_2cm),
            "orientation": (-0.500, 0.500, -0.500, 0.500),
        },
        "gripper_position": gripper_box_2cm,
    }

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

    # Define CSV inputs/outputs if necessary
    stage_b_input_csv = (
        Path.home()
        / "ros2_ws"
        / "src"
        / "gp8_ros2"
        / "data"
        / "gp8_stageB_50_xyz_targets.csv"
    )
    
    stage_b_output_csv = (
        Path.home()
        / "ros2_ws"
        / "src"
        / "gp8_ros2"
        / "data"
        / "gp8_stageB_results.csv"
    )

    # Load CSV
    csv_targets = load_position_targets(stage_b_input_csv)
    
    print(
        f"Loaded {len(csv_targets)} position targets "
        f"from {stage_b_input_csv}"
    )
    
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
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, display_node)
    
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

        while True:
            print("\n---------- GP8 CONTROL MENU ----------")
            print("z = execute full tool0 pose sequence")
            print("x = execute position-only tool0 sequence")
            print("v = execute position-only targets from CSV")
            print("o = open gripper")
            print("c = close gripper")
            print("s = move to safe position")
            print("r = move to rest position")
            print("b = add box to scene")
            print("p = pick up box")
            print("Enter = exit")
            print("--------------------------------------")

            command = input("Command: ").strip().lower()

            if command == "":
                print("Closing GP8 control script.")
                break

            elif command == "z":
                execute_pose_targets(
                    moveit=moveit,
                    arm=arm,
                    display_publisher=display_publisher,
                    targets=targets,
                    move_mode=move_mode,
                    state_update_wait=state_update_wait,
                    rviz_display_wait=rviz_display_wait,
                    interactive_mode=interactive_mode,
                    pose_link="tool0",
                    position_only=False,
                )

            
            elif command == "x":
                execute_pose_targets(
                    moveit=moveit,
                    arm=arm,
                    display_publisher=display_publisher,
                    targets=targets,
                    move_mode=move_mode,
                    state_update_wait=state_update_wait,
                    rviz_display_wait=rviz_display_wait,
                    interactive_mode=interactive_mode,
                    pose_link="tool0",
                    position_only=True,
                )

            elif command == "v":
                run_csv_position_sequence(
                    moveit=moveit,
                    arm=arm,
                    display_publisher=display_publisher,
                    display_node=display_node,
                    tf_buffer=tf_buffer,
                    csv_targets=csv_targets,
                    home_state=home_state,
                    output_csv_path=stage_b_output_csv,
                    state_update_wait=state_update_wait,
                    waypoint_pause=waypoint_pause,
                    rviz_display_wait=rviz_display_wait,
                    interactive_mode=interactive_mode,
                )

            elif command == "o":
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

            elif command == "s":
                move_to_joint_state(
                    moveit=moveit,
                    arm=arm,
                    display_publisher=display_publisher,
                    joint_positions=safe_state,
                    state_name="safe position",
                    state_update_wait=state_update_wait,
                    waypoint_pause=waypoint_pause,
                )

            elif command == "r":
                move_to_joint_state(
                    moveit=moveit,
                    arm=arm,
                    display_publisher=display_publisher,
                    joint_positions=home_state,
                    state_name="rest position",
                    state_update_wait=state_update_wait,
                    waypoint_pause=waypoint_pause,
                )

            elif command == "b":
                box_spawn = spawn_box_in_gazebo(display_node, box_name, box_sdf_path, box_position_world)
                if box_spawn:
                    add_box_to_planning_scene(moveit, box_name, box_size, box_position_base)
                    print("Box added to Gazebo and Moveit...")
                else:
                    print("Box failed to spawn into Gazebo and was not added to MoveIt...")

            elif command == "p":
                pick_object(
                    moveit=moveit,
                    arm=arm,
                    display_publisher=display_publisher,
                    display_node=display_node,
                    gripper_client=gripper_client,
                    object_data=box_2cm,
                    state_update_wait=state_update_wait,
                    rviz_display_wait=rviz_display_wait,
                    interactive_mode=interactive_mode,
                    gripper_max_effort=gripper_max_effort,
                )

            else:
                print(f"Unknown command: '{command}'")


    finally:
        display_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
