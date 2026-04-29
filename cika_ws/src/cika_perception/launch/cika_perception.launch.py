import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory("cika_perception")

    # ── Arguments ─────────────────────────────────────────────────────────────
    backend_arg = DeclareLaunchArgument(
        name="backend",
        default_value="sim",
        choices=["sim", "hardware"],
        description="sim = .pt on CPU/GPU  |  hardware = .blob on OAK-D VPU",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Use simulation clock",
    )

    backend      = LaunchConfiguration("backend")
    use_sim_time = LaunchConfiguration("use_sim_time")

    is_sim      = PythonExpression(["'", backend, "' == 'sim'"])
    is_hardware = PythonExpression(["'", backend, "' == 'hardware'"])

    # ── Model paths ────────────────────────────────────────────────────────────
    detector_model   = os.path.join(pkg, "models", "detector",   "kaggle_run_100_epochs.onnx")
    classifier_model = os.path.join(pkg, "models", "classifier", "efficientnetb0.onnx")

    config_sim = os.path.join(pkg, "config", "perception_sim.yaml")
    config_hw  = os.path.join(pkg, "config", "perception_hardware.yaml")

    # ── Detector node — SIM ────────────────────────────────────────────────────
    detector_sim = Node(
        condition=IfCondition(is_sim),
        package="cika_perception",
        executable="detector_node.py",
        name="detector_node",
        output="screen",
        parameters=[
            config_sim,
            {"model_path": detector_model, "use_sim_time": use_sim_time},
        ],
    )

    # ── Detector node — HARDWARE ───────────────────────────────────────────────
    detector_hw = Node(
        condition=IfCondition(is_hardware),
        package="cika_perception",
        executable="detector_node.py",
        name="detector_node",
        output="screen",
        parameters=[
            config_hw,
            {"model_path": detector_model, "use_sim_time": use_sim_time},
        ],
    )

    # ── Classifier node — SIM ──────────────────────────────────────────────────
    classifier_sim = Node(
        condition=IfCondition(is_sim),
        package="cika_perception",
        executable="classifier_node.py",
        name="classifier_node",
        output="screen",
        parameters=[
            config_sim,
            {"model_path": classifier_model, "use_sim_time": use_sim_time},
        ],
    )

    # ── Classifier node — HARDWARE ─────────────────────────────────────────────
    classifier_hw = Node(
        condition=IfCondition(is_hardware),
        package="cika_perception",
        executable="classifier_node.py",
        name="classifier_node",
        output="screen",
        parameters=[
            config_hw,
            {"model_path": classifier_model, "use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription([
        backend_arg,
        use_sim_time_arg,
        detector_sim,
        detector_hw,
        classifier_sim,
        classifier_hw,
    ])