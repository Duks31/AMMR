import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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

    use_sim_time = LaunchConfiguration("use_sim_time")

    # ── Resolve paths at launch time (Python, not substitution — intentional) ──
    # Model files are installed into the package share directory via CMakeLists.
    # We resolve them here so nodes receive absolute paths as parameters.
    detector_model = os.path.join(
        pkg, "models", "detector", "kaggle_run_50_epochs.pt"
    )
    classifier_model = os.path.join(
        pkg, "models", "classifier", "efficientnetb0_finetuned.pth"
    )
    config_sim  = os.path.join(pkg, "config", "perception_sim.yaml")
    config_hw   = os.path.join(pkg, "config", "perception_hardware.yaml")

    # ── Detector node ──────────────────────────────────────────────────────────
    detector_node = Node(
        package="cika_perception",
        executable="detector_node.py",
        name="detector_node",
        output="screen",
        parameters=[
            config_sim,                          # base config
            {
                "model_path": detector_model,    # override with resolved path
                "use_sim_time": use_sim_time,
            },
        ],
    )

    # ── Classifier node ────────────────────────────────────────────────────────
    classifier_node = Node(
        package="cika_perception",
        executable="classifier_node.py",
        name="classifier_node",
        output="screen",
        parameters=[
            config_sim,
            {
                "model_path": classifier_model,
                "use_sim_time": use_sim_time,
            },
        ],
    )

    return LaunchDescription([
        backend_arg,
        use_sim_time_arg,
        detector_node,
        classifier_node,
    ])