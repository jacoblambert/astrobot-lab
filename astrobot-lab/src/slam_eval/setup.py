from setuptools import find_packages, setup


package_name = "slam_eval"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jacob Lambert",
    maintainer_email="jacob@example.com",
    description="Phase 2 SLAM evaluation tools.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "compare_tum_trajectories = slam_eval.compare_tum_trajectories:main",
            "compare_bev_maps = slam_eval.compare_bev_maps:main",
            "compare_bev_accumulated = slam_eval.compare_bev_accumulated:main",
            "compare_bev_registered_clouds = slam_eval.compare_bev_registered_clouds:main",
            "compare_occupancy_maps = slam_eval.compare_occupancy_maps:main",
            "evaluate_costmap_reachability = slam_eval.evaluate_costmap_reachability:main",
            "analyze_pointcloud_height = slam_eval.analyze_pointcloud_height:main",
            "analyze_slam_error = slam_eval.analyze_slam_error:main",
            "pose_stamped_recorder = slam_eval.pose_stamped_recorder:main",
            "transform_tum = slam_eval.transform_tum:main",
            "evaluate_loop_closure = slam_eval.evaluate_loop_closure:main",
        ],
    },
)
