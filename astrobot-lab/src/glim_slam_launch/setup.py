from glob import glob
from setuptools import find_packages, setup


package_name = "glim_slam_launch"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config/glim_astrobot", glob("config/glim_astrobot/*.json")),
        (
            f"share/{package_name}/config/glim_astrobot_lidar_only",
            glob("config/glim_astrobot_lidar_only/*.json"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jacob Lambert",
    maintainer_email="jacob@example.com",
    description="GLIM SLAM launch and configuration for Astrobot.",
    license="Apache-2.0",
)
