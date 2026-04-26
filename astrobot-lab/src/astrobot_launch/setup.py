from glob import glob
from setuptools import find_packages, setup


package_name = "astrobot_launch"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/behavior_trees", glob("behavior_trees/*.xml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jacob Lambert",
    maintainer_email="jacob@example.com",
    description="Launch and configuration package for Astrobot Phase 1 bring-up.",
    license="Apache-2.0",
)
