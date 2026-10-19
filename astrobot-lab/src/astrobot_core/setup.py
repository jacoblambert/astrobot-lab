from setuptools import find_packages, setup

setup(
    name="astrobot_core", version="0.1.0", packages=find_packages(exclude=["test"]),
    data_files=[("share/ament_index/resource_index/packages", ["resource/astrobot_core"]),
                ("share/astrobot_core", ["package.xml"])],
    install_requires=["setuptools"], tests_require=["pytest"],
    maintainer="Jacob Lambert", maintainer_email="jacob@example.com", license="Apache-2.0",
)
