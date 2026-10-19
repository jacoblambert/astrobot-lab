from setuptools import find_packages, setup


package_name = "synthetic_rtk"


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
    description="Synthetic RTK / surface-beacon absolute-position source for the lunar rover.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "synthetic_rtk = synthetic_rtk.synthetic_rtk_node:main",
        ],
    },
)
