from setuptools import find_packages, setup


package_name = "astrobot_diagnostics"


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
    description="Reusable runtime diagnostics for Astrobot autonomy bring-up.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "diagnostics_node = astrobot_diagnostics.diagnostics_node:main",
        ],
    },
)
