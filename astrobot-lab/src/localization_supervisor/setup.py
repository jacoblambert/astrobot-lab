from setuptools import setup, find_packages
setup(name="localization_supervisor", version="0.1.0", packages=find_packages(exclude=["test"]),
      data_files=[("share/ament_index/resource_index/packages", ["resource/localization_supervisor"]),
                  ("share/localization_supervisor", ["package.xml"])],
      install_requires=["setuptools"], license="Apache-2.0", maintainer="Jacob Lambert",
      maintainer_email="jacob@example.com",
      entry_points={"console_scripts": ["localization_supervisor = localization_supervisor.node:main"]})
