from setuptools import setup

package_name = "tello_driver"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Pedro Soares",
    maintainer_email="pmbs.123@gmail.com",
    description="Simple ROS wrapper to control a DJI Tello drone.",
    license="BSD3",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "tello_driver_node = tello_driver.tello_driver_ros_node:main",
        ],
    },
)
