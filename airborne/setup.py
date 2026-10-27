from glob import glob
from setuptools import find_packages, setup

setup(
    name="airborne_telemetry",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/airborne_telemetry"]),
        ("share/airborne_telemetry", ["package.xml"]),
        ("share/airborne_telemetry/launch", glob("launch/*.launch.py")),
        ("share/airborne_telemetry/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "pymavlink>=2.4.41", "pyserial>=3.5"],
    zip_safe=True,
    maintainer="Telemetry Communication maintainers",
    maintainer_email="maintainers@example.com",
    description="Pixhawk MAVLink radio bridge and ROS 2 GPS telemetry",
    license="TODO",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["airborne_bridge = airborne_telemetry.node:main"]},
)
