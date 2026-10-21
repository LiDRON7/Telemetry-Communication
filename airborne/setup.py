from glob import glob

from setuptools import find_packages, setup


setup(
    name="airborne",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/airborne"]),
        ("share/airborne", ["package.xml"]),
        ("share/airborne/launch", glob("launch/*.launch.py")),
        ("share/airborne/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "pymavlink>=2.4.41", "pyserial>=3.5"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="LiDRON",
    maintainer_email="maintainers@example.com",
    description="ROS 2 telemetry interface and MAVLink radio bridge",
    license="TODO",
    entry_points={"console_scripts": ["airborne_bridge = airborne.bridge:main"]},
)
