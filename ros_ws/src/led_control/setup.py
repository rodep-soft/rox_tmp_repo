from setuptools import find_packages, setup

package_name = "led_control"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "rclpy", "gpiozero", "board", "neopixel"],
    zip_safe=True,
    maintainer="rodep",
    maintainer_email="yano.tatsuki439@mail.kyutech.jp",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "led_control_node = led_control.led_control:main",
        ],
    },
)
