from setuptools import find_packages, setup

package_name = "lifting_motor"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    py_modules=['lifting_motor','state_machine', 'motor_driver'], 
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rodep",
    maintainer_email="yano.tatsuki439@mail.kyutech.jp",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["lifting_motor = lifting_motor.lifting_motor:main"],
    },
)
