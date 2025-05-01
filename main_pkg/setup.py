from setuptools import find_packages, setup

package_name = "main_pkg"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ('share/main_pkg/launch', ['launch/main.launch.py']),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="icynunnymumu",
    maintainer_email="icynunnymumu@todo.todo",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "main = main_pkg.main:main",
            "dataset_capture = main_pkg.dataset_capture:main",
            "custom_joint_state_publisher_gui = main_pkg.custom_joint_state_publisher_gui:main",
            "dataset_fusing_capture = main_pkg.dataset_fusing_capture:main",
        ],
    },
)
