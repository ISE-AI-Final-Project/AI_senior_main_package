from setuptools import find_packages, setup

package_name = 'best_grasp_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    # install_requires=['setuptools'],
    # install_requires=['setuptools', 'rclpy', 'tf2_msgs'],
    # install_requires=['setuptools', 'rclpy', 'tf2_ros', 'geometry_msgs', 'tf2_geometry_msgs',],
    install_requires=[
    'setuptools',
    'rclpy',
    'geometry_msgs',
    'tf2_ros',
    'numpy',
    'scipy',
],



    zip_safe=True,
    maintainer='icynunnymumu',
    maintainer_email='nunun_cute@hotmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # "sub_tf = best_grasp_pkg.sub_tf:main",
            'pose_transform = best_grasp_pkg.pose_transform:main',
            'best_grasp_server = best_grasp_pkg.best_grasp_server:main',
        ],
    },
)
