from setuptools import find_packages, setup

package_name = 'kinematic_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='icetenny',
    maintainer_email='ice.tennison@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'endeffector_pubsub = kinematic_pkg.pos_endeffector_pub:main',
            'joint_pos = kinematic_pkg.joint_pos:main',
            'ik = kinematic_pkg.ik:main',
            'ik_joint_pub = kinematic_pkg.ik_joint_pub:main',
            'ik_server = kinematic_pkg.ik_server:main',
        ],
    },
)
