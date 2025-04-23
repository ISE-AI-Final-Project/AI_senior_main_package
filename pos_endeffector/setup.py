from setuptools import find_packages, setup

package_name = 'pos_endeffector'

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
    maintainer='tunyaluck',
    maintainer_email='85739324+Tunyalucklie@users.noreply.github.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['endeffector_pubsub = pos_endeffector.pos_endeffector_pub:main',
                            'joint_pos = pos_endeffector.joint_pos:main',
        ],
    },
)