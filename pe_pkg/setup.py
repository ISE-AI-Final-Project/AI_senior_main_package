from setuptools import find_packages, setup

package_name = 'pe_pkg'

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
    maintainer='icynunnymumu',
    maintainer_email='icynunnymumu@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_server = pe_pkg.pose_server:main',
            'pose_client = pe_pkg.pose_client:main',
        ],
    },
)
