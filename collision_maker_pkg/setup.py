from setuptools import find_packages, setup

package_name = 'collision_maker_pkg'

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
    maintainer_email='nunun_cute@hotmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'collision_server = collision_maker_pkg.collision_server:main',
            'collision_client = collision_maker_pkg.collision_client:main',
            'collision_server_with_mask = collision_maker_pkg.collision_server_with_mask:main',
        ],
    },
)
