from setuptools import find_packages, setup

package_name = 'my_zed_pkg'

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
            "zed_sub = my_zed_pkg.zed_sub:main",
            "zed_client_test = my_zed_pkg.zed_client_test:main"
        ],
    },
)
