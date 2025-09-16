from setuptools import find_packages, setup

package_name = 'ros2_k3s_cli'

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
    maintainer='asus',
    maintainer_email='sachinkum123567@gmail.com',
    description='ROS2 K3S CLI for cluster management',
    license='Apache 2.0',
    tests_require=['pytest'],
    test_suite='pytest',
    entry_points={
        'console_scripts': [
            'cli = ros2_k3s_cli.cli:main',
        ],
    },
)
