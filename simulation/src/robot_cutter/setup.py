from setuptools import find_packages, setup

package_name = 'robot_cutter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/rviz.launch.py']),
        ('share/' + package_name + '/launch', ['launch/gazebo.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/robot_cutter.urdf.xacro']),
        ('share/' + package_name + '/urdf', ['urdf/gazebo.urdf.xacro']),
        ('share/' + package_name + '/config', ['config/initial_positions.yaml', 'config/controllers.yaml', 'config/ros2_controllers.yaml', 'config/joint_limits.yaml', 'config/moveit_controllers.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rui',
    maintainer_email='ruimartins203@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
