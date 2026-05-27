from setuptools import find_packages, setup

package_name = 'controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/controller_final_pose.launch.py']),
        ('share/' + package_name + '/config', ['config/moveit_py_config.yaml']),
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
            'node_controller_final_pose = controller.node_controller_final_pose:main',
            'node_controller_buffer = controller.node_controller_buffer:main',
            'node_goto_initial_pose = controller.node_goto_initial_pose:main',
            'node_move_to_point = controller.node_move_to_point:main',
            'node_state_machine = controller.node_state_machine:main',
        ],
    },
)
