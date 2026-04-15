from setuptools import find_packages, setup

package_name = 'branch_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', ['models/yolov8n.pt', 'models/yolo_tree_detection.pt']),
    ],
    install_requires=['setuptools', 'ultralytics'],
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
            'node_detect_branch = branch_detection.node_detect_branch:main',
            'service_compute_world_coordinates = branch_detection.service_compute_world_position:main'
        ],
    },
)
