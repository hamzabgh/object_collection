from setuptools import setup
from glob import glob


package_name = 'object_collection_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name,             ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gazebo_launch.py']),
        ('share/' + package_name + '/urdf',   ['urdf/drone.urdf', 'urdf/box.urdf','urdf/model.sdf']),
        ('share/' + package_name + '/config', ['config/controller.yaml']),
        ('share/' + package_name + '/meshes', glob('meshes/*')),


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gemini',
    maintainer_email='gemini@todo.todo',
    description='ROS 2 package for object collection using computer vision',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_collection_node = object_collection_py.object_collection_node:main'
        ],
    },
)