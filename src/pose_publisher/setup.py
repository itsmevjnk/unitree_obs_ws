from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'pose_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hotdog',
    maintainer_email='hotdog@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pub_node = pose_publisher.pose_publisher:main',
            'pcl_node = pose_publisher.pose_to_pcl:main'
            'tf_node = pose_publisher.pose_to_tf:main'
        ],
    },
)
