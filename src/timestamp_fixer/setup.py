from setuptools import find_packages, setup

package_name = 'timestamp_fixer'

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
    maintainer='itsmevjnk',
    maintainer_email='ngtv0404@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scan_fixer = timestamp_fixer.scan_fixer:main',
            'imu_fixer = timestamp_fixer.imu_fixer:main',
            'odom_fixer = timestamp_fixer.odom_fixer:main',
        ],
    },
)