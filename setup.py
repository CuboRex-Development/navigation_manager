from setuptools import setup
import os
from glob import glob

package_name = 'navigation_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name), glob('waypoints/*.csv')),
        (os.path.join('share', package_name), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yuta',
    maintainer_email='yutanakamura01@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_manager = navigation_manager.navigation_manager:main',
            'waypoint_saver = navigation_manager.waypoint_saver:main',
            'waypoint_saver_for_map = navigation_manager.waypoint_saver_for_map:main',
            'waypoint_viewer = navigation_manager.waypoint_viewer:main',
            'param_test = navigation_manager.param_test:main',
            'gps_on = navigation_manager.gps_on_amcl_off:main',
            'amcl_on = navigation_manager.amcl_on_gps_off:main'
        ],
    },
)
