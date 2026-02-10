from setuptools import find_packages, setup

package_name = 'trackdlo_utils'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='todo@todo.com',
    description='TrackDLO2: Utility scripts for testing, evaluation, and visualization',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulate_occlusion = trackdlo_utils.simulate_occlusion:main',
            'simulate_occlusion_eval = trackdlo_utils.simulate_occlusion_eval:main',
            'tracking_test = trackdlo_utils.tracking_test:main',
            'collect_pointcloud = trackdlo_utils.collect_pointcloud:main',
            'mask_node = trackdlo_utils.mask:main',
            'tracking_result_img = trackdlo_utils.tracking_result_img_from_pointcloud_topic:main',
            'depth_format_converter = trackdlo_utils.depth_format_converter:main',
        ],
    },
)
