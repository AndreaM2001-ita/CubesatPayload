import os
from glob import glob
from setuptools import setup

package_name = 'satellite_payload'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    include_package_data=True,  # Optional, for non-Python files
    data_files=[
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ['launch/launch_payload.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fusion_node = satellite_payload.fusion_node:main',
            'telemetry_uploader = satellite_payload.telemetry_uploader:main',
            'light_node = satellite_payload.light_node:main',
            'cam_node = satellite_payload.cam_node:main',
            'imu_node = satellite_payload.imu_node:main',
            'env_node = satellite_payload.env_node:main',
            'scheduler_node = satellite_payload.scheduler_node:main'
        ]
    }
)
