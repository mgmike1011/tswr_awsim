from setuptools import setup
import os
from glob import glob
package_name = 'tswr_awsim'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['resource/path.csv']),
        (os.path.join('share', package_name), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AMMM',
    maintainer_email='ammm@pp.com',
    description='TSwR Project - AWSIM',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_script = tswr_awsim.test_script:main',
            'lin_MPC_controller = tswr_awsim.lin_MPC_controller:main',
            'iLQR_controller = tswr_awsim.iLQR_controller:main',
            'stanley_controller = tswr_awsim.stanley_controller:main',
            'pure_pursuit_controller = tswr_awsim.pure_pursuit_controller:main',
            'reference_trajectory_generator = tswr_awsim.reference_trajectory:main',
            'path_publisher = tswr_awsim.path_pub:main',
        ],
    },
)
