from setuptools import setup

package_name = 'tswr_awsim'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        ],
    },
)
