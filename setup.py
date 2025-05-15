from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'smach_test_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
    (os.path.join('share', package_name), ['package.xml']),
    (os.path.join('share', package_name, 'config'), glob('src/smach_test_pkg/config/*.yaml')),
],
    install_requires=[
        'setuptools',
        'rclpy',
        'smach',
        'std_msgs',
        'geometry_msgs',
    ],
    zip_safe=True,
    maintainer='csrobot',
    maintainer_email='flynn.m.brian@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_state_machine = smach_test_pkg.state_machines.simple_state_machine:main',
            'pick_and_place = smach_test_pkg.state_machines.pick_and_place:main',
        ],
    },
)
