from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'ur10_python_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='lhs223@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = ur10_python_interface.publisher_member_function:main',
            'listener = py_pubsub.subscriber_member_function:main',
            'input = ur10_python_interface.input:main',
            'config = ur10_python_interface.config_example:main',
            'mode_manager = ur10_python_interface.mode_manager:main',
        ],
    },
)
