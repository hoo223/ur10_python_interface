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
            'talker = example_script.publisher_member_function:main',
            'listener = example_script.subscriber_member_function:main',
            'service = example_script.service_member_function:main',
            'client = example_script.client_member_function:main',
            'config = example_script.config_example:main',
            'list_con = example_script.list_controller_example:main',
            'input = script.input:main',
            'mode_manager = script.mode_manager:main',
        ],
    },
)
