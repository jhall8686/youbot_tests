from setuptools import find_packages, setup

package_name = 'youbot_name_printer'

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
    maintainer='jackson.hall',
    maintainer_email='jackson.hall@du.edu',
    description='Write your name with the Kuka YouBot',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ik_writer = youbot_name_printer.ik_writer:main',
            'trajectory_to_joint_states = youbot_name_printer.trajectory_to_joint_states:main'
        ],
    },
)
