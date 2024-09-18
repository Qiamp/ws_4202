from setuptools import setup

package_name = 'turtle_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jay',
    maintainer_email='21099345d@connect.polyu.hk',
    description='Turtle control and pose show nodes',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'turtle_control_node = turtle_control.turtle_control_node:main',
            'turtle_pose_show_node = turtle_control.turtle_pose_show_node:main',
        ],
    },
)
