from setuptools import setup

package_name = 'my_robot_description'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='URDF description of a custom mobile robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'speed = my_robot_description.speed:main',
        ],
    },
)
