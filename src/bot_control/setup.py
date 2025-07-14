from setuptools import setup

package_name = 'bot_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/obstacle_stop.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tarun',
    maintainer_email='cs24b2028@iiitdm.ac.in',
    description='LIDAR obstacle stopper',
    entry_points={
        'console_scripts': [
            'obstacle_stop = bot_control.obstacle_stop:main',
        ],
    },
)

