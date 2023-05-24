from setuptools import setup

package_name = 'turtlebot4_pid'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marmol',
    maintainer_email='thomas.marmol@utbm.fr',
    description='PID control for the turtlebot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'PID = turtlebot4_pid.turtlebot4_pid:main'
        ],
    },
)
