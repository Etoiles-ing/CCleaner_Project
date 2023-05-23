from setuptools import setup

package_name = 'torturebot4_navigation'

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
    maintainer='Harlock',
    maintainer_email='nathan.rassie@utbm.fr',
    description='Navigation Stack for Torturebot4',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'torturebot4_map_navigation = torturebot4_navigation.torturebot4_map_navigation:main'
        ],
    },
)
