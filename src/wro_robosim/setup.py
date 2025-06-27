from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'wro_robosim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'description', 'wro_car'), glob('description/wro_car/*')),
        (os.path.join('share', package_name, 'description', 'wro_obstacle_world'), glob('description/wro_obstacle_world/*')),
        (os.path.join('share', package_name, 'description', 'wro_open_world'), glob('description/wro_open_world/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eneji',
    maintainer_email='enejiohieku@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
