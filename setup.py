from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'reto_m6'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name), glob('urdf/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Miguel',
    maintainer_email='miguel.chumacero.b@uni.pe',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = reto_m6.reto_topic_pub:main',
            'listener = reto_m6.reto_topic_sub:main',
            'pointcloud2 = reto_m6.reto_pointcloud2_pub:main',
            'figures_pub = reto_m6.reto_figures_pub:main',
            'figures_sub = reto_m6.reto_figures_sub:main',
            'filter_pub = reto_m6.reto_filter_pub:main',
            'filter_sub = reto_m6.reto_filter_sub:main',
        ],
    },
)
