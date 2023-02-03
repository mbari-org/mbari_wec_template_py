from glob import glob
import os

from setuptools import setup


package_name = 'mbari_wec_template_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[f'{package_name}'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anderson',
    maintainer_email='anderson@mbari.org',
    description='MBARI Power Buoy Controller Template (python)',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'controller = {package_name}.controller:main',
        ],
    },
)
