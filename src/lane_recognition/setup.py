from setuptools import setup
import os
from glob import glob

package_name = 'lane_recognition'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', 'lane_recognition' ,'launch'),glob(os.path.join('launch','*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rubu',
    maintainer_email='ra.hourani@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_recognition = lane_recognition.lane_recognition:main',
            'new_lane_recognition=lane_recognition.new_lane_recognition:main',
            'image_pub=lane_recognition.image_pub:main',
            'check=lane_recognition.check:main',
            'dashed=lane_recognition.dashed:main'
        ],
    },
)
