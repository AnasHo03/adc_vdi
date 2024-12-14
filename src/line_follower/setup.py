from setuptools import setup

package_name = 'line_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'line_follower = line_follower.line_follower:main',
            'parallel_parking_aus = line_follower.parallel_parking_aus:main',
            'teleop = line_follower.teleop:main',
            'cross_parking_aus = line_follower.cross_parking_aus:main',
            'static_overtaking = line_follower.static_overtaking:main',
            'parking2 = line_follower.parking2:main',
            'ausparkenm = line_follower.ausparkenm:main',
            'cross_parking = line_follower.cross_parking:main',
            'parallel_parking = line_follower.parallel_parking:main',
            'follower = line_follower.follower:main',
            'self_aligning=line_follower.self_aligning:main',
            'lidar=line_follower.lidar:main'




        ],
    },
)
