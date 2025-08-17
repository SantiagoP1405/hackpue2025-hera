from setuptools import find_packages, setup

package_name = 'hera_robot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='santiago_gomez',
    maintainer_email='santiago_gomez@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hera_nav2 = hera_robot_navigation.hera_nav2:main',
            'patrol_voice_detector = hera_robot_navigation.patrol_voice_detector:main',
            'set_initial_pose = hera_robot_navigation.set_initial_pose:main',
        ],
    },
)
