from setuptools import setup

package_name = 'warmup_project'

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
    maintainer='seungu',
    maintainer_email='seungu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wall_follower = warmup_project.wall_follower:main',
            'teleop = warmup_project.teleop:main',
            'drive_square = warmup_project.drive_square:main',
            'person_follower = warmup_project.person_follower:main'
        ],
    },
)
