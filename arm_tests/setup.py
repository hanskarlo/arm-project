from setuptools import find_packages, setup

package_name = 'arm_tests'

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
    maintainer='aroarm0',
    maintainer_email='hansjarales@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_group_test = arm_tests.move_group_test:main',
            'move_to_saved = arm_tests.move_to_saved:main',
            'planning_scene_test = arm_tests.planning_scene_test:main',
            'pga_test = arm_tests.pose_goal_array_test:main',
            'save_test = arm_tests.save_test:main',
        ],
    },
)
