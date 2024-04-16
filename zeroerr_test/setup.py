from setuptools import find_packages, setup

package_name = 'zeroerr_test'

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
    maintainer_email='hjarales@arosystems.ai',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_group_test = zeroerr_test.move_group_test:main',
            'save_test = zeroerr_test.save_test:main',
        ],
    },
)
