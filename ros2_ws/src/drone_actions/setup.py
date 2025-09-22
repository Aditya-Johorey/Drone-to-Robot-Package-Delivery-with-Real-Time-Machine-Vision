from setuptools import find_packages, setup

package_name = 'drone_actions'

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
    maintainer='ninja',
    maintainer_email='lourenco.sequeira@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_mission_server = drone_actions.drone_mission_server:main',
            'drone_mission_client = drone_actions.drone_mission_client:main'
        ],
    },
)
