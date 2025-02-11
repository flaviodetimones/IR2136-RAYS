from setuptools import find_packages, setup

package_name = 'tarea3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'
    'pymavlink'
    'rclpy'],
    zip_safe=True,
    maintainer='usuario',
    maintainer_email='mgarciablasco26@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
    'console_scripts': [
     	'battery_gps_node = tarea3.battery_gps_node:main',
        'mission_control_node = tarea3.mission_control_node:main',
    	],
    },

)
