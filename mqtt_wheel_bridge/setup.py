from setuptools import find_packages, setup

package_name = 'mqtt_wheel_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/controls.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ryank',
    maintainer_email='rjkaddis@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'mqtt_wheel_bridge = {package_name}.mqtt_wheel_bridge:main'
        ],
    },
)
