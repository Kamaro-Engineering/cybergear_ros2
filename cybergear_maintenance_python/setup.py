from setuptools import find_packages, setup

package_name = 'cybergear_maintenance_python'

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
    maintainer='jonas',
    maintainer_email='joecarverde@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'send_config_message = cybergear_maintenance_python.send_config_message:change_id',
            'set_ki_gain = cybergear_maintenance_python.send_config_message:set_ki',
            'read_parameter = cybergear_maintenance_python.send_config_message:read_parameter',
            'service = cybergear_maintenance_python.zero_motor_service:main'
        ],
    },
)
