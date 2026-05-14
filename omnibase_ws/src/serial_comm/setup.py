from setuptools import find_packages, setup

package_name = 'serial_comm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={
        package_name: ['dashboard.html'],
    },
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roger',
    maintainer_email='joserogelioruiz12@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'serial_communication = serial_comm.serial_communication:main',
            'pedro_dashboard = serial_comm.pedro_dashboard:main',
            'simple_rx = serial_comm.simple_rx:main',
        ],
    },
)
