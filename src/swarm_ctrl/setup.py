from setuptools import setup
package_name = 'swarm_ctrl'
setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Hybrid Python swarm controller for PX4 + MAVROS2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'swarm_controller = swarm_ctrl.swarm_controller:main',
            'monitor = swarm_ctrl.monitor:main',
            'uwb_stub = swarm_ctrl.uwb_stub:main',
        ],
    },
)
