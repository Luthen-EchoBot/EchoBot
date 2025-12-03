from setuptools import setup

package_name = 'pkg_movement_sequence'

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
    maintainer='ut',
    maintainer_email='teuliere@insa-toulouse.fr',
    description='TODO: Package description',
    license='Licence Apache 2.0',
    entry_points={
        'console_scripts': [
		'movement_sequence_publisher = pkg_movement_sequence.movement_sequence:main',
        ],
    },
)
