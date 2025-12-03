from setuptools import setup

package_name = 'pkg_ultrasonic_detection'

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
    maintainer='greg',
    maintainer_email='greg.drenou@gmail.com',
    description='Ultrasonic detection package',
    license='License Apache-2.0',
    entry_points={
        'console_scripts': [
            'ultrasonic_listener = pkg_ultrasonic_detection.ultrasonic:main',
        ],
    },
)

