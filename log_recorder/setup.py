from setuptools import find_packages, setup

package_name = 'log_recorder'

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
    maintainer='curtis',
    maintainer_email='curtisburke.eng@gmail.com',
    description='A wrapper around ros2 bag record API.',
    license='MIT-License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'log_recorder = log_recorder.log_recorder:main'
        ],
    },
)
