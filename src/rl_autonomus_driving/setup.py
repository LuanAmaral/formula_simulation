from setuptools import find_packages, setup

package_name = 'rl_autonomus_driving'

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
    maintainer='luan',
    maintainer_email='luan.rocha.amaral@gmail.com',
    description='RL algorithms to develop a driver using EUFS simulator',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rl_autonomous_driving = rl_autonomous_driving.rl_autonomous_driving:main',
        ],
    },
)
