from setuptools import find_packages, setup

package_name = 'rl_py_pid_uros'

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
    maintainer='arvindh_r',
    maintainer_email='arvindh1793@gmail.com',
    description='A reinforcement learning based \
                 PID controller for an encoder motor \
                 using ros2 and microros as middleware',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid_tuner = rl_py_pid_uros.rl_pid:main'
        ],
    },
)
