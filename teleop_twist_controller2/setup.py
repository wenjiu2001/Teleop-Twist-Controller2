from setuptools import setup

package_name = 'teleop_twist_controller2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    author='Wenjiu',
    author_email='tp6u4jo6@gmail.com',
    maintainer='Wenjiu',
    maintainer_email='tp6u4jo6@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='A ROS 2 package for teleoperation using a joystick with Pygame',
    license='CC BY-NC-SA 4.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_twist_controller = teleop_twist_controller.teleop_twist_controller:main'
        ],
    },
)
