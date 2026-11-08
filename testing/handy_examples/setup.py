from setuptools import setup

package_name = 'handy_examples'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Owen Connell',
    maintainer_email='oconnell7@gatech.edu',
    description='Example MoveIt Servo clients for the Handy arm',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'servo_demo = handy_examples.servo_demo:main',
            'servo_jog = handy_examples.servo_jog:main',
            'servo_wave = handy_examples.servo_wave:main',
        ],
    },
)
