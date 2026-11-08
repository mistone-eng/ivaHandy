from setuptools import setup

package_name = 'handy_api'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Owen Connell',
    maintainer_email='oconnell7@gatech.edu',
    description='Modular Python API for the Handy arm (Servo, MoveIt, gripper, state, transforms)',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'handy_api_selftest = handy_api.selftest:main',
        ],
    },
)
