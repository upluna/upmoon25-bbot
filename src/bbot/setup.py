from setuptools import find_packages, setup

package_name = 'bbot'

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
    maintainer='Max',
    maintainer_email='woodsm25@up.edu',
    description='Nodes for remote controlling the baby bot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
			'bbot_driver_kb = bbot.keyboard_driver:main',
            'bbot_driver_js = bbot.joystick_driver:main'
        ],
    },
)
