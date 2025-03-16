from setuptools import find_packages, setup

package_name = 'turtlebot_launcher'

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
    maintainer='Chong Kai Jie',
    maintainer_email='kaijie.chong@gmail.com',
    description='Service for running flywheel based ball launcher on Turtlebot 3. NUS CDE2310 AY24/25',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
