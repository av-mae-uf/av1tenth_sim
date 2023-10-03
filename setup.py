from glob import glob
from setuptools import find_packages, setup

package_name = 'av1tenth_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/models', glob('models/*.STL'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='patrick',
    maintainer_email='patrick@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulator=av1tenth_sim.simulator_node:main',
            'visualizer=av1tenth_sim.visualizer_node:main'
        ],
    },
)
