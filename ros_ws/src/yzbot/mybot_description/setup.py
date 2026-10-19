from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mybot_description'

world_files = []
for folder, _, filenames in os.walk('worlds'):
    if filenames:
        world_files.append((os.path.join('share', package_name, folder),
                            [os.path.join(folder, name) for name in filenames]))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # urdf xacro files
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.*'))),
        # world files
    ] + world_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
