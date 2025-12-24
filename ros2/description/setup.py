from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'description'

def package_files(directory):
    """Recursively collect files for installation preserving subfolders."""
    paths = []
    for (path, _, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(path, filename))
    return paths

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "urdf"), package_files("urdf")),
        (os.path.join("share", package_name, "meshes"), package_files("meshes")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
