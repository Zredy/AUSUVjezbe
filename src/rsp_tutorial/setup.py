import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'rsp_tutorial'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch','*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name), glob('urdf/*')),
        (os.path.join('share', package_name), glob('resource/meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ascuric',
    maintainer_email='as239650@stud.fsb.hr',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_publisher = rsp_tutorial.state_publisher:main',
            'iiwa14_publisher = rsp_tutorial.iiwa14_publisher:main',
        ],
    },
)
