from setuptools import find_packages, setup
from glob import glob
package_name = 'projektDM'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
     data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/robotModel', glob('robotModel/*')),
        ('share/' + package_name + '/launch', glob('launch/*launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='dominik.stawarczyk@student.put.poznan.pl',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'simulate_joints= projektDM.simulate_joints:main',
	    'workspace = projektDM.workspace:main'	
        ],
    },
)
