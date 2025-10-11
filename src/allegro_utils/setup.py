from setuptools import find_packages, setup
import os
import glob

package_name = 'allegro_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name,
            ['package.xml']),
        #(os.path.join('share', package_name, 'description', 'urdf'),
        #    [glob(os.path.join(package_name, 'description', 'urdf', '*'))]),
        #(os.path.join('share', package_name,  'description', 'urdf', 'mesh'), glob(os.path.join(package_name, 'description', 'urdf', 'mesh', '*'))),
    ],
    install_requires=['setuptools',
        'numpy',
        'scipy',
        'pinocchio',   # this installs Pinocchio
        'PyQt5', # for your GUI
    ],
    zip_safe=True,
    maintainer='leo',
    maintainer_email='leonardo.marcello_99@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'moveit_joint_broadcaster = allegro_utils.moveit_joint_broadcaster:main',
            'wave = allegro_utils.wave:main',
            'gui = allegro_utils.gui:main',
            # add here other executables
        ],
    },
)
