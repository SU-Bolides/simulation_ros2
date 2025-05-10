from setuptools import find_packages, setup

package_name = 'rviz_simulation'
data_files = []

data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/rviz.launch.py']))
data_files.append(('share/' + package_name + '/urdf', ['urdf/voiture.xacro']))
data_files.append((
    'share/' + package_name + '/urdf/meshes',
    [
        'urdf/meshes/ChevroletCamaroLight.stl',
        'urdf/meshes/left_wheel.stl',
        'urdf/meshes/right_wheel.stl',
        'urdf/meshes/rplidar.stl',
        'urdf/meshes/hinge.stl',
    ]
))
data_files.append((
    'share/' + package_name + '/config',
    ['config/bolide_viewer.rviz']
))

data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='imad',
    maintainer_email='imadeddine.ghomari@yahoo.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'webot_driver = voiture_simulation.webot_driver:main',
        ],
},
)
