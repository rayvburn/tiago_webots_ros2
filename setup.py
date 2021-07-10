"""webots_ros2 package setup file."""

from glob import glob
from setuptools import setup


package_name = 'tiago_webots_ros2'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, [
    'launch/tiago_webots.launch.py',
    'launch/tiago_mapping.launch.py',
    'launch/cartographer.launch.py',
    'launch/occupancy_grid.launch.py'
]))
data_files.append(('share/' + package_name + '/resource', [
    'resource/map/intralogistics.pgm',
    'resource/map/intralogistics.yaml'
]))
data_files.append(('share/' + package_name + '/worlds/textures', glob('worlds/textures/*')))
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/intralogistics.wbt',
    'worlds/.intralogistics.wbproj',
    'worlds/retirement_home.wbt',
    'worlds/.retirement_home.wbproj',
]))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.9.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    author='Jarosław Karwowski',
    author_email='jaroslaw.karwowski.dokt@pw.edu.pl',
    maintainer='Jarosław Karwowski',
    maintainer_email='jaroslaw.karwowski.dokt@pw.edu.pl',
    keywords=['ROS', 'Webots', 'Robot', 'Simulation', 'Examples'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Tiago Iron robot ROS2 interface for Webots.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tiago_driver = tiago_webots_ros2.tiago_driver:main'
        ],
        'launch.frontend.launch_extension': ['launch_ros = launch_ros']
    }
)
