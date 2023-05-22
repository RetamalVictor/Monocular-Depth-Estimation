from glob import glob
from setuptools import setup

package_name = 'depth_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        package_name + '.midas_node',
        package_name + '.transform_pipeline'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='victor',
    maintainer_email='v.retamalguiberteau@student.vu.nl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'midas_node = depth_pkg.midas_node:main',
        ],
        'ros2cli.command': [
            'midas_node = depth_pkg.midas_node:main',
        ],
        'rclpy_components': [
            'depth_pkg.midas_node = depth_pkg.midas_node:MiDaSNode',
        ],
    },
)
