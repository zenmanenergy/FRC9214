from setuptools import find_packages, setup

package_name = 'team9214'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/team9214.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='usv',
    maintainer_email='you@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ros_to_nt = team9214.ros_to_nt:main',
            'nt_to_ros = team9214.nt_to_ros:main',
        ],
    },
)
