from setuptools import find_packages, setup

package_name = 'pc_util_monitor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kamiko',
    maintainer_email='s24c1050qg@s.chibakoudai.jp',
    description='a package for practice',
    license='BSD-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pc_util_monitor = pc_util_monitor.node:main',
        ],
    },
)
