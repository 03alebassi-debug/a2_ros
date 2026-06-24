from setuptools import find_packages, setup

package_name = 'artifact_logger'

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
    maintainer='team7',
    maintainer_email='gizemfiliz@sabanciuniv.edu',
    description='Logs detected artifacts to a CSV file in the map frame.',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'artifact_logger_node = artifact_logger.artifact_logger_node:main'
        ],
    },
)
