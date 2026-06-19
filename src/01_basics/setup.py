from setuptools import find_packages, setup

setup(
    name='basics',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/basics']),
        ('share/basics', ['package.xml']),
        ('share/basics/launch', ['launch/pub_sub.launch.py']),
    ],
    install_requires=['setuptools'],
    maintainer='Daeun Song',
    maintainer_email='songd@ewha.ac.kr',
    entry_points={
        'console_scripts': [
            'publisher = basics.publisher:main',
            'subscriber = basics.subscriber:main',
        ],
    },
)