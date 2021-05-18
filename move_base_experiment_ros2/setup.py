from setuptools import setup

PACKAGE_NAME = 'move_base_experiment'

setup(
    name=PACKAGE_NAME,
    version='0.3.0',
    packages=[PACKAGE_NAME],
    data_files=[
        ('share/' + PACKAGE_NAME + '/launch/*.launch.py')
    ],
    install_requires=['setuptools'],
    zip_safe=True
)