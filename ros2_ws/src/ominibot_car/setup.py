from setuptools import setup

package_name = 'ominibot_car'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Wei-Chih Lin',
    maintainer_email='weichih.lin@prontonmail.com',
    description='Connect ominibot carV1.2, communication protocol(https://github.com/CIRCUSPi/OminiBotHV/blob/master/communication/ominibo_car_V008_1225.pdf)',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f"ominibot_car_driver = {package_name}.ominibot_car_driver:main",
        ],
    },
)
