from setuptools import find_packages, setup

package_name = 'pub_sub_tutorial'

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
    maintainer='sagar',
    maintainer_email='sagar16812@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'led_publisher = pub_sub_tutorial.led_publisher:main',
            'led_subscriber = pub_sub_tutorial.led_subscriber:main',
        ],
    },
)
