from setuptools import find_packages, setup

package_name = 'foodsam_node' 

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
    maintainer='Eunae Park',
    maintainer_email='eapark@korasrobotics.com',
    description='FoodSAM-ROS2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	"subscriber = foodsam_node.subscriber:main",
            "detection = foodsam_node.pubsub:main",
        ],
    },
)
