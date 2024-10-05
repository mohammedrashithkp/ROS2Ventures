from setuptools import find_packages, setup

package_name = 'my_robot_controller'

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
    maintainer='rashi',
    maintainer_email='rashi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "my_first_node = my_robot_controller.myfirstnode:main",
            "draw_circle_node  = my_robot_controller.drawcircle:main",
            "pose_subscriber_node = my_robot_controller.pose_subscriber:main",
            "turtle_controller_node = my_robot_controller.turtlecontroller:main",
        ],
    },
)
