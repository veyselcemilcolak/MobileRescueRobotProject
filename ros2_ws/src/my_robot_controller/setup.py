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
    maintainer='imdl3',
    maintainer_email='imdl3@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "tn = my_robot_controller.trial0talk:main",
            "ln = my_robot_controller.trial0listen:main",
            "command_node = my_robot_controller.command_node:main",
            "control_node  = my_robot_controller.control_node:main",
            "input_node = my_robot_controller.input_node:main",
            "uart_node = my_robot_controller.uart_node:main",
            "robot_node = my_robot_controller.fake_odom_and_ir:main"
        ],
    },
)
