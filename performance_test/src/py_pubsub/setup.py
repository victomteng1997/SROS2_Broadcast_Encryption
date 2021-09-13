from setuptools import setup

package_name = 'py_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gelei',
    maintainer_email='victomteng1997@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'talker = py_pubsub.publisher_member_function:main',
             'listener = py_pubsub.subscriber_member_function:main',
             'ROS2_perf_test = py_pubsub.ros2_perf:main',
             'ROS2_perf_test_enc = py_pubsub.ros2_perf_encrypt:main',
             'ROS2_perf_test_4 = py_pubsub.ros2_perf_4:main',
             'ROS2_perf_test_frequency = py_pubsub.ros2_perf_frequency:main',
             'ROS2_perf_test_enc_frequency = py_pubsub.ros2_perf_encrypt_frequency:main',
        ],
    },
)
