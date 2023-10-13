from setuptools import setup

setup(
    name='rclpy_decorators',
    version='1.0.0',
    description='Decorator functions for commonly used functionality of the ROS 2 Python client library',
    url='https://github.com/hmakelin/gisnav/rclpy_decorators',
    author='Harri Makelin',
    author_email='hmakelin@protonmail.com',
    license='MIT',
    packages=['rclpy_decorators'],
    install_requires=[
        'rclpy'
    ],
    zip_safe=True
)