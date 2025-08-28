from setuptools import setup

package_name = 'gesture_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YOUR_NAME',
    maintainer_email='your@email.com',
    description='',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gesture_controller = gesture_controller.gesture_controller:main',
            'drone_initializer = gesture_controller.drone_initializer:main',
        ],
    },
)
