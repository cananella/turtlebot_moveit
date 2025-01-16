from setuptools import setup

package_name = 'turtlebot_moveit'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'scripts.aruco_marker_detect',  # Add your Python scripts here
        'scripts.aruco_move_test',  # Add your Python scripts here
        'scripts.srv_call_test',  # Add your Python scripts here
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_marker_detect = scripts.aruco_marker_detect:main',  # Add your entry points here
            'aruco_move_test = scripts.aruco_move_test:main',  # Add your entry points here
            'srv_call_test = scripts.srv_call_test:main',
            
        ],
    },
    package_data={
        package_name: ['config/calibration_params.yaml'],  # Include the YAML file
    },
)