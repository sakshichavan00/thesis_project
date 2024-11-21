from setuptools import find_packages, setup

package_name = 'camera_test_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # add the lunch file in the launch folder, any launch file
        ('share/' + package_name + '/launch', ['launch/panther_fusion.launch.py']),
        # add the param file in the config folder
        ('share/' + package_name + '/config', ['config/panther_fusion_cam.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sakshi',
    maintainer_email='sakshijchavan@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_subscriber_node = camera_test_pkg.camera_subscriber_node:main',
            'camera_transformer_node = camera_test_pkg.camera_transformer_node:main',
            'camera_transformer_node2 = camera_test_pkg.camera_transformer_node2:main',
            'integrated_camera_node = camera_test_pkg.integrated_camera_node:main',
            'sensor_fusion_covariance= camera_test_pkg.sensor_fusion_covariance:main',
            'map_cam_odom_tf = camera_test_pkg.map_cam_odom_tf:main',
            'fusion_plot = camera_test_pkg.fusion_plot:main',
        ],
    },
)
