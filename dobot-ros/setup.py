from setuptools import setup, find_packages

package_name = 'dobot_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/dobot_config.yaml']),
    ],
    package_data={
        'dobot_ros': [
            'web/static/**/*',
            'web/static/css/*.css',
            'web/static/js/*.js',
        ],
    },
    include_package_data=True,
    install_requires=[
        'setuptools',
        'click>=8.0.0',
        'rich>=10.0.0',
        'pyyaml>=5.4.0',
        'prompt_toolkit>=3.0.0',
        'fastapi>=0.100.0',
        'uvicorn[standard]>=0.20.0',
        'requests>=2.28.0',
    ],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='ROS2 CLI for Dobot CR series robots',
    license='MIT',
    entry_points={
        'console_scripts': [
            'dobot-ros = dobot_ros.cli:main',
        ],
    },
)
