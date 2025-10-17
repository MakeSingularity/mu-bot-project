from setuptools import setup, find_packages

package_name = 'emu_vision'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/emu_vision_launch.py']),
        ('share/' + package_name + '/config', ['config/emu_vision_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Emu Bot Team',
    maintainer_email='dev@mubot.org',
    description='Vision processing package for emu droid companion robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'emu_observer = emu_vision.emu_observer:main',
            'emu_tracker = emu_vision.emu_tracker:main',
            'emu_pose_estimator = emu_vision.emu_pose_estimator:main',
        ],
    },
)
