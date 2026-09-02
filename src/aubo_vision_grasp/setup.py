from setuptools import find_packages, setup

package_name = 'aubo_vision_grasp'

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
    maintainer='aiseon',
    maintainer_email='harry_ma03@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'setup_grasp_scene = aubo_vision_grasp.setup_grasp_scene:main',
            'auto_suction_grasp = aubo_vision_grasp.auto_suction_grasp:main',
            'red_target_detector = aubo_vision_grasp.red_target_detector:main',
        ],
    },
)
