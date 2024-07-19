from setuptools import find_packages, setup

package_name = 'communication_controller'

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
    maintainer='anh',
    maintainer_email='89755193+anhp4561@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "identification = communication_controller.identification:main",
            "info_status = communication_controller.info_status:main",
            "exploration = communication_controller.exploration:main",
            "static_map_broadcaster = communication_controller.static_map_broadcaster:main",
        ],
    },
)
