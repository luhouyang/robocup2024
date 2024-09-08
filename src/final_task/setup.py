from setuptools import setup
from glob import glob

package_name = 'final_task'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name,
         ['package.xml']),
        ('share/' + package_name + '/launch',
         glob('launch/*launch.py')),
        ('share/' + package_name + '/config',
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lu',
    maintainer_email='luhouyang@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'final_script = final_task.final_script:main',
            'fallen = final_task.fallen:main'
        ],
    },
)
