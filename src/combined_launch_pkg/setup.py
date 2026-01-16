from setuptools import find_packages, setup

package_name = 'combined_launch_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', ['launch/combined.launch.py']),
],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tamara Kraus',
    maintainer_email='tamara.kraus@stud.tu-darmstadt.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
 
    entry_points={
    'console_scripts': [
        'mcl_liveplot = combined_launch_pkg.mcl_liveplot:main',
    ],
},

)
