from setuptools import setup
import glob

package_name = 'project3'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob.glob('launch/*')),
        #('share/' + package_name + '/msg', ['msg/XYArrays.msg']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shuruixu',
    maintainer_email='1330762153@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #'playBag = project3.playBag:main',
            'peopleCounter = project3.peopleCounter:main',
            #'xy_arrays_publisher = project3.xy_arrays_publisher:main',
        ],
    },
)
