from setuptools import setup, find_packages

package_name = 'hello'

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
    maintainer='otete',
    maintainer_email='otete@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = hello.talker:main',
            'ROB2 = hello.ROB2:main',
            'listener = hello.listener:main',
            'picture = hello.picture:main',
        ]
    },
)

