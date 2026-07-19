from setuptools import find_packages, setup

package_name = 'tf_follower'

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
    maintainer='dev',
    maintainer_email='dev@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_follower_node = tf_follower.tf_follower:main',
            'tf_follower_node_v2 = tf_follower.tf_follower_v2:main',
            'tf_follower_node_v2pt2 = tf_follower.tf_follower_v2pt2:main',
            "tf_follower_task3 = tf_follower.tf_follower_task3:main",
            "tf_follower_task3v2 = tf_follower.tf_follower_task3v2:main",
        ],
    },
)
