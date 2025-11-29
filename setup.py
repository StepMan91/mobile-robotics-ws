from setuptools import setup

package_name = 'unitree_rl'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wsluser',
    maintainer_email='wsluser@todo.todo',
    description='RL Agent for Unitree G1',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rl_agent = unitree_rl.rl_agent:main',
        ],
    },
)
