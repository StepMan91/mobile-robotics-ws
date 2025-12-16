from setuptools import setup
import os
from glob import glob

package_name = 'ros2_bridge'

# Read the long description from README.md
with open('README.md', 'r', encoding='utf-8') as f:
    long_description = f.read()

setup(
    # ---------------------------------------------------------
    # Core Package Identity
    # ---------------------------------------------------------
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    
    # ---------------------------------------------------------
    # Resource Installation
    # ---------------------------------------------------------
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'cpp_assets'), glob('cpp_assets/*')),
    ],
    scripts=['test_udp_sender.py', 'listen_debug.py'],
    entry_points={
        'console_scripts': [
            'human_bridge_node = ros2_bridge.human_bridge_node:main',
        ],
    },
    
    # ---------------------------------------------------------
    # Dependencies & Testing (Active)
    # ---------------------------------------------------------
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    
    # ---------------------------------------------------------
    # Maintainer & Description (ROS Standard)
    # ---------------------------------------------------------
    maintainer='basti',
    maintainer_email='bastien.caspani@gmail.com',
    description='Bridge between external UDP vision app and ROS2 for Humanoid Teleoperation',
    long_description=long_description,
    long_description_content_type='text/markdown',
    license='MIT',
    
    # ---------------------------------------------------------
    # Domain Context & Searchability
    # ---------------------------------------------------------
    keywords=['ROS2', 'Robotics', 'Computer Vision', 'Humanoid', 'Unitree G1', 'Teleoperation', 'Sim-to-Real'],
    url='https://github.com/bastien-caspani/mobile-robotics-ws',
    project_urls={
        'Source': 'https://github.com/bastien-caspani/mobile-robotics-ws',
        'Tracker': 'https://github.com/bastien-caspani/mobile-robotics-ws/issues',
        'Documentation': 'https://github.com/bastien-caspani/mobile-robotics-ws/blob/main/README.md',
    },
    classifiers=[
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Developers',
        'Intended Audience :: Science/Research',
        'Topic :: Scientific/Engineering :: Robotics',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.10',
        'Operating System :: Microsoft :: Windows',
        'Operating System :: POSIX :: Linux',
    ],

    # ---------------------------------------------------------
    # Extended / Optional / Legacy Arguments (For Completeness)
    # ---------------------------------------------------------
    # These fields are often not strictly required by colcon but are part of standard setuptools
    
    # Author identity (often redundant with maintainer in ROS, but good for PyPI)
    author='Bastien Caspani',
    author_email='bastien.caspani@gmail.com',
    
    # Platform specifications
    platforms=['Windows', 'Linux'],
    
    # Python version constraints
    python_requires='>=3.8',
    
    # Packaging options
    include_package_data=True, # Auto-include files from MANIFEST.in if present
    
    # Optional dependencies (e.g. for development or extra features)
    extras_require={
        'test': ['pytest', 'ament_copyright', 'ament_flake8', 'ament_pep257'],
        'dev': ['ptvsd'], # Debugging tools
    },
    
    # Deprecated / Rarely used fields
    provides=[package_name],
    obsoletes=[],
)
