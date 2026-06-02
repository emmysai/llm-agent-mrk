from setuptools import setup
import os
from glob import glob

package_name = 'llm_agent'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eeman',
    maintainer_email='emmy.saifee@gmail.com',
    description='LLM Agent with 3 required tools (get_robot_pose, get_waypoints, calculate_distance) + token/call tracking',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'llm_agent_node = llm_agent.node:main',
            'llm_agent_ui   = llm_agent.ui:main',
            'chat_cli       = llm_agent.chat:main',
        ],
    },
)
