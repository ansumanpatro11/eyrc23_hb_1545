from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'hb_task2b'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*'))
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ertslab',
    maintainer_email='srivenkateshwar@e-yantra.org',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'controller_1 = hb_task2b.bot_controller_1:main',
             'controller_2 = hb_task2b.bot_controller_2:main',
             'controller_3 = hb_task2b.bot_controller_3:main',
             'nextgoalpub= hb_task2b.nextGoalPub:main',
             'feedback = hb_task2b.feedback_2:main',
             'feedback2 = hb_task2b.feedback_2_mod:main'

         ],
    },
)