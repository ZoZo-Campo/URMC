from setuptools import setup,find_packages
from glob import glob
name='urmc_mission_05'
setup(name=name,version='0.1.0',packages=find_packages(),data_files=[
 ('share/ament_index/resource_index/packages',['resource/'+name]),
 ('share/'+name,['package.xml']),('share/'+name+'/launch',glob('launch/*.launch.py')),
 ('share/'+name+'/config',glob('config/*.yaml'))],install_requires=['setuptools'],
 zip_safe=True,maintainer='Équipe URMC',maintainer_email='team@example.invalid',
 description='Préparation expérimentale URMRC 2026',license='Apache-2.0',
 entry_points={'console_scripts':['mission = urmc_mission_05.main:main']})
