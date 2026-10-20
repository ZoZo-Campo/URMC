from setuptools import setup,find_packages
from glob import glob
name='urmc_core'
setup(name=name,version='0.1.0',packages=find_packages(),data_files=[
 ('share/ament_index/resource_index/packages',['resource/'+name]),
 ('share/'+name,['package.xml']),('share/'+name+'/launch',glob('launch/*.launch.py')),
 ('share/'+name+'/config',glob('config/*.yaml')),('share/'+name+'/web',glob('web/*.html'))],install_requires=['setuptools'],
 zip_safe=True,maintainer='Équipe URMC',maintainer_email='team@example.invalid',
 description='Préparation expérimentale URMRC 2026',license='Apache-2.0',
 entry_points={'console_scripts':['simulator = urmc_core.sim:main', 'dashboard = urmc_core.dashboard:main']})
