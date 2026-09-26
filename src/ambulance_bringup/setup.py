from setuptools import setup
from glob import glob
setup(name='ambulance_bringup',version='0.1.0',packages=['ambulance_bringup'],data_files=[('share/ament_index/resource_index/packages',['resource/ambulance_bringup']),('share/ambulance_bringup/launch',glob('launch/*.py')),('share/ambulance_bringup/config',glob('config/*')),('share/ambulance_bringup/maps',glob('maps/*')),('share/ambulance_bringup',['package.xml'])],zip_safe=True)
