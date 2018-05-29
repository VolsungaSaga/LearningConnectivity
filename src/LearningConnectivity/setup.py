## ! DO NOT MANUALLY INVOKE THIS WITH PYTHON, USE CATKIN INSTEAD 
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
	packages=['LearningConnectivity'],
	package_dir={'': 'scripts'},
	requires=['std_msgs','rospy','message_filters','sensor_msgs','geometry_msgs','nav_msgs','tf']


	)

setup(**setup_args)