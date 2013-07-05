from __future__ import print_function
import argparse
import os
import stat
import sys

# find the import relatively if available to work before installing catkin or overlaying installed version
if os.path.exists(os.path.join('/opt/ros/groovy/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/opt/ros/groovy/share/catkin/cmake', '..', 'python'))
from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/home/balto/rosbuild_ws/rosserial-experimental/rosserial_xbee/build/devel/env.sh')

output_filename = '/home/balto/rosbuild_ws/rosserial-experimental/rosserial_xbee/build/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    #print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
