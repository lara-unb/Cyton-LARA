#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib; roslib.load_manifest('dq_robotics_openrave')
import rospy
import os
from optparse import OptionParser
from openravepy import *
from openravepy.misc import OpenRAVEGlobalArguments

if __name__ == "__main__":

    rospy.init_node('dq_robotics_openrave_server')

    parser = OptionParser(description='openrave planning example')
    OpenRAVEGlobalArguments.addOptions(parser)
    (options, args) = parser.parse_args()
    try:

        #Load openraveros Server
        env = OpenRAVEGlobalArguments.parseAndCreate(options,defaultviewer=False)
        env.SetViewer('qtcoin')
        RaveLoadPlugin(os.path.join(roslib.packages.get_pkg_dir('openraveros'),'lib','openraveros'))
        namespace = 'openrave'
        if env.AddModule(RaveCreateModule(env,'rosserver'),namespace) != 0:
            raise ValueError('failed to create openraveros server')
        
        rospy.spin()
   

    finally:
        RaveDestroy()
