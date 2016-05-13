#!/usr/bin/env python

#
# Copyright (c) 2013
# Automation and Robotics Lab (LARA) at University of Brasilia (UnB)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     - Neither the name of the Automation and Robotics Lab (LARA) nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License LGPL along with this program.
# If not, see <http://www.gnu.org/licenses/>.
#
# ################################################################
#
#   ROS stack name: robot
#   ROS package name: robot_sim
#
#   Authors: Murilo M. Marinho, email: murilomarinho@lara.unb.br
#
# ################################################################


##############################################################################
#                                INCLUDES
##############################################################################
import roslib; roslib.load_manifest('dq_robotics_openrave')
import rospy
import openraveros

#Messages
from dq_robotics_openrave.msg  import jointvalues

#Services
from openraveros.srv import env_loadscene
from openraveros.srv import env_getbody
from openraveros.srv import body_setjointvalues
from openraveros.srv import body_getjointvalues

from dq_robotics_openrave.srv   import GetJointValues
from dq_robotics_openrave.srv   import GetJointValuesResponse
from dq_robotics_openrave.srv   import SetJointValues
from dq_robotics_openrave.srv   import SetJointValuesResponse

##############################################################################
#                            NODE MAIN ROUTINE
##############################################################################

def main():
    
    # Initialize Node
    rospy.init_node('dq_robotics_openrave_client')

    # Initialize RobotSimClient object
    robotsimclient = RobotSimClient()
    robotsimclient.Initialize()
    
    # Create main loop
    rate = rospy.Rate(robotsimclient.samplerate);

    while not rospy.is_shutdown(): 
        rate.sleep()
        robotsimclient.loop()

##############################################################################
#                            CLASS RobotPlot
##############################################################################

class RobotSimClient:

##############################################################################
#                         CONSTRUCTORS AND DESTRUCTORS
##############################################################################

    def __init__(self):
    
        # Wait for services to exist
        rospy.wait_for_service('/openrave/env_loadscene')
        rospy.wait_for_service('/openrave/env_getbody'  )
        rospy.wait_for_service('/openrave/body_getjointvalues')

        # Obtain robotname and robotdofs from parameter server
        self.GetParameters()

        # Member Variables
        self.robot_bodyid = 0

        # Initialize of service clients
        self.service_client_env_loadscene       = rospy.ServiceProxy('/openrave/env_loadscene'      , env_loadscene)
        self.service_client_env_getbody         = rospy.ServiceProxy('/openrave/env_getbody'        , env_getbody)
        self.service_client_body_setjointvalues = rospy.ServiceProxy('/openrave/body_setjointvalues', body_setjointvalues)
        self.service_client_body_getjointvalues = rospy.ServiceProxy('/openrave/body_getjointvalues', body_getjointvalues)

        # Initialize of service servers
        services_prefix = '/dq_robotics_openrave/robot/'
        self.service_server_setjointvalues      = rospy.Service(services_prefix    + str(self.robotname) + '/setjointvalues'    , SetJointValues , self.SetJointValuesCallback)
        self.service_server_getjointvalues      = rospy.Service(services_prefix    + str(self.robotname) + '/getjointvalues'    , GetJointValues , self.GetJointValuesCallback)

        # Initialize publishers
        self.publisher_getjointvalues           = rospy.Publisher(services_prefix  + str(self.robotname) + '/getjointvalues'  , jointvalues)
        self.publisher_getjointvalues_msg       = jointvalues()
    
        # Initialize subscribers
        self.subscriber_setjointvalues          = rospy.Subscriber(services_prefix + str(self.robotname) + '/setjointvalues' , jointvalues, self.SetJointValuesSubscriberCallback)
        
    def loop(self):
       
        try:
          # Publish joint values
          body_getjointvalues_response                   = self.service_client_body_getjointvalues(bodyid=self.robot_bodyid)
          self.publisher_getjointvalues_msg.header.stamp = rospy.Time.now()
          self.publisher_getjointvalues_msg.jointvalues  = body_getjointvalues_response.values
          self.publisher_getjointvalues.publish(self.publisher_getjointvalues_msg)
        except Exception as e:
          print e
                
##############################################################################
#                     CLASS INITIALIZATION
##############################################################################

    def Initialize(self):

        # Load environment file in robot_sim package folder
        str_package_path = roslib.packages.get_pkg_dir('dq_robotics_openrave')
        env_loadscene_response = self.service_client_env_loadscene(filename=(str_package_path + '/env/' + str(self.openraveenvironmentfilename) ),resetscene=1)

        # Get bodyid of the RobotSimClient inside the environment
        env_getbody_response   = self.service_client_env_getbody(str(self.robotname))
        self.robot_bodyid = env_getbody_response.bodyid
        

        return True

##############################################################################
#                     SERVICE SERVER CALLBACK FUNCTIONS
##############################################################################

### SETJOINTVALUES CALLBACK
    def SetJointValuesCallback(self, req):

        if(len(req.jointvalues)  != self.robotdofs ):
            rospy.loginfo('RobotSimClient_client: Incorrect size of jointvalues '+str(self.robotdofs)+'!=' + str(len(req.jointvalues)))
        else:
            body_setjointvalues_response = self.service_client_body_setjointvalues(self.robot_bodyid, req.jointvalues, [])

        return SetJointValuesResponse()

### GETJOINTVALUES CALLBACK
    def GetJointValuesCallback(self, req):

        try:
          body_getjointvalues_response = self.service_client_body_getjointvalues(bodyid=self.robot_bodyid)
        except Exception as e:
          print 'GetJointValuesCallback thrown exception: ',e        

        return GetJointValuesResponse(body_getjointvalues_response.values)

##############################################################################
#                     SUBSCRIBER CALLBACK FUNCTIONS
##############################################################################

### SETJOINTVALUES CALLBACK
    def SetJointValuesSubscriberCallback(self, msg):

        if(len(msg.jointvalues)  != self.robotdofs ):
            rospy.loginfo('RobotSimClient_client: Incorrect size of jointvalues '+str(self.robotdofs)+'!=' + str(len(msg.jointvalues)))
        else:
            body_setjointvalues_response = self.service_client_body_setjointvalues(self.robot_bodyid, msg.jointvalues, [])

        return

##############################################################################
#                         PARAMETER LOADING FUNCTIONS
##############################################################################        
        
    def GetParameters(self):
    
        error = False     
            
        # Robot name
        if  rospy.has_param('/robot/name'):
            self.robotname = rospy.get_param('/robot/name')
            rospy.logdebug('Parameter read: robotname = ' + str(self.robotname))                                  
        else:
            rospy.logerr("Parameter robotname not set, shutting down node...")
            rospy.signal_shutdown("Parameter Reading Error")
            error = True

        # Robot dofs
        if  rospy.has_param('/robot/dofs'):
            self.robotdofs = rospy.get_param('/robot/dofs')
            rospy.logdebug('Parameter read: robotdofs = ' + str(self.robotdofs))                                  
        else:
            rospy.logerr("Parameter robotdofs not set, shutting down node...")
            rospy.signal_shutdown("Parameter Reading Error")
            error = True    

        # OpenRave environment
        if  rospy.has_param('/openrave/environment/filename'):
            self.openraveenvironmentfilename = rospy.get_param('/openrave/environment/filename')
            rospy.logdebug('Parameter read: openraveenvironmentfilename = ' + str(self.openraveenvironmentfilename))                                  
        else:
            rospy.logerr("Parameter openraveenvironmentfilename not set, shutting down node...")
            rospy.signal_shutdown("Parameter Reading Error")
            error = True  

        # Samplerate environment
        if  rospy.has_param('/node/samplerate'):
            self.samplerate = rospy.get_param('/node/samplerate')
            rospy.logdebug('Parameter read: samplerate = ' + str(self.samplerate))                                  
        else:
            rospy.logerr("Parameter samplerate not set, shutting down node...")
            rospy.signal_shutdown("Parameter Reading Error")
            error = True  
        
        return not error



##############################################################################
#                          RUNNING THE MAIN ROUTINE
############################################################################## 

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

