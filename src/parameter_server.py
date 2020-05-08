#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from mallard_teleop.cfg import ParametersConfig

# This node serves as a parameter server. Every time parameter is changed 
# it publishes it on topic. The launch file on client side has to be called
# with initial parameter values.

def callback(config,level):
    rospy.loginfo("Kp gain: %s", config.Kp)
    return config # publish paramters, each on its own topic 


if __name__=="__main__":
    try:
        rospy.init_node("parameter_server", anonymous = False)
        srv = Server(ParametersConfig,callback) 
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass