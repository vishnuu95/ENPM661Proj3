#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import math
import time
from std_srvs.srv import Empty
import numpy as np
                       



if __name__ == '__main__':
    try:
        pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
        rospy.init_node('turtle_move', anonymous=True)

        rate = rospy.Rate(10)
        vel_command = Twist()
        fname = './nodes_optimal.txt'
        f = open(fname,"r+")
        l = f.readlines()
        l_data = [line.rstrip() for line in l]
        location = []
        for pt in l_data:
            location.append((float(pt.split(',')[0]),float(pt.split(',')[1]),float(pt.split(',')[2])))

        loc = np.asarray(location)
        loc[:,:2] = loc[:,:2]/20
        for a in range(loc.shape[0]):
            if loc[a,2]>180:
                loc[a,2] = loc[a,2] -360
        temp = []
        for i in range(1,len(loc)):
            temp.append((np.sqrt((loc[i][0]-loc[i-1][0])**2+(loc[i][1]-loc[i-1][1])**2),loc[i][2]-loc[i-1][2]))

        temp = np.asarray(temp) 
        for p in temp:
            vel_command.angular.z = np.deg2rad(p[1])/5
            vel_command.linear.x = p[0]/5
            
            for i in range(50):
                rospy.loginfo(vel_command)
                pub.publish(vel_command)
                rate.sleep()


        #rospy.init_node('turtlesim_motion_pose', anonymous=True)

        
            
                   
        
        
       
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
