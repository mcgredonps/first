#!/usr/bin/env python
#
# Simulates a GPS sensor sending NavSatFix messages,
# see http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html

import rospy
from sensor_msgs.msg import NavSatFix

def gps_fix_sim():
    pub=rospy.Publisher('sensor_msgs/NavSatFix', NavSatFix, queue_size=10)
    rospy.init_node('gps_sim', anonymous=True)
    rate = rospy.Rate(2)
    
    while not rospy.is_shutdown():
       gps_fix = NavSatFix()
       gps_fix.latitude = 36.6
       gps_fix.longitude = -121.1
       gps_fix.altitude = 1.0
       pub.publish(gps_fix)
       rate.sleep()
       
if __name__ == '__main__':
    try:
        gps_fix_sim()
    except rospy.ROSInterruptException:
        pass
