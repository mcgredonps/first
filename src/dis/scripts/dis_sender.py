#!/usr/bin/env python
# Converts GPS coordinate fixes to DIS messages.
# see http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html
#
# This subscribes to and receies messages from the sensor_msgs/NavSatFix
# topic, which is a typical default source for GPS fixes. It converts
# the GPS lat/lon/alt (geodetic) coordinates to DIS geocentric coordinates
# and publishes to the network DIS Entity State PDUs. 
#
# Uses the Python DIS code I did earlier. The network bcast address
# and port need to be set. (UDP_PORT and DESTINATION_ADDRESS).
#
# Whether GPS is sufficiently accurate for DIS updates is up to the
# user. There are other topics in ROS that may publish more accureate
# position information in a local coordinate system.
#
# DMcG

# imports from standard python for networking, etc.
import socket
import sys
import time

# imports from ros
import rospy
from sensor_msgs.msg import NavSatFix

# DIS import
sys.path.append("dis_io")
sys.path.append("distributed_interactive_simulation")

from DataInputStream import DataInputStream
from DataOutputStream import DataOutputStream

from dis7 import EntityStatePdu
from io import BytesIO
from RangeCoordinates import GPS

udpSocket = 0
UDP_PORT = 3000
DESTINATION_ADDRESS = "172.20.159.255"

def gps_fix_callback(data):
    latitude = data.latitude
    longitude = data.longitude
    altitude = data.altitude
    rospy.loginfo(rospy.get_caller_id() + "latitude is about %s", data.latitude)
    
    espdu = EntityStatePdu()
    
    # Set the entity ID, the unique identifiers for an entity in DIS
    espdu.entityID.entityID = 42
    espdu.entityID.siteID = 17
    espdu.entityID.applicationID = 23
    
    # Set entity type, etc:
    
    # Set the location. This requires a change in coordinate systems from
    # the geodetic lat/lon/alt to the geocentric coordinate system of
    # DIS.
    
    gps = GPS()
    montereyLocation = gps.lla2ecef((latitude, longitude, altitude) )
    espdu.entityLocation.x = montereyLocation[0]
    espdu.entityLocation.y = montereyLocation[1]
    espdu.entityLocation.z = montereyLocation[2]
    
    # Serialize the PDU and put it on the network
    memoryStream = BytesIO()
    outputStream = DataOutputStream(memoryStream)
    espdu.serialize(outputStream)
    data = memoryStream.getvalue()
    
    # Send message to the network
    udpSocket.sendto(data, (DESTINATION_ADDRESS, UDP_PORT))

    

# Set up a UDP socket to send
def setup_networking():
   
    
    global udpSocket
    
    udpSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udpSocket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    rospy.loginfo(rospy.get_caller_id() + "set up UDP networking");
    
# Subscribe to the sensor_msgs/NavSatFix topic, which should provide
# us with GPS position updates. Specify the gps_fix_callback function
# as what gets called when we receive a NavSatFix update.

# Note that logging won't work until init_node is called. anonymous=True causes
# a unique name to be used for the node name; otherwise an old node that has
# the same name will be kicked off and replaced when a new node with the same
# name starts.

def dis_sender():
     rospy.init_node('dis_sender', anonymous=True)
     setup_networking()
     
     # In theory there are nodes publishing to this standard topic, typically
     # a GPS receiver. The message format is standard within ROS.
     
     rospy.Subscriber("sensor_msgs/NavSatFix", NavSatFix, gps_fix_callback)
     rospy.spin() 
 

if __name__ == '__main__':
    try:
        dis_sender()
    except rospy.ROSInterruptException:
        pass
