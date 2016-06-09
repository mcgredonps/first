A simple first node program in ROS.

Uses Python and the DIS Python implementation to send DIS updates
whenever a GPS fix is received and published on the topic.

To run, in Ubuntu with ROS Kinetic installed:

0. Make sure the ROS config file is sourced. It's a good idea to put this
in your ~/.bashrc:

# ROS environment
source /opt/ros/kinetic/setup.bash

This specifies the kinetic kame release. There may be other releases
installed under /opt/ros.

1. Run roscore in a terminal window:

roscore

This starts up the messaging infrastructure that allows topics to be published
and received.

2. In another terminal window, start a node that simulates sending GPS fixes.
ROS has a standard topic for publishing GPS fixes, and this simulates a sensor
providing these fixes. First, source the module setup.bash

cd first
source devel/setup.bash

Then start the ROS node that simulates a GPS sensor publishing position fixes
to the standard topic for GPS position fixes:

rosrun dis gps_sensor_sim.py

This uses the module "dis" (in the src directory) and runs the node gps_sensor_sim.py
(in the src/dis/scripts directory). 

3. Likewise, start up a node that will receive the GPS position updates and
publish DIS updates:

cd first
source devel/setup.bash
rosrun dis dis_sender.py

This will start receiving simulated GPS position fixes on the standard ROS topic
and send DIS entity state PDUs that reflext the position of the robot. You should
change the code in first/src/dis/scripts/dis_sender.py to reflect the correct
bcast address, port, and entity state PDU settings, such as the entity type.

4. Visualize the node structure. ROS uses cooperating processes that communicate
via topics. You can see these processes ("nodes") and the topics that connect
them by running rqt_graph:

rqt_graph

This will show the gps_sensor_sim node (started in step 2) publishing to the
sensor_msgs/NavSatFix topic, and the dis_sender node receiving the messages
from the sensor_msgs/NavSatFix topic. 

Automating Launch

Since ROS robots often require many nodes to be started up, as with the
multi-step rosrun commands above, the entire process can be automated via
the roslaunch command. This uses the xml file named dis.launch to automate
launching several nodes.

Start roscore in a terminal window:

roscore

Then open another window and source the package setup.bash file

cd first
source devel/setup.bash

and then launch the nodes:

roslaunch dis.launch

And then visualize the node structure with rqt_graph:

rqt_graph




