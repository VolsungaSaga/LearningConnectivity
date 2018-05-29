
#! /usr/bin/env python
import rospy
import roslib
import tf

import tf2_ros

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import geometry_msgs.msg

#In order for our system to work, the following ROS nodes must be active:
#   mocap_optitrack - For the motion capture position data.
#   turtlebot_bringup minimal launch - For the robot to broadcast odometry data.
#       turtlebot_teleop - For optionally moving the robot.
class worldToOdom_tf:
    def __init__(self):
        self.num_msgs = 0   
        self.transform = geometry_msgs.msg.TransformStamped()
        #self.odomSub = rospy.Subscriber("/odom", Odometry, self.worldToOdom_tf_callback)
        self.robot1Sub = rospy.Subscriber("/Robot_1/pose", PoseStamped, self.worldToOdom_tf_callback)



    def worldToOdom_tf_callback(self,msg):
            #Here, we supply a transform from the world frame to the odom frame
            br = tf2_ros.TransformBroadcaster()
            if self.num_msgs == 0:
                self.transform.header.stamp = rospy.Time.now()
                self.transform.header.frame_id = "map"
                self.transform.child_frame_id = "odom"
                self.transform.transform.translation.x = msg.pose.position.x
                self.transform.transform.translation.y = msg.pose.position.y
                self.transform.transform.translation.z = msg.pose.position.z

                self.transform.transform.rotation.x = msg.pose.orientation.x
                self.transform.transform.rotation.y = msg.pose.orientation.y
                self.transform.transform.rotation.z = msg.pose.orientation.z
                self.transform.transform.rotation.w = msg.pose.orientation.w
                #Try inverting z (yaw), so that the results from tf make a modicum of sense.
                quat = self.transform.transform.rotation
                quat_list = [quat.x, quat.y, quat.z, quat.w]
                roll, pitch, yaw = euler_from_quaternion(quat_list)
                yaw = -yaw
                quat = quaternion_from_euler(roll,pitch,yaw)

                self.num_msgs += 1

            br.sendTransform(self.transform)

    


if __name__ == '__main__':

#We are attempting to find the transform between the world and odom by finding the transform between the world
# and the robot1's base link as reported by the motion capture node. After this world-to-link transform is found,
# the world-to-odom transform is made to be that initial transform. Thus, we acquire the world-to-odom link.


    rospy.init_node("worldToOdom_tf")
    worldToOdom_tf = worldToOdom_tf()

    rospy.spin()