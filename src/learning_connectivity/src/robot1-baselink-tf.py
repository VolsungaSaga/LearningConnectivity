#!/usr/bin/env python

import rospy
import roslib
import tf

import tf2_ros

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import geometry_msgs.msg

#This node handles the broadcasting of all the transforms we need to translate between the motion capture's frame and the odometry's frame.
# They include:
# Transform from world to map.
# Transform from map to robot 1


#In order for our system to work, the following ROS nodes must be active:
#   mocap_optitrack - For the motion capture position data.
#   turtlebot_bringup minimal launch - For the robot to broadcast odometry data.
#       (OPTIONAL) turtlebot_teleop - For optionally moving the robot.
class Robot1BaseLink_tf:
    def __init__(self):
        self.num_pose_msgs = 0

        self.tf_worldToMap = geometry_msgs.msg.TransformStamped()
        self.tf_mapToRobot1 = geometry_msgs.msg.TransformStamped()
        self.odomSub = rospy.Subscriber("/odom", Odometry, self.robot1BaseLink_tf_callback)
        self.robot1Sub = rospy.Subscriber("/Robot_1/pose", PoseStamped, self.worldToMap_tf_callback)



    def worldToMap_tf_callback(self, msg):
        br = tf2_ros.TransformBroadcaster()
        if self.num_pose_msgs == 0:
            self.tf_worldToMap.header.stamp = rospy.Time.now()
            self.tf_worldToMap.header.frame_id = "world"
            self.tf_worldToMap.child_frame_id = "map"
            self.tf_worldToMap.transform.translation.x = msg.pose.position.x
            self.tf_worldToMap.transform.translation.y = msg.pose.position.y
            self.tf_worldToMap.transform.translation.z = msg.pose.position.z

            self.tf_worldToMap.transform.rotation.x = msg.pose.orientation.x
            self.tf_worldToMap.transform.rotation.y = msg.pose.orientation.y
            self.tf_worldToMap.transform.rotation.z = msg.pose.orientation.z
            self.tf_worldToMap.transform.rotation.w = msg.pose.orientation.w
            #Try inverting z (yaw), so that the results from tf make a modicum of sense.
            quat = self.tf_worldToMap.transform.rotation
            quat_list = [quat.x, quat.y, quat.z, quat.w]
            roll, pitch, yaw = euler_from_quaternion(quat_list)
            yaw = yaw
            quat = quaternion_from_euler(roll,pitch,yaw)

            self.num_pose_msgs += 1

        br.sendTransform(self.tf_worldToMap)

    def robot1BaseLink_tf_callback(self,msg):
            #Here, we supply a transform from the world frame to the odom frame
            br = tf2_ros.TransformBroadcaster()
            self.tf_mapToRobot1.header.stamp = rospy.Time.now()
            self.tf_mapToRobot1.header.frame_id = "map"
            self.tf_mapToRobot1.child_frame_id = "Robot_1/base_link"
            self.tf_mapToRobot1.transform.translation.x = msg.pose.pose.position.x
            self.tf_mapToRobot1.transform.translation.y = msg.pose.pose.position.y
            self.tf_mapToRobot1.transform.translation.z = msg.pose.pose.position.z

            self.tf_mapToRobot1.transform.rotation.x = msg.pose.pose.orientation.x
            self.tf_mapToRobot1.transform.rotation.y = msg.pose.pose.orientation.y
            self.tf_mapToRobot1.transform.rotation.z = msg.pose.pose.orientation.z
            self.tf_mapToRobot1.transform.rotation.w = msg.pose.pose.orientation.w
            #Try inverting z (yaw), so that the results from tf make a modicum of sense.
            quat = self.tf_mapToRobot1.transform.rotation
            quat_list = [quat.x, quat.y, quat.z, quat.w]
            roll, pitch, yaw = euler_from_quaternion(quat_list)
            yaw = yaw
            quat = quaternion_from_euler(roll,pitch,yaw)


            br.sendTransform(self.tf_mapToRobot1)

    


if __name__ == '__main__':

#We are attempting to find the transform between the world and odom by finding the transform between the world
# and the robot1's base link as reported by the motion capture node. After this world-to-link transform is found,
# the world-to-odom transform is made to be that initial transform. Thus, we acquire the world-to-odom link.


    rospy.init_node("robot1_baselink_tf")
    Robot1BaseLink_tf = Robot1BaseLink_tf()

    rospy.spin()