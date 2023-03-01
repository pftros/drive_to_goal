#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from math import atan2
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class DriveToGoal:
    def __init__(self):
      # initialize subscribers (/goal and /base_pose_ground_truth)
      # initialize `cmd_vel` publisher
        self.goal = PoseStamped()
        self.goal_reached = True
        self.pose_subscriber = rospy.Subscriber("pose", Odometry, self.pose_callback)
        self.goal_subscriber = rospy.Subscriber("goal", PoseStamped, self.goal_callback)
        self.cmdvel_publisher = rospy.Publisher("cmd_vel", Twist)

    def goal_callback(self, msg):
        rospy.loginfo(msg)
        self.goal_reached = False
        self.goal = msg
        
    def pose_callback(self, msg):
        rospy.loginfo(msg)
        vel_msg = Twist()
        if(not self.goal_reached):
          vel_msg.angular.z = 0.2
        else:
            vel_msg.angular.z = 0
        
        self.cmdvel_publisher.publish(vel_msg)
        return

    
if __name__ == '__main__':
    rospy.init_node('drive_to_goal')
    DriveToGoal()
    rospy.spin()