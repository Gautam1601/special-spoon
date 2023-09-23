#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult
from std_msgs.msg import Header
from math import atan2

class MazeSolver:
    def __init__(self):
        rospy.init_node('maze_solver')
        
        # Initialize your variables here
        self.robot_pose = None
        self.target_pose = None
        self.navigation_status = None
        
        # Create a subscriber to get the robot's current pose
        self.pose_subscriber = rospy.Subscriber('/robot_pose', PoseStamped, self.pose_callback)
        
        # Create a subscriber to listen for navigation status
        self.navigation_status_subscriber = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.navigation_status_callback)
        
        # Create a publisher to send navigation goals
        self.navigation_goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        
        # Initialize your target pose
        self.set_target_pose(x=5.0, y=5.0, theta=0.0)  # Example goal
        
    def pose_callback(self, pose):
        # Update the robot's pose when it's received
        self.robot_pose = pose
        
    def navigation_status_callback(self, status):
        # Update the navigation status when it changes
        self.navigation_status = status.status.status
        
    def set_target_pose(self, x, y, theta):
        # Create a new target pose
        self.target_pose = PoseStamped()
        self.target_pose.header = Header()
        self.target_pose.header.stamp = rospy.Time.now()
        self.target_pose.header.frame_id = "map"
        self.target_pose.pose.position.x = x
        self.target_pose.pose.position.y = y
        self.target_pose.pose.orientation.z = theta
        self.target_pose.pose.orientation.w = 1.0
        
    def navigate_to_goal(self):
        while not rospy.is_shutdown():
            if self.navigation_status == GoalStatus.SUCCEEDED:
                # Goal reached, set a new target pose
                self.set_target_pose(x=1.0, y=1.0, theta=0.0)  # Example next goal
                self.navigation_status = None  # Reset status
                
            if self.robot_pose is not None:
                # Calculate the angle to the target
                target_angle = atan2(self.target_pose.pose.position.y - self.robot_pose.pose.position.y,
                                     self.target_pose.pose.position.x - self.robot_pose.pose.position.x)
                
                # Send the goal to the navigation stack
                self.navigation_goal_publisher.publish(self.target_pose)
                
                # Adjust robot's orientation to face the target
                # You may need to implement this part

if __name__ == '__main__':
    try:
        maze_solver = MazeSolver()
        maze_solver.navigate_to_goal()
    except rospy.ROSInterruptException:
        pass
