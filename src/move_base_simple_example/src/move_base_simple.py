#!/bin/python3

import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult

class Node:
    def __init__(self) -> None:
        rospy.init_node(NODE_NAME)
        self.move_base_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)
        rospy.sleep(2)
    
    def send_goal(self, x, y):
        '''
        Read the target coordinates specified by the user, then create the target message and publish it to move_base simple topic
        '''
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.orientation.w = 1
        self.move_base_pub.publish(goal)
        rospy.loginfo("Goal sent")
        
    def loop(self):
        '''
        Send the fixed navigation goals
        '''
        rospy.loginfo("Startup the robot position then press enter")
        input()
        self.send_goal(x=-1.6, y=-1.55)
        rospy.wait_for_message("move_base/result", MoveBaseActionResult) #Wwait for the end of navigation
        rospy.sleep(1)
        self.send_goal(x=-0.5, y=-2)
        rospy.wait_for_message("move_base/result", MoveBaseActionResult)
        rospy.sleep(1)
        self.send_goal(x=1.2, y=-1.7)
        rospy.wait_for_message("move_base/result", MoveBaseActionResult)
            

if __name__ == "__main__":
    NODE_NAME = "move_base_simple_node"
    node = Node()
    node.loop()
