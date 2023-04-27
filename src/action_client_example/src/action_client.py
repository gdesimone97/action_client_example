#!/bin/python3

import rospy
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback



class Node:
    def __init__(self) -> None:
        rospy.init_node(NODE_NAME)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction) #Action client startup
        rospy.loginfo("Waiting for move base...")
        self.client.wait_for_server()
        rospy.loginfo("Move base loaded")
        rospy.sleep(2)
        
    def feedback(self, msg: MoveBaseFeedback):
        '''
        Feedback callback.
        This function is called by the action server from the time the goal is sent until it is reached.
        The feedback message contains the current position of the robot in the world
        '''
        pose = msg.base_position.pose
        rospy.loginfo(f"Base position:\n{pose}")
    
    def done_cb(self, goal_status, result):
        '''
        Callback that gets called when the planner considers that the navigation is completed. 
        The callback should take two parameters: the terminal state (as an integer from actionlib_msgs/GoalStatus) and the result.
        '''
        if goal_status == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached")
        else:
            rospy.loginfo("Navigation error")
    
    def send_goal(self, x, y):
        '''
        Read the target coordinates specified by the user, then create the target message and send it to the move_base action server
        '''
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = float(x)
        goal.target_pose.pose.position.y = float(y)
        goal.target_pose.pose.orientation.w = 1
        self.client.send_goal(goal, feedback_cb=self.feedback, done_cb=self.done_cb)
        rospy.loginfo("Goal sent")
        self.client.wait_for_result()
    
    def loop(self):
        '''
        Send the fixed navigation goals
        '''
        rospy.loginfo("Startup the robot position then press enter")
        input()
        self.send_goal(x=-1.6, y=-1.55)
        rospy.sleep(1)
        self.send_goal(x=-0.5, y=-2)
        rospy.sleep(1)
        self.send_goal(x=1.2, y=-1.7)
            

if __name__ == "__main__":
    NODE_NAME = "action_client_node"
    node = Node()
    node.loop()
