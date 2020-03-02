#!/usr/bin/env python
import rospy
import numpy as np
from math import floor
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from time import time, sleep

currentPos = [-1.5,0,0,0,0]

# processes the data from the "joint_states" rostopic
# updates the Current Position
def joint_callback(data):
    global currentPos
    currentPos = [0]+list(data.position[2:7])
    #print("Msg: {}".format(data.header.seq))
    #print("Wheel Positions:\n\tLeft: {0:.2f}rad\n\tRight:{0:.2f}rad\n\n".format(data.position[0],data.position[1]))
    #print("Joint Positions:\n\tShoulder1: {0:.2f}rad\n\tShoulder2:{0:.2f}rad\n\tElbow: {0:.2f}rad\n\tWrist:{0:.2f}rad\n\n".format(data.position[2],data.position[3],data.position[4],data.position[5]))
    #print("Gripper Position:\n\tGripper:{0:.2f}rad\n".format(data.position[6]))
    #print("----------")# listens to the "joint_states" topic and sends them to "joint_callback"for processing

# read the joint states for 0.5s and update the current position
# !!! The time needs to be increased to 1s when LIDAR is also running scripts
def readCurrentPos():
    start_time = time()
    while time() < start_time + 0.5:
        read_joint_states()

def read_joint_states():
    rospy.Subscriber("/om_with_tb3/joint_states",JointState,joint_callback)

# Makes sure the joints do not go outside the joint limits/break the servos
def clean_joint_states(data):
    lower_limits = [0.5, -3.14, -1.57, -1.57, -1.57]
    upper_limits = [-0.8, 3.14, 1.57, 1.57, 1.57]
    clean_lower = np.maximum(lower_limits,data)
    clean_upper = np.minimum(clean_lower,upper_limits)
    return list(clean_upper)

# publishes a set of joint commands to the 'joint_trajectory_point' topic
# Joint Position vector should contain 6 elements
# [0, shoulder1, shoulder2, elbow, wrist, gripper]
def publish_move(data):
    jointpub = rospy.Publisher('/om_with_tb3/joint_trajectory_point',Float64MultiArray,queue_size =10)
    joint_pos = Float64MultiArray()
    joint_pos.data = clean_joint_states(data)
    jointpub.publish(joint_pos)
 
def moveTo(**kwargs):
    destination = kwargs.get('destination', [-1.5,0,0,0,0])
    readCurrentPos()
    initialPos = currentPos

    rospy.logwarn("Planning to move now")
    print("Initial pos: {}".format(initialPos))
    print("Destination pos: {}".format(destination))

    read_joint_states()
    print("now moving to next pos")
    exe(destination)

def exe(destination):
    rate = rospy.Rate(20)
    read_joint_states()
    print("Currently at: {}".format(np.array(currentPos)))
    print("want to go to: {}".format(np.array(destination)))
    publish_move(np.array(destination))
    
    start_time = time()
    while time() < start_time + 1.5:
        rate.sleep()
    


def move_arm():    
    moveTo(destination=[-1.5,0,0,0,0])
    #go back
    moveTo(destination=[-1.5,0.15,1.25,-0.96,0.7])
    #turn towards shelf
    moveTo(destination=[-1.5,-1.48,1.22,-1.04,-0.3])
    #lean in
    moveTo(destination=[-1.5,-1.48, -0.12,0.26,-0.4])
    #lean back out
    moveTo(destination=[-1.5,-1.48,1.17,-0.8,-0.3])

    moveTo(destination=[-1.5,-1.61,1.04,-0.65,-0.75])

    moveTo(destination=[-1.5,-3.08,1.25,-0.82,-0.75])

    moveTo(destination=[-1.5,-3.08,1.04,-0.5,-0.12])

    moveTo(destination=[-1.5,-3.08,0.16,-0.16,-0.12])

    moveTo(destination=[-1.5,-3.01,1.12,-1.2,0.12])

    moveTo(destination=[-1.5,0.02,1.2,-1.25,0.1])

    moveTo(destination=[-1.5,0,0,0,0])


    
    #loops over the commands at 20Hz until shut down
    
    
if __name__ == '__main__':    
    rospy.init_node('move_arm',anonymous=True)    
    rate = rospy.Rate(20)
    move_arm()
    rospy.logwarn("movement has ended")
    rospy.spin()
