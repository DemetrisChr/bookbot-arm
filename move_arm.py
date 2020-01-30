#!/usr/bin/env python
import rospy
import numpy as np
from math import floor
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from time import time, sleep

currentPos = [0,1,1,1,1,0]

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
    rospy.Subscriber("joint_states",JointState,joint_callback)

# Makes sure the joints do not go outside the joint limits/break the servos
def clean_joint_states(data):
    lower_limits = [0, -1.57, -1.57, -1.57, -1.57,   -1]
    upper_limits = [0,  1.57,  1.57,  1.57,  1.57, 1.57]
    clean_lower = np.maximum(lower_limits,data)
    clean_upper = np.minimum(clean_lower,upper_limits)
    return list(clean_upper)

# publishes a set of joint commands to the 'joint_trajectory_point' topic
# Joint Position vector should contain 6 elements
# [0, shoulder1, shoulder2, elbow, wrist, gripper]
def publish_move(data):
    jointpub = rospy.Publisher('joint_trajectory_point',Float64MultiArray,queue_size =10)
    joint_pos = Float64MultiArray()
    joint_pos.data = clean_joint_states(data)
    jointpub.publish(joint_pos)

# Unused. Should be deprecated
def atDestination(data):
    print("checking if it's at destination...")
    print("destination difference: {}".format(np.linalg.norm(np.array(data[:-1]) - np.array(currentPos[:-1]))))
    return np.linalg.norm(np.array(data[:-1]) - np.array(currentPos[:-1])) <= 0.06

# Breaks down the move from origin -> destination into smaller intermediate steps
def getIntermediateSteps(destination, step):
    dif = np.array(destination) - np.array(currentPos)
    divFactor = np.max(np.abs(dif)) / step
    stepVector = dif / divFactor

    intermediateSteps = [np.array(currentPos) + np.array(stepVector)*i for i in range(1, int(divFactor)+1)]
    intermediateSteps.append(destination)
    return intermediateSteps

# Executes the move sequence at 20hz. After the moveSequence is done, it will keep trying to reach the destination for another 0,5s
# moveSequence is an array of moves (where a move is an array of joint states)
def executeMoveSequence(moveSequence):
    rate = rospy.Rate(20)
    for move in moveSequence:
        read_joint_states()  # loops over the commands at 20Hz until shut down
        print("Currently at: {}".format(np.array(currentPos)))
        print("Want to go to: {}".format(move))
        publish_move(np.array(move))
        rate.sleep()

    # keep trying to get to the destination for another 1s
    start_time = time()
    while time() < start_time + 1:
        publish_move(np.array(moveSequence[-1]))
        rate.sleep()

# Moves to the destination at a set speed (low, normal, fast, jump)
def moveTo(**kwargs):
    # Read arguments. Set destination and speed
    destination = kwargs.get('destination', [0,0,0,0,0,0])
    speed = kwargs.get('speed', 'normal')
    speed_switch = {
        'slow': 0.03,
        'normal': 0.05,
        'fast': 0.07,
        'jump': 1
    }
    step = speed_switch.get(speed)

    # read current(initial) position
    readCurrentPos()
    initialPos = currentPos

    # Break the move into intermediate steps. Create the moveSequence
    if speed == 'jump':
        moveSequence = [destination for i in range(5)]
    else:
        moveSequence = getIntermediateSteps(destination, step)

    # DEBUGGING
    rospy.logwarn("Planning move sequence!")
    print("Initial pos: {}".format(initialPos))
    print("Destination: {}".format(destination))
    print("Planned move sequence:")
    np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})
    for move in moveSequence:
        print move

    # Execute move sequence
    executeMoveSequence(moveSequence)

if __name__ == '__main__':
    rospy.init_node('move_arm',anonymous=True)
    rospy.logwarn("moveTo started")
    #moveTo(destination=[0,1,1,0,0,0], speed="normal")
    #moveTo(destination=[0,0,0,0,0,0], speed="normal")
    moveTo(destination=[0,-1,0.5,-0.4,0,0], speed="normal")
    rospy.logwarn("moveTo ended")
    rospy.spin()
