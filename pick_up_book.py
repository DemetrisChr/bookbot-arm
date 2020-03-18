#!/usr/bin/env python
import rospy
import numpy as np
from math import floor
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from time import time, sleep

currentPos = [-0.5, 0, 0, 0, 0]
currentGripper = 0
curr_effort = [0, 0, 0, 0, 0]


# processes the data from the "joint_states" rostopic
# updates the Current Position
def joint_callback(data):
    global currentPos
    global currentGripper
    global curr_effort
    currentPos = list(data.position[2:7])
    curr_effort = data.effort[2:]
    currentGripper = data.position[-1]


# read the joint states for 0.5s and update the current position
# !!! The time needs to be increased to 1s when LIDAR is also running scripts
def readCurrentPos():
    start_time = time()
    while time() < start_time + 1.5:
        read_joint_states()


def read_joint_states():
    rospy.Subscriber("/joint_states", JointState, joint_callback)


# Makes sure the joints do not go outside the joint limits/break the servos
def clean_joint_states(data):
    lower_limits = [0.5, -3.14, -1.57, -1.57, -1.57]
    upper_limits = [-0.8, 3.14, 1.57, 1.57, 1.57]
    clean_lower = np.maximum(lower_limits, data)
    clean_upper = np.minimum(clean_lower, upper_limits)
    return list(clean_upper)


def clean_joint_states_gripper(data):
    lower_limits = -0.01
    upper_limits = 0.01
    clean_lower = np.maximum(lower_limits, data)
    clean_upper = np.minimum(clean_lower, upper_limits)
    return list(clean_upper)


# publishes a set of joint commands to the 'joint_trajectory_point' topic
# Joint Position vector should contain 6 elements
# [0, shoulder1, shoulder2, elbow, wrist, gripper]

def publish_move(data):
    jointpub = rospy.Publisher('/joint_trajectory_point', Float64MultiArray, queue_size=10, latch=True)
    rospy.Rate(1).sleep()
    joint_pos = Float64MultiArray()
    joint_pos.data = clean_joint_states(data)
    jointpub.publish(joint_pos)


def publish_move_gripper(data):
    jointpub = rospy.Publisher('/gripper_position', Float64MultiArray, queue_size=10, latch=True)
    rospy.Rate(1).sleep()
    joint_pos = Float64MultiArray()
    # joint_pos.data = clean_joint_states_gripper(data)
    lower = [-0.015]
    upper = [0.015]
    clean_lower = np.maximum(lower, data)
    clean_upper = np.minimum(clean_lower, upper)
    joint_pos.data = clean_upper
    jointpub.publish(joint_pos)


def moveTo(**kwargs):
    destination = kwargs.get('destination', [-0.5, 0, 0, 0, 0])
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
    print("EFFORT: {}".format(curr_effort))
    print("want to go to: {}".format(np.array(destination)))
    print("CURRENT GRIPPER: {}".format(currentGripper))
    publish_move(np.array(destination))

    start_time = time()
    while time() < start_time + 0.5:
        rate.sleep()


def ready_to_pick_up():
    rospy.logwarn("Moving to initial position")
    rospy.logwarn("Align arm with book")
    moveTo(destination=[-0.5, 1.61, 0.4, 0.7, -0.4])
    publish_move_gripper([-0.013])
    rospy.logwarn("Lean towards book")
    moveTo(destination=[-0.5, 1.61, 0.25, 0.57, -0.4])
    rospy.logwarn("Slightly increase gripper width")
    publish_move_gripper([-0.013])

    grip_book()
    rospy.Rate(1).sleep()


# Open gripper = -0.01
# High negative effort happens when you try to close the gripper and there is something in the middle
# High positive effort happens in the opposite case

def grip_book():
    read_joint_states()
    gripper = currentGripper
    counter = 0
    while curr_effort[-1] > -160 and currentPos[1] > -0.1:
        read_joint_states()
        destination = [-0.5] + currentPos[:-1]
        if curr_effort[-1] > -100 and (curr_effort[1] <= 60 or curr_effort[2] <= 60) and counter <= 6:
            # if enough effort is being put on other joints, close gripper
            gripper += 0.002
            publish_move_gripper([gripper])
            counter += 1
        elif curr_effort[1] > 60:
            # get closer to book if not enough effort
            destination[2] -= 0.1
            moveTo(destination=destination)
        else:
            # if it's gripped hard enough, end the loop
            break

    print("ENDED LOOP")

    gripper -= 0.002
    publish_move_gripper([gripper])
    moveTo(destination=[-0.5, 1.61, -0.1, 0.7, -0.4])
    gripper += 0.002
    publish_move_gripper([gripper])
    moveTo(destination=[-0.5, 1.61, 0.4, 0.7, -0.3])
    gripper -= 0.004
    publish_move_gripper([gripper])
    rospy.Rate(1).sleep()
    gripper += 0.004
    publish_move_gripper([gripper])
    moveTo(destination=[-0.5, 1.61, 0.4, 0.7, 0.2])
    moveTo(destination=[-0.5, 1.61, 0, 0.3, 0.2])
    moveTo(destination=[-0.5, 1.61, 0.53, 0.87, -0.17])

    rospy.Rate(1).sleep()


if __name__ == '__main__':
    rospy.init_node('move_arm', anonymous=True)
    rate = rospy.Rate(20)
    ready_to_pick_up()
    rospy.logwarn("movement has ended")
    rospy.spin()
