#!/usr/bin/env python3

## @package motion_controller
#
# makes the robot move in the map respecting the normal, sleep and play behaviour

import rospy
import random
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import CompressedImage
import math
import actionlib
import actionlib.msg
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


VERBOSE = False

# default behaviour
behaviour = None
room = None
room_pos = None

# home position
home = [rospy.get_param('home_x'), rospy.get_param('home_y')]
# Action client goal init
goal_pos = MoveBaseGoal()

# action client init
act_c = None
# publishers for home poisition reaching
pub_home_reached = rospy.Publisher("/home_reached", Bool, queue_size=1)
pub_human_reached = rospy.Publisher("/human_reached", Bool, queue_size=1)
pub_no_room = rospy.Publisher("/no_room",Bool, queue_size=1)

# global variables init
home_reached = False
human_reached = False

rate = None

## function get_random_position
#
# get a random position on the map
def get_random_position():
    randX = random.randint(-6, 6)
    randY = random.randint(-8, 8)
    randPos = [randX, randY]
    return randPos

## function get_room_pos
#
# function to check if the room position is known
def get_room_pos():
    while(not rospy.is_shutdown()):
        if (room != None):
            break
        rate.sleep()

    colour = rospy.get_param(room)
    if rospy.search_param(colour) != None:
        room_pos = rospy.get_param(colour)
    else:
        #room_pos = "no_room"
        room_pos = None
    return room_pos
    

## function get_behaviour
#
# subscriber callback to the behaviour topic
def get_behaviour(state):
    global behaviour
    behaviour = state.data

## function get_go_to_command
#
# get the go to command from human
def get_go_to_command(command):
    global room
    room = command.data 
    

## function feedback_cb
#
# callback to send_goal function
def feedback_cb(feedback):
    # while the goal is being reached, check if the behaviour changes
    if behaviour == 'play' or behaviour == 'sleep' or behaviour == 'track_normal':
        rospy.loginfo("The behaviour has changed! Canceling goal...")
        act_c.cancel_all_goals()

## function move_normal
#
# movement in the NORMAL state
def move_normal():
    
    # get a random position
    pos = get_random_position()

    goal_pos.target_pose.header.frame_id = "map"
    goal_pos.target_pose.header.stamp = rospy.Time.now()
    # set robot goal position
    goal_pos.target_pose.pose.position.x = pos[0]
    goal_pos.target_pose.pose.position.y = pos[1]
    goal_pos.target_pose.pose.position.z = 0
    goal_pos.target_pose.pose.orientation.w = 2
    # send robot position and wait that the goal is reached within 60 seconds
    act_c.send_goal(goal_pos, feedback_cb=feedback_cb)
    rospy.loginfo("Robot goal position sent:")
    rospy.loginfo(goal_pos.target_pose.pose.position)
    act_c.wait_for_result(rospy.Duration.from_sec(180.0))
    result = act_c.get_result()
    if result:
        rospy.loginfo("Robot has reached the goal or the behaviour has changed")


## function move_sleep
#
# movement in the SLEEP state
def move_sleep():
    global home_reached

    goal_pos.target_pose.header.frame_id = "map"
    goal_pos.target_pose.header.stamp = rospy.Time.now()
    # set robot goal position
    goal_pos.target_pose.pose.position.x = home[0]
    goal_pos.target_pose.pose.position.y = home[1]
    goal_pos.target_pose.pose.position.z = 0
    goal_pos.target_pose.pose.orientation.w = 2
    # send robot position and wait that the goal is reached
    act_c.send_goal(goal_pos)
    rospy.loginfo("Robot returns home...")
    rospy.loginfo(goal_pos.target_pose.pose.position)
    act_c.wait_for_result(rospy.Duration.from_sec(240.0))
    result = act_c.get_result()
    if result:
        rospy.loginfo("Robot has reached the home position in time, now sleeps")
        home_reached = True

## function move_play
#
# movement in the PLAY state
def move_play():
    global human_reached, room

    if not human_reached:
        goal_pos.target_pose.header.frame_id = "map"
        goal_pos.target_pose.header.stamp = rospy.Time.now()
        # set robot goal position
        goal_pos.target_pose.pose.position.x = home[0]
        goal_pos.target_pose.pose.position.y = home[1]
        goal_pos.target_pose.pose.position.z = 0
        goal_pos.target_pose.pose.orientation.w = 2
        # send robot position and wait that the goal is reached within 60 seconds
        act_c.send_goal(goal_pos)
        rospy.loginfo("Robot returns in front of the human...")
        act_c.wait_for_result(rospy.Duration.from_sec(240.0))
        result = act_c.get_result()
        if result:
            rospy.loginfo("Robot has reached the human position in time")
            human_reached = True
            room_pos = None
            pub_human_reached.publish(human_reached)
    else:
        room_position = get_room_pos()
        # if the room position is unknown, publish to 'no_room' topic (go to Find behaviour)
        if  room_position == None:
            rospy.loginfo("The location is unknown!")
        
            # signal that the location is not known
            pub_no_room.publish(True)
            pub_human_reached.publish(False)
            rospy.sleep(2)
            human_reached = False
            room = None

        # if the room position is known, reach the room
        elif room_position != None:
            goal_pos.target_pose.header.frame_id = "map"
            goal_pos.target_pose.header.stamp = rospy.Time.now()
            # set robot goal position
            goal_pos.target_pose.pose.position.x = room_position[0]
            goal_pos.target_pose.pose.position.y = room_position[1]
            goal_pos.target_pose.pose.position.z = 0
            goal_pos.target_pose.pose.orientation.w = 2
            # send robot position and wait that the goal is reached within 60 seconds
            act_c.send_goal(goal_pos)
            rospy.loginfo("Robot going to the %s (%s)", room, rospy.get_param(room))
            #rospy.loginfo(goal_pos.target_pose.pose.position)
            act_c.wait_for_result(rospy.Duration.from_sec(240.0))
            result = act_c.get_result()
            if result:
                rospy.loginfo("Robot has reached the %s (%s) in time", room, rospy.get_param(room))
                # wait some time before returning to the human
                rospy.sleep(random.randint(3,10))
                human_reached = False
                room = None
                room_position = None
                

## function main
#
# init action client and choose how the robot moves checking its state (behaviour)
def main():
    # init node
    rospy.init_node("motion_controller")
    global act_c, home_reached, human_reached, rate

    rate = rospy.Rate(20)

    # subscriber to go_to command
    rospy.Subscriber("/go_to_command", String, get_go_to_command)

    # subscriber to current behaviour
    rospy.Subscriber("/behaviour", String, get_behaviour)
    rospy.loginfo("Subscribed to the behaviour")

    # initialize action client
    act_c = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # wait for the initialization of the server for 10 seconds
    act_c.wait_for_server(rospy.Duration(10))

    while not rospy.is_shutdown():
        # move the robot according to behaviour
        if behaviour == "sleep":
            if not home_reached:
                move_sleep()
                if home_reached:
                    pub_home_reached.publish(home_reached)
        else:
            # reinitialize home_reached
            home_reached = False

            if behaviour == "normal":
                move_normal()
                rospy.sleep(1)
            
            if behaviour == "play":
                move_play()
                
        rate.sleep()
            


if __name__ == "__main__":
    main()
