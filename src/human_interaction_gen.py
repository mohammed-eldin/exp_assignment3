#!/usr/bin/env python

## @package human_interaction_gen
#
# Human interactions with the robot: sends play commands and goTo + location commands

import rospy
import random
from geometry_msgs.msg import PoseStamped
import actionlib
import actionlib.msg
from std_msgs.msg import String, Bool

## variables
pub_play = None
pub_go_to = None
human_reached = None

## function get_random_position
#
# get a random room for the go to command
def get_random_position():
    pos_array = ["Entrance", "Closet", "Livingroom", "Kitchen", "Bathroom", "Bedroom"]
    randPos = pos_array[random.randint(0, 5)]
    return randPos

## function get_human_reached
#
# subscriber callback, gets if the robot is in front of the human
def get_human_reached(home):  
    global human_reached
    human_reached = home.data
    
## function main
#
# initialize node, and loop for play and go to commands
def main():
    global pub_play, pub_go_to, human_reached

    # init node
    rospy.init_node("human_interaction_generator")
    rate = rospy.Rate(20)

    # publishers
    pub_play = rospy.Publisher("/play_command", Bool, queue_size=1)
    pub_go_to = rospy.Publisher("/go_to_command", String, queue_size=1)

    # subscriber for human_reached topic
    rospy.Subscriber("/human_reached", Bool, get_human_reached)

    while not rospy.is_shutdown():
        # send play command randomly
        if random.randint(1,rospy.get_param("play_freq")) == 1:
            # publish play command
            pub_play.publish(True)
            
        # if the robot is in front of the human, send a go to command
        if human_reached:    
            # wait random time
            rospy.sleep(random.randint(2,5))
            pos = get_random_position()
            rospy.loginfo("Human says: go to the %s (%s)", pos, rospy.get_param(pos))
            # publish goto command
            pub_go_to.publish(pos)
            # reinit 
            human_reached = False

        rate.sleep()


if __name__ == "__main__":
    main()
