#!/usr/bin/env python3

## @package ball_tracking
#
# uses cv2 libraries to track the balls in the map, reach them 
# and save the positions of the rooms, while avoiding obstacles thanks to the laser scan

# Python libs
import sys
import time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

import imutils

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String, Float64
from nav_msgs.msg import Odometry
from random import randint
import math

VERBOSE = False

# define colours limits
greenLower = (50, 50, 20)
greenUpper = (70, 255, 255)

blackLower = (0, 0, 0)
blackUpper = (5, 50, 50)

redLower = (0, 50, 50)
redUpper = (5, 255, 255)

yellowLower = (25, 50, 50)
yellowUpper = (35, 255, 255)

blueLower = (100, 50, 50)
blueUpper = (130, 255, 255)

magentaLower = (125, 50, 50)
magentaUpper = (150, 255, 255)



## class ball_tracking
#
# implements a ball tracking algorithm using cv2
class ball_tracking:
    # method __init__
    #
    # initialization of the ball_tracking class
    def __init__(self):
        # variables initialization
        self.ball_detected = False
        self.near_ball = False
        self.center = None
        self.radius = None
        self.behaviour = None
        self.ball_reached = False
        self.colour = None
        self.current_pos = None
        self.room = None


        self.regions_ = {
            'right': 0,
            'fright': 0,
            'front': 0,
            'fleft': 0,
            'left': 0,
        }

        # publishers
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.pub_ball = rospy.Publisher("/ball_detected", Bool, queue_size=1)
        self.pub_reach = rospy.Publisher("/ball_reached", Bool, queue_size=1)
        self.pub_room_found = rospy.Publisher("/room_found", Bool, queue_size=1)
        # subscriber to camera
        self.cam_sub = rospy.Subscriber(
            "/camera1/image_raw/compressed", CompressedImage, self.callback,  queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.clbk_laser)
        self.room_sub = rospy.Subscriber('/go_to_command', String, self.get_room)

        # subscriber to current behaviour
        rospy.Subscriber("/behaviour", String, self.get_behaviour)

    ## method get_behaviour
    #
    # subscriber callback to the behaviour topic
    def get_behaviour(self, state):
        self.behaviour = state.data

    ## method get_room
    #
    # subscriber callback to the room topic
    def get_room(self, room):
        self.room = room.data

    ## method odom_callback
    #
    # subscriber callback to the odometry topic
    def odom_callback(self, msg):
        self.current_pos = msg.pose.pose.position

    ## method clbk_laser
    #
    # subscriber callback for the laser topic
    def clbk_laser(self, msg):

        self.regions_ = {
            'right':  min(min(msg.ranges[0:143]), 10),
            'fright': min(min(msg.ranges[144:287]), 10),
            'front':  min(min(msg.ranges[288:431]), 10),
            'fleft':  min(min(msg.ranges[432:575]), 10),
            'left':   min(min(msg.ranges[576:713]), 10),
        }

    ## method follow_avoid_obstacles
    #
    # sends velocities to the robot to reach the ball avoiding obstacles
    def follow_avoid_obstacles(self):
        regions = self.regions_
        msg = Twist()
        angular_z = -0.003*(self.center[0] - 400)
        linear_x = -0.01*(self.radius - 100)
        state_description = ''

        # linear velocity saturation
        if linear_x > 0.4:
            linear_x = 0.4

        #d0 = 0.15
        #d = 0.30

        d0 = 0.1
        d = 0.15

        if regions['front'] > d0 and regions['fleft'] > d and regions['fright'] > d:
            state_description = 'case 1 - nothing'
            # # go towards the ball
            twist_msg = Twist()
            twist_msg.linear.x = linear_x
            twist_msg.angular.z = angular_z
            self.vel_pub.publish(twist_msg)

        elif regions['front'] < d0 and regions['fleft'] > d and regions['fright'] > d:
            state_description = 'case 2 - front'
            # turn
            twist_msg = Twist()
            twist_msg.linear.x = 0
            twist_msg.angular.z = 0.3
            self.vel_pub.publish(twist_msg)

        elif regions['front'] > d0 and regions['fleft'] > d and regions['fright'] < d:
            state_description = 'case 3 - fright'
            # turn left a little
            twist_msg = Twist()
            twist_msg.linear.x = 0
            twist_msg.angular.z = 0.3
            self.vel_pub.publish(twist_msg)

        elif regions['front'] > d0 and regions['fleft'] < d and regions['fright'] > d:
            state_description = 'case 4 - fleft'
            # turn right a little
            twist_msg = Twist()
            twist_msg.linear.x = 0
            twist_msg.angular.z = -0.3
            self.vel_pub.publish(twist_msg)
            
        elif regions['front'] < d0 and regions['fleft'] > d and regions['fright'] < d:
            state_description = 'case 5 - front and fright'
            # turn right a little
            twist_msg = Twist()
            twist_msg.linear.x = 0
            twist_msg.angular.z = 0.3
            self.vel_pub.publish(twist_msg)

        elif regions['front'] < d0 and regions['fleft'] < d and regions['fright'] > d:
            state_description = 'case 6 - front and fleft'
            # turn right a little
            twist_msg = Twist()
            twist_msg.linear.x = 0
            twist_msg.angular.z = -0.3
            self.vel_pub.publish(twist_msg)

        elif regions['front'] < d0 and regions['fleft'] < d and regions['fright'] < d:
            state_description = 'case 7 - front and fleft and fright'
            # go towards the ball
            twist_msg = Twist()
            twist_msg.angular.z = linear_x
            twist_msg.linear.x = angular_z
            self.vel_pub.publish(twist_msg)

        elif regions['front'] > d0 and regions['fleft'] < d and regions['fright'] < d:
            state_description = 'case 8 - fleft and fright'
            # go towards the ball
            twist_msg = Twist()
            twist_msg.angular.z = linear_x
            twist_msg.linear.x = angular_z
            self.vel_pub.publish(twist_msg)
        else:
            state_description = 'unknown case'
            #rospy.loginfo(regions)
        #rospy.loginfo(state_description)

    ## method get_mask_colour
    #
    # method to decide which colour mask will be applied
    def get_mask_colour(self, maskGreen, maskBlack, maskRed, maskYellow, maskBlue, maskMagenta):
        sumGreen = np.sum(maskGreen)
        sumBlack = np.sum(maskBlack)
        sumRed = np.sum(maskRed)
        sumYellow = np.sum(maskYellow)
        sumBlue = np.sum(maskBlue)
        sumMagenta = np.sum(maskMagenta)

        # rospy.loginfo([sumGreen, sumBlack, sumRed, sumYellow, sumBlue, sumMagenta])
        sumArray = np.array([sumGreen, sumBlack, sumRed,
                            sumYellow, sumBlue, sumMagenta])
        max_ind = np.argmax(sumArray)

        # return the mask related to the colour with higher value of detection
        if max_ind == 0:
            return [maskGreen, 'Green']
        elif max_ind == 1:
            return [maskBlack, 'Black']
        elif max_ind == 2:
            return [maskRed, 'Red']
        elif max_ind == 3:
            return [maskYellow, 'Yellow']
        elif max_ind == 4:
            return [maskBlue, 'Blue']
        elif max_ind == 5:
            return [maskMagenta, 'Magenta']
        else:
            return [maskGreen, 'None']    # default (the masks are all zeroes)

    ## method follow_ball
    #
    # publish velocities to reach the ball
    def follow_ball(self):
        if self.near_ball:
            # if near enough to the ball start following it
            self.follow_avoid_obstacles()
        else:
            # if not near enough go towards the ball
            twist_msg = Twist()
            twist_msg.linear.x = 0.4
            self.vel_pub.publish(twist_msg)
        
    ## method callback
    #
    # Callback function of subscribed topic.
    # Here images get converted and features detected
    def callback(self, ros_data):

        if VERBOSE:
            print('received image of type: "%s"' % ros_data.format)

        angular_z = None
        linear_x = None
        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:

        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # create masks for all colours
        maskGreen = cv2.inRange(hsv, greenLower, greenUpper)
        maskBlack = cv2.inRange(hsv, blackLower, blackUpper)
        maskRed = cv2.inRange(hsv, redLower, redUpper)
        maskYellow = cv2.inRange(hsv, yellowLower, yellowUpper)
        maskBlue = cv2.inRange(hsv, blueLower, blueUpper)
        maskMagenta = cv2.inRange(hsv, magentaLower, magentaUpper)
        # choose the correct mask
        mask_colour = self.get_mask_colour(maskGreen, maskBlack, maskRed, maskYellow, maskBlue, maskMagenta)

        mask = cv2.erode(mask_colour[0], None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        # cv2.imshow('mask', mask)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        self.center = None

        # rospy.loginfo(mask_colour[1])

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            self.radius = radius
            M = cv2.moments(c)
            self.center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            
            self.ball_detected = True

            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(image_np, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(image_np, self.center, 5, (0, 0, 255), -1)

                self.near_ball = True
            else:
                self.near_ball = False
            
        else:
            self.ball_detected = False

        # publish if the ball has been detected
        # self.pubBall.publish(self.ball_detected)

        # update the points queue
        # pts.appendleft(center)
        cv2.imshow('window', image_np)
        cv2.waitKey(2)

        # if behaviour is track_normal, go near the ball
        if self.behaviour == "track_normal" or self.behaviour == "track_find":
            if self.ball_detected or self.near_ball:
                angular_z = -0.003*(self.center[0] - 400)
                linear_x = -0.01*(self.radius - 100)
                # if the robot is almost still (in front of the ball)
                if abs(angular_z) < 0.02 and abs(linear_x) < 0.02:
                    # signal that the ball has been reached
                    self.ball_reached = True
                    self.colour = mask_colour[1]
                    
                    # save information about ball position
                    self.save_info(self.colour)
                    
                    rospy.sleep(2)
                    # rospy.loginfo("The ball has been reached!")
                    if self.behaviour == "track_find":
                        # get the colour of the room that we want to track
                        room_colour = rospy.get_param(self.room)
                        if self.colour == room_colour:
                            rospy.loginfo("The %s (%s) has been found! Returning to the PLAY behaviour...", self.room, room_colour)
                            self.pub_room_found.publish(True)
                        else:
                            rospy.loginfo("The %s (%s) has not been found! Returning to the FIND behaviour", self.room, room_colour)
                            self.pub_room_found.publish(False)

                    # publish that the ball has been reached
                    self.pub_reach.publish(self.ball_reached)

                    # reinitialize variable
                    self.ball_reached = False

                # else follow the ball
                else: 
                    # track the ball 
                    self.follow_ball()
            
        
        # publish if the ball has been detected
        if mask_colour[1] != self.colour:
            self.pub_ball.publish(self.ball_detected)
        else:
            self.pub_ball.publish(False)


    ## method save_info
    #
    # Saves the position of the tracked ball
    def save_info(self, colour):
        rospy.loginfo("Saving position of the %s ball", colour)
       
        ball_pos = [self.current_pos.x, self.current_pos.y]
        rospy.loginfo("Ball Position: %s", str(ball_pos))

        rospy.set_param(colour, ball_pos)


## function main
#
# init node and ball tracking class
def main(args):
    # initialize ball tracking node
    rospy.init_node('ball_tracking', anonymous=True)

    # Initializes class
    bt = ball_tracking()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
