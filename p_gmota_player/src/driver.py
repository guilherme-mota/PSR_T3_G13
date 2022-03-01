#!/usr/bin/python3

# -------------------------------------------------------------------------------
# Name:        Driver
# Purpose:     Drive turtleboot
# Author:      Grupo 13
# Created:     01/03/2022
# -------------------------------------------------------------------------------

# ------------------------
#   IMPORTS
# ------------------------
import sys
import numpy as np
import cv2
import copy
import math
import rospy
import tf2_ros
from geometry_msgs.msg import Twist, PoseStamped
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# GLOBAL VARIABLES
# -----------------------------------------------------
x_last = None
y_last = None
show_windows = None


class Driver:
    def __init__(self):
        self.goal = PoseStamped()
        self.goal_active = False

        self.angle = 0
        self.speed = 0

        self.name = rospy.get_name()
        self.name = self.name.strip('/')  # Remove initial /
        print('Player name is ' + self.name)  # Print Player name

        # Get Game Parameters
        redTeam = rospy.get_param('/red_players')
        greenTeam = rospy.get_param('/green_players')
        blueTeam = rospy.get_param('/blue_players')

        # Print Player Information
        if (self.name == redTeam[0]) or (self.name == redTeam[1]) or (self.name == redTeam[2]):
            print('Player team is Red')
            print('Player ' + self.name + ' hunts ' + str(greenTeam) + ' and runs from ' + str(blueTeam))
            self.prey = "GREEN"
        elif (self.name == greenTeam[0]) or (self.name == greenTeam[1]) or (self.name == greenTeam[2]):
            print('Player team is Green')
            print('Player ' + self.name + ' hunts ' + str(blueTeam) + ' and runs from ' + str(redTeam))
            self.prey = "BLUE"
        elif (self.name == blueTeam[0]) or (self.name == blueTeam[1]) or (self.name == blueTeam[2]):
            print('Player team is Blue')
            print('Player ' + self.name + ' hunts ' + str(redTeam) + ' and runs from ' + str(greenTeam))
            self.prey = "RED"

        self.publisher_command = rospy.Publisher('/' + self.name + '/cmd_vel', Twist, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.timer = rospy.Timer(rospy.Duration(0.1), self.sendCommandCallback)

        self.goal_subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goalReceivedCallback)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/" + self.name + "/camera/rgb/image_raw", Image, self.callback)

    def callback(self, data):
        global show_windows

        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")  # cv_image type: numpy.ndarray

        if self.prey == "GREEN":
            mask = cv2.inRange(cv_image, (0, 100, 0), (0, 255, 0))
        elif self.prey == "RED":
            mask = cv2.inRange(cv_image, (0, 0, 0), (0, 0, 255))
        elif self.prey == "BLUE":
            mask = cv2.inRange(cv_image, (0, 0, 0), (255, 0, 0))

        image_processed = copy.deepcopy(cv_image)
        image_processed[np.logical_not(mask)] = 0

        kernel = np.ones((5, 5), np.uint8)
        img_dilation = cv2.dilate(image_processed, kernel, iterations=2)

        if show_windows:
            cv2.imshow("Mask", mask)
            cv2.imshow("Image Processed", image_processed)
            cv2.imshow("Image Dilated", img_dilation)

        # Get Object
        image_grey = cv2.cvtColor(image_processed, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(image_grey, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(thresh, 4, cv2.CV_32S)

        try:
            # Get Object Max Area Centroid
            max_area = 0
            max_area_Label = None
            for i in range(num_labels):
                if i != 0 and max_area < stats[i, cv2.CC_STAT_AREA]:
                    max_area = stats[i, cv2.CC_STAT_AREA]
                    max_area_Label = i

            if max_area_Label is not None:
                mask2 = cv2.inRange(labels, max_area_Label, max_area_Label)
                mask2 = mask2.astype(bool)
                if self.prey == "GREEN":
                    cv_image[mask2] = (0, 255, 0)
                elif self.prey == "RED":
                    cv_image[mask2] = (0, 0, 255)
                elif self.prey == "BLUE":
                    cv_image[mask2] = (255, 0, 0)

                # Draw Line on Centroid
                global x_last, y_last
                x = int(centroids[max_area_Label, 0])
                y = int(centroids[max_area_Label, 1])
                if x_last is not None and y_last is not None:
                    # Cross On Centroid
                    cv2.line(cv_image, (x, y), (x_last + 5, y_last), (0, 0, 255), 1, cv2.LINE_4)
                    cv2.line(cv_image, (x, y), (x_last - 5, y_last), (0, 0, 255), 1, cv2.LINE_4)
                    cv2.line(cv_image, (x, y), (x_last, y_last + 5), (0, 0, 255), 1, cv2.LINE_4)
                    cv2.line(cv_image, (x, y), (x_last, y_last - 5), (0, 0, 255), 1, cv2.LINE_4)
                x_last = x
                y_last = y

                height, width, _ = cv_image.shape

                if x == 150:
                    self.angle = 0
                elif x > 150:
                    self.angle = -1.00
                elif x < 150:
                    self.angle = 1.00

                # Publish position of the target
                twist = Twist()
                twist.linear.x = 0.6
                twist.angular.z = self.angle
                self.publisher_command.publish(twist)
        finally:
            print("No player detected")

        if show_windows:
            cv2.imshow("Camera Image", cv_image)

        cv2.waitKey(3)

    def goalReceivedCallback(self, msg):
        print('Received New Goal on Frame ID ' + msg.header.frame_id)
        target_frame = self.name + '/odom'
        try:
            self.goal = self.tf_buffer.transform(msg, target_frame, rospy.Duration(1))
            self.goal_active = True
            rospy.logwarn('Setting new goal')
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.goal_active = False
            rospy.logerr(
                'Could not transform goal from ' + msg.header.frame_id + ' to ' + target_frame + '. Will ignore this '
                                                                                                 'goal.')

    def driveStraight(self):
        goal_copy = copy.deepcopy(self.goal)  # make sure we don't change the stamp field of the goa
        goal_copy.header.stamp = rospy.Time.now()

        print('Transforming pose')
        goal_in_base_link = self.tf_buffer.transform(goal_copy, self.name + '/base_footprint', rospy.Duration(1))
        print('Pose transformed')

        x = goal_in_base_link.pose.position.x
        y = goal_in_base_link.pose.position.y

        self.angle = math.atan2(y, x)
        self.speed = 0.5

    def sendCommandCallback(self, event):
        print('Sending Twist Command')

        if not self.goal_active:  # no goal, no movement
            self.angle = 0
            self.speed = 0
        else:
            self.driveStraight()

        twist = Twist()
        twist.linear.x = self.speed
        twist.angular.z = self.angle
        self.publisher_command.publish(twist)


def main():
    # ----------------------------
    # INITIALIZATION
    # ----------------------------
    rospy.init_node('p_gmota_driver', anonymous=False)

    args = rospy.myargv(argv=sys.argv)

    global show_windows
    show_windows = args[1]
    print(show_windows)

    # Start driving
    driver = Driver()

    rospy.spin()


if __name__ == '__main__':
    main()
