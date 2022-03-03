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
import time
import random
import numpy as np
import colorama
from colorama import Fore, Back, Style
import cv2
import copy
import math
import rospy
import tf2_ros
from geometry_msgs.msg import Twist, PoseStamped
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from gazebo_msgs.msg import ModelState, ModelStates, ContactsState

# GLOBAL VARIABLES
# -----------------------------------------------------
prey_x_last = None
prey_y_last = None
hunter_x_last = None
hunter_y_last = None

show_windows = None
lastPosition = {'posX': 0, 'posY': 0, 'posZ': 0,
                'oriX': 0, 'oriY': 0, 'oriZ': 0, 'oriW': 0}
actualPosition = {'posX': 0, 'posY': 0, 'posZ': 0,
                  'oriX': 0, 'oriY': 0, 'oriZ': 0, 'oriW': 0}


class Driver:
    def __init__(self):
        global lastPosition, actualPosition
        self.goal = PoseStamped()
        self.goal_active = False
        self.random_goal_active = False
        self.angle = 0
        self.speed = 0

        self.name = rospy.get_name()
        self.name = self.name.strip('/')  # Remove initial /

        # Get Game Parameters
        redTeam = rospy.get_param('/red_players')
        greenTeam = rospy.get_param('/green_players')
        blueTeam = rospy.get_param('/blue_players')

        # Print Player Information
        if (self.name == redTeam[0]) or (self.name == redTeam[1]) or (self.name == redTeam[2]):
            print('Player name is ' + Fore.RED + self.name + Style.RESET_ALL)  # Print Player name
            print('Player team is' + Fore.RED + 'Red' + Style.RESET_ALL)
            print('Player ' + Fore.RED + self.name + Style.RESET_ALL + ' hunts ' + Fore.GREEN + str(greenTeam)
                  + Style.RESET_ALL + ' and runs from ' + Fore.BLUE + str(blueTeam) + Style.RESET_ALL)
            self.team = "RED"
            self.prey = "GREEN"
            self.hunter = "BLUE"
            self.state = "stop"
        elif (self.name == greenTeam[0]) or (self.name == greenTeam[1]) or (self.name == greenTeam[2]):
            print('Player name is ' + Fore.GREEN + self.name + Style.RESET_ALL)  # Print Player name
            print('Player team is' + Fore.GREEN + 'Green' + Style.RESET_ALL)
            print('Player ' + Fore.GREEN + self.name + Style.RESET_ALL + ' hunts ' + Fore.BLUE + str(blueTeam) +
                  Style.RESET_ALL + ' and runs from ' + Fore.RED + str(redTeam) + Style.RESET_ALL)
            self.team = "GREEN"
            self.prey = "BLUE"
            self.hunter = "RED"
            self.state = "stop"
        elif (self.name == blueTeam[0]) or (self.name == blueTeam[1]) or (self.name == blueTeam[2]):
            print('Player name is ' + Fore.BLUE + self.name + Style.RESET_ALL)  # Print Player name
            print('Player team is ' + Fore.BLUE + 'Blue' + Style.RESET_ALL)
            print('Player ' + Fore.BLUE + self.name + Style.RESET_ALL + ' hunts ' + Fore.RED + str(
                redTeam) + Style.RESET_ALL + ' and runs from ' + Fore.GREEN + str(greenTeam) + Style.RESET_ALL)
            self.team = "BLUE"
            self.prey = "RED"
            self.hunter = "GREEN"
            self.state = "stop"

        self.publisher_command = rospy.Publisher('/' + self.name + '/cmd_vel', Twist, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.timer = rospy.Timer(rospy.Duration(0.1), self.sendCommandCallback)

        self.goal_subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goalReceivedCallback)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/" + self.name + "/camera/rgb/image_raw", Image, self.callback)

        self.modelStates = rospy.Subscriber('/gazebo/model_states', ModelStates, self.updateActualPos)

        self.randomGoalTimer = rospy.Timer(rospy.Duration(3), self.sendRandomGoal)

    def callback(self, data):
        global show_windows

        # Convert image to openCV
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")  # cv_image type: numpy.ndarray
        cv_image_g = copy.deepcopy(cv_image)

        # Convert to grey scale
        cv_grey = cv2.cvtColor(cv_image_g, cv2.COLOR_BGR2GRAY)

        # Image processing for Red team
        if self.team == "RED" and self.prey == "GREEN" and self.hunter == "BLUE":
            prey_cv_image = copy.deepcopy(cv_image)
            hunter_cv_image = copy.deepcopy(cv_image)
            prey_mask = cv2.inRange(prey_cv_image, (0, 100, 0), (0, 255, 0))  # green team
            hunter_mask = cv2.inRange(hunter_cv_image, (0, 0, 0), (255, 0, 0))  # blue team

        # Image processing for Blue team:
        elif self.team == "BLUE" and self.prey == "RED" and self.hunter == "GREEN":
            prey_cv_image = copy.deepcopy(cv_image)
            hunter_cv_image = copy.deepcopy(cv_image)
            prey_mask = cv2.inRange(prey_cv_image, (0, 0, 0), (0, 0, 255))  # red team
            hunter_mask = cv2.inRange(hunter_cv_image, (0, 100, 0), (0, 255, 0))  # green team

        # Image processing for Green team:
        elif self.team == "GREEN" and self.prey == "BLUE" and self.hunter == "RED":
            prey_cv_image = copy.deepcopy(cv_image)
            hunter_cv_image = copy.deepcopy(cv_image)
            prey_mask = cv2.inRange(prey_cv_image, (0, 0, 0), (255, 0, 0))  # blue team
            hunter_mask = cv2.inRange(hunter_cv_image, (0, 0, 0), (0, 0, 255))  # red team

        # Image processing for prey and hunter mask:
        # Colored Mask
        prey_img_processed = copy.deepcopy(cv_image)
        hunter_img_processed = copy.deepcopy(cv_image)
        prey_img_processed[np.logical_not(prey_mask)] = 0
        hunter_img_processed[np.logical_not(hunter_mask)] = 0

        # Kernel 5x5 filter
        kernel = np.ones((5, 5), np.uint8)

        # Dilation from both masks
        prey_img_dilation = cv2.dilate(prey_img_processed, kernel, iterations=2)
        hunter_img_dilation = cv2.dilate(hunter_img_processed, kernel, iterations=2)

        # Convert both masks to grey img:
        prey_img_grey = cv2.cvtColor(prey_img_dilation, cv2.COLOR_BGR2GRAY)
        hunter_img_grey = cv2.cvtColor(hunter_img_dilation, cv2.COLOR_BGR2GRAY)

        # Closing operation from grey img:
        prey_img_closing = cv2.morphologyEx(prey_img_grey, cv2.MORPH_CLOSE, kernel)
        hunter_img_closing = cv2.morphologyEx(hunter_img_grey, cv2.MORPH_CLOSE, kernel)

        # Thresholding from grey image:
        _, prey_thresh = cv2.threshold(prey_img_closing, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        _, hunter_thresh = cv2.threshold(hunter_img_closing, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        # Merge Prey with Hunter Mask:
        ph_img = cv2.add(prey_img_dilation, hunter_img_dilation)

        # Get connected components from threshold:
        prey_num_labels, prey_labels, prey_stats, prey_centroids = cv2.connectedComponentsWithStats(prey_thresh, 4,
                                                                                                    cv2.CV_32S)
        hunter_num_labels, hunter_labels, hunter_stats, hunter_centroids = cv2.connectedComponentsWithStats(
            hunter_thresh, 4, cv2.CV_32S)

        # Compare Max Area from Prey and Hunter Mask
        # Decide if state is hunting or running

        # Get Object Max Area Centroid
        prey_max_area = 0
        hunter_max_area = 0
        prey_max_area_Label = None
        hunter_max_area_Label = None

        try:

            # Get Object Max Area from Prey
            for i in range(prey_num_labels):

                if i != 0 and prey_max_area < prey_stats[i, cv2.CC_STAT_AREA]:
                    p_x = prey_stats[i, cv2.CC_STAT_LEFT]
                    p_y = prey_stats[i, cv2.CC_STAT_TOP]
                    p_w = prey_stats[i, cv2.CC_STAT_WIDTH]
                    p_h = prey_stats[i, cv2.CC_STAT_HEIGHT]
                    prey_max_area = prey_stats[i, cv2.CC_STAT_AREA]
                    prey_max_area_Label = i

                    if prey_max_area_Label is not None:
                        global prey_x_last, prey_y_last
                        (prey_cX, prey_cY) = prey_centroids[i]
                        prey_cX, prey_cY = int(prey_cX), int(prey_cY)
                        prey_rect_img = cv2.rectangle(prey_thresh, (p_x, p_y), (p_x + p_w, p_y + p_h), (255, 255, 0),
                                                      -1)

                        # Draw Line on Centroid
                        prey_x = int(prey_centroids[prey_max_area_Label, 0])
                        prey_y = int(prey_centroids[prey_max_area_Label, 1])
                        if prey_x_last is not None and prey_y_last is not None:

                            p_dist = ((prey_x - prey_x_last) ** 2 + (prey_y - prey_y_last) ** 2) ** (1 / 2)
                            if p_dist < 10:
                                # Cross On Centroid
                                cv2.line(prey_rect_img, (prey_x, prey_y), (prey_x_last + 5, prey_y_last), (0, 0, 255),
                                         1, cv2.LINE_4)
                                cv2.line(prey_rect_img, (prey_x, prey_y), (prey_x_last - 5, prey_y_last), (0, 0, 255),
                                         1, cv2.LINE_4)
                                cv2.line(prey_rect_img, (prey_x, prey_y), (prey_x_last, prey_y_last + 5), (0, 0, 255),
                                         1, cv2.LINE_4)
                                cv2.line(prey_rect_img, (prey_x, prey_y), (prey_x_last, prey_y_last - 5), (0, 0, 255),
                                         1, cv2.LINE_4)

                            else:
                                cv2.line(prey_rect_img, (prey_x, prey_y), (prey_x + 5, prey_y), (0, 0, 255), 1,
                                         cv2.LINE_4)
                                cv2.line(prey_rect_img, (prey_x, prey_y), (prey_x - 5, prey_y), (0, 0, 255), 1,
                                         cv2.LINE_4)
                                cv2.line(prey_rect_img, (prey_x, prey_y), (prey_x, prey_y + 5), (0, 0, 255), 1,
                                         cv2.LINE_4)
                                cv2.line(prey_rect_img, (prey_x, prey_y), (prey_x, prey_y - 5), (0, 0, 255), 1,
                                         cv2.LINE_4)

                                # cv2.imshow("Prey Area Image", prey_rect_img)
                        prey_x_last = prey_x
                        prey_y_last = prey_y
                        height, width, _ = cv_image.shape

                        # Hunting
                        if prey_x == 150:
                            self.angle = 0
                        elif prey_x > 150:
                            self.angle = -1.5
                        elif prey_x < 150:
                            self.angle = 1.5

            # Get Max Area from Hunting
            for j in range(hunter_num_labels):

                if j != 0 and hunter_max_area < hunter_stats[j, cv2.CC_STAT_AREA]:
                    h_x = hunter_stats[j, cv2.CC_STAT_LEFT]
                    h_y = hunter_stats[j, cv2.CC_STAT_TOP]
                    h_w = hunter_stats[j, cv2.CC_STAT_WIDTH]
                    h_h = hunter_stats[j, cv2.CC_STAT_HEIGHT]
                    hunter_max_area = hunter_stats[j, cv2.CC_STAT_AREA]
                    hunter_max_area_Label = j

                    if hunter_max_area_Label is not None:
                        global hunter_x_last, hunter_y_last
                        (hunter_cX, hunter_cY) = hunter_centroids[j]
                        hunter_cX, hunter_cY = int(hunter_cX), int(hunter_cY)
                        hunter_rect_img = cv2.rectangle(hunter_thresh, (h_x, h_y), (h_x + h_w, h_y + h_h),
                                                        (255, 255, 255), -1)

                        # Draw Line on Centroid
                        hunter_x = int(hunter_centroids[hunter_max_area_Label, 0])
                        hunter_y = int(hunter_centroids[hunter_max_area_Label, 1])
                        if hunter_x_last is not None and hunter_y_last is not None:

                            h_dist = ((hunter_x - hunter_x_last) ** 2 + (hunter_y - hunter_y_last) ** 2) ** (1 / 2)
                            if h_dist < 10:
                                # Cross On Centroid
                                cv2.line(hunter_rect_img, (hunter_x, hunter_y), (hunter_x_last + 5, hunter_y_last),
                                         (0, 0, 0), 1, cv2.LINE_4)
                                cv2.line(hunter_rect_img, (hunter_x, hunter_y), (hunter_x_last - 5, hunter_y_last),
                                         (0, 0, 0), 1, cv2.LINE_4)
                                cv2.line(hunter_rect_img, (hunter_x, hunter_y), (hunter_x_last, hunter_y_last + 5),
                                         (0, 0, 0), 1, cv2.LINE_4)
                                cv2.line(hunter_rect_img, (hunter_x, hunter_y), (hunter_x_last, hunter_y_last - 5),
                                         (0, 0, 0), 1, cv2.LINE_4)

                            else:
                                cv2.line(hunter_rect_img, (hunter_x, hunter_y), (hunter_x + 5, hunter_y), (0, 0, 0), 1,
                                         cv2.LINE_4)
                                cv2.line(hunter_rect_img, (hunter_x, hunter_y), (hunter_x - 5, hunter_y), (0, 0, 0), 1,
                                         cv2.LINE_4)
                                cv2.line(hunter_rect_img, (hunter_x, hunter_y), (hunter_x, hunter_y + 5), (0, 0, 0), 1,
                                         cv2.LINE_4)
                                cv2.line(hunter_rect_img, (hunter_x, hunter_y), (hunter_x, hunter_y - 5), (0, 0, 0), 1,
                                         cv2.LINE_4)

                                # cv2.imshow("Hunter Area  img", hunter_rect_img)
                        hunter_x_last = hunter_x
                        hunter_y_last = hunter_y
                        height, width, _ = cv_image.shape

                        # Running
                        if hunter_x == 150:
                            self.angle = 0
                        elif hunter_x > 150:
                            self.angle = -1.5
                        elif hunter_x < 150:
                            self.angle = 1.5

            self.state = "stop"  # Reset State

            if prey_max_area > hunter_max_area:
                # Hunting
                self.state = "hunting"
                self.goal_active = False
                twist = Twist()
                twist.linear.x = 0.75
                twist.angular.z = self.angle
                self.publisher_command.publish(twist)
            elif prey_max_area < hunter_max_area:
                # Running
                self.state = "running"
                self.goal_active = False
                twist = Twist()
                twist.linear.x = -0.75
                twist.angular.z = self.angle
                self.publisher_command.publish(twist)
            elif prey_max_area_Label is None and hunter_max_area_Label is None:
                if self.state is not "hunting" and self.state is not "running" and not self.goal_active:
                    twist = Twist()
                    twist.linear.x = 0.3
                    x = random.random() * 16 - 8
                    y = random.random() * 5 - 2.5
                    twist.angular.z = math.atan2(y, x)
                    self.publisher_command.publish(twist)


        finally:
            pass

        # Show image processing in output
        if show_windows == "true":
            cv2.imshow("Prey Image Dilated", prey_img_dilation)
            cv2.imshow("Hunter Image Dilated", hunter_img_dilation)
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

        goal_in_base_link = self.tf_buffer.transform(goal_copy, self.name + '/base_footprint', rospy.Duration(1))

        x = goal_in_base_link.pose.position.x
        y = goal_in_base_link.pose.position.y

        self.angle = math.atan2(y, x)
        self.speed = 0.5

    def sendCommandCallback(self, event):
        if not self.goal_active and not self.random_goal_active:  # no goal, no movement
            self.angle = 0
            self.speed = 0
        elif self.goal_active and not self.random_goal_active:
            self.driveStraight()

        twist = Twist()
        twist.linear.x = self.speed
        twist.angular.z = self.angle
        self.publisher_command.publish(twist)

    def updateActualPos(self, models):
        global lastPosition, actualPosition
        rospy.sleep(1)
        i = 0
        for name in models.name:
            if self.name == name:
                actualPosition['posX'] = models.pose[i].position.x
                actualPosition['posY'] = models.pose[i].position.y
                actualPosition['posZ'] = models.pose[i].position.z
                actualPosition['oriX'] = models.pose[i].orientation.x
                actualPosition['oriY'] = models.pose[i].orientation.y
                actualPosition['oriZ'] = models.pose[i].orientation.z
                actualPosition['oriW'] = models.pose[i].orientation.w
            i += 1

    def sendRandomGoal(self, event):
        global lastPosition, actualPosition
        if (actualPosition['posX'] == lastPosition['posX'] and
                actualPosition['posY'] == lastPosition['posY'] and
                actualPosition['posZ'] == lastPosition['posZ'] and
                actualPosition['oriX'] == lastPosition['oriX'] and
                actualPosition['oriY'] == lastPosition['oriY'] and
                actualPosition['oriZ'] == lastPosition['oriZ'] and
                actualPosition['oriW'] == lastPosition['oriW']):

            x = random.random() * 16 - 8
            y = random.random() * 5 - 2.5

            # Publish position of the target
            self.random_goal_active = True
            self.speed = 0.5
            self.angle = math.atan2(y, x)
        else:
            lastPosition['posX'] = actualPosition['posX']
            lastPosition['posY'] = actualPosition['posY']
            lastPosition['posZ'] = actualPosition['posZ']
            lastPosition['oriX'] = actualPosition['oriX']
            lastPosition['oriY'] = actualPosition['oriY']
            lastPosition['oriZ'] = actualPosition['oriZ']
            lastPosition['oriW'] = actualPosition['oriW']

            self.random_goal_active = False


def main():
    # ----------------------------
    # INITIALIZATION
    # ----------------------------
    rospy.init_node('p_gmota_driver', anonymous=False)

    rospy.sleep(0.2)  # make sure the rospy time works

    global show_windows
    args = rospy.myargv(argv=sys.argv)
    show_windows = args[1]

    # Start driving
    driver = Driver()

    rospy.spin()


if __name__ == '__main__':
    main()
