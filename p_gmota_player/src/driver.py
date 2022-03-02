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
x_last = None
y_last = None
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
        print('Player name is ' + self.name)  # Print Player name

        # Get Game Parameters
        redTeam = rospy.get_param('/red_players')
        greenTeam = rospy.get_param('/green_players')
        blueTeam = rospy.get_param('/blue_players')

        # Print Player Information and assing their team and states
        if (self.name == redTeam[0]) or (self.name == redTeam[1]) or (self.name == redTeam[2]):
            print('Player team is Red')
            print('Player ' + self.name + ' hunts ' + str(greenTeam) + ' and runs from ' + str(blueTeam))
            self.team = "RED"
            self.prey = "GREEN"
            self.hunter = "BLUE"
            self.state = "hunt"
        elif (self.name == greenTeam[0]) or (self.name == greenTeam[1]) or (self.name == greenTeam[2]):
            print('Player team is Green')
            print('Player ' + self.name + ' hunts ' + str(blueTeam) + ' and runs from ' + str(redTeam))
            self.team = "GREEN"
            self.prey = "BLUE"
            self.hunter = "RED"
            self.state = "hunt"
        elif (self.name == blueTeam[0]) or (self.name == blueTeam[1]) or (self.name == blueTeam[2]):
            print('Player team is Blue')
            print('Player ' + self.name + ' hunts ' + str(redTeam) + ' and runs from ' + str(greenTeam))
            self.team = "BLUE"
            self.prey = "RED"
            self.hunter = "GREEN"
            self.state = "hunt"

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

        #Convert image to openCV
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")  # cv_image type: numpy.ndarray
        cv_image_g = copy.deepcopy(cv_image)
        
        #Convert to grey scale
        cv_grey = cv2.cvtColor(cv_image_g, cv2.COLOR_BGR2GRAY)

        # Image processing for Red team
        if self.team == "RED" and self.prey == "GREEN" and self.hunter == "BLUE":
            prey_cv_image = copy.deepcopy(cv_image)
            hunter_cv_image = copy.deepcopy(cv_image)
            prey_mask = cv2.inRange(prey_cv_image, (0, 100, 0), (0, 255, 0)) #green team
            hunter_mask = cv2.inRange(hunter_cv_image, (0, 0, 0), (255, 0, 0)) #blue team

        # Image processing for Blue team:
        elif self.team == "BLUE" and self.prey == "RED" and self.hunter == "GREEN":
            prey_cv_image = copy.deepcopy(cv_image)
            hunter_cv_image = copy.deepcopy(cv_image)
            prey_mask = cv2.inRange(prey_cv_image, (0, 0, 0), (0, 0, 255)) #red team
            hunter_mask = cv2.inRange(hunter_cv_image, (0, 100, 0), (0, 255, 0)) #green team
        
        # Image processing for Green team:
        elif self.team == "GREEN" and self.prey == "BLUE" and self.hunter == "RED":
            prey_cv_image = copy.deepcopy(cv_image)
            hunter_cv_image = copy.deepcopy(cv_image)
            prey_mask = cv2.inRange(prey_cv_image, (0, 0, 0), (255, 0, 0)) #blue team
            hunter_mask = cv2.inRange(hunter_cv_image, (0, 0, 0), (0, 0, 255)) #red team
            
        #Image processing for prey and hunter mask:
        #Colored Mask
        prey_img_processed = copy.deepcopy(cv_image)
        hunter_img_processed = copy.deepcopy(cv_image)
        prey_img_processed[np.logical_not(prey_mask)] = 0
        hunter_img_processed[np.logical_not(hunter_mask)] = 0
        
        #Kernel 5x5 filter
        kernel = np.ones((5, 5), np.uint8)

        #Dilation from both masks 
        prey_img_dilation = cv2.dilate(prey_img_processed, kernel, iterations=2)
        hunter_img_dilation = cv2.dilate(hunter_img_processed, kernel, iterations=2)
          
        #Convert both masks to grey img:
        prey_image_grey = cv2.cvtColor(prey_img_dilation, cv2.COLOR_BGR2GRAY)
        hunter_image_grey = cv2.cvtColor(hunter_img_dilation, cv2.COLOR_BGR2GRAY)
        
        #Thresholding from grey image:
        _, prey_thresh = cv2.threshold(prey_image_grey, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        _, hunter_thresh = cv2.threshold(hunter_image_grey, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        
        #Merge Prey with Hunter Mask:
        ph_img = cv2.add(prey_img_dilation, hunter_img_dilation)

        #Get connected components from threshold:
        prey_num_labels, prey_labels, prey_stats, prey_centroids = cv2.connectedComponentsWithStats(prey_thresh, 4, cv2.CV_32S)
        hunter_num_labels, hunter_labels, hunter_stats, hunter_centroids = cv2.connectedComponentsWithStats(hunter_thresh, 4, cv2.CV_32S)
        
        
        #Compare Max Area from Prey and Hunter Mask
        #Decide if state is hunting or running
        try:
            # Get Object Max Area Centroid
            prey_max_area = 0
            hunter_max_area = 0
            prey_max_area_Label = None
            hunter_max_area_Label = None
            
            for i in range(prey_num_labels):

                if i != 0 and prey_max_area < prey_stats[i, cv2.CC_STAT_AREA]:
                    p_x = prey_stats[i, cv2.CC_STAT_LEFT]
                    p_y = prey_stats[i, cv2.CC_STAT_TOP]
                    p_w = prey_stats[i, cv2.CC_STAT_WIDTH]
                    p_h = prey_stats[i, cv2.CC_STAT_HEIGHT]
                    p_area = prey_stats[i, cv2.CC_STAT_AREA]
                    prey_max_area_Label = i

                    if p_area > 10:
                        global x_last, y_last
                        to_delete = []
                        exists = False
                        (cX, cY) = prey_centroids[i]
                        cX, cY = int(cX), int(cY)
                        rect_img = cv2.rectangle(prey_thresh, (p_x, p_y), (p_x + p_w, p_y + p_h), (255, 255, 0), -1)
                         
                        # Draw Line on Centroid
                        x = int(prey_centroids[prey_max_area_Label, 0])
                        y = int(prey_centroids[prey_max_area_Label, 1])
                        if x_last is not None and y_last is not None:
                            # Cross On Centroid
                            cv2.line(rect_img, (x, y), (x_last + 5, y_last), (0, 0, 255), 1, cv2.LINE_4)
                            cv2.line(rect_img, (x, y), (x_last - 5, y_last), (0, 0, 255), 1, cv2.LINE_4)
                            cv2.line(rect_img, (x, y), (x_last, y_last + 5), (0, 0, 255), 1, cv2.LINE_4)
                            cv2.line(rect_img, (x, y), (x_last, y_last - 5), (0, 0, 255), 1, cv2.LINE_4)
                            cv2.imshow("Rect img", rect_img)
                        x_last = x
                        y_last = y
                        height, width, _ = cv_image.shape

                if prey_max_area_Label is not None:
                    mask2 = cv2.inRange(prey_labels, prey_max_area_Label, prey_max_area_Label)
                    mask2 = mask2.astype(bool)
                    cv_image2 = copy.deepcopy(cv_image)
                    if self.prey == "GREEN":
                        cv_image2[mask2] = (0, 255, 0)
                    elif self.prey == "RED":
                        cv_image2[mask2] = (0, 0, 255)
                    elif self.prey == "BLUE":
                        cv_image2[mask2] = (255, 0, 0)
              
            print(x)
            #Hunting
            if x == 150:
                self.angle = 0
            elif x > 150:
                self.angle = -1.5
            elif x < 150:
                self.angle = 1.5

            # Publish position of the target
            twist = Twist()
            twist.linear.x = 0.75
            twist.angular.z = self.angle
            self.publisher_command.publish(twist)
        finally:
            # print("No player detected")
            pass

        #Show image processing in output
        if show_windows == "true":
            #cv2.imshow("Prey Mask", prey_mask)
            #cv2.imshow("Prey Image Processed", prey_img_processed)
            cv2.imshow("Prey Image Dilated", prey_img_dilation)
            cv2.imshow("Prey-Hunter image", ph_img)
            #cv2.imshow("Hunter Mask", hunter_mask)
            #cv2.imshow("Hunter Image Processed", hunter_img_processed)
            #cv2.imshow("Hunter Image Dilated", hunter_img_dilation)
            
            #cv2.imshow("Camera Image", cv_image)
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
        # print('Sending Twist Command')

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

            self.speed = 0.5
            self.angle = math.atan2(y, x)

            self.random_goal_active = True
            print(self.name + 'Sending Random Goal')
        else:
            actualPosition['posX'] = lastPosition['posX']
            actualPosition['posY'] = lastPosition['posY']
            actualPosition['posZ'] = lastPosition['posZ']
            actualPosition['oriX'] = lastPosition['oriX']
            actualPosition['oriY'] = lastPosition['oriY']
            actualPosition['oriZ'] = lastPosition['oriZ']
            actualPosition['oriW'] = lastPosition['oriW']

            print(actualPosition['posX'])
            print(lastPosition['posX'])


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
