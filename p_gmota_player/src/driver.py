#!/usr/bin/python3


import copy
import math
import rospy
import tf2_ros
from geometry_msgs.msg import Twist, PoseStamped
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped


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

        # Print Player team
        if (self.name == redTeam[0]) or (self.name == redTeam[1]) or (self.name == redTeam[2]):
            print('Player team is Red')
        elif (self.name == greenTeam[0]) or (self.name == greenTeam[1]) or (self.name == greenTeam[2]):
            print('Player team is Green')
        elif (self.name == blueTeam[0]) or (self.name == blueTeam[1]) or (self.name == blueTeam[2]):
            print('Player team is Blue')

        self.publisher_command = rospy.Publisher('/' + self.name + '/cmd_vel', Twist, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.timer = rospy.Timer(rospy.Duration(0.1), self.sendCommandCallback)

        self.goal_subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goalReceivedCallback)

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
                'Could not transform goal from ' + msg.header.frame_id + ' to ' + target_frame + '. Will ignore this goal.')

    def driveStraight(self, minumum_speed=0.1, maximum_speed=1.5):
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

    # Start driving
    driver = Driver()

    rospy.spin()


if __name__ == '__main__':
    main()
