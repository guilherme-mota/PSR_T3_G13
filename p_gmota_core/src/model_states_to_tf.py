#!/usr/bin/python3


# ------------------------
#   IMPORTS
# ------------------------
import rospy
import std_msgs.msg
from std_msgs.msg import String
from sensor_msgs import point_cloud2
from math import sin, cos
import tf_conversions  # Because of transformations
import tf2_ros
import geometry_msgs.msg
from gazebo_msgs.msg import ModelState, ModelStates, ContactsState


def callbackMsgReceived(models):
    rospy.loginfo('Received Model States Message! ')

    # Get Game Parameters
    redTeam = rospy.get_param('/red_players')
    greenTeam = rospy.get_param('/green_players')
    blueTeam = rospy.get_param('/blue_players')

    for player in redTeam + greenTeam + blueTeam:
        for name in models.name:
            if player == name:
                print(name)


def main():
    # ----------------------------
    # INITIALIZATION
    # ----------------------------
    rospy.init_node('model_states_to_tf', anonymous=False)
    rospy.Subscriber('/gazebo/model_states', ModelStates, callbackMsgReceived)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    main()
