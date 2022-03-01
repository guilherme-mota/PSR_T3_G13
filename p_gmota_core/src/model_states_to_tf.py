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
    print('Received Model States Message!')

    # Get Game Parameters
    redTeam = rospy.get_param('/red_players')
    greenTeam = rospy.get_param('/green_players')
    blueTeam = rospy.get_param('/blue_players')

    br = tf2_ros.TransformBroadcaster()

    for player in redTeam + greenTeam + blueTeam:
        i = 0
        for name in models.name:
            if player == name:
                print(name)

                # Create and Populate my Transformation
                t = geometry_msgs.msg.TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = 'world'
                t.child_frame_id = '/' + name + '/odom'
                t.transform.translation.x = models.pose[i].position.x
                t.transform.translation.y = models.pose[i].position.y
                t.transform.translation.z = models.pose[i].position.z
                t.transform.rotation.x = models.pose[i].orientation.x
                t.transform.rotation.y = models.pose[i].orientation.y
                t.transform.rotation.z = models.pose[i].orientation.z
                t.transform.rotation.w = models.pose[i].orientation.w

                # Send Transformation
                br.sendTransform(t)

            i += 1


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
