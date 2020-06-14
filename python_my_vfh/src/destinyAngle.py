#!/usr/bin/env python

import rospy
import numpy as np
import copy
import math
from math import degrees, radians, atan2, pi
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3
from std_msgs.msg import Float32, Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

positionGoal = None
positionRobot = None
previusPosition = None

def goalAngle(goalAnglePublisher, yaw):
    global positionGoal
    global positionRobot
    if(positionGoal == None or positionRobot == None):
        return

    x = positionGoal.pose.position.x - positionRobot.pose.pose.position.x
    y = positionGoal.pose.position.y - positionRobot.pose.pose.position.y

    radians = 0
    if (x or y): 
        radians = atan2(y, x)
    else:
        radians = 0

    resultAngle = radians - yaw
    
    goalAnglePublisher.publish( resultAngle)

def robotPositionCallback(msg, goalAnglePublisher, markerPublisherTrackLine):
    global positionRobot
    positionRobot = copy.deepcopy(msg)

    # uhol otocenia
    q = positionRobot.pose.pose.orientation
    # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = atan2(siny_cosp, cosy_cosp)
    
    goalAngle(goalAnglePublisher, yaw)
    trackLine(positionRobot.pose.pose.position, markerPublisherTrackLine)


def robotGoalPositionCallback(msg):
    global positionGoal
    positionGoal = copy.deepcopy(msg)

count = 0
def trackLine(actualPosition, markerPublisherTrackLine):
    global previusPosition
    global count

    x = actualPosition.x
    y = actualPosition.y
    previusPosition.points.append(Point( x = x, y = y, z = 0 ))

    if (len(previusPosition.points) < 1) or (count > 1000):
        count = 0
        return

    count += 1
    markerPublisherTrackLine.publish(previusPosition)



def main():
    global previusPosition
    rospy.init_node('myOdometry')

    previusPosition = Marker(
          header = Header(
            frame_id = "world",
            stamp = rospy.Time.now()
          ),
          id = 500,
          type = Marker.LINE_STRIP,
          action = Marker.ADD,
          pose = Pose(
            position = Point( x = 0, y = 0, z = 0 ),
            orientation = Quaternion(0, 0, 0, 5)
          ),
          scale = Vector3( 0.1, 0.1, 0.1),
          color =  ColorRGBA( 0, 0, 0, 1)
    )

    robotName = rospy.get_param('robotName', '/robot0')


    goalAnglePublisher = rospy.Publisher('destinyAngle', Float32, queue_size=2)
    markerPublisherTrackLine = rospy.Publisher('visualizationMyVFHTrackLine', Marker, queue_size=0)
    
    robotGoalPositionCallback_lambda = lambda x: robotPositionCallback(x, goalAnglePublisher, markerPublisherTrackLine)
    robotPositionSub = rospy.Subscriber( robotName + '/odom', Odometry, robotGoalPositionCallback_lambda)
    
    robotGoalPositionSub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, robotGoalPositionCallback)

    rospy.spin()


if __name__ == "__main__":
    main()