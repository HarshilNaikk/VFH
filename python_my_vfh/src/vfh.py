#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from math import pi, cos, sin, floor, tan, radians, sqrt, atan2, atan
import numpy as np
import time
import matplotlib.pyplot as plt
from python_my_vfh.msg import plotH
import copy

from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3, Twist, PoseStamped
from std_msgs.msg import Header, ColorRGBA, Float32MultiArray, Float32

class configVFH:
  #config
  name            = None
  sizeGrid        = None
  sizeSquare      = None
  numberOfSector  = None
  robotRadius     = None
  safeZone        = None
  Smax            = None
  Tlow            = None
  Thigh           = None
  rotation        = None
  velocity        = None
  goalAngle       = None
  mi1             = None
  mi2             = None
  mi3             = None

  #private
  __globalGoalPoint__     = None
  __globalRobotOdometry__ = None

  # Markers
  markerPublisherSektor           = None
  markerPublisherSektorBinary     = None
  markerPublisherSektorMask       = None
  markerPublisherSectorMagnitude  = None
  markerPublisherRobotCMD         = None
  markerPublisherGoalLine         = None
  markerPublisherICR              = None

class vfhSector:
  def __init__(self):
    self.h = 0.0
    self.binary = True
    self.mask = True

  def __str__(self):
    print("H ", self.h)
    print("Binary ", self.binary)
    print("Mask ", self.mask)

class vfhNode:
  def __init__(self, x , y, angle):
    self.magnitude = float()
    self.x = x
    self.y = y
    self.angle = angle

  def distance(self, deltaX, deltaY):
    one = (abs(self.x) - deltaX)**2
    two = (abs(self.y) - deltaY)**2
    return sqrt(one + two)

class openingNode:
  score = 0

  def __init__(self, Kl, Kr):
    self.Kl = Kl
    self.Kr = Kr

  def calculateScore(self, targetSector, previouslySector, configVFH):
    distance = distanceSector(self.Kl, self.Kr, configVFH)
    selectedSector = targetSector
    #print("targetSector ", targetSector)
    if distance <= configVFH.Smax:
      #print("1.")
      #print("Kl ", self.Kl)
      #print("Kr ", self.Kr)
      selectedSector = selectSector(self.Kl, self.Kr, configVFH)
      self.score = gfunction(selectedSector, targetSector, previouslySector, configVFH)

      #print("selectedSector ", selectedSector)

    else:
      #print("2.")
      cl = round_down(self.Kl - (configVFH.Smax/2)) % configVFH.numberOfSector 
      cr = round_down(self.Kr + (configVFH.Smax/2)) % configVFH.numberOfSector
      #print("Kl ", self.Kl)
      #print("Kr ", self.Kr)
      #print("cl ", cl)
      #print("cr ", cr)
      
      firstSelectedSector = selectSector(self.Kl, cl, configVFH)
      secondSelectedSector = selectSector(cr, self.Kr, configVFH)
      #print("firstSelectedSector", firstSelectedSector)
      #print("secondSelectedSector", secondSelectedSector)

      firstScore = gfunction(firstSelectedSector, targetSector, previouslySector, configVFH)
      secondScore = gfunction(secondSelectedSector, targetSector, previouslySector, configVFH)
      #print("firstScore ", firstScore)
      #print("secondScore ", secondScore)

      temp1 = (cl > targetSector) and (targetSector > cr) and (cl > cr)
      temp2 = (cr > cl) and ((cl > targetSector) or (cr < targetSector))
      #print("temp1", temp1)
      #print("temp2", temp2)
      if (temp1 or temp2):
        targetScore = gfunction(targetSector, targetSector, previouslySector, configVFH)
        if targetScore <= firstScore and targetScore <= secondScore:
          #print("Target !!! ", targetScore, " ", targetSector)
          return targetScore, targetSector 

        else:
          print("Target score nie je najlepsie :", targetScore)

      if firstScore <= secondScore:
        self.score = firstScore
        selectedSector = firstSelectedSector
        Kr = cl
      else:
        self.score = secondScore
        selectedSector = secondSelectedSector
        Kl = cr
    
    #print("Best Score ", self.score)
    return self.score, selectedSector

  

previouslySector = 0

def laserScanSubscribeCallback(msg, configVFH):
  irc = configVFH.velocity/configVFH.rotation

  drawIRC(irc, configVFH)

  if (configVFH.__globalGoalPoint__ == None) or (configVFH.__globalGoalPoint__ == None):
    return

  calculateGoalAngle(configVFH)

  laserData = np.copy(msg.ranges)

  middleSizeGrid = (configVFH.sizeGrid/2)
  gridRange = configVFH.sizeSquare * middleSizeGrid
  sizeSector = radians(360.0/configVFH.numberOfSector)
  
  listOfNode = []
  listOfSector = [vfhSector() for x in range(configVFH.numberOfSector)]

  # Prevedenie dat do mriezky
  for i, val in enumerate(laserData):
    if(val == np.inf):
      continue

    angle = float(msg.angle_min + (msg.angle_increment * i))
    if angle < 0:
      angle += 2*pi

    x = abs(laserData[i]) * cos(angle)
    y = abs(laserData[i]) * sin(angle)

    # out of range
    if abs(x) > gridRange or abs(y) > gridRange or ( x == 0 and y == 0):
      continue
  
    gridX = round_down(x/configVFH.sizeSquare)*configVFH.sizeSquare
    gridY = round_down(y/configVFH.sizeSquare)*configVFH.sizeSquare

    filterItem = next(iter(filter(lambda item: (item.x == gridX) and (item.y == gridY), listOfNode)), None)
    
    if filterItem == None:
      listOfNode.append( vfhNode( gridX, gridY, angle))
      listOfNode[-1].magnitude = 1
    else:
      filterItem.magnitude += 1.0


  ## First State
  # Vypocet konstant pre nulovy vpli okrajovych buniek
  dMax = (configVFH.sizeGrid - 1)/2
  rRS = configVFH.robotRadius + configVFH.safeZone
  a = dMax / (dMax - rRS)
  b = a / dMax

  for node in listOfNode:
    distance = node.distance(0, 0)
    if distance <= rRS:
      distance = rRS

    enlargementAngle = np.arcsin(rRS/distance)

    i = 0
    if distance > dMax:
      continue
    elif distance < rRS:
      i = 1
    else:
      i = a - b*distance

    node.magnitude = (node.magnitude**2)*i
    enlargementSectorLeft = angleToSector(node.angle + enlargementAngle, sizeSector)
    enlargementSectorRight = angleToSector(node.angle - enlargementAngle, sizeSector)
    temp = enlargementSectorRight
    while temp != (enlargementSectorLeft+1)%configVFH.numberOfSector:
      listOfSector[temp].h += node.magnitude
      temp = (temp+1)%configVFH.numberOfSector
    
    listOfSector[enlargementSectorLeft].h += node.magnitude

  drawSectors(listOfSector, configVFH)
  
  for sector in listOfSector:
    print(sector.h)
  #drawSectors(listOfSector, configVFH)
  #drawSectorsMagnitude(listOfSector, configVFH)

  ## Second State
  clear = True
  for i, sector in enumerate(listOfSector):
    if sector.h < configVFH.Tlow:
      sector.binary = False
    elif sector.h > configVFH.Thigh:
      sector.binary = True
      clear = False
    else:
      sector.binary = listOfSector[i-1].binary
      clear = False

  drawSectorsBinary(listOfSector, configVFH)

  ## Third State
  robotAngle = pi/2

  # ICR
  # left +, right -, y = rl
  listOfOpening = []
  velocity = configVFH.velocity
  rotation = configVFH.rotation
  while(True):
    velocity /=1.001
    rotation *=1.001
    #print("New velocity ", velocity)
    #print("New rotation ", rotation)

    irc = velocity/rotation

    drawIRC(irc, configVFH)

    goalAngle = configVFH.goalAngle
    if goalAngle < 0:
      goalAngle += 2*pi

    fiB = pi
    fiR = fiB
    fiL = fiB

    targetSector = angleToSector(goalAngle, sizeSector)
    for node in listOfNode:
      d = node.distance(0, irc) 
      if ((d) <= (rRS + irc)):
        if (node.angle <= fiL) and node.angle <= pi:
          fiL = node.angle
        elif (node.angle >= fiR) and node.angle >= pi:
          fiR = node.angle

    if fiL > pi:
      fiL = pi
    if fiR < pi:
      fiR = pi

    fiLSector = angleToSector(fiL, sizeSector)
    fiRSector = angleToSector(fiR, sizeSector)
  
    block = True
    for i, sector in enumerate(listOfSector):
      if sector.binary == False:
        if (i >= fiRSector) or (fiLSector >= i): 
          sector.mask = False
          block = False
        else:
          sector.mask = True

    if block == False:
      break

  global previouslySector

  temp = 0
  while(not clear):
    Kr = findREdge(listOfSector, temp, configVFH)
    Kl = findLEdge(listOfSector, Kr, configVFH)
    temp = Kl
    filterItem = next(iter(filter(lambda item: (item.Kl == Kl) and (item.Kr == Kr), listOfOpening)), None)
    if filterItem == None:
      listOfOpening.append( openingNode(Kl, Kr))
    else:
      break

  print(len(listOfOpening))
  bestScore = 10000
  bestSector = targetSector
  for opening in listOfOpening:
    score, sector = opening.calculateScore( targetSector, previouslySector, configVFH)
    if score < bestScore:
      bestScore = score
      bestSector = sector
  
  drawSectorsMask(listOfSector, bestSector, configVFH)
  selectedAngle = bestSector * sizeSector
  if selectedAngle > pi:
    selectedAngle -= 2*pi

  print("Angle ", selectedAngle)
  
  twist = Twist()
  if abs(selectedAngle) > configVFH.rotation:
    if selectedAngle > 0:
      selectedAngle  = configVFH.rotation
    else:
      selectedAngle  = configVFH.rotation*(-1)
  twist.angular.z = selectedAngle
  twist.linear.x = velocity
  configVFH.markerPublisherRobotCMD.publish(twist)
  


#############################################################################x  

def drawIRC(irc, configVFH):
  marker = Marker(
          header = Header(
            frame_id = configVFH.robotName,
            stamp = rospy.Time.now()
          ),
          id = 500,
          type = Marker.LINE_STRIP,
          action = Marker.ADD,
          pose = Pose(
            position = Point( x = 0, y = 0, z = 0 ),
            orientation = Quaternion(0, 0, 0, 5)
          ),
          scale = Vector3( 0.05, 0.05, 0.05),
          color =  ColorRGBA( 0, 0, 0, 1)
  )

  number = 100
  piece = 2*pi/number
  for i in range(number):
    x = irc * sin(i * piece)
    y = irc * cos(i * piece)
    marker.points.append( Point( x = x, y = y + irc, z = 0 ))
  configVFH.markerPublisherICR.publish(marker)

  for i in range(number):
    x = irc * sin(i * piece)
    y = irc * cos(i * piece)
    marker.points.append( Point( x = x, y = y - irc, z = 0 ))

  configVFH.markerPublisherICR.publish(marker)

def findREdge(listOfSector, startPosition, configVFH):
  for i in range(0, configVFH.numberOfSector):
    index = (startPosition + i) % configVFH.numberOfSector
    if (listOfSector[index].mask == False) and (listOfSector[index-1].mask == True):
      return index
  return -1

def findLEdge(listOfSector, startPosition, configVFH):
  for i in range(0, configVFH.numberOfSector):
    index = (startPosition + i) % configVFH.numberOfSector
    if (listOfSector[index].mask == True) and (listOfSector[index-1].mask == False):
      return (index - 1) % configVFH.numberOfSector
  return -1

def selectSector(leftSector, rightSector, configVFH):
  if rightSector > leftSector:
    selectSector = (rightSector + round_down((configVFH.numberOfSector + leftSector - rightSector)/2))%configVFH.numberOfSector
  else:
    selectSector = round_down(leftSector + rightSector)/2

  return selectSector

def gfunction(c, targetSector, previouslySector, configVFH):
  firstPart = configVFH.mi1 * minSector(c, targetSector, configVFH)
  secondPart = configVFH.mi2 * minSector(c, 0, configVFH)
  thitdPart = configVFH.mi3 * minSector(c, previouslySector, configVFH)

  return firstPart + secondPart + thitdPart

def angleToSector(angle, sizeSector):
  if angle < 0:
    angle += 2*pi
  elif angle > (2*pi):
    angle -= 2*pi
    
  return round_down(angle/sizeSector)

def minSector(a, b, configVFH):
  one = abs(a - b)
  two = abs(a - b - configVFH.numberOfSector)
  three = abs(a - b + configVFH.numberOfSector)
  if (one < two) and (one  < three):
    return one
  elif (two < three):
    return two
  else:
    return three 

def drawSectors(listOfSector, configVFH):
  sizeSector = (2*pi) / configVFH.numberOfSector
  marker = Marker(
          header = Header(
            frame_id = configVFH.robotName,
            stamp = rospy.Time.now()
          ),
          id = 500,
          type = Marker.TRIANGLE_LIST,
          action = Marker.ADD,
          pose = Pose(
            position = Point( x = 0, y = 0, z = 0 ),
            orientation = Quaternion(0, 0, 0, 5)
          ),
          scale = Vector3( 1, 1, 1),
          color =  ColorRGBA( 0, 0, 0, 0.8)
  )

  for i in range(configVFH.numberOfSector):
    if listOfSector[i].h == 0:
      marker.colors.append( ColorRGBA( 0, 1, 0, 1)) # green
    else:
      marker.colors.append( ColorRGBA( 1, 0, 0, 1)) # red

    marker.points.append( Point( x = 0, y = 0, z = 0 ))

    for o in range(2):
      alfa = (i + o) * sizeSector
      size = (configVFH.sizeSquare * configVFH.sizeGrid)/2
      
      c = cos(alfa)
      s = sin(alfa)
      if abs(s) < abs(c):
        x = np.sign(c) * size
        y = (1 if (c == 0) else (s/c)) * x
      else:
        y = np.sign(s) * size
        x = (1 if (s == 0) else (c/s)) * y    
      
      marker.points.append( Point( x = x, y = y, z = 0 ))
    
  configVFH.markerPublisherSektor.publish( marker)

def drawSectorsBinary(listOfSector, configVFH):
  sizeSector = (2*pi) / configVFH.numberOfSector
  marker = Marker(
          header = Header(
            frame_id = configVFH.robotName,
            stamp = rospy.Time.now()
          ),
          id = 500,
          type = Marker.TRIANGLE_LIST,
          action = Marker.ADD,
          pose = Pose(
            position = Point( x = 0, y = 0, z = 0 ),
            orientation = Quaternion(0, 0, 0, 5)
          ),
          scale = Vector3( 1, 1, 1),
          color =  ColorRGBA( 0, 0, 0, 0.8)
  )

  for i in range(configVFH.numberOfSector):
    if listOfSector[i].binary == False:
      marker.colors.append( ColorRGBA( 0, 1, 0, 1)) # green
    else:
      marker.colors.append( ColorRGBA( 1, 0, 0, 1)) # red

    marker.points.append( Point( x = 0, y = 0, z = 0 ))

    for o in range(2):
      alfa = (i + o) * sizeSector
      size = (configVFH.sizeSquare * configVFH.sizeGrid)/2
      
      c = cos(alfa)
      s = sin(alfa)
      if abs(s) < abs(c):
        x = np.sign(c) * size
        y = (1 if (c == 0) else (s/c)) * x
      else:
        y = np.sign(s) * size
        x = (1 if (s == 0) else (c/s)) * y    
      
      marker.points.append( Point( x = x, y = y, z = 0 ))
    
  configVFH.markerPublisherSektorBinary.publish( marker)

def drawSectorsMask(listOfSector, bestSector, configVFH):
  sizeSector = (2*pi) / configVFH.numberOfSector
  marker = Marker(
          header = Header(
            frame_id = configVFH.robotName,
            stamp = rospy.Time.now()
          ),
          id = 500,
          type = Marker.TRIANGLE_LIST,
          action = Marker.ADD,
          pose = Pose(
            position = Point( x = 0, y = 0, z = 0 ),
            orientation = Quaternion(0, 0, 0, 5)
          ),
          scale = Vector3( 1, 1, 1),
          color =  ColorRGBA( 0, 0, 0, 0.8)
  )

  for i in range(configVFH.numberOfSector):
    if i == bestSector:
      marker.colors.append( ColorRGBA( 1, 1, 0, 1))
    elif listOfSector[i].mask == False:
      marker.colors.append( ColorRGBA( 0, 1, 0, 1)) # green
    else:
      marker.colors.append( ColorRGBA( 1, 0, 0, 1)) # red

    marker.points.append( Point( x = 0, y = 0, z = 0 ))

    for o in range(2):
      alfa = (i + o) * sizeSector
      size = (configVFH.sizeSquare * configVFH.sizeGrid)/2
      
      c = cos(alfa)
      s = sin(alfa)
      if abs(s) < abs(c):
        x = np.sign(c) * size
        y = (1 if (c == 0) else (s/c)) * x
      else:
        y = np.sign(s) * size
        x = (1 if (s == 0) else (c/s)) * y    
      
      marker.points.append( Point( x = x, y = y, z = 0 ))
    
  configVFH.markerPublisherSektorMask.publish( marker)

def drawSectorsMagnitude(listOfSector, configVFH):
  sizeSector = (2*pi) / configVFH.numberOfSector
  startAngle = sizeSector/2

  marker = Marker(
          header = Header(
            frame_id = configVFH.robotName,
            stamp = rospy.Time.now()
          ),
          id = 500,
          type = Marker.LINE_LIST,
          action = Marker.ADD,
          pose = Pose(
            position = Point( x = 0, y = 0, z = 0 ),
            orientation = Quaternion(0, 0, 0, 5)
          ),
          scale = Vector3( 0.1, 0.1, 0.1),
          color =  ColorRGBA( 0, 0, 0, 1)
  )

  for i, sector in enumerate(listOfSector):
    firstPoint = Point( x = 0, y = 0, z = 0 )
    angle = startAngle + (sizeSector * i)
    size = sector.h * 0.1

    c = cos(angle)
    s = sin(angle)
    if abs(s) < abs(c):
      x = np.sign(c) * size
      y = (1 if (c == 0) else (s/c)) * x
    else:
      y = np.sign(s) * size
      x = (1 if (s == 0) else (c/s)) * y

    secondPoint = Point( x = x, y = y, z = 0)

    marker.points.append(firstPoint)
    marker.points.append(secondPoint)

  configVFH.markerPublisherSectorMagnitude.publish(marker)

def calculateGoalAngle(configVFH):
  positionRobot = configVFH.__globalRobotOdometry__
  positionGoal = configVFH.__globalGoalPoint__

  # uhol otocenia
  q = positionRobot.pose.pose.orientation
  # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  siny_cosp = 2 * (q.w * q.z + q.x * q.y)
  cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
  yaw = atan2(siny_cosp, cosy_cosp)

  x = positionGoal.pose.position.x - positionRobot.pose.pose.position.x
  y = positionGoal.pose.position.y - positionRobot.pose.pose.position.y

  radians = 0
  if (x or y): 
      radians = atan2(y, x)
  else:
      radians = 0

  configVFH.goalAngle =  radians - yaw
  drawGoalAngle(configVFH)


def drawGoalAngle(configVFH):
  sizeLine = 2
  marker = Marker(
          header = Header(
            frame_id = configVFH.robotName,
            stamp = rospy.Time.now()
          ),
          id = 500,
          type = Marker.LINE_LIST,
          action = Marker.ADD,
          pose = Pose(
            position = Point( x = 0, y = 0, z = 0 ),
            orientation = Quaternion(0, 0, 0, 5)
          ),
          scale = Vector3( 0.1, 0.1, 0.1),
          color =  ColorRGBA( 0, 1, 1, 1)
  ) 

  marker.points.append(Point(x = 0, y = 0, z = 0))

  c = cos(configVFH.goalAngle)
  s = sin(configVFH.goalAngle)
  if abs(s) < abs(c):
    x = np.sign(c) * sizeLine
    y = (1 if (c == 0) else (s/c)) * x
  else:
    y = np.sign(s) * sizeLine
    x = (1 if (s == 0) else (c/s)) * y

  marker.points.append(Point( x = x, y = y, z = 0))

  configVFH.markerPublisherGoalLine.publish(marker)

def distanceSector(leftSector, rightSector, configVFH):
  if leftSector >= rightSector:
    return leftSector - rightSector
  else:
    return configVFH.numberOfSector - rightSector + leftSector 

def round_down(n):
    return int(floor(n))

def robotOdometryCallback(msg, configVFH):
  configVFH.__globalRobotOdometry__ = copy.deepcopy(msg)

def robotGoalPositionCallback(msg, configVFH):
  configVFH.__globalGoalPoint__ = copy.deepcopy(msg)

def main():
  rospy.init_node('myVFH')
  config = configVFH()

  # configure
  config.robotName      = rospy.get_param('robotName', '/robot0')
  config.sizeGrid       = rospy.get_param('sizeGrid', 100)
  config.sizeSquare     = rospy.get_param('sizeSquare', 0.05)
  config.numberOfSector = rospy.get_param('numberOfSector', 100)
  config.robotRadius    = rospy.get_param('robotRadius', 0.3)
  config.safeZone       = rospy.get_param('safeZone', 0.2)
  config.Smax           = rospy.get_param('Smax', 10)
  config.Tlow           = rospy.get_param('Tlow', 5)
  config.Thigh          = rospy.get_param('Thigh', 15)
  config.rotation       = rospy.get_param('rotation', 1)
  config.velocity       = rospy.get_param('velocity', 1)
  config.mi1            = rospy.get_param('mi1', 5)
  config.mi2            = rospy.get_param('mi2', 2)
  config.mi3            = rospy.get_param('mi3', 2)
  
  # marker
  config.markerPublisherRobotCMD        = rospy.Publisher(config.robotName + '/cmd_vel', Twist)
  config.markerPublisherSektor          = rospy.Publisher('visualizationMyVFHSektor', Marker, queue_size=2)
  config.markerPublisherSektorBinary    = rospy.Publisher('visualizationMyVFHSektorBinary', Marker, queue_size=2)
  config.markerPublisherSektorMask      = rospy.Publisher('visualizationMyVFHSektorMask', Marker, queue_size=2)
  config.markerPublisherSectorMagnitude = rospy.Publisher('visualizationMyVFHSectorMagnitude', Marker, queue_size=2)
  config.markerPublisherGoalLine        = rospy.Publisher('visualizationMyVFHSGoalLine', Marker, queue_size=2)
  config.markerPublisherICR             = rospy.Publisher('visualizationMyVFHIRC', Marker, queue_size=2)
  
  robotOdometrySubscribe                = rospy.Subscriber( config.robotName + '/odom', Odometry, robotOdometryCallback, config)
  robotGoalPositionSubscribe            = rospy.Subscriber('/move_base_simple/goal', PoseStamped, robotGoalPositionCallback, config)
  # Create topic
  laserScanTopic = rospy.get_param('laserScanTopic', config.robotName + '/laser_0')

  #main funciton
  laserScanSubscribeCallback_lambda = lambda x:laserScanSubscribeCallback(x, config)
  laserScanSubscribe = rospy.Subscriber(laserScanTopic, LaserScan, laserScanSubscribeCallback_lambda)

  rospy.spin()


if __name__ == '__main__':
  main()