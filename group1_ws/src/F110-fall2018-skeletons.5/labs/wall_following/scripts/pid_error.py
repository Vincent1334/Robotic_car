#!/usr/bin/env python

PKG = 'wall_following'
import roslib; roslib.load_manifest(PKG)
import rospy
import math
import numpy as np
import yaml
import sys
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import pdb
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
pub = rospy.Publisher('pid_error', Float64, queue_size=10)
fieldpub = rospy.Publisher('field_error', numpy_msg(Floats), queue_size=10)
xpub = rospy.Publisher('x', Float64, queue_size=10)
ypub = rospy.Publisher('y', Float64, queue_size=10)
gap_pub = rospy.Publisher('gap', Float64, queue_size=10)

lastdirection = 1

# You can define constants in Python as uppercase global names like these.
MIN_DISTANCE = 0.1
MAX_DISTANCE = 30.0
MIN_ANGLE = -45.0
MAX_ANGLE = 225.0



# data: single message from topic /scan
# angle: between -45 to 225 degrees, where 0 degrees is directly to the right
# Outputs length in meters to object with angle in lidar scan field of view
def getRange(data, angle):
  
  return min(data.ranges[angle*4],MAX_DISTANCE)
  
  
  
def getRangeCentered(data, angle):
  angle+=135
  return min(data.ranges[angle*4],MAX_DISTANCE)

def getRangeCenteredAv(data,angle):
  angle+=135
  av = (data.ranges[angle*4]+data.ranges[angle*4+1]+data.ranges[angle*4+2]+data.ranges[angle*4+3])/4
  return min(av,MAX_DISTANCE)

def getMaxEdge(data):
  maxEdge=[0,0]
  viewrange = 130
  for i in range(-viewrange,viewrange):
    d=getRangeCenteredAv(data,i)
    dif = 0
    if i<0:
      dif = getRangeCenteredAv(data,i+1)-d
    else:
      dif = d-getRangeCenteredAv(data,i+1)

   
    if dif>maxEdge[0]:
      maxEdge=[dif,i]

  return maxEdge

def getFrontDistance(data):
  mini = getRangeCentered(data, 0)
  for i in range(-5,5):
    mini = min(getRangeCentered(data, i),mini)
  return mini
    

def distanceLeft(data):
  a=getRange(data,180)
  b=getRange(data,225)
  theta = math.pi/4
  alpha=math.atan((a*math.cos(theta)-b)/(a*math.sin(theta)))
  return b*math.cos(alpha)


def distanceRight(data):
  a=getRange(data,90)
  b=getRange(data,45)
  theta = math.pi/4
  alpha=math.atan((a*math.cos(theta)-b)/(a*math.sin(theta)))
  return b*math.cos(alpha)
  
  
# data: single message from topic /scan
# desired_distance: desired distance to the left wall [meters]
# Outputs the PID error required to make the car follow the left wall.
def followLeft(data, desired_distance):
  
  
  AB =distanceLeft(data)
  
  return -(AB-desired_distance)


# data: single message from topic /scan
# desired_distance: desired distance to the right wall [meters]
# Outputs the PID error required to make the car follow the right wall.
def followRight(data, desired_distance):
  
  AB=distanceRight(data)
  
  return AB-desired_distance

# data: single message from topic /scan
# Outputs the PID error required to make the car drive in the middle
# of the hallway.
def followCenter(data):
  
  return distanceRight(data)-distanceLeft(data)

# Callback for receiving LIDAR data on the /scan topic.
# data: the LIDAR data, published as a list of distances to the wall.
def scan_callback(data):
  global lastdirection
  error = 0.0 # TODO: replace with followLeft, followRight, or followCenter
  desired_distance=0.5
  rechts =  followRight(data, desired_distance)
  links = followLeft(data, desired_distance)
  
  #print 'rechts: '+str(getRange(data,90))
  #print 'links: '+str(getRange(data,180))
  
  error = 0
  #print 'errordifference'+str(abs(rechts-links))
  #if abs(rechts-links)<2.0:
   # error = followCenter(data)
    #print 'center'
  #elif links<rechts:
  #error = followLeft(data, desired_distance)
   # print 'links'
  #else:
  #  error = followRight(data, desired_distance)
  #  print 'rechts'
  #print "error: "+str(error)
  
  #vectors=np.zeros(180,2)
  #vectorIndex =0;
  vector = np.array([0.0,0.0], dtype=np.float32)
  edge = getMaxEdge(data)
  steer=1
  counter_steer=5.0#2.3
  curve_steer=5.0#2.9
  breite = getRangeCenteredAv(data,90)+getRangeCenteredAv(data,-90)
  #print breite 
  if edge[0]>0.5 and edge[0]<1.9 and breite > 1.0 and False:
    #print breite 

    #print "Kantenwinkel: "+str(edge[1])+" Laenge: "+str(edge[0])
    #links hindernis links lenken
    if edge[1]>57:
      steer = curve_steer
      #vector = np.array([100.0,0.0], dtype=np.float32)
      print "llinks"+"Kantenwinkel: "+str(edge[1])+" Laenge: "+str(edge[0])
    #links hindernis rechts lenken
    elif edge[1]>0:
      steer = 1/counter_steer
      #vector = np.array([-100.0,0.0], dtype=np.float32)
      print "lrechts"+"Kantenwinkel: "+str(edge[1])+" Laenge: "+str(edge[0])
    #rechts hindernis rechts lenken
    if edge[1]<-57:
      steer = 1/curve_steer
      print "rrechts" +"Kantenwinkel: "+str(edge[1])+" Laenge: "+str(edge[0])
    #rechts hinderniss links lenken
    elif edge[1]<-0:
      steer = counter_steer
      print "rlinks"+"Kantenwinkel: "+str(edge[1])+" Laenge: "+str(edge[0])
    
    


  max_edge=[0,0]

  for i in range(-90,90):
    alpha=np.deg2rad(i)
    #d=getRangeCentered(data,i)*math.sin(abs(alpha))*np.cos(alpha)
    d=getRangeCentered(data,i)
    winkel=np.rad2deg(np.arctan(vector[0]/vector[1]))
    #VIELLEICHT RAUSNEHMEN/ANPASSEN!!
    #d=d*math.cos(alpha)
    if i>0:
      d=d*steer
    else:
      d = d/steer

    space = getRangeCenteredAv(data,45)+getRangeCenteredAv(data,-45)
    if space<0.78 and i>0 and getRangeCenteredAv(data,-90)<0.5:
      d=d*2.3
      print "eng"+str(space)

    #if getRangeCentered(data,-i)>getRangeCentered(data,i)

      
    #if i>0:
      #d*=1.5

    #if i>-90 and i<-80:
    #  d=d*5.0
    #elif i<90 and i>80:
    #  d=d*5.0
    #d=abs(np.sin(alpha))*d
    #p=getRangeCentered(data,-90) + getRangeCentered(data,90)
    #print p
    #if p>0.7:
     # if winkel>0 and i<0:
      #  d=d*2
      #if winkel<0 and i>0:
       # d=d*2


    x=np.sin(alpha)*d;
    y=np.cos(alpha)*d
    
    vector=vector+np.array([x,y], dtype=np.float32)
  detect = "no edge"
  if edge[0]>0.2 and edge[0]<1.9 and breite > 1.0 and sum(data.ranges)>1000.0 and False:
    #print breite 
    betrag= math.sqrt(math.pow(vector[0],2)+math.pow(vector[1],2))
    #print "Kantenwinkel: "+str(edge[1])+" Laenge: "+str(edge[0])
    #links hindernis links lenken
    if edge[1]>57:
      vector = np.array([betrag,0.1], dtype=np.float32)
      detect= "llinks"
    #links hindernis rechts lenken
    elif edge[1]>0:
      vector = np.array([-1*betrag,0.1], dtype=np.float32)
      detect =" lrechts"
    #rechts hindernis rechts lenken
    if edge[1]<-57:
      vector = np.array([-1*betrag,0.1], dtype=np.float32)
      detect= " rrechts" 
    #rechts hinderniss links lenken
    elif edge[1]<-0:
      vector = np.array([betrag,0.1], dtype=np.float32)
      detect= " rlinks"
  #print "Kantenwinkel: "+str(edge[1])+" Laenge: "+str(edge[0])+" Breite: " +str(breite)+" Summe: "+str(sum(data.ranges))+ detect 
  #print getFrontDistance(data)
  if getFrontDistance(data)<0.46:
    vector[0]=np.sign(lastdirection)*90
    vector[1]=-1
  else:
    lastdirection = vector[0]

  fieldmsg=numpy_msg(Floats)
  fieldmsg.data=vector
  #fieldpub.publish(fieldmsg)
  
  xmsg = Float64()
  xmsg.data = vector[0]
  xpub.publish(xmsg)
  ymsg = Float64()
  ymsg.data = vector[1]
  ypub.publish(ymsg)
  
  msg = Float64()
  msg.data = error
  pub.publish(msg)
  




# Boilerplate code to start this ROS node.
# DO NOT MODIFY!
if __name__ == '__main__':
	rospy.init_node('pid_error_node', anonymous = True)
	rospy.Subscriber("scan", LaserScan, scan_callback)
	rospy.spin()
