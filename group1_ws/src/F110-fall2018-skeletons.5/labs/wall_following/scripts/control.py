#!/usr/bin/env python

import rospy
import math
from race.msg import drive_param
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import numpy as np
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import sensor_msgs.msg
import math
# TODO: modify these constants to make the car follow walls smoothly.
#scaleV: 241.0 addV: 1.1 velocity: 2.9492299432Winkel: 0.246930426878 KP: 0.4KD: 5.8
#scaleV: 167.0 addV: 1.05 velocity: 2.67636038805Winkel: 0.292038858615 KP: 0.6KD: 6.65  


#scaleV: 155.0 addV: 2.65 velocity: 3.00230738461Winkel: 0.127920613401 KP: 0.6KD: 8.1 <--bester

#scaleV: 119.0 addV: 1.9 velocity: -1.81777398421Winkel: -0.6 KP: 0.6KD: 6.65 nur potential 15.16
#scaleV: 101.0 addV: 2.15 velocity: 2.74991191938Winkel: 0.328912755585 KP: 0.6KD: 6.65
#scaleV: 93.0 addV: 1.65 velocity: 3.11822036292Winkel: -0.133485387206 KP: 0.6KD: 6.65 Breite: 0.660000000149

#scaleV: 93.0 addV: 1.75 velocity: 3.93441624115Winkel: 0.279861482225 KP: 0.6KD: 6.65 Breite: 0.338250003755




KP = 0.6
KD = 6.65  #Abtastrate 40Hz
prev_e = 0
le = 0

x=0.0
y=0.0

scaleV=93.0
addV=1.75
scan=0


pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)

# Callback for receiving PID error data on the /pid_error topic 	
# data: the PID error from pid_error_node, published as a Float64
def control_callback(data):
    global le
    global x
    global y
    global scaleV
    global addV
    global scan
    
    # TODO: Based on the error (data.data), determine the car's required velocity
    # amd steering angle.
    global prev_e
    #e=data.data

    V=-np.arctan(x/y)
    e=V
    V=KP*e+KD*(e-prev_e)
    #print "Winkel"+str(V)+" KP: "+str(KP)+"KD"+str(KD)
    if V>0.6:
        V=0.6
    if V<-0.6:
        V=-0.6

    #le=min(data.ranges[135*4],30)
    msg = drive_param()
    #msg.velocity = 1.2/(0.6*abs(V)+0.5)*(le/30)  # TODO: implement PID for velocity
    #msg.velocity = 4.5*abs(e)+1.0
    

    #msg.velocity=math.sqrt(math.pow(x,2)+math.pow(y,2))/70
    breite = getRangeCenteredAv(scan,90)+getRangeCenteredAv(scan,-90)
    addEng=0
    if breite<0.8:
      addEng=1.2
    p=np.sign(y)
    velocity=p*(math.sqrt(math.pow(x,2)+math.pow(y,2))/scaleV+addV+addEng)
    #velocity=0.7
    velocity = velocity/(abs(V)+1)
    msg.velocity=velocity

    print "scaleV: "+str(scaleV)+" addV: "+str(addV)+" velocity: "+str(msg.velocity) + "Winkel: "+str(V)+" KP: "+str(KP)+"KD: "+str(KD) +" Breite: "+str(breite)
    


   
    #Ziel 5.5
    #math.sqrt(math.pow(x,2)+math.pow(y,2))/85+0.6 =>6.656s
    #math.sqrt(math.pow(x,2)+math.pow(y,2))/70 => 6,924



    #msg.velocity=0.5
    msg.angle = -V   # TODO: implement PID for steering angle
    pub.publish(msg)
    #print "Winkel: "+str(V)
    #print "Geschwindigkeit: "+str(msg.velocity)
    prev_e=e
    
    
    #print "vector"+str(x)+"y"+str(y)

    #print msg.velocity
    
    



        
def scan_callback(data):
  global le
  global scan
  le=min(data.ranges[135*4],30)
  scan = data
    
def x_callback(data):
  global x
  x=data.data
  
def y_callback(data):
  global y
  y=data.data

def getRangeCenteredAv(data,angle):
  angle+=135
  av = (data.ranges[angle*4]+data.ranges[angle*4+1]+data.ranges[angle*4+2]+data.ranges[angle*4+3])/4
  return min(av,30.0)
 

def joy_callback(data):
  global scaleV
  global addV
  global KP
  global KD
  #try:
    #for c in self.command_list:
    #if match_command(c, data.buttons):
      #print "cccc"
      #self.run_command(c, data)
      # Only run 1 command at a time
      
  #except JoyTeleopException as e:
    #rospy.logerr("error while parsing joystick input: %s", str(e))
  #self.old_buttons = data.buttons
  #print "buttons"+str(data.buttons)
  #y Button  
  if data.buttons[3]:
    addV+=0.05
    print "Y"
  if data.buttons[1]:
    addV-=0.05
    print "A"
  if data.buttons[0]:
    scaleV+=2
    print "X"
  if data.buttons[2]:
    scaleV-=2
    print "B"


  if data.axes[5]==1.0:
    KP+=0.05
    print "UP"
  if data.axes[5]==-1.0:
    KP-=0.05
    print "DOWN"
  if data.axes[4]==-1.0:
    KD+=0.05
    print "RIGHT"
  if data.axes[4]==1.0:
    KD-=0.05
    print "LEFT"


# Boilerplate code to start this ROS node.
# DO NOT MODIFY!
if __name__ == '__main__':
    rospy.init_node('pid_controller_node', anonymous=True)
    rospy.Subscriber("pid_error", Float64, control_callback)
    rospy.Subscriber("scan", LaserScan, scan_callback)
    #rospy.Subscriber("field_error", numpy_msg(Floats), field_callback)
    
    rospy.Subscriber("x", Float64, x_callback)
    rospy.Subscriber("y", Float64, y_callback)
    rospy.Subscriber('vesc/joy', sensor_msgs.msg.Joy, joy_callback)
    
    rospy.spin()

