#!/usr/bin/env python 

import rospy
import serial
import time 
import math
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

#create a new publisher. we specify the topic name, then type of message then the queue size
rospy.init_node('talker_RS', anonymous=True)

pub = rospy.Publisher('chatter_RS', String, queue_size=10)
pub_test = rospy.Publisher('chatter', String, queue_size=10)


ser = serial.Serial('/dev/ttyUSB0', 115200)
ser.write("!g 1 0_!g 2 0\r")
ser.write("!g 1 0_!g 2 0\r")
ser.write("^ECHOF 1\r")                 # echo is closed
ser.write("# c\r")                      # Clear buffer
ser.write("!C 1 0\r")     # +\r
ser.write("!C 2 0\r")     # +\r
ser.write("# c\r")                      # Clear buffer
ser.write("?C\r")                       # select CB for hall sensor or C for encoder
ser.write("# 200\r")                     # read data every (100!!) ms



#set the loop rate
# rate = rospy.Rate(0.2) #cycle is running once for each 10 seconds

# rate.sleep()

# global i  
# i = i + 1 
# print(i) 


global straight_speed 
global rotation_speed
global straight_speed_low

global distance_between_wheels
global wheel_rad
global encoder_for_one_cycle 


straight_speed = 400  # forward and back 
rotation_speed = 250  # CW and CCW 
straight_speed_low = int(straight_speed/2)

distance_between_wheels = 31.8   #in cm 31.8 
wheel_rad = 5                    #in cm 5 
encoder_for_one_cycle = 36000    #????


new_location_theta      = 0
motor1_encoder_prev     = 0 
motor2_encoder_prev     = 0 

i = 0 


def read_controller():  
  
  #pub.publish(hello_str)
  #send to hello_str via serialCom. With 115200 baudrate, 6 chactare(6*8 = 48 bit) is send in 416,66 us.
  #But do not forget that start bit, stop bit and parity bits are not included in this calculate!   
  #Commands that i found from internet    https://answers.ros.org/question/306418/how-to-open-a-serial-port-using-a-ros-node/

  while not rospy.is_shutdown():
  
   message = ser.read_until('\r\r')
   message = message.replace('\r\r', '')
   message_1 = message.split("C=")
   message_2 = message_1[1].split(":")
   print(message_2[0])
   print(message_2[1])
  
   print("*********************************************")
   motor1ENC = int(message_2[0])
   motor2ENC = int(message_2[1]) 
   print("motor1 encoder value is:")
   print(motor1ENC)
   print("motor2 encoder value is:")
   print(motor2ENC)  

   #make sure that everything is under "float"

   print(float(wheel_rad))
   print(math.pi)
   print(float(float(motor1ENC) / float(encoder_for_one_cycle)))

   ds = ((2.0 * float(math.pi) * float(wheel_rad) * float(float(motor1ENC)/float(encoder_for_one_cycle))) + (2.0 * float(math.pi) * float(wheel_rad) * float(float(motor2ENC)/float(encoder_for_one_cycle)))) / 2.0  
   print("straight distance value is: [in cm]")
   print(ds)
   encoder_processing(motor1ENC,motor2ENC) #sending for processing 
   

   # global i  
   # i = i + 1 
   # print(i) 
    


def get_cmd_vel(id_message):

    if (id_message.linear.x == 0.5 and id_message.angular.z == 0 ):       #forward
       motor1_order = straight_speed
       motor2_order = straight_speed

    if (id_message.linear.x == -0.5 and id_message.angular.z == 0 ):       #back
       motor1_order = -straight_speed
       motor2_order = -straight_speed 
    if (id_message.linear.x == 0 and id_message.angular.z == -1 ):       #cw
       motor1_order = -rotation_speed  
       motor2_order = rotation_speed
    if (id_message.linear.x == 0 and id_message.angular.z == 1 ):       #ccw
      motor1_order = rotation_speed  
      motor2_order = -rotation_speed  
    if (id_message.linear.x == 0.5 and id_message.angular.z == -1):          #forward-right
      motor1_order = straight_speed_low  
      motor2_order = straight_speed  
    if (id_message.linear.x == 0.5 and id_message.angular.z == 1):          #forward-left
      motor1_order = straight_speed 
      motor2_order = straight_speed_low 
    if (id_message.linear.x == -0.5 and id_message.angular.z == 1): #back-right
      motor1_order = -straight_speed_low 
      motor2_order = -straight_speed
    if(id_message.linear.x == -0.5 and id_message.angular.z == -1):   #back-left
      motor1_order = -straight_speed 
      motor2_order = -straight_speed_low
    if (id_message.linear.x == 0 and id_message.angular.z == 0):  #stop
      motor1_order = 0 
      motor2_order = 0  

    send_cmd_to_motorcontrol(motor1_order, motor2_order)

def send_cmd_to_motorcontrol(motor1,motor2):

    # format for Roboteq controller
    motor1_info = '!g 1 {}\r'.format(motor1)         #there was a underscope!
    motor2_info = '!g 2 {}\r'.format(motor2)         #there was a underscope!

    #print message
    print motor1_info
    print motor2_info

    # send by serial
    ser.write(motor1_info)
    ser.write(motor2_info)

    # rate_1 = rospy.Rate(0.2)
    # rate_1.sleep()
    pub_test.publish('test_1234')
    

def encoder_processing(motor1_encoder,motor2_encoder):

  # prev_location_x = 
  # prev_location_y = 
  # prev_location_theta =  
  
  # new_location_x = 
  # new_location_y =
  # new_location_theta = 

  global motor1_encoder_prev 
  global motor2_encoder_prev


  global new_location_theta 
  global new_location_theta_snap

  global i 

  real_motor1_encoder = motor1_encoder - motor1_encoder_prev
  real_motor2_encoder = motor2_encoder - motor2_encoder_prev    

  motor1_encoder_prev = motor1_encoder 
  motor2_encoder_prev = motor2_encoder 

  motor1_distance_snap = 2.0 * math.pi * float(wheel_rad) * float(real_motor1_encoder) / float(encoder_for_one_cycle) 
  motor2_distance_snap = 2.0 * math.pi * float(wheel_rad) * float(real_motor2_encoder) / float(encoder_for_one_cycle)
 
  new_location_theta_snap = (motor1_distance_snap - motor2_distance_snap) / float(distance_between_wheels) 
  new_location_theta = new_location_theta + new_location_theta_snap 
  
  print("new_location_theta is: [in radian]")
  print(new_location_theta)
  print("new_location_theta is: [in degree]")
  print(math.degrees(new_location_theta))

  #ser.write("!C 1 0_!C 2 0\r")     # +\r





if __name__ == '__main__':
    try:
      rospy.Subscriber('/cmd_vel', Twist,get_cmd_vel)
      read_controller()
    except rospy.ROSInterruptException:
     pass