#!/usr/bin/python

# This node sends rate setpoints to a MAVLINK enabled vehicle using a joystick. It does this
# by putting the vechicle into an OFFBOARD armed state, and publishing 
# to the vehicle_rates_setpoint topic (RPYT commands). By default it uses the first
# 4 axis of a joystick, but if using a DroidPad 
# (https://www.digitalsquid.co.uk/droidpad/) interface, you can select the
# correct axes by passing 'droidpad' as an argument to this script

import rospy
import sys

from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
from geometry_msgs.msg import *

class JoystickToMAVROS():
  def __init__(self, droidpad=False):
    
    self.droidpad = droidpad

    rospy.init_node('joystick_to_mavros_rates_setpoint')
    
    self.set_mode_proxy = rospy.ServiceProxy('mavros/set_mode',SetMode)
    self.arm_proxy = rospy.ServiceProxy('mavros/cmd/arming',CommandBool)
   
    self.throttle_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
    self.rates_pub = rospy.Publisher('/mavros/setpoint_attitude/cmd_vel', TwistStamped, queue_size=1)
    rospy.set_param('/mavros/setpoint_attitude/reverse_throttle', True)
    
    
    self.js_sub = rospy.Subscriber('joy', Joy, self.joy_callback, queue_size=1)
    self.state_sub = rospy.Subscriber('mavros/state', State, self.state_callback, queue_size = 10)

    self.state = State()
    
    self.informed = False

    self.joystick_input = None

    self.rate = rospy.Rate(20)

  def run(self):
    while (not rospy.is_shutdown()):
      
      if self.state.mode != 'OFFBOARD':
        #print 'Setting Offboard mode'
        try:
          self.set_mode_proxy(0,'OFFBOARD')
        except rospy.ServiceException:
          #print 'Waiting for set_mode service'
          pass

      elif not self.state.armed:
        #print 'Offboard mode set, arming'
        try:
          self.arm_proxy(True)
        except rospy.ServiceException:
          # print 'Waiting for arming service'
          pass
      
      elif not self.informed:
        print 'Offboard mode armed, ready to command'
        self.informed = True
        
      self.publish_joystick_to_mavros(self.joystick_input)

      self.rate.sleep()

  def state_callback(self, msg):
    self.state = msg

  def joy_callback(self, msg):
    if self.droidpad:
      nr,t,p,y = msg.axes[-4:]
    else:
      nr,t,p,y = msg.axes[:4]
    self.joystick_input = [t,-nr,p,y] 
  
  def publish_joystick_to_mavros(self,joystick_input=None):
    if joystick_input is None:
      joystick_input = [0,0,0,0]
    throttle = AttitudeTarget()
    rates = TwistStamped()

    throttle.thrust = (joystick_input[0])
    rates.header.stamp = rospy.Time.now()
    rates.twist.angular.x = joystick_input[1]
    rates.twist.angular.y = joystick_input[2]
    rates.twist.angular.z = joystick_input[3]



    self.throttle_pub.publish(throttle)
    self.rates_pub.publish(rates)
    
if __name__ == '__main__':
  droidpad = len(sys.argv) > 1 and sys.argv[1] == 'droidpad'
  jtm = JoystickToMAVROS(droidpad)
  jtm.run()
