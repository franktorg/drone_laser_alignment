#!/usr/bin/env python2.7

# ROS python API
import rospy
# Joy message structure
from sensor_msgs.msg import Joy
# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped, TwistStamped
from gazebo_msgs.msg import *
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
import numpy as np

import tf
from tf.transformations import quaternion_from_euler

import time

# Flight modes class
# Flight modes are activated using ROS services
class fcuModes:
    def __init__(self):
        pass

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print "Service arming call failed: %s"%e

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException, e:
            print "Service disarming call failed: %s"%e

    def setStabilizedMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Stabilized Mode could not be set."%e

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Offboard Mode could not be set."%e

    def setAltitudeMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Altitude Mode could not be set."%e

    def setPositionMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Position Mode could not be set."%e

    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException, e:
              print "service set_mode call failed: %s. Autoland Mode could not be set."%e


# Main class: Converts joystick commands to position setpoints
class Controller:

    # initialization method
    def __init__(self):
        # Drone state
        self.state = State()
        # Instantiate a setpoints message
        self.sp = PositionTarget()
        # set the flag to use velocity setpoints and yaw angle
        self.sp.type_mask = int('010111000111', 2) # int('010111111000', 2)
        # LOCAL_NED
        self.sp.coordinate_frame = 1

        # Instantiate a velocity setpoint message
        #self.vel_sp = TwistStamped()

        # We will fly at a fixed altitude for now
        # Altitude setpoint, [meters]
        self.ALT_SP = 1.0
        # update the setpoint message with the required altitude
        self.sp.position.z = self.ALT_SP

        # Instantiate a joystick message
        self.joy_msg = Joy()
        # initialize
        self.joy_msg.axes = [0.0, 0.0, 0.0]

        # Step size for position update
        self.STEP_SIZE = 2.0

        # Fence. We will assume a square fence for now
        self.FENCE_LIMIT = 5.0

        # A Message for the current local position of the drone(Anchor)
        self.local_pos = Point(0.0, 0.0, 0.0)
        self.local_vel = Point(0.0, 0.0, 0.0)

        self.modes = fcuModes()

		self.subGazStates = rospy.Subscriber('/gazebo/model_states', ModelStates, self.cbGazStates, queue_size=1)
		rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)

        # Position controllers
        self.current_time = time.time()
        self.last_time_z = self.current_time
        self.last_time_y = self.current_time
        self.last_time_x = self.current_time

        self.windup_guard = 20

        self.u_z = 0.0
        self.ITerm_z = 0.0
        self.SetPoint_z  = self.ALT_SP

        self.u_x = 0.0
        self.ITerm_x = 0.0
        self.SetPoint_x  = 0

        self.u_y = 0.0
        self.ITerm_y = 0.0
        self.SetPoint_y  = 0


    # Callbacks
    def PID_z(self, current_z):
        Kp_z = 1.5
        Ki_z = 0.01

        self.current_time = time.time()
        delta_time = self.current_time - self.last_time_z
        self.last_time_z = self.current_time       

        error_z = self.SetPoint_z - current_z
        PTerm_z =  Kp_z * error_z
        self.ITerm_z += error_z * delta_time

        if (self.ITerm_z < -self.windup_guard):
            self.ITerm_z = -self.windup_guard

        elif (self.ITerm_z > self.windup_guard):
            self.ITerm_z = self.windup_guard

        self.u_z = PTerm_z + (Ki_z * self.ITerm_z)

        

    def PID_x(self, current_x):
        Kp_x = 1.5
        Ki_x = 0.01


        self.current_time = time.time()
        delta_time = self.current_time - self.last_time_x
        self.last_time_x = self.current_time

        error_x = self.SetPoint_x - current_x
        PTerm_x =  Kp_x * error_x
        self.ITerm_x += error_x * delta_time

        if (self.ITerm_x < -self.windup_guard):
            self.ITerm_x = -self.windup_guard
        elif (self.ITerm_x > self.windup_guard):
            self.ITerm_x = self.windup_guard

        self.u_x = PTerm_x + (Ki_x * self.ITerm_x)

    def PID_y(self,current_y):
        Kp_y = 1.5
        Ki_y = 0.01
        
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time_y
        self.last_time_y = self.current_time

        error_y = self.SetPoint_y - current_y
        PTerm_y =  Kp_y * error_y
        self.ITerm_y += error_y * delta_time

        if (self.ITerm_y < -self.windup_guard):
            self.ITerm_y = -self.windup_guard
        elif (self.ITerm_y > self.windup_guard):
            self.ITerm_y = self.windup_guard

        self.u_y = PTerm_y + (Ki_y * self.ITerm_y)

  
    ## local position callback
    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z

     ## joystick callback
    def joyCb(self, msg):
        self.joy_msg = msg
        if msg.buttons[0] > 0 :
          self.modes.setArm()

        if msg.buttons[1] > 0 :
            self.modes.setAutoLandMode()
       
        if msg.buttons[2] > 0 :
            self.modes.setOffboardMode()  
       
        if msg.buttons[10] > 0 :
           self.modes.setDisarm()

    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    ## Update setpoint message
    def updateSp(self):
        x = -1.0*self.joy_msg.axes[1]
        y = -1.0*self.joy_msg.axes[0]
        z = self.joy_msg.axes[2]

   

        distance_x = abs(self.local_pos.x-self.leader_pos.x)
        distance_y = abs(self.local_pos.y-self.leader_pos.y)


 

        if (z > 0) :
            self.SetPoint_z  = self.ALT_SP
            self.PID_z(self.local_pos.z)
            self.sp.velocity.z = self.u_z
            self.SetPoint_x  = 0
            self.SetPoint_y  = 0
            self.PID_x(distance_x)
            self.PID_y(distance_y)
            self.u_x= -np.sign(self.local_pos.x-self.leader_pos.x)*self.u_x
            self.u_y= -np.sign(self.local_pos.y-self.leader_pos.y)*self.u_y
            self.sp.velocity.x = self.u_x
            self.sp.velocity.y = self.u_y
            print "ex : ",self.SetPoint_x-distance_x," u_x : ",self.u_x
            print "ey : ",self.SetPoint_y-distance_y," u_y : ",self.u_y
            print "ez : ",self.ALT_SP-self.local_pos.z," u_z : ",self.u_z
        
        elif (z>0) and abs(self.leader_pos.z- self.ALT_SP)<0.01:
            self.sp.velocity.z = 0 

        #landing
        if (z<0):
            self.SetPoint_z  = 0
            self.PID_z(self.leader_pos.z)
            self.sp.velocity.z = self.u_z
            print "ez : ",self.ALT_SP-self.leader_pos.z," u_z : ",self.u_z
        elif (z<0) and abs(self.leader_pos.z - 0)<0.01:
            self.sp.velocity.z = 0

# Main function
def main():

    # initiate node
    rospy.init_node('setpoint_node', anonymous=True)


    # controller object
    cnt = Controller()

    # ROS loop rate, [Hz]
    rate = rospy.Rate(20.0)

    # Subscribe to drone state
    rospy.Subscriber('mavros/state', State, cnt.stateCb)

    # subscribe to joystick topic
    rospy.Subscriber('joy', Joy, cnt.joyCb)

  
    # Setpoint publisher
    sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)

    pos_anchor_pub = rospy.Publisher('position_Anchor_gt',Point,queue_size=1) 
    pos_tag_pub = rospy.Publisher('position_Tag_gt',Point,queue_size=1) 

    vel_anchor_pub = rospy.Publisher('velocity_Anchor_gt',Point,queue_size=1)
    vel_tag_pub = rospy.Publisher('velocity_Tag_gt',Point,queue_size=1)
   
   

    # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
    k=0
    while k<10:
        sp_pub.publish(cnt.sp)
        rate.sleep()
        k = k+1

    # activate OFFBOARD mode
    cnt.modes.setOffboardMode()

    # ROS main loop
    while not rospy.is_shutdown():
        cnt.updateSp()
        sp_pub.publish(cnt.sp)
        pos_anchor_pub.publish(cnt.local_pos)
        vel_anchor_pub.publish(cnt.local_vel)
        pos_tag_pub.publish(cnt.leader_pos)
        vel_tag_pub.publish(cnt.leader_vel)
        #dist_anchor_tag_pub.publish(cnt.distance_anchor_tag)
        #vel_sp_pub.publish(cnt.vel_sp)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
