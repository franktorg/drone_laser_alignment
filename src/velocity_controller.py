#!/usr/bin/env python2.7

# ROS python API
import rospy
# Laser pixel coordinates message structure
from drone_laser_alignment.msg import Pixel_coordinates
# Joy message structure
from sensor_msgs.msg import Joy
# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, Vector3, PoseStamped, TwistStamped
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
        # self.state = State()
        # Instantiate laser pixel coordinates
        self.coordinates = Pixel_coordinates()
        # Instantiate a setpoints message
        self.sp = PositionTarget()
        # set the flag to use velocity and position setpoints, and yaw angle
        self.sp.type_mask = int('010111000000', 2) # int('010111111000', 2)
        # BODY_NED
        self.sp.coordinate_frame = 8
        # Yaw Setpoint
        self.sp.yaw = 0.0
        # Joystick button
        self.alignment_flag = 0
        # Instantiate a velocity setpoint message
        #self.vel_sp = TwistStamped()

        # We will fly at a fixed altitude for now
        # Altitude setpoint, [meters]
        self.ALT_SP = 1.5
        # update the setpoint message with the required altitude
        self.sp.position.z = self.ALT_SP

        # Instantiate a joystick message
        self.joy_msg = Joy()
        # initialize
        self.joy_msg.axes = [0.0, 0.0, 0.0]

        # Step size for position update
        self.STEP_SIZE = 2.0

        # Fence. We will assume a rectangular fence [Cage flight area]
        self.FENCE_LIMIT_X = 1.5
        self.FENCE_LIMIT_Y = 2.0
        
        # A Message for the current local position of the drone(Anchor)
        self.local_pos = Point(0.0, 0.0, 0.0)
        self.local_vel = Vector3(0.0, 0.0, 0.0)

        self.modes = fcuModes()

        # Position controllers
        self.current_time = time.time()
        self.last_time_z = self.current_time
        self.last_time_y = self.current_time
        self.last_time_x = self.current_time

        self.last_error_z = 0.0
        self.last_error_y = 0.0
        self.last_error_x = 0.0
        self.windup_guard = 20.0

        self.u_z = 0.0
        self.ITerm_z = 0.0
        self.DTerm_z = 0.0
        self.SetPoint_z  = self.ALT_SP

        self.u_x = 0.0
        self.ITerm_x = 0.0
        self.DTerm_x = 0.0
        self.SetPoint_x  = 0.0

        self.u_y = 0.0
        self.ITerm_y = 0.0
        self.DTerm_y = 0.0
        self.SetPoint_y  = 0.0

        # Controller values
        self.kp_val = 0.003 
        self.ki_val = 0.0004
        self.kd_val  = 0.0009 
        self.pxl_err = 4

    # Keep drone inside the cage area limits
    def bound(self, v, low, up):
            r = v
            if v > up:
                r = up
            if v < low:
                r = low

            return r

    # Callbacks
    def PID_z(self, current_z):
        Kp_z = 1.5
        Ki_z = 0.01
        Kd_z = 0.1

        error_z = self.SetPoint_z - current_z

        self.current_time = time.time()
        delta_time = self.current_time - self.last_time_z
        delta_error = error_z - self.last_error_z

        PTerm_z =  Kp_z * error_z
        self.ITerm_z += error_z * delta_time

        if (self.ITerm_z < -self.windup_guard):
            self.ITerm_z = -self.windup_guard
        elif (self.ITerm_z > self.windup_guard):
            self.ITerm_z = self.windup_guard

        self.DTerm_z = 0.0
        if delta_time > 0:
            self.DTerm_z = delta_error / delta_time

        # Remember last time and last error for next calculation
        self.last_time_z = self.current_time 
        self.last_error_z = error_z          

        self.u_z = PTerm_z + (Ki_z * self.ITerm_z) + (Kd_z * self.DTerm_z)

        

    def PID_x(self, current_x):
        Kp_x = self.kp_val
        Ki_x = self.ki_val
        Kd_x = self.kd_val 

        error_x = abs(self.SetPoint_x - current_x)

        self.current_time = time.time()
        delta_time = self.current_time - self.last_time_x
        delta_error = error_x -self.last_error_x

        PTerm_x =  Kp_x * error_x
        self.ITerm_x += error_x * delta_time

        if (self.ITerm_x < -self.windup_guard):
            self.ITerm_x = -self.windup_guard
        elif (self.ITerm_x > self.windup_guard):
            self.ITerm_x = self.windup_guard

        self.DTerm_x = 0.0
        if delta_time > 0:
            self.DTerm_x = delta_error / delta_time

        # Remember last time and last error for next calculation
        self.last_time_x = self.current_time 
        self.last_error_x = error_x 

        self.u_x = PTerm_x + (Ki_x * self.ITerm_x) #+ (Kd_x * self.DTerm_x)

    def PID_y(self, current_y):

        Kp_y = self.kp_val 
        Ki_y = self.ki_val
        Kd_y = self.kd_val
        
        error_y = abs(self.SetPoint_y - current_y)

        self.current_time = time.time()
        delta_time = self.current_time - self.last_time_y
        delta_error = error_y -self.last_error_y

        PTerm_y =  Kp_y * error_y
        self.ITerm_x += error_y * delta_time

        if (self.ITerm_y < -self.windup_guard):
            self.ITerm_y = -self.windup_guard
        elif (self.ITerm_y > self.windup_guard):
            self.ITerm_y = self.windup_guard

        self.DTerm_y = 0.0
        if delta_time > 0:
            self.DTerm_y = delta_error / delta_time

        # Remember last time and last error for next calculation
        self.last_time_y = self.current_time 
        self.last_error_y = error_y 

        self.u_y = PTerm_y + (Ki_y * self.ITerm_y) #+ (Kd_y * self.DTerm_y)

  
    ## local position callback
    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z
        quater = (msg.pose.orientation.x, msg.pose.orientation.y,\
                  msg.pose.orientation.z, msg.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quater)
        self.current_yaw = euler[2]

    ## local velocity callback
    def velCb(self, msg):
        self.local_vel.x = msg.twist.linear.x
        self.local_vel.y = msg.twist.linear.y
        self.local_vel.z = msg.twist.linear.z

    ## Pixel coordinates callback
    def pxl_coordCb(self, msg):
        self.coordinates.xp = msg.xp
        self.coordinates.yp = msg.yp
        self.coordinates.blob = msg.blob

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

        if msg.buttons[4] > 0 :
            self.alignment_flag = 1

        if msg.buttons[3] > 0 :
            self.alignment_flag = 0

    ## Drone State callback
    # def stateCb(self, msg):
    #     self.state = msg

    ## Update setpoint message
    def updateSp(self):

        x = -1.0*self.joy_msg.axes[1]
        y = -1.0*self.joy_msg.axes[0]

        self.sp.yaw = 0.0
        self.sp.yaw_rate = 0.0


        # Switch to velocity setpoints (Laser coordinates)       
        if self.alignment_flag and self.coordinates.blob:

            print "Velocity Controller"
            # Set the flag to use velocity setpoints and yaw angle
            #self.sp.type_mask = int('010111000111', 2)

            # Altitude controller based on local position
            self.SetPoint_z  = self.ALT_SP
            self.PID_z(self.local_pos.z)
            ez = abs(self.ALT_SP - self.local_pos.z)
            # if ez < 0.01 :
            #      self.sp.velocity.z = 0
            # elif ez > 0.01 :
            self.sp.velocity.z = self.u_z 
            

            # x and y controller based on distance from blob center to image center (0,0)
            if ez < 0.1:
                self.SetPoint_x  = 0
                self.SetPoint_y  = 0
                self.PID_x(self.coordinates.xp)
                self.PID_y(self.coordinates.yp)
                self.u_x= -np.sign(self.SetPoint_x - self.coordinates.xp)*self.u_x
                self.u_y= -np.sign(self.SetPoint_y - self.coordinates.yp)*self.u_y
                
                ex = abs(self.SetPoint_x - self.coordinates.xp)
                ey = abs(self.SetPoint_x - self.coordinates.yp)
                
                
                if ex < self.pxl_err:
                    self.sp.velocity.x = 0
                elif ex > self.pxl_err:
                    self.sp.velocity.x = self.u_x

                if ey < self.pxl_err:
                    self.sp.velocity.y = 0    
                elif ey > self.pxl_err:
                    self.sp.velocity.y = self.u_y
                

                #print "ex : ",self.SetPoint_x - self.coordinates.xp, " u_x : ",self.u_x
                #print "ey : ",self.SetPoint_y - self.coordinates.yp, " u_y : ",self.u_y
                #print "ez : ",self.ALT_SP - self.local_pos.z," u_z : ",self.u_z
            
                
                    

                #landing
                # if z < 0 or z == 0:
                #     #print("Landing mode")
                #     self.SetPoint_z  = 1
                #     self.PID_z(self.local_pos.z)
                #     self.sp.velocity.z = self.u_z
                #     print "ez : ",self.ALT_SP-self.sp.position.z," u_z : ",self.u_z
                # elif (z<0) and abs(self.local_pos.z - 0)<0.01:
                #     self.sp.velocity.z = 0
            else:
                self.sp.velocity.x = 0
                self.sp.velocity.y = 0


        # Switch to position setpoints (Joystick)    
        else:
            # set the flag to use position setpoints and yaw angle
            print "Manual mode (Joystick)"
            # self.sp.type_mask = int('010111111000', 2)
            # Update 
            xsp = self.local_pos.x + self.STEP_SIZE*x
            ysp = self.local_pos.y + self.STEP_SIZE*y


            # limit
            self.sp.position.x = self.bound(xsp, -1.0*self.FENCE_LIMIT_X, self.FENCE_LIMIT_X)
            self.sp.position.y = self.bound(ysp, -1.0*self.FENCE_LIMIT_Y, self.FENCE_LIMIT_Y)
            
         

        

# Main function
def main():

    # initiate node
    rospy.init_node('setpoint_node', anonymous = True)


    # controller object
    cnt = Controller()

    # ROS loop rate, [Hz]
    rate = rospy.Rate(20.0)

    # Subscribe to drone state
    #rospy.Subscriber('mavros/state', State, cnt.stateCb)

    # Subscribe to drone's local position
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)

    # Subscribe to drone's local velocity
    rospy.Subscriber('mavros/local_position/velocity_local', TwistStamped, cnt.velCb)

    # Subscribe to laser pixel coordinates
    rospy.Subscriber('laser_alignment/coordinates', Pixel_coordinates, cnt.pxl_coordCb)

    # Subscribe to joystick topic
    rospy.Subscriber('joy', Joy, cnt.joyCb)

    # Setpoint publisher
    sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size = 1)

    joy_pub = rospy.Publisher('/joy', Joy, queue_size=1)
    
    # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
    k = 0
    while k < 10:
        sp_pub.publish(cnt.sp)
        rate.sleep()
        k = k + 1

    # start with 
    # activate OFFBOARD mode
    cnt.modes.setOffboardMode()
    print("OFFBOARD mode active")
    # ROS main loop
    while not rospy.is_shutdown():
        cnt.updateSp()
        sp_pub.publish(cnt.sp)
        
        #dist_anchor_tag_pub.publish(cnt.distance_anchor_tag)
        #vel_sp_pub.publish(cnt.vel_sp)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
