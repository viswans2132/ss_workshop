#!/usr/bin/env python
# ROS python API


import rospy

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped, TwistStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from nav_msgs.msg import Odometry
from mavros_msgs.srv import *
from std_msgs.msg import *
import numpy as np
from numpy import linalg as la
from tf.transformations import *

# import rosservice

import random
import string
import time

# Flight modes class
# Flight modes are activated using ROS services
class fcuModes:
    def __init__(self):
        pass

    def setTakeoff(self):
        rospy.wait_for_service('mavros/cmd/takeoff')
        try:
            takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
            takeoffService(altitude = 3)
        except rospy.ServiceException as e:
            print("Service takeoff call failed: ,%s")%e

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print("Service arming call failed: %s")%e

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException as e:
            print("Service disarming call failed: %s")%e

    def setStabilizedMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Stabilized Mode could not be set.")%e

    def setLoiterMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LOITER')
            # rosservice.call_service('mavros/set_mode', mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Loiter Mode could not be set.")%e

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
            # rosservice.call_service('mavros/set_mode', mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Offboard Mode could not be set.")%e

    def setAltitudeMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Altitude Mode could not be set.")%e

    def setPositionMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Position Mode could not be set.")%e

    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException as e:
               print("service set_mode call failed: %s. Autoland Mode could not be set.")%e

class Controller:
    # initialization method
    def __init__(self):
        # Drone state
        self.state = State()
        # Instantiate a setpoints message
        self.sp = PoseStamped()
        self.sp_yaw = -0.0
        self.pos_sp = np.array([-0.0, 0.0, 2.0])
        self.cur_pose = PoseStamped()
        self.cur_vel = TwistStamped()
        
        # Step size for position update
        self.STEP_SIZE = 2.0
        # Fence. We will assume a square fence for now
        self.FENCE_LIMIT = 5.0
        

        # initial values for setpoints
        self.sp.pose.position.x = 0.0
        self.sp.pose.position.y = 0.0
        self.ALT_SP = 2.0
        self.sp.pose.position.z = self.ALT_SP
        self.local_pos = Point(0.0, 0.0, self.ALT_SP)

        self.Kpos = np.array([-0.8, -0.8, -1.5])

        self.safety_flag = False


    # Callbacks

    ## local position callback
    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z


    def odomCb(self, msg):
        self.cur_pose.pose.position.x = msg.pose.pose.position.x
        self.cur_pose.pose.position.y = msg.pose.pose.position.y
        self.cur_pose.pose.position.z = msg.pose.pose.position.z

        self.cur_pose.pose.orientation.w = msg.pose.pose.orientation.w
        self.cur_pose.pose.orientation.x = msg.pose.pose.orientation.x
        self.cur_pose.pose.orientation.y = msg.pose.pose.orientation.y
        self.cur_pose.pose.orientation.z = msg.pose.pose.orientation.z

        q = [self.cur_pose.pose.orientation.x, self.cur_pose.pose.orientation.y, self.cur_pose.pose.orientation.z, self.cur_pose.pose.orientation.w]

        euler_angles = euler_from_quaternion(q)
        self.yaw = euler_angles[2]

        self.cur_vel.twist.linear.x = msg.twist.twist.linear.x
        self.cur_vel.twist.linear.y = msg.twist.twist.linear.y
        self.cur_vel.twist.linear.z = msg.twist.twist.linear.z

        self.cur_vel.twist.angular.x = msg.twist.twist.angular.x
        self.cur_vel.twist.angular.y = msg.twist.twist.angular.y
        self.cur_vel.twist.angular.z = msg.twist.twist.angular.z

    # Provide position setpoint
    def newPoseCB(self, msg):
        self.sp.pose.position.x = msg.pose.position.x
        self.sp.pose.position.y = msg.pose.position.y
        self.sp.pose.position.z = msg.pose.position.z

    # Provide yaw setpoint in degrees
    def yawAngleCB(self,msg):
        yaw_angle = msg.data
        quat = quaternion_from_euler(0.0, 0.0, yaw_angle)
        self.sp.pose.orientation.x = quat[0]
        self.sp.pose.orientation.y = quat[1]
        self.sp.pose.orientation.z = quat[2]
        self.sp.pose.orientation.w = quat[3]

    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    #### Useful datatype conversions
    def vector2Arrays(self, vector):        
        return np.array([vector.x, vector.y, vector.z])

    def vector3Arrays(self, vector):        
        return np.array([vector.x, vector.y, vector.z , vector.w])

    def array2Vector3(self, array, vector):
        vector.x = array[0]
        vector.y = array[1]
        vector.z = array[2]

    def array2Vector4(self, array, vector):
        vector.x = array[0]
        vector.y = array[1]
        vector.z = array[2]
        vector.w = array[3]
    ####

    # Not used currently
    def safety_filter(self, desVel, errPos):        
        P = np.eye(3)
        u = cp.Variable(3)

        a = 4.0

        l = errPos[1]*errPos[1] + errPos[2]*errPos[2]
        l = np.maximum(l, 0.001)

        h = errPos[0] - a*l**0.25

        dhdx = 1
        dhdy = -a*errPos[1]/(2*(l**0.75))
        dhdz = -a*errPos[2]/(2*(l**0.75))
        dhdp = np.array([dhdx, dhdy, dhdz])

        constraints = [dhdp@u >= -1.4*h]
        prob = cp.Problem(cp.Minimize(cp.quad_form(u-desVel, P)), constraints)
        result = prob.solve()

        val = u.value

        if np.linalg.norm(val) > 0.8:
            val = 0.8*val/np.linalg.norm(val)

        return val

    # Velocity controller
    def get_des_vel(self):
        curPos = self.vector2Arrays(self.cur_pose.pose.position)
        desPos = self.vector2Arrays(self.sp.pose.position) 

        yaw_sp = self.sp_yaw

        errPos = curPos - desPos
        desVel = self.Kpos*errPos


        if curPos[2] < 0.5:
            desVel[0] = 0.0
            desVel[1] = 0.0
            yaw_sp = self.yaw

        if la.norm(curPos - desPos)<0.1:
            self.safety_flag = True

        errYaw = self.yaw - yaw_sp

        if np.abs(errYaw) > np.pi:
            errYaw = np.sign(errYaw)*(np.abs(errYaw) - 2*np.pi)
        
        desYawVel = -0.3*errYaw


        if np.linalg.norm(desVel) > 1.0:
            desVel = (1.0/np.linalg.norm(desVel))*desVel
        return desVel, desYawVel

    # Velocity setpoint publisher
    def get_vel_sp(self):        
        desVel, desYawVel = self.get_des_vel()
        now = rospy.Time.now()

        vel_sp = TwistStamped()
        vel_sp.twist.linear.x = desVel[0]
        vel_sp.twist.linear.y = desVel[1]
        vel_sp.twist.linear.z = desVel[2]
        vel_sp.twist.angular.z = desYawVel
        vel_sp.header.stamp = now


# Main function
def main():
    rospy.init_node('setpoint_node', anonymous=True)
    modes = fcuModes()  #flight modes
    cnt = Controller()  # controller object
    rate = rospy.Rate(300)
    rospy.Subscriber('mavros/state', State, cnt.stateCb)

    # Subscribe to drone's local position
    rospy.Subscriber('mavros/local_position/odom', Odometry, cnt.odomCb)

    rospy.Subscriber('new_pose', PoseStamped, cnt.newPoseCB) #new waypoint
    rospy.Subscriber('yaw_in_deg',Float32,cnt.yawAngleCB)

    sp_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

    # Setpoint publisher    
    movement_cmd = AttitudeTarget()
    thrust_pub = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)

    modes.setLoiterMode()


    print("ARMING")
    while cnt.state.armed == False:
        modes.setArm()
        time.sleep(0.03)

    cnt.sp.pose.orientation.x = 0
    cnt.sp.pose.orientation.y = 0
    cnt.sp.pose.orientation.z = 0
    cnt.sp.pose.orientation.w = 1

    k=0
    while k<20:
        print("offboard trying")
        sp_pub.publish(cnt.sp)
        time.sleep(0.03)
        modes.setOffboardMode()
        k = k + 1

    time.sleep(0.5)

    print(cnt.state.mode)
    if cnt.state.mode == "OFFBOARD":
        print("---------")
        print("OFFBOARD")
        print("---------")


        # ROS main loop
        while not rospy.is_shutdown():
            vel_sp = cnt.get_vel_sp()
            vel_pub.publish(vel_sp)
            time.sleep(0.03)

    else:
        print("Unable to switch to offboard. Please try again. Shutting down the controller.")
        rospy.signal_shutdown("Velocity controller node shut down.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("Shutting down the velocity controller node. Error calling main loop.")
        rospy.signal_shutdown("Velocity controller node shut down.")
