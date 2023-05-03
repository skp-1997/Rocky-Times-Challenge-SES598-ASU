#! /usr/bin/python

# Import the libraries
import rospy
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Point, Quaternion, TwistStamped
import std_msgs.msg
import math
import numpy as np

from tf.transformations import quaternion_from_euler
from mavros_msgs.srv import CommandBool, SetMode, SetModeRequest, CommandBoolRequest, CommandTOL

radius = 8
omega = 0.5
# Define the class

class mapRock:

    def __init__(self):

        self.curr_drone_pose = PoseStamped()
        self.current_state = State()
        self.vel_pose = TwistStamped()
        self.des_pose = PoseStamped()
        self.isReadyToFly = False
        self.rate = rospy.Rate(5)
        self.orientation = quaternion_from_euler(0, 0, 0)
        self.distThreshold = 0.5
        
        drone_state = rospy.Subscriber("/mavros/state", State, callback=self.state_cb)
        self.pose_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
        self.arming_cl = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback=self.poseCallback)
        
        self.offboardCheck = False
        self.pose = PoseStamped()

        self.pose.pose.position.x = 0
        self.pose.pose.position.y = 0
        self.pose.pose.position.z = 5
        
        self.probe = np.array([40.5, 3.8, 15])
        self.rock = np.array([60.2, -7, 20.6]) #[58.20, -14.5, 19.5, #2]
        self.rover = np.array([12.6, -65.0, -3.0])
        self.reachAltitude = False
        self.reachedLocation = False
        self.orbitRock = True


    def state_cb(self, msg):
        self.current_state = msg

    def poseCallback(self, msg):
        self.curr_drone_pose = msg

    def drone_state_cb(self, msg):
        print('[INFO] Drone Mode : ', msg.mode)
        if msg.mode == 'OFFBOARD':
            self.isReadyToFly = True
            print('[INFO] Ready to Fly!')

    
    def set_Offboard(self):
        print('Setting mode')
        try:
            while not rospy.is_shutdown() and not self.offboardCheck:

                for i in range(15):

                    if (rospy.is_shutdown()):
                        break

                    self.pose_pub.publish(self.pose)
                    self.rate.sleep()
                set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
                mode = set_mode(custom_mode="OFFBOARD")
                self.offboardCheck = True
                self.isReadyToFly = True
                self.reachAltitude = True
                rospy.loginfo(mode)
                print(mode)

        except rospy.ServiceException as e:
            rospy.loginfo('SetMode is not Offboard')
    



    def arm_drone(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print("Service arm call failed: %s" % e)

    def disArm_drone(self):
        print('[INFO] Trying to DisArm!')
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            arming_cl = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
            response = arming_cl(value=False)
            rospy.loginfo(response)
        except rospy.ServiceException as e:
            print('[ALERT] DisArming failed : ', e)

    '''
    def offboard(self):
        # Wait for Flight Controller connection
        print('[INFO] Enabling offboard!')

        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True

        if (self.arming_cl.call(arm_cmd).success == True):
            rospy.loginfo("Vehicle armed")

        # Send a few setpoints before starting
        for i in range(30):
            if (rospy.is_shutdown()):
                break

            self.pose_pub.publish(self.pose)
            self.rate.sleep()

        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = 'OFFBOARD'

        if (self.set_mode_client.call(offb_set_mode).mode_sent == True):
            self.offboardCheck = True
            rospy.loginfo("OFFBOARD enabled")



        last_req = rospy.Time.now()
        while not rospy.is_shutdown() and not self.offboardCheck:
            
            if (self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if (self.set_mode_client.call(offb_set_mode).mode_sent == True):
                    self.offboardCheck = True
                    rospy.loginfo("OFFBOARD enabled")
                    break

                last_req = rospy.Time.now()
            else:
                
                if (not self.current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                    if (self.arming_cl.call(arm_cmd).success == True):
                        rospy.loginfo("Vehicle armed")

            last_req = rospy.Time.now()

    '''
                    
            

    '''
    def setmode_offb(self):
        rospy.wait_for_service("/mavros/cmd/arming")
        arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        rospy.wait_for_service("/mavros/set_mode")
        set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)

        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True
        if arming_client.call(arm_cmd).success == True:
            rospy.loginfo("Vehicle armed")
        for i in range(100):
            if rospy.is_shutdown():
                break
            self.local_pose_pub.publish(self.pose)
            self.rate.sleep()

        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = "OFFBOARD"
        if set_mode_client.call(offb_set_mode).mode_sent == True:
            self.offboardCheck = True
            rospy.loginfo("OFFBOARD enabled")

        last_req = rospy.Time.now()
        while not rospy.is_shutdown() and not self.offboardCheck:
            if self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if set_mode_client.call(offb_set_mode).mode_sent == True:
                    self.offboardCheck = True
                    rospy.loginfo("OFFBOARD enabled")
                    break
                last_req = rospy.Time.now()
            else:
                if not self.current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                    if arming_client.call(arm_cmd).success == True:
                        rospy.loginfo("Vehicle armed")
            last_req = rospy.Time.now()
    '''

    def takeOff(self):
        print('[INFO] Taking-off the Ground!')
        self.des_pose.pose.position.x = self.curr_drone_pose.pose.position.x
        self.des_pose.pose.position.y = self.curr_drone_pose.pose.position.y
        self.des_pose.pose.position.z = float(self.altitude)
        des = self.des_pose.pose.position
        print('[INFO] des : ',des)
        try:
            while not rospy.is_shutdown():
                self.isReadyToFly = True
                if self.isReadyToFly:
                    curr = self.curr_drone_pose.pose.position
                    dist = math.sqrt(
                        (curr.x - des.x)**2 + (curr.y - des.y)**2 + (curr.z - des.z)**2)
                    if dist > self.distThreshold:
                        self.pose_pub.publish(self.des_pose)
                        self.rate.sleep()
        except rospy.ServiceException as e:
            print('[ALERT] Not able to Take-Off!', e)


    def setStabilizeMode(self):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            isModeChanged = flightModeService(custom_mode='OFFBOARD')  # return true or false
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. GUIDED Mode could not be set. Check that GPS is enabled" % e)

    def setTakeoffMode(self):
        rospy.wait_for_service('/mavros/cmd/takeoff')
        try:
            takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
            takeoffService(altitude=5, latitude=0, longitude=0, min_pitch=0, yaw=0)
        except rospy.ServiceException as e:
            print("Service takeoff call failed: %s" % e)


    def land(self):
        print('[INFO] Trying to Land on site!')
        rospy.wait_for_service('/mavros/cmd/land')
        try:
            while not rospy.is_shutdown():
                land_cl = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
                response = land_cl(altitude=10, latitude=0, longitude=0, min_pitch=0, yaw=0)
                rospy.loginfo(response)
        except rospy.ServiceException as e:
            print('[ALERT] Not able to Land!', e)



    def got_to_location(self, location):
        print('[INFO] Heading to new location')
        self.reachedLocation = False
        self.des_pose.pose.position.x = location[0]
        self.des_pose.pose.position.y = location[1]
        self.des_pose.pose.position.z = location[2]
        self.des_pose.pose.orientation.x = self.orientation[0]
        self.des_pose.pose.orientation.y = self.orientation[1]
        self.des_pose.pose.orientation.z = self.orientation[2]
        self.des_pose.pose.orientation.w = self.orientation[3]
        des = self.des_pose.pose.position
        try:
            while not rospy.is_shutdown():
                if self.isReadyToFly:
                    curr = self.curr_drone_pose.pose.position
                    dist = math.sqrt(
                        (curr.x - des.x)**2 + (curr.y - des.y)**2 + (curr.z - des.z)**2)
                    if dist > self.distThreshold:
                        self.pose_pub.publish(self.des_pose)
                
                    else:
                        print('[INFO] Reached Location!')
                        self.reachedLocation = True
                        break
        except rospy.ServiceException as e:
            print('[ALERT] Not able to Fly to Location!', e)

    '''
    def revolve_rock(self):
        rospy.loginfo('Revolving around the rock initiated!')


        pass
    '''

    def revolve_rock(self):
        rospy.loginfo("Orbitting Rock")
        self.t0 = rospy.get_time()
        while not rospy.is_shutdown() and self.orbitRock:
            t = rospy.get_time() - self.t0
            if t > 15:
                self.orbitRock = False
                break
            '''
            self.pose.pose.position.x = self.rock[0] + (radius * math.cos(omega * t))
            self.pose.pose.position.y = self.rock[1] + (radius * math.sin(omega * t))
            self.pose.pose.position.z = self.rock[2]
            '''
            self.vel_pose.twist.linear.x = (-5 * math.cos(omega * t))
            self.vel_pose.twist.linear.y = (-5 * math.sin(omega * t))
            self.vel_pose.twist.linear.z = 0
            heading = math.atan2(self.curr_drone_pose.pose.position.y - 0,
                                 self.curr_drone_pose.pose.position.x - 0)
            
            self.vel_pose.twist.angular.z = 0.49    #self.orientation[2]
            self.vel_pose.header.stamp = rospy.Time.now()
            self.vel_pub.publish(self.vel_pose)
            self.rate.sleep()

    def activate_slam(self):
        pass


    def main(self):
        print('[INFO] Started!')
        while not rospy.is_shutdown() and not self.current_state.connected:
            self.rate.sleep()
            
        self.arm_drone()
        if not self.offboardCheck:
            self.set_Offboard()
        
        if self.reachAltitude:
            self.got_to_location(self.probe)

        if self.reachedLocation:
            self.orientation = quaternion_from_euler(0, 0, -3.14/2)
            self.got_to_location(self.rock)
        
        if self.reachedLocation:
            self.revolve_rock()
        
        if self.reachedLocation:
            self.got_to_location(self.rover)
        
        self.disArm_drone()
        
        print('[INFO] Mission Accomplished')
        


if __name__ == '__main__':
    rospy.init_node('offboard')
    obj = mapRock()
    obj.main()
    rospy.spin()


