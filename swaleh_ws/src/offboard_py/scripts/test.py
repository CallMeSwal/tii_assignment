#!/usr/bin/env python

import rospy

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped

from geographic_msgs.msg import GeoPoseStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

import inspect

class fcuModes:
    def __init__(self):
        pass

    def setTakeoff(self):
    	rospy.wait_for_service('mavros/cmd/takeoff')
    	try:
    		takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
    		takeoffService(altitude = 3)
    	except rospy.ServiceException, e:
    		print "Service takeoff call failed: %s"%e

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

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Offboard Mode could not be set."%e
    
    def setMissionMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.MISSION')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Mission Mode could not be set."%e
    
    def setMode(self, mode):
        rospy.wait_for_service('mavros/set_mode')
        try:
            print("setting mode", mode)
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode=mode)
        except rospy.ServiceException, e:
            print("service set_mode call failed:", e, ". ", mode, " mode could not be set.")
    
    def pullMission(self):
        rospy.wait_for_service('mavros/mission/pull')
        try:
            pullMissionService = rospy.ServiceProxy('mavros/mission/pull', mavros_msgs.srv.WaypointPull)
            pullMissionService()
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Mission could not be pulled."%e
        
    def clear_points(self):
        rospy.wait_for_service('mavros/mission/clear')
        try:
            armService = rospy.ServiceProxy('mavros/mission/clear', mavros_msgs.srv.WaypointClear)
            armService(False)
        except rospy.ServiceException, e:
            print "Service disarming call failed: %s"%e
    

# Main class: Converts joystick commands to position setpoints
class Controller:
    # initialization method
    def __init__(self):
        # Drone state
        self.state = State()
        # Instantiate a setpoints message
        self.sp         = PositionTarget()
        # set the flag to use position setpoints and yaw angle
        self.sp.type_mask    = int('010111111000', 2)
        # LOCAL_NED
        self.sp.coordinate_frame= 1

        # We will fly at a fixed altitude for now
        # Altitude setpoint, [meters]
        self.ALT_SP        = rospy.get_param('alt_sp', 1.0)
        # update the setpoint message with the required altitude
        self.sp.position.z    = self.ALT_SP

        # Instantiate a joystick message
        #self.joy_msg        = Joy()
        # initialize
        #self.joy_msg.axes=[0.0, 0.0, 0.0]

        # Step size for position update
        #self.STEP_SIZE = rospy.get_param('joy_step_scale',2.0)

        # Fence. We will assume a square fence for now
        #self.FENCE_LIMIT = 5.0

        # A Message for the current local position of the drone
        self.local_pos = Point(0.0, 0.0, 0.0)

        #Waypoint uploaded to drone
        self.waypoint_list = None

        self.waypoint_counter = 0
        self.reached_waypoints = []

        self.mode = "AUTO.MISSION"
        self.clear = False
    
    # Getters and Setters
    def get_waypoint_list(self):
        return self.waypoint_list

    def is_armed(self):
        if self.state.armed:
            return self.state.armed
        return self.state.armed    
    def get_state_mode(self):
        return self.state.mode
    def get_req_mode(self):
        return self.mode

    # Callbacks

    ## local position callback
    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z

    ## joystick callback
#    def joyCb(self, msg):
#        self.joy_msg = msg

    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg
    
    ## Drone State callback
    def waypoints_cb(self, msg):
        self.waypoint_list = msg
    
    ## Drone State callback
    def waypoint_counter_cb(self, msg):
        if msg.wp_seq not in self.reached_waypoints:
            self.reached_waypoints+=[msg.wp_seq]
            self.waypoint_counter+=1
            print("current_waypoint", self.waypoint_counter)
            if self.waypoint_counter==2:
                self.mode = "OFFBOARD"
            elif self.waypoint_counter==8:
                self.mode = "AUTO.MISSION"

    ## Update setpoint message
#    def updateSp(self):
#        x = -1.0*self.joy_msg.axes[0]
#        y = self.joy_msg.axes[1]

#        self.sp.position.x = self.local_pos.x + self.STEP_SIZE*x
#        self.sp.position.y = self.local_pos.y + self.STEP_SIZE*y


# Main function
# takes of in offboard mode at current positin to  self.ALT_SP altitude, specified in the takeoff_test_params.yaml
def main():
    # First Time
    first_time = True
    # initiate node
    rospy.init_node('takeoff_offboard_test_node', anonymous=True)

    # flight mode object
    fcu_modes = fcuModes()

    # controller object
    cnt = Controller()

    # ROS loop rate, [Hz]
    rate = rospy.Rate(20.0)

    # Subscribe to drone state
    rospy.Subscriber('mavros/state', State, cnt.stateCb)

    # Subscribe to drone's local position
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)
    
    # subscribe to joystick topic
    #rospy.Subscriber('joy', Joy, cnt.joyCb)

    # Subscribe to drone's waypoint list
    rospy.Subscriber('mavros/mission/waypoints', mavros_msgs.msg.WaypointList, cnt.waypoints_cb)

    # Subscribe to drone's reached waypoints
    rospy.Subscriber('mavros/mission/reached', mavros_msgs.msg.WaypointReached, cnt.waypoint_counter_cb)

    # Setpoint publisher    
    sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)
    sp_pub_glb = rospy.Publisher('mavros/setpoint_position/global', GeoPoseStamped, queue_size=1)
    #sp_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    #Make sure the drone is armed
    while not cnt.is_armed():
        fcu_modes.setArm()
        rate.sleep()
    
    #Download mission from drone
    mission = cnt.get_waypoint_list()
    print(mission)

    #Create middle setpoints
    middle_setpoints = []
    for point in mission:
        print(point)
        middle_point = GeoPoseStamped()
        middle_point.position.x = 0 
        middle_point.position.y = 0
        middle_point.position.z = 0

    # ROS main loop
    while not rospy.is_shutdown():
        if cnt.get_state_mode()!=cnt.get_req_mode():
            if cnt.get_req_mode() == "AUTO.MISSION":
                fcu_modes.setMissionMode()
                print("auto_mission")
            elif cnt.get_req_mode() == "OFFBOARD":
                if first_time:
                    first_time=False
                    for i in range(100):
                        sp_pub.publish(pose)

                fcu_modes.setOffboardMode()
                print("offboard")
            '''
            print("changing_mode", cnt.get_state_mode(), cnt.get_req_mode()) 
            fcu_modes.setMode(cnt.get_req_mode())
            print("hi_here")
            '''
        #sp_pub.publish(cnt.sp)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass