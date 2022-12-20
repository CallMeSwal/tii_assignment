#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Point, PoseStamped
from geographic_msgs.msg import GeoPoseStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *

import inspect

class fcuModes:
    def __init__(self):
        pass

    def set_takeoff(self):
    	rospy.wait_for_service('mavros/cmd/takeoff')
    	try:
    		takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
    		takeoffService(altitude = 3)
    	except rospy.ServiceException, e:
    		print "Service takeoff call failed: %s"%e

    def set_arm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print "Service arming call failed: %s"%e

    def set_disarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException, e:
            print "Service disarming call failed: %s"%e

    def set_offboard_mode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Offboard Mode could not be set."%e
    
    def set_mission_mode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.MISSION')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Mission Mode could not be set."%e
    
    def pull_mission(self):
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
    

# Drone Controller, updates drone values like mode, state, etc
class Controller:
    # initialization method
    def __init__(self):
        # Drone state
        self.state = State()
        self.sp = PositionTarget()
        self.sp.type_mask    = int('010111111000', 2)
        self.sp.coordinate_frame= 1
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
    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z
    
    def stateCb(self, msg):
        self.state = msg
    
    def waypoints_cb(self, msg):
        self.waypoint_list = msg
    
    def waypoint_counter_cb(self, msg):
        if msg.wp_seq not in self.reached_waypoints:
            self.reached_waypoints+=[msg.wp_seq]
            self.waypoint_counter+=1
            print("current_waypoint", self.waypoint_counter)
            if self.waypoint_counter==2:
                self.mode = "OFFBOARD"
            elif self.waypoint_counter==8:
                self.mode = "AUTO.MISSION"

def main():
    # initiate node
    rospy.init_node('run_drone', anonymous=True)

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

    # Subscribe to drone's waypoint list
    rospy.Subscriber('mavros/mission/waypoints', mavros_msgs.msg.WaypointList, cnt.waypoints_cb)

    # Subscribe to drone's reached waypoints
    rospy.Subscriber('mavros/mission/reached', mavros_msgs.msg.WaypointReached, cnt.waypoint_counter_cb)

    # Setpoint publisher    
    sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)
    sp_pub_glb = rospy.Publisher('mavros/setpoint_position/global', GeoPoseStamped, queue_size=1)

    # Wait until drone is armed 
    while not cnt.is_armed():
        fcu_modes.set_arm()
        rate.sleep()
    
    # Download mission from drone
    mission = cnt.get_waypoint_list()

    '''
    # Transcribe waypoint into position setpoints for the drone to follow in offboard mode
    # Process to calculate setpoints
    # 1. Calcualte X,Y,Z difference between current waypoint and subsequent waypoint. This is known as the direction vector.
    # 2. Use mavros/local_position/pose topic to find local position of drone.
    # 3. Add direction vector to local position to find subseqquent setpoint for drone.
    # 4. Repeat for all remaining waypoints in mission.
    # Should be moved to controller class or made into seperate class
    middle_setpoints = []
    for point in mission:
        print(point)
        middle_point = GeoPoseStamped()
        middle_point.position.x = 0 
        middle_point.position.y = 0
        middle_point.position.z = 0
    '''

    # ROS main loop
    while not rospy.is_shutdown():
        if cnt.get_state_mode()!=cnt.get_req_mode():
            if cnt.get_req_mode() == "AUTO.MISSION":
                fcu_modes.set_mission_mode()
            elif cnt.get_req_mode() == "OFFBOARD":
                fcu_modes.set_offboard_mode()
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass