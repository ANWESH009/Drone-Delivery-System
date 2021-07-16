import time
import math
import simulator
from stoppable_thread import StoppableThread
from flight_data import FlightData
from flight_commands import FlightCommands
from sensors import Sensors
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import argparse

# from vehicle import Vehicle


#################################################################################___merging___#####################################################################################################







parser = argparse.ArgumentParser()
parser.add_argument('--connect', default = '')
args = parser.parse_args()


connection_string = args.connect

#--------------------------------------------------
#-------------- FUNCTIONS  
#--------------------------------------------------
#-- Define arm and takeoff

def arm_and_takeoff(altitude):

   while not vehicle.is_armable:
      print("waiting to be armable")
      time.sleep(1)

   print("Arming motors")
   vehicle.mode = VehicleMode("GUIDED")
   vehicle.armed = True

   while not vehicle.armed: time.sleep(1)

   print("Taking Off")
   vehicle.simple_takeoff(altitude)

   while True:
      v_alt = vehicle.location.global_relative_frame.alt
      print(">> Altitude = %.1f m"%v_alt)
      if v_alt >= altitude - 1.0:
          print("Target altitude reached")
          break
      time.sleep(1)

 #-- Define the function for sending mavlink velocity command in body frame
def set_velocity_body(vehicle, vx, vy, vz):
    
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111, #-- BITMASK -> Consider only the velocities
            0, 0, 0,        #-- POSITION
            vx, vy, vz,     #-- VELOCITY
            0, 0, 0,        #-- ACCELERATIONS
            0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()
    

def clear_mission(vehicle):
    """
    Clear the current mission.
    """
    cmds = vehicle.commands
    vehicle.commands.clear()
    vehicle.flush()

    # After clearing the mission you MUST re-download the mission from the vehicle
    # before vehicle.commands can be used again
    # (see https://github.com/dronekit/dronekit-python/issues/230)
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()

def download_mission(vehicle):
    """
    Download the current mission from the vehicle.
    """
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready() # wait until download is complete.
    

def get_current_mission(vehicle):
    """
    Downloads the mission and returns the wp list and number of WP 
    
    Input: 
        vehicle
        
    Return:
        n_wp, wpList
    """

    print "Downloading mission"
    download_mission(vehicle)
    missionList = []
    n_WP        = 0
    for wp in vehicle.commands:
        missionList.append(wp)
        n_WP += 1 
        
    return n_WP, missionList
    

def ChangeMode(vehicle, mode):
    while vehicle.mode != VehicleMode(mode):
            vehicle.mode = VehicleMode(mode)
            time.sleep(0.5)
    return True


def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5



def distance_to_current_waypoint(vehicle):
    """
    Gets distance in metres to the current waypoint. 
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint = vehicle.commands.next
    if nextwaypoint==0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint

def bearing_to_current_waypoint(vehicle):
    nextwaypoint = vehicle.commands.next
    if nextwaypoint==0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    bearing = get_bearing(vehicle.location.global_relative_frame, targetWaypointLocation)
    return bearing

def get_bearing(my_location, tgt_location):
    """
    Aproximation of the bearing for medium latitudes and sort distances
    """
    dlat = tgt_location.lat - my_location.lat
    dlong = tgt_location.lon - my_location.lon
    
    return math.atan2(dlong,dlat)


def condition_yaw(heading, relative=False):
    
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,       # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0,          #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)


def saturate(value, minimum, maximum):
    if value > maximum: value = maximum
    if value < minimum: value = minimum
    return value

def add_angles(ang1, ang2):
    ang = ang1 + ang2
    if ang > 2.0*math.pi:
        ang -= 2.0*math.pi
    
    elif ang < -0.0:
        ang += 2.0*math.pi
    return ang

#--------------------------------------------------
#-------------- INITIALIZE  
#--------------------------------------------------      
#-- Setup the commanded flying speed
gnd_speed = 8 # [m/s]
radius    = 80
max_lat_speed = 4
k_err_vel   = 0.2
n_turns     = 3
direction   = 1 # 1 for cw, -1 ccw

mode      = 'GROUND'

#--------------------------------------------------
#-------------- CONNECTION  
#--------------------------------------------------    
#-- Connect to the vehicle
print('Connecting...')
vehicle = connect(connection_string)
#vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True)


#--------------------------------------------------
#-------------- MAIN FUNCTION  
#--------------------------------------------------    
while True:
    
    if mode == 'GROUND':
        #--- Wait until a valid mission has been uploaded
        n_WP, missionList = get_current_mission(vehicle)
        time.sleep(2)
        if n_WP > 0:
            print ("A valid mission has been uploaded: takeoff!")
            mode = 'TAKEOFF'
            
    elif mode == 'TAKEOFF':
        time.sleep(1)
        #-- Takeoff
        arm_and_takeoff(5)
        
      
        #-- Change mode, set the ground speed
        vehicle.groundspeed = gnd_speed
        mode = 'MISSION'
        vehicle.commands.next = 1
        
        vehicle.flush()

        #-- Calculate the time for n_turns
        time_flight = 2.0*math.pi*radius/gnd_speed*n_turns
        time0 = time.time()

        print ("Swiitch mode to MISSION")
        
    elif mode == 'MISSION':
        #-- We command the velocity in order to maintain the vehicle on track
        #- vx = constant
        #- vy = proportional to off track error
        #- heading = along the path tangent
        
        my_location = vehicle.location.global_relative_frame
        bearing     = bearing_to_current_waypoint(vehicle)
        dist_2_wp   = distance_to_current_waypoint(vehicle)
        
        try:
            print "bearing  %.0f  dist = %.0f"%(bearing*180.0/3.14, dist_2_wp)
            heading = add_angles(bearing,-direction*0.5*math.pi)
            #print heading*180.0/3.14
            condition_yaw(heading*180/3.14)
            
            v_x     = gnd_speed
            v_y     = -direction*k_err_vel*(radius - dist_2_wp)
            v_y     = saturate(v_y, -max_lat_speed, max_lat_speed)
            print "v_x = %.1f  v_y = %.1f"%(v_x, v_y)
            set_velocity_body(vehicle, v_x, v_y, 0.0)
            
        except Exception as e:
            print e


        if time.time() > time0 + time_flight: 
            ChangeMode(vehicle, 'RTL')    
            clear_mission(vehicle)        
            mode = 'BACK'
            print (">> time to head Home: switch to BACK")
            
    elif mode == "BACK":
        if vehicle.location.global_relative_frame.alt < 1:
            print (">> Switch to GROUND mode, waiting for new missions")
            mode = 'GROUND'
    
    
    
    
    time.sleep(0.5)

class ObstacleAvoidance(StoppableThread):
    # Class constants
    LEFT = "left"
    RIGHT = "right"
    UP = "up"
    DEVIATION_HORIZONTAL = 20
    DEVIATION_VERTICAL = 500
    NO_OBSTACLES_AHEAD = 1
    CAUTION_DISTANCE = 200    # Should be 4000, Measured in Centimeters
    DANGER_DISTANCE = 125     # Should be 1000, Measured in Centimeters
    CAUTION_ALTITUDE = 375    # Should be 3000, Measured in Centimeters.
    DANGER_ALTITUDE = 4000    # Should be 4000, Measured in Centimeters.
    CONSTANT_HEIGHT = 1000    # Should be 1000, Measured in Centimeters.
    MAX_LEFT_RIGHT = 0.875    # Should be 7.0, Maximum distance the drone would go right/left until trying to pass and obstacle from above. Measured in Meters.
    LEFT_SENSOR = "leftSensor"
    RIGHT_SENSOR = "rightSensor"
    FRONT_SENSOR = "frontSensor"
    BOTTOM_SENSOR = "bottomSensor"

    # In the final software, the __init__ function signature should look like:
    # def __init__(self, vehicle):
    # vehicle object is an object from external module that has been developed by a different person within
    # the company. Since this module use other modules which is not related
    # to this project, I put all the relevant code line for proper functioning in comment.

    # def __init__(self, vehicle):
    def __init__(self):
        # # Check for correct input
        # if isinstance(vehicle, Vehicle) is False:
        #     raise TypeError('Expected object of type Vehicle, got '+type(vehicle).__name__)

        # Calling super class constructor
        StoppableThread.__init__(self)

        # These are the distances the drone passed in a certain direction.
        # They are used to know if I need to try to pass the obstacle from another direction.
        self.__right_distance = 0.0
        self.__left_distance = 0.0
        self.__up_distance = 0.0

        # Always keep track of my last move in order to know better what to do in certain situations.
        self.__last_move = None

        # In case the drone need to pass an obstacle from above - keeping 2 flags. One for indicating its in the
        # process of climbing, and another one to indicate it finished climbing and should now keep its altitude
        # until passing the obstacle.
        self.__keep_altitude = False
        self.__climbing = False

        # I want to maintain the drone's altitude for a short period of time before descending back to the
        # original constant altitude, so for that I keep tracking of the time since it ascended.
        self.__start_time_measure = 0.0

        # In case of emergency, keeping a flag to order the drone to land.
        self.__safety_protocol = False

        # Other classes within the software for giving flight orders, get sensors data and
        # calculate distances by geographic positioning system data.
        # the __flight_commands & __location objects should get as a parameter the vehicle object
        self.__flight_commands = FlightCommands()   # FlightCommands(vehicle)
        self.__sensors = Sensors() 
        self.__flight_data = FlightData()           # FlightData(vehicle)

        # Get the drone's location and height for further calculations.
        self.__last_latitude = self.__flight_data.get_current_latitude()
        self.__last_longitude = self.__flight_data.get_current_longitude()
        self.__last_height = self.__flight_data.get_current_height()

        # Counter and flag for being aware of a sequence of illegal sensor inputs in order to be aware of a
        # malfunction in the system
        self.__illegal_input_counter = 0
        self.__last_input_legitimacy = True

        self.__num_of_lines = int(self.__get_number_of_line_in_file())  # Delete when moving to a full functioning system.

        # Initializing the sensors
        if self.__sensors.connect() == -1:
            raise ConnectionError('Cannot connect to sensors')

        print("Connected Successfuly to Sensors!")

        # Flag that indicates if the system is activated in order to give the user the knowledge if it should override
        # other flight commands given to the drone. The flag is 'True' only for left/right/up maneuvers and False
        # for all other cases including fixing the drone's altitude, since the altitude is fixed for 10 meters and
        # any other running software within the drone shouldn't change its height.
        self.__is_active = False

    def run(self):
        while not self.stopped():
            simulator.altitude_change()     # Delete when moving to a full functioning system.

            self.__num_of_lines -= 1        # Delete when moving to a full functioning system.
            if self.__num_of_lines == 0:    # Delete when moving to a full functioning system.
                self.stopit()               # Delete when moving to a full functioning system.
                continue                    # Delete when moving to a full functioning system.

            print("-----------------------------------------------------------")
            if self.__safety_protocol is True:
                self.__follow_safety_protocol()

            ahead_distance = self.__get_sensor_reading(self.FRONT_SENSOR)
            print("Distance Ahead: %d" % ahead_distance)

            # In case the path ahead is clear of obstacles, reset counters and fix altitude.
            if ahead_distance == self.NO_OBSTACLES_AHEAD or ahead_distance >= self.CAUTION_DISTANCE:
                print("Way is Clear")
                self.__check_flags()
                self.__fix_altitude()
                self.__last_move = None
                self.__is_active = False
                print("Is Active = " + str(self.__is_active))
                self.__right_distance = self.__left_distance = self.__up_distance = 0.0
                continue






           if ahead_distance <= self.DANGER_DISTANCE:
                # There's an obstacle in less than 10 meters - DANGER!
                # In such case the algorithm would slow down the drone drastically in order to give it more
                # time to manouver and avoid a collision.
                print("CAUTION: Obstacle in less than 1.25 meters!")
                print("Slowing Down to avoid collision")
            self.__flight_commands.hold(ahead_distance)
            else:
                print("Obstacle in less than 2 meters")

            




            # Get a reading from the left side sensor.
            left_side_distance = self.__get_sensor_reading(self.LEFT_SENSOR)
            # Get a reading from the right side sensor.
            right_side_distance = self.__get_sensor_reading(self.RIGHT_SENSOR)
            print("Distance Left: %d, Distance Right: %d" % (left_side_distance, right_side_distance))

           
