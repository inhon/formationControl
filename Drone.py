from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil # Needed for command message definitions
import numpy as np
import time
from geopy.distance import geodesic
from helpers import calculate_distance_lla


class Drone():
    # (22.9049399147239,120.272397994995,27.48,0) 長榮大學 圖書館前 機頭朝北
    def __init__(self, connection_string):  
        print("Connecting to vehicle on: %s" % connection_string)
        self.connected = True
        self.home=None
        try:
            self.vehicle = connect(connection_string, wait_ready=True)            
        except Exception as e:
            print(e)
            self.connected = False
             
    def set_guided_and_arm(self):
        """
        Set the UAV to GUIDED mode and arm the UAV 
        """
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)
        
        # Copter should arm in GUIDED mode
        while self.vehicle.mode != VehicleMode("GUIDED"):
            self.vehicle.mode = VehicleMode("GUIDED")
            time.sleep(1)

        self.vehicle.armed = True
        # Confirm vehicle armed
        while not self.vehicle.armed:
            self.vehicle.armed = True
            print(" Waiting for arming...")
            time.sleep(1)
        # Let the propeller spin for a while to warm up so as to increase stability during takeoff
        time.sleep(2)

    def takeoff(self, aTargetAltitude):
        """
        In Guided mode, take off the UAV to the target altitude (aTargetAltitude). 
        """
        self.vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude
        # Wait until the vehicle reaches a safe height
        while True:
            if self.vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
               break
            time.sleep(1)
  
    def land(self):
        while(self.vehicle.mode != VehicleMode("LAND")):
            self.vehicle.mode = VehicleMode("LAND")
            time.sleep(0.2)
        print("Landing")
    
    def get_state(self):
        """
        Return the states of the UAV in a dictionary
        """
        stateobj = {
            "Mode" : self.vehicle.mode.name,
            "BatteryVoltage" :self.vehicle.battery.voltage, 
            "BatteryCurrent" :self.vehicle.battery.current,
            "BatteryLevel":self.vehicle.battery.level,
            "IsArmable" : self.vehicle.is_armable,
            "armed" : self.vehicle.armed,
            "airspeed": self.vehicle.airspeed,
            "SystemStatus" : self.vehicle.system_status.state,
            "GlobalLat" : self.vehicle.location.global_frame.lat,
            "GlobalLon" : self.vehicle.location.global_frame.lon,
            "SeaLevelAltitude" : self.vehicle.location.global_frame.alt,
            "RelativeAlt" : self.vehicle.location.global_relative_frame.alt,
            "localAlt":self.vehicle.location.local_frame.down
        }
        if(self.vehicle.home_location!=None):
            stateobj["homeLocationAlt"]=self.vehicle.home_location.alt
            stateobj["homeLocationLat"]=self.vehicle.home_location.lat
            stateobj["homeLocationLon"]=self.vehicle.home_location.lon

        #print(stateobj)
        return stateobj
    
    def get_home_location(self):
        while self.vehicle.home_location is None:
            print(f"waiting for UAV home location.")
            self.vehicle.commands.download()
            self.vehicle.commands.wait_ready()
            time.sleep(1)
        self.home=self.vehicle.home_location
        print(f"get UAV home location.")
        return self.home      

    def condition_yaw(self,heading, relative=False):
        """
        yaw speed: 10 deg/s
        """
        if relative:
            is_relative = 1 #yaw relative to direction of travel
        else:
            is_relative = 0 #yaw is an absolute angle
        # create the CONDITION_YAW command using command_long_encode()
        msg =self.vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0, #confirmation
            heading,    # param 1, yaw in degrees
            10,         # param 2, yaw speed deg/s
            1,          # param 3, direction -1 ccw, 1 cw
            is_relative, # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        # send command to vehicle
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def send_global_velocity(self, north, east, down=0):
        
        msg = self.vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, # lat_int - X Position in WGS84 frame in 1e7 * meters
        0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        north, # X velocity in NED frame in m/s
        east, # Y velocity in NED frame in m/s
        down, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()
        # send command to vehicle on 1 Hz cycle
        #for x in range(0,duration):
        #    vehicle.send_mavlink(msg)
        #    time.sleep(1)   
    
    def read_global_position(self):
        """
        return LocationGlobalRelative: (p.lat, p.lon, p.alt)
        """
        return self.vehicle.location.global_relative_frame
    
    def read_local_velocity(self):
        """
        return a list [vx, vy, vz] in meter/sec
        """
        return self.vehicle.velocity
 
    def send_global_velocity(self, north, east, down=0):        
        msg = self.vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, # lat_int - X Position in WGS84 frame in 1e7 * meters
        0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        north, # X velocity in NED frame in m/s
        east, # Y velocity in NED frame in m/s
        down, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()
        
    def close_conn(self):
        #print("Close connection to vehicle")
        self.vehicle.close()

    def set_rtl_alt(self, rtl_alt=2000):
        self.vehicle.parameters['RTL_ALT']=rtl_alt
        while self.vehicle.parameters['RTL_ALT'] != rtl_alt:
            self.vehicle.parameters['RTL_ALT']=rtl_alt
            time.sleep(1)
        self.vehicle.parameters['WP_YAW_BEHAVIOR']=1
        while self.vehicle.parameters['WP_YAW_BEHAVIOR'] !=1:
            self.vehicle.parameters['WP_YAW_BEHAVIOR']=1
            time.sleep(1)
        
        return True
    
    def rtl(self):
        while(self.vehicle.mode != VehicleMode("RTL")):
            self.vehicle.mode = VehicleMode("RTL")
            time.sleep(0.2)
        while True:
            current_location = self.vehicle.location.global_relative_frame
            distance = calculate_distance_lla(current_location, self.home)
            if distance < 1.5:  # 設定 1.5 米的容忍範圍
                break
            time.sleep(0.5)           

    
    




