from dronekit import LocationGlobalRelative, LocationGlobal
from Drone import Drone
import numpy as np
from helpers import calculate_desired_positions_global, calculate_yaw_angle, interpolate_waypoints
import time
from geopy.distance import geodesic


''' Loosely based on the following paper: https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=6798711'''

class FormationFlying(object):
    def __init__(self, num_uavs: int, port: int, takeoff_altitude: int, collision_threshold: int=10.0, rtl_alt: int =2000):
        self.num_uavs = num_uavs
        self.drones = []
        for i in range(num_uavs):
            #self.drones.append(Vehicle(f'udpin:localhost:{port + 10 * i}'))
            self.drones.append(Drone(f'tcp:localhost:{port + 10 * i}')) #port: 5762

        self.takeoff_altitude = takeoff_altitude # meter
        self.max_velocity = 4  # Reduced for better control(m/sec)
        self.wp_radius = 1.5  # Adjusted for better tolerance
        self.formation_tolerance = 0.5  # Tolerance for formation achievement
        self.collision_threshold = collision_threshold
        self.rtl_alt=rtl_alt #cm
        
        # Formation control gains
        self.K1 = np.array([[0.7, 0], [0, 0.7]])  # Acts on the position error (Proportional Gain)
        self.damping = 0.4  # Damping factor to control high frequency velocity changes (overshoot and oscillations)
       
        # Define square formation offsets (in meters, relative to the formation center)
        self.formation_offsets = [
            np.array([-10, -10]), # north, east , unit: meter 
            np.array([10, -10]),
            np.array([-10, 10]),
            np.array([10, 10])            
        ]
        """
        2          3
           center
        0          1
        """
    
    def set_rtl_alt_all(self):
        for index, drone in enumerate(self.drones):
            if (drone.set_rtl_alt(self.rtl_alt)==True):
                print(f"set the UAV {index} RTL_ALT successful")
                
    def limit_vector(self, vector):
        if np.linalg.norm(vector) > self.max_velocity:
            vector = (vector / np.linalg.norm(vector)) * self.max_velocity
        return vector
    
    def calculate_formation_velocity(self, i, uav_i_pos):
        ''' 
        This function calculates a velocity component which would act as repulsion
        bwtween the UAVs if they are closer than a certain threshold (10m for instance).
        This would enable inter-UAV collision avoidance.
        return np.array([north_vel east_vel])
        '''

        #self.collision_threshold = 10
        v_formation = np.array([0.0, 0.0])
        for j in range(self.num_uavs):
            if i != j:
                uav_j_pos = self.drones[j].read_global_position() #LocationGlobalRelative
                uav_i_to_uav_j_dst = geodesic((uav_i_pos.lat, uav_i_pos.lon), (uav_j_pos.lat, uav_j_pos.lon)).meters
                # print(f"Distance between UAV {i} and UAV {j}: {uav_i_to_uav_j_dst}")
                # desired_uav_j_pos = np.array([uav_i_pos.lat + self.formation_offsets[j][0], uav_i_pos.lon + self.formation_offsets[j][1]])
                
                if uav_i_to_uav_j_dst < self.collision_threshold:
                    #print(f"Collision Detected between UAV {i} and UAV {j}!")
                    w = (uav_i_to_uav_j_dst - self.collision_threshold) / uav_i_to_uav_j_dst
                    #w(+)遠離時會將無人機聚集，w(-)太靠近時會推開
                    direction_vector = np.array([uav_j_pos.lat - uav_i_pos.lat, uav_j_pos.lon - uav_i_pos.lon])
                    
                    normalized_direction = direction_vector / np.linalg.norm(direction_vector)
                    v_formation += 50 * w * normalized_direction
                    #print(f"Formation Velocity for UAV {i} due to UAV {j}: {v_formation}")
        return v_formation 

    def calculate_control_input_global(self, current_pos, desired_pos, velocity, i):
        """
        return np.array([north_vel east_vel])
        """
        current_pos = np.array([current_pos.lat, current_pos.lon])
        desired_pos = np.array([desired_pos.lat, desired_pos.lon])

        # Calculate error in meters
        error_meters = np.array([
            geodesic((current_pos[0], current_pos[1]), (desired_pos[0], current_pos[1])).meters,
            geodesic((current_pos[0], current_pos[1]), (current_pos[0], desired_pos[1])).meters
        ])
        # [diff_lat, diff_lon]
        # Adjust sign based on direction
        if desired_pos[0] < current_pos[0]:
            error_meters[0] = -error_meters[0]
        if desired_pos[1] < current_pos[1]:
            error_meters[1] = -error_meters[1]

        velocity = np.array(velocity) #velocity:a list [vx, vy] in meter/sec
        control_input_vel = self.K1 @ error_meters - self.damping * velocity

        formation_velocity = self.calculate_formation_velocity(i, self.drones[i].read_global_position())
        control_input_vel += formation_velocity

        return self.limit_vector(control_input_vel)
    
    def initialize_formation(self, formation_center=None): # take off and format UAVs  
        print("Starting Mission!")
        self.home=[]
        for i in range(self.num_uavs):
            home=self.drones[i].get_home_location()
            print(f"UAV {i} home location set: {home.lat}, {home.lon}, {home.alt}")
            self.home.append(home)          
 
        while(input("\033[93m {}\033[00m" .format("Change UAVs to GUIDED mode and takeoff? y/n\n")) != "y"):
            pass

        for i in range(self.num_uavs): # change drone to GUIDED mode and arm
            self.drones[i].set_guided_and_arm()
            print(f"UAV {i} changed mode to GUIDED and armed successfully!")
            self.drones[i].takeoff(self.takeoff_altitude)
            print(f"UAV {i} took off successfully!") 
        
        while(input("\033[93m {}\033[00m" .format("Make sure All UAVs in the air! ? y/n\n")) != "y"):
            pass
        # Formation Initialization
        self.yaw = [0,0,0,0]
        # formation_center = self.drones[0].read_global_position()  # Use the first drone position as the formation center
        desired_positions = calculate_desired_positions_global(formation_center, self.formation_offsets) 

        forming = True
        self.yaw_update_interval =30  # Correct yaw every 30 seconds during waypoint following
        self.yaw_update_time = time.time()

        print("Initializing Formation!")
        #print("Desired Positions:", [(desired_positions[i].lat, desired_positions[i].lon) for i in range(self.num_uavs)])
        while forming:
            forming = False
            for i in range(self.num_uavs):
                current_pos = self.drones[i].read_global_position()
                desired_pos = desired_positions[i] #LocationGlobalRelative(lat, lon , 0)
                velocity = np.array(self.drones[i].read_local_velocity()) #a list [vx, vy, vz] in meter/sec

                self.yaw[i] = calculate_yaw_angle(current_pos, formation_center) #yaw_degree
                control_input = self.calculate_control_input_global(current_pos, desired_pos, velocity[:2], i)
                #current_pos, desired_pos : #LocationGlobalRelative(lat, lon , 0)                
                self.drones[i].condition_yaw(self.yaw[i]) #yaw speed: 90 degree/sec
                self.drones[i].send_global_velocity(control_input[0],control_input[1])
                # Check if the UAV is within the formation radius with tolerance
                distance_to_formation = geodesic((current_pos.lat, current_pos.lon), (desired_pos.lat, desired_pos.lon)).meters
                # print(f"Drone {i} Distance to Formation: {distance_to_formation}")
                if distance_to_formation > self.wp_radius + self.formation_tolerance:
                    forming = True

            time.sleep(1) #0.5
        print("Formation Achieved! Proceeding to Waypoints")

    def waypoint_following(self, waypoint):

        formation_center = waypoint #LocationGlobalRelative(lat, lon , 0)
        desired_positions = calculate_desired_positions_global(formation_center, self.formation_offsets)
        #a list of LocationGlobalRelative(lat, lon , 0)
        reached_waypoint = [False] * self.num_uavs
               
        #while not reached_waypoint:
        while not all(reached_waypoint):  #檢查每台UAV是否到達，還未到達者要繼續移動
            for i in range(self.num_uavs):
                if not reached_waypoint[i]: 
                    current_pos = self.drones[i].read_global_position() #LocationGlobalRelative
                    desired_pos = desired_positions[i]
                    velocity = np.array(self.drones[i].read_local_velocity())

                    control_input = self.calculate_control_input_global(current_pos, desired_pos, velocity[:2], i)
                    self.drones[i].send_global_velocity(control_input[0],control_input[1])
                    # Check if the UAV is within the waypoint radius
                    distance_to_waypoint = geodesic((current_pos.lat, current_pos.lon), (desired_pos.lat, desired_pos.lon)).meters
                    if distance_to_waypoint < self.wp_radius:
                        reached_waypoint[i] = True
                time.sleep(0.2)
            
            """
            # Periodically update yaw
            if time.time() - self.yaw_update_time > self.yaw_update_interval:
                for i in range(self.num_uavs):
                    current_pos = self.drones[i].read_global_position()
                    self.yaw[i] = calculate_yaw_angle(current_pos, formation_center)
                    #print(f"Yaw for UAV {i}: {self.yaw[i]}")
                    self.drones[i].condition_yaw(self.yaw[i])
                self.yaw_update_time = time.time()

                time.sleep(0.2)
            """ 
        time.sleep(0.2)

    def rtl_all(self):
        for drone in self.drones:
            drone.rtl()
            #time.sleep(0.2)

if __name__ == "__main__":
    num_uavs = 4
    port = 5762
    takeoff_altitude = 15
    #formation_center=LocationGlobalRelative(22.9051598, 120.2721620 , 0) 
    waypoints = [
        LocationGlobalRelative(22.9051598, 120.2721620, 0), # formation_center
        LocationGlobalRelative(22.9061728, 120.2742004, 0),
        LocationGlobalRelative(22.9067361, 120.2721834, 0),
        LocationGlobalRelative(22.9056885, 120.2706814, 0),
        LocationGlobalRelative(22.9051598, 120.2721620, 0)  # formation_center
    ]
    new_waypoints=[]
    formation_flying = FormationFlying(num_uavs, port, takeoff_altitude)# takeoff_altitude=15 m
    formation_flying.set_rtl_alt_all() #RTL_ALT=2000 cm
    formation_flying.initialize_formation(waypoints[0])
    # interpolate_waypoints
    for i in range(len(waypoints) - 1):
        segment = [waypoints[i], waypoints[i + 1]]
        interpolated = interpolate_waypoints(segment, 3)
        new_waypoints.extend(interpolated)
    #perform mission
    for waypoint in new_waypoints:
        formation_flying.waypoint_following(waypoint)
    #return to home locations
    formation_flying.rtl_all()
    
    print("Mission Completed!")

    