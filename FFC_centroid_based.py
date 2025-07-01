from dronekit import LocationGlobalRelative
from Drone import Drone
import numpy as np
from helpers import Waypoint, calculate_desired_positions_global, calculate_yaw_angle
import time
from geopy.distance import geodesic


''' Loosely based on the following paper: https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=6798711'''

class FormationFlying(object):
    def __init__(self, num_uavs: int, port: int, takeoff_altitude: int, collision_threshold=10.0):
        self.num_uavs = num_uavs
        self.drones = []
        for i in range(num_uavs):
            #self.drones.append(Vehicle(f'udpin:localhost:{port + 10 * i}'))
            self.drones.append(Drone(f'tcp:localhost:{port + 10 * i}')) #port: 5762

        self.takeoff_altitude = takeoff_altitude
        self.max_velocity = 1.2  # Reduced for better control
        self.wp_radius = 1.5  # Adjusted for better tolerance
        self.formation_tolerance = 0.5  # Tolerance for formation achievement
        self.collision_threshold = collision_threshold        

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

        velocity = np.array(velocity) 
        control_input_vel = self.K1 @ error_meters - self.damping * velocity

        formation_velocity = self.calculate_formation_velocity(i, self.drones[i].read_global_position())
        control_input_vel += formation_velocity

        return self.limit_vector(control_input_vel)
    
    def initialize_formation(self, formation_center=None): # take off and format UAVs  
        print("Starting Mission!")
        while(input("\033[93m {}\033[00m" .format("Change UAVs to GUIDED mode and arm? y/n\n")) != "y"):
            pass

        for i in range(self.num_uavs): # change drone to GUIDED mode and arm
            self.drone[i].set_guided_and_arm()
            print(f"UAV {i} changed mode to GUIDED and armed successfully!")

        while(input("\033[93m {}\033[00m" .format("Take off  UAVs ? y/n\n")) != "y"):
            pass
        
        #takeoff_check_flag = True
        for i in range(self.num_uavs): # take off all the UAVs
            self.drone[i].takeoff(self.take_off_altitude) 
            print(f"UAV {i} took off successfully!")

        #if not takeoff_check_flag:
        #    print("Error in arming and taking off! Aborting mission!")
        #    return

        print("All UAVs in the air!")

        # Formation Initialization
        self.yaw = [0,0,0,0]
        # formation_center = self.drones[0].read_global_position()  # Use the first drone position as the formation center
        desired_positions = calculate_desired_positions_global(formation_center, self.formation_offsets) 

        forming = True
        self.yaw_update_interval = 30  # Correct yaw every 30 seconds during waypoint following
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
                
                # Check if the UAV is within the formation radius with tolerance
                distance_to_formation = geodesic((current_pos.lat, current_pos.lon), (desired_pos.lat, desired_pos.lon)).meters
                # print(f"Drone {i} Distance to Formation: {distance_to_formation}")
                if distance_to_formation > self.wp_radius + self.formation_tolerance:
                    forming = True

            time.sleep(0.1)
        print("Formation Achieved! Proceeding to Waypoints")

    def waypoint_following(self, waypoint):

        formation_center = waypoint
        desired_positions = calculate_desired_positions_global(formation_center, self.formation_offsets)

        reached_waypoint = False
        while not reached_waypoint:
            for i in range(self.num_uavs):
                current_pos = self.drones[i].read_global_position()
                desired_pos = desired_positions[i]
                velocity = np.array(self.drones[i].read_local_velocity())

                control_input = self.calculate_control_input_global(current_pos, desired_pos, velocity[:2], i)

                # Check if the UAV is within the waypoint radius
                distance_to_waypoint = geodesic((current_pos.lat, current_pos.lon), (desired_pos.lat, desired_pos.lon)).meters
                # print(f"Drone {i} Distance to Waypoint: {distance_to_waypoint}")
                # print(distance_to_waypoint<self.wp_radius)
                if distance_to_waypoint < self.wp_radius:
                    reached_waypoint = True

                #if not reached_waypoint:
                #    self.drones[i].update_velocity_yaw(control_input[0], control_input[1], 0, self.yaw[i])

            # Periodically update yaw
            if time.time() - self.yaw_update_time > self.yaw_update_interval:
                for i in range(self.num_uavs):
                    current_pos = self.drones[i].read_global_position()
                    self.yaw[i] = calculate_yaw_angle(current_pos, formation_center)
                    # print(f"Yaw for UAV {i}: {self.yaw[i]}")
                    self.drones[i].condition_yaw(self.yaw[i])
                self.yaw_update_time = time.time()

                time.sleep(0.1)

        time.sleep(0.1)

if __name__ == "__main__":
    num_uavs = 4
    port = 5762
    takeoff_altitude = 10
    waypoints = [
        Waypoint(), #formation center
        Waypoint(22.9074872, 120.2767968),
        Waypoint(22.9104125, 120.2715182),
        Waypoint(22.9070326, 120.2673769)
    ]

    formation_flying = FormationFlying(num_uavs, port, takeoff_altitude)
    formation_flying.initialize_formation()
    for waypoint in waypoints:
        formation_flying.waypoint_following(waypoint)
    print("Mission Completed!")
    