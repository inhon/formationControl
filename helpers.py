from dronekit import LocationGlobalRelative
from geopy import distance
from geopy.distance import geodesic
import math
import numpy as np

def calculate_distance_lla(pos1:LocationGlobalRelative, pos2:LocationGlobalRelative) ->float:
    # Calculate the distance between two points in meters
    return distance.distance((pos1.lat, pos1.lon), (pos2.lat, pos2.lon)).m

def calculate_desired_positions_global(formation_center:LocationGlobalRelative, offsets:np.array) -> list:
    """
    formation_center is a LocationGlobalRelative(lat, lon , 0) 
    return a list of LocationGlobalRelative(lat, lon , 0)
    """
    desired_positions = []
    for offset in offsets:
        # Calculate the new latitude and longitude based on the offset
        new_position = geodesic(meters=offset[0]).destination((formation_center.lat, formation_center.lon), 0)  # Offset in the north direction
        new_lat = new_position.latitude
        new_position = geodesic(meters=offset[1]).destination((new_lat, formation_center.lon), 90)  # Offset in the east direction
        new_lon = new_position.longitude
        desired_positions.append(LocationGlobalRelative(new_lat, new_lon, 0))
    return desired_positions

def interpolate_waypoints(waypoints: list, num_points: int)->list: 
    '''
    Interpolate between the waypoints given in the list to generate a new waypoint_list.
    waypoints: List of waypoints(LocationGlobalRelative(lat, lon , 0)) to interpolate between.
    num_points: Number of points to interpolate between two consecutive waypoints.
    '''
    interpolated_waypoints = []
    for i in range(len(waypoints) - 1):
        lat_diff = (waypoints[i + 1].lat - waypoints[i].lat) / num_points
        lon_diff = (waypoints[i + 1].lon - waypoints[i].lon) / num_points
        for j in range(num_points):
            interpolated_waypoints.append(LocationGlobalRelative(waypoints[i].lat + j * lat_diff, waypoints[i].lon + j * lon_diff, 0))
   
    interpolated_waypoints.append(waypoints[-1])
    return interpolated_waypoints

def calculate_yaw_angle(current_pos: LocationGlobalRelative, center_pos: LocationGlobalRelative) -> float:
    delta_lat = center_pos.lat - current_pos.lat
    delta_lon = center_pos.lon - current_pos.lon
    yaw_rad = math.atan2(delta_lon, delta_lat)  # 注意經度放在前面
    yaw_deg = math.degrees(yaw_rad)
    return (yaw_deg + 360) % 360  # 確保角度範圍是 [0, 360)