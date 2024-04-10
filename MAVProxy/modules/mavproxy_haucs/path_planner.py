import math
import pandas as pd
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

###
### https://developers.google.com/optimization/routing/tsp
###

UAV_SPEED = {"WP_NAVSPEED": 1000,
            # "RTL_SPEED": 0, set if different from WP_NAVSPEED
            "LAND_SPEED": 50,
            "WPNAV_SPEED_UP": 250}

UAV_MAX_FTIME = 540 #seconds

def geo_measure(lat1, lon1, lat2, lon2):
    """Converts GPS coordinates to distances"""
    R = 6_378_137;  #Radius of earth in meters
    lat1 *= math.pi / 180
    lat2 *= math.pi / 180
    lon1 *= math.pi / 180
    lon2 *= math.pi / 180
    
    dx = (lon2 - lon1) * math.cos((lat2 + lat1)/2) * R
    dy = (lat2 - lat1) * R
    return dx, dy

def compute_euclidean_distance_matrix(locations):
    """Creates callback to return distance between points."""
    distances = {}
    for from_counter, from_node in enumerate(locations):
        distances[from_counter] = {}
        for to_counter, to_node in enumerate(locations):
            if from_counter == to_counter:
                distances[from_counter][to_counter] = 0
            else:
                # Euclidean distance
                distances[from_counter][to_counter] = int(
                    math.hypot((from_node[0] - to_node[0]), (from_node[1] - to_node[1]))
                )
    return distances

def distance_callback(from_index, to_index):
    """Returns the distance between the two nodes."""
    # Convert from routing variable Index to distance matrix NodeIndex.
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    return distance_matrix[from_node][to_node]

def create_data_model(file, home):
    df = pd.read_csv(file)

    num_points = df.shape[0]
    coords = [(home[0], home[1])]
    dists = [(0,0)]
    for i in range(num_points):
        lat = df.iloc[i].lat
        lon = df.iloc[i].lon
        x,y = geo_measure(home[0], home[1], lat, lon)
        dists.append((int(x), int(y)))
        coords.append((lat, lon))
    
    data = {}
    data["locations"] = dists
    data["num_vehicles"] = 1
    data["depot"] = 0
    data["coordinates"] = coords
    return data

def print_solution(manager, routing, solution):
    """Prints solution on console."""
    indices = []
    index = routing.Start(0)
    plan_output = "route:\n"
    route_distance = 0
    while not routing.IsEnd(index):
        plan_output += f" {manager.IndexToNode(index)} ->"
        previous_index = index
        index = solution.Value(routing.NextVar(index))
        route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
        indices.append(index)
    plan_output += f" {manager.IndexToNode(index)}\n"
    plan_output += f"distance: {route_distance}m"
    print(plan_output)
    return indices, route_distance

def generate_mission(file, home, coords, alt, land, delay):
    #start with header and a takeoff
    with open(file,'w') as m:
        m.write('QGC WPL 110\n')
        m.write(f"0\t0\t0\t16\t0\t0\t0\t0\t{home[0]}\t{home[1]}\t{home[2]}\t1\n") #set home
        m.write(f"0\t0\t0\t22\t0\t0\t0\t0\t0\t0\t{alt}\t1\n") #takeoff
        #set each pond/waypoint
        for i in coords:
            lat = i[0]
            lon = i[1]
            m.write(f"0\t0\t0\t16\t0\t0\t0\t0\t{lat}\t{lon}\t{alt}\t1\n") #nav_wp
            if land == 'True':
                m.write("0\t0\t0\t21\t0\t0\t0\t0\t0\t0\t0\t1\n") #land
            if delay > 0:
                m.write(f"0\t0\t0\t93\t{delay}\t0\t0\t0\t0\t0\t0\t1\n") #delay
            if land == 'True':
                m.write(f"0\t0\t0\t22\t0\t0\t0\t0\t0\t0\t{alt}\t1\n") #takeoff
        #set return to launch and disarm
        m.write("0\t0\t0\t20\t0\t0\t0\t0\t0\t0\t0\t1\n") #RTL
        #disarm
        m.write("0\t0\t0\t218\t41\t0\t0\t0\t0\t0\t0\t1\n") #DISARM

def estimate_missionTime(distance, wps, alt, land, delay):
    flight_time = distance / UAV_SPEED["WP_NAVSPEED"] * 100 #moving time
    if land == 'True':
        flight_time += (wps + 1) * alt / UAV_SPEED["LAND_SPEED"] * 100 #landing time
        flight_time += (wps + 1) * alt / UAV_SPEED["WPNAV_SPEED_UP"] * 100 #takeoff time
        total_time = flight_time + (wps * delay)
    else:
        flight_time += alt / UAV_SPEED["LAND_SPEED"] * 100 #landing time
        flight_time += alt / UAV_SPEED["WPNAV_SPEED_UP"] * 100 #takeoff time
        flight_time += (wps * delay)
        total_time = flight_time

    return flight_time, total_time

def main(source, output, home, alt, land, delay):
    """Entry point of the program."""
    # Instantiate the data problem.
    data = create_data_model(source, home)

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(data["locations"]), data["num_vehicles"], data["depot"]
    )

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    distance_matrix = compute_euclidean_distance_matrix(data["locations"])

    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    # search_parameters.log_search = True


    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        indices, distance = print_solution(manager, routing, solution)
        sorted_coords = [data["coordinates"][i] for i in indices[:-1]]
        generate_mission(output, home, coords=sorted_coords, alt=alt, land=land, delay=delay)
        
        flight_time, mission_time = estimate_missionTime(distance, len(indices) - 1, alt, land, delay)
        if mission_time >= UAV_MAX_FTIME:
            print("ROUTE SIZE WARNING: UAV may end mission early with low battery")
        print(f"estimated mission time: {mission_time//60}mins {round(mission_time%60)}secs")
        print(f" estimated flight time: {flight_time//60}mins {round(flight_time%60)}secs")
        print(f"   max UAV flight time: {UAV_MAX_FTIME//60}mins {UAV_MAX_FTIME%60}secs")
    else:
        raise Exception('no solution found, try again ...')