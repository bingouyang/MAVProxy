import math
import pandas as pd
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

###
### https://developers.google.com/optimization/routing/tsp
###

UAV_SPEED = {"WPNAV_SPEED": 1000,
            "LAND_SPEED": 50,
            "WPNAV_SPEED_UP": 250,
            "WPNAV_SPEED_DN": 100,
            "WPNAV_ACCEL":250,
            "WPNAV_ACCEL_Z":100}

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

def generate_mission(args, coords):
    """Generates WP Mission Files.txt
    """
    home = args['home']
    alt = args['alt']
    delay = args['delay']
    dive = args['dive']
    land = args['land']
    #start with header and a takeoff
    with open(args['output'],'w') as m:
        m.write('QGC WPL 110\n')
        m.write(f"0\t1\t0\t16\t0\t0\t0\t0\t{home[0]}\t{home[1]}\t{home[2]}\t1\n") #set home
        m.write(f"1\t0\t3\t22\t0\t0\t0\t0\t0\t0\t{alt}\t1\n") #takeoff
        #set each pond/waypoint
        index = 1
        for i in coords:
            lat = i[0]
            lon = i[1]
            index += 1
            m.write(f"{index}\t0\t3\t16\t0\t0\t0\t0\t{lat}\t{lon}\t{alt}\t1\n") #nav_wp
            if land == 'true':
                if dive != alt:
                    index += 1
                    m.write(f"{index}\t0\t3\t16\t0\t0\t0\t0\t{lat}\t{lon}\t{dive}\t1\n") #dive low
                index += 1
                m.write(f"{index}\t0\t3\t21\t0\t0\t0\t0\t0\t0\t0\t1\n") #land       
            if delay > 0:
                index += 1
                m.write(f"{index}\t0\t3\t93\t{delay}\t0\t0\t0\t0\t0\t0\t1\n") #delay
            if land == 'true':
                index += 1
                m.write(f"{index}\t0\t3\t22\t0\t0\t0\t0\t0\t0\t{alt}\t1\n") #takeoff

        index += 1
        m.write(f"{index}\t0\t3\t20\t0\t0\t0\t0\t0\t0\t0\t1\n") #RTL


# ── WINCH MISSION GENERATOR ───────────────────────────────────────────────────
# NEW FORMAT — distinct from generate_mission() above.
# DO NOT merge these two functions — servo sequence and landing logic differ.
#
# Mission structure per pond:
#   NAV_TAKEOFF (cruise_alt)
#   DO_SET_SERVO ch8=1500   neutral/safe at takeoff
#   NAV_WAYPOINT (pond)     fly to pond
#   NAV_LAND                land on water (FC gates here until touchdown confirmed)
#   NAV_DELAY (stab_sec)    stabilization after landing
#   DO_SET_SERVO ch8=1900   trigger winch DEPLOY
#   NAV_DELAY (soak_sec)    soak time (must cover RELEASE_SEC+PAUSE_SEC+RETRACT_SEC)
#                           set SCR_USER1/2/3 in Mission Planner to tune winch timing
#   NAV_TAKEOFF (cruise_alt)
#   CONDITION_CHANGE_ALT    wait until upload_alt reached
#   DO_SET_SERVO ch8=1200   trigger data UPLOAD at altitude
#   ... repeat per pond ...
#   RTL

_SERVO_CH      = 8
_SERVO_NEUTRAL = 1500   # safe/neutral — set at every takeoff
_SERVO_DEPLOY  = 1900   # winch deploy — set after landing confirmed
_SERVO_UPLOAD  = 1200   # data upload  — set after climbing to upload_alt

_CMD_TAKEOFF     = 22
_CMD_WAYPOINT    = 16
_CMD_LAND        = 21
_CMD_DELAY       = 93
_CMD_DO_SERVO    = 183
_CMD_COND_ALT    = 113
_CMD_RTL         = 20
_FRAME_REL       = 3    # relative altitude (AGL)
_FRAME_MISSION   = 2    # no position


def _wprow(seq, current, frame, cmd,
           p1=0, p2=0, p3=0, p4=0,
           lat=0, lon=0, alt=0, autocontinue=1):
    return (f"{seq}\t{current}\t{frame}\t{cmd}\t"
            f"{p1:.8f}\t{p2:.8f}\t{p3:.8f}\t{p4:.8f}\t"
            f"{lat:.8f}\t{lon:.8f}\t{alt:.6f}\t{autocontinue}")


def generate_mission_winch(args, coords):
    """
    Generates a winch-drone mission .waypoints file.

    *** NEW FORMAT — verify in Mission Planner before uploading ***
    *** DO NOT reuse old .waypoints files — servo sequence has changed ***

    args keys:
        home        (lat, lon, alt) tuple
        output      output filename (.waypoints)
        alt         cruise altitude AGL in metres
        upload_alt  minimum altitude before data upload trigger (m)
        stab_sec    stabilization delay after landing (s)
        soak_sec    soak time on water — must cover full winch cycle (s)
                    must be >= SCR_USER1 + SCR_USER2 + SCR_USER3
    coords:
        list of (lat, lon) tuples, TSP-sorted by caller
    """
    home       = args['home']
    alt        = float(args['alt'])
    upload_alt = float(args.get('upload_alt', min(10, alt)))
    stab_sec   = float(args.get('stab_sec', 3))
    soak_sec   = float(args.get('soak_sec', 60))
    output     = args['output']

    print()
    print("=" * 62)
    print("  HAUCS WINCH MISSION GENERATOR  *** NEW FORMAT ***")
    print("=" * 62)
    print(f"  Servo ch{_SERVO_CH} mapping:")
    print(f"    {_SERVO_NEUTRAL} = NEUTRAL  (set at every takeoff — safety)")
    print(f"    {_SERVO_DEPLOY} = DEPLOY   (set after landing on water confirmed)")
    print(f"    {_SERVO_UPLOAD} = UPLOAD   (set after climbing to {upload_alt}m)")
    print()
    print(f"  Cruise altitude : {alt}m AGL")
    print(f"  Upload altitude : {upload_alt}m AGL")
    print(f"  Stab delay      : {stab_sec}s after landing")
    print(f"  Soak time       : {soak_sec}s on water")
    print()
    print("  Winch timing set via Mission Planner SCR_USER params:")
    print("    SCR_USER1 = RELEASE_SEC  (descent time, default 20s)")
    print("    SCR_USER2 = PAUSE_SEC    (soak at bottom,  default 2s)")
    print("    SCR_USER3 = RETRACT_SEC  (retract time,  default 35s)")
    print(f"  WARNING: soak_sec ({soak_sec}s) must be >= SCR_USER1+USER2+USER3")
    print()
    print("  *** VERIFY ALL VALUES IN MISSION PLANNER BEFORE UPLOAD ***")
    print("=" * 62)

    rows = []
    seq  = 0

    # HOME (seq 0)
    rows.append(_wprow(seq, 1, 0, _CMD_WAYPOINT,
                       lat=home[0], lon=home[1], alt=home[2]))
    seq += 1

    for lat, lon in coords:
        # 1. NAV_TAKEOFF to cruise altitude
        rows.append(_wprow(seq, 0, _FRAME_REL, _CMD_TAKEOFF,
                           p1=0, lat=home[0], lon=home[1], alt=alt))
        seq += 1

        # 2. DO_SET_SERVO neutral at takeoff (safety)
        rows.append(_wprow(seq, 0, _FRAME_MISSION, _CMD_DO_SERVO,
                           p1=_SERVO_CH, p2=_SERVO_NEUTRAL))
        seq += 1

        # 3. NAV_WAYPOINT — fly to pond at cruise altitude
        rows.append(_wprow(seq, 0, _FRAME_REL, _CMD_WAYPOINT,
                           lat=lat, lon=lon, alt=alt))
        seq += 1

        # 4. NAV_LAND — descend and land on water
        #    FC gates here until touchdown confirmed by EXTENDED_SYS_STATE
        rows.append(_wprow(seq, 0, _FRAME_REL, _CMD_LAND,
                           lat=lat, lon=lon, alt=0))
        seq += 1

        # 5. NAV_DELAY — stabilization after landing
        rows.append(_wprow(seq, 0, _FRAME_MISSION, _CMD_DELAY,
                           p1=stab_sec))
        seq += 1

        # 6. DO_SET_SERVO — trigger winch DEPLOY
        rows.append(_wprow(seq, 0, _FRAME_MISSION, _CMD_DO_SERVO,
                           p1=_SERVO_CH, p2=_SERVO_DEPLOY))
        seq += 1

        # 7. NAV_DELAY — soak time (covers full winch cycle)
        #    Tune via SCR_USER1/2/3 in Mission Planner
        rows.append(_wprow(seq, 0, _FRAME_MISSION, _CMD_DELAY,
                           p1=soak_sec))
        seq += 1

        # 8. NAV_TAKEOFF — climb back to cruise altitude
        rows.append(_wprow(seq, 0, _FRAME_REL, _CMD_TAKEOFF,
                           p1=0, lat=lat, lon=lon, alt=alt))
        seq += 1

        # 9. CONDITION_CHANGE_ALT — wait until upload_alt reached
        #    Safe to use here: runs concurrently with NAV_TAKEOFF (active NAV cmd)
        rows.append(_wprow(seq, 0, _FRAME_REL, _CMD_COND_ALT,
                           p1=upload_alt, alt=upload_alt))
        seq += 1

        # 10. DO_SET_SERVO — trigger data UPLOAD at altitude
        rows.append(_wprow(seq, 0, _FRAME_MISSION, _CMD_DO_SERVO,
                           p1=_SERVO_CH, p2=_SERVO_UPLOAD))
        seq += 1

    # RTL
    rows.append(_wprow(seq, 0, _FRAME_MISSION, _CMD_RTL))
    seq += 1

    with open(output, 'w') as f:
        f.write("QGC WPL 110\n")
        for row in rows:
            f.write(row + "\n")

    print()
    print(f"  Saved: {output}")
    print(f"  Ponds: {len(coords)}  Items/pond: 10  Total items: {seq}")
    print()


def estimate_missionTime(hmodule, distances, wps, args):
    """
    Estimates the mission time for a given waypoint misison
    Includes:
    - drone vertical/horizontal speeds
    - drone vertical/horziontal accelerations
    - multi-speed landings
    """
    # UAV parameters
    # for i in UAV_SPEED:
    #     UAV_SPEED[i] = int(hmodule.get_mav_param(i))
    # mission parameters
    land = args['land']
    delay = args['delay']
    alt = args['alt']
    dive = args['dive']

    flight_time = 0

    horz_accel_t = UAV_SPEED["WPNAV_SPEED"] / UAV_SPEED["WPNAV_ACCEL"]
    horz_accel_d = UAV_SPEED["WPNAV_ACCEL"] / 100 * horz_accel_t**2 #distance to accel & deaccel

    #calculate flight times between waypoints
    for dist in distances:
        #hits max velocity
        if horz_accel_d < dist:
            t = 2 * horz_accel_t + (dist - horz_accel_d) / UAV_SPEED["WPNAV_SPEED"] * 100
        #stays below max velocity
        else:
            dist = dist / 2
            t = 2 * math.sqrt(2 * dist / UAV_SPEED["WPNAV_ACCEL"] * 100)
        flight_time += t  

    #calculate takeoff time
    vert_accel_t = UAV_SPEED["WPNAV_SPEED_UP"] / UAV_SPEED["WPNAV_ACCEL_Z"]
    vert_accel_d = UAV_SPEED["WPNAV_ACCEL_Z"] / 100 * vert_accel_t**2
    #hits max vertical velocity
    if vert_accel_d < alt:
        takeoff_time = 2 * vert_accel_t + (alt - vert_accel_d) / UAV_SPEED["WPNAV_SPEED_UP"] * 100
    #stays below max vertical velocity
    else:
        dist = alt / 2
        takeoff_time = 2 * math.sqrt(2 * dist / UAV_SPEED["WPNAV_SPEED_UP"] * 100)
    
    #calculate dive time
    vert_accel_t = UAV_SPEED["WPNAV_SPEED_DN"] / UAV_SPEED["WPNAV_ACCEL_Z"]
    vert_accel_d = UAV_SPEED["WPNAV_ACCEL_Z"] / 100 * vert_accel_t**2
    #hits max vertical velocity
    if vert_accel_d < (alt - dive):
        landing_time = 2 * vert_accel_t + ((alt - dive) - vert_accel_d) / UAV_SPEED["WPNAV_SPEED_DN"] * 100
    #stays below max vertical velocity
    else:
        dist = (alt - dive) / 2
        landing_time = 2 * math.sqrt(2 * dist / UAV_SPEED["WPNAV_SPEED_DN"] * 100)

    #calculate landing time
    vert_accel_t = UAV_SPEED["LAND_SPEED"] / UAV_SPEED["WPNAV_ACCEL_Z"]
    vert_accel_d = UAV_SPEED["WPNAV_ACCEL_Z"] / 100 * vert_accel_t**2
    #hits max vertical velocity
    if vert_accel_d < dive:
        landing_time += 2 * vert_accel_t + (dive - vert_accel_d) / UAV_SPEED["LAND_SPEED"] * 100
    #stays below max vertical velocity
    else:
        dist = dive / 2
        landing_time += 2 * math.sqrt(2 * dist / UAV_SPEED["LAND_SPEED"] * 100)
    #calculate number of landings and takeoffs and delay locations
    if land == 'true':
        landing_time = (wps) * landing_time
        #final landing time
        landing_time += 2 * vert_accel_t + (alt - vert_accel_d) / UAV_SPEED["LAND_SPEED"] * 100
        takeoff_time = (wps + 1) * takeoff_time
        flight_time +=  landing_time + takeoff_time
        total_time = flight_time + (wps * delay)
    else:
        flight_time += landing_time  + takeoff_time + (wps * delay)
        total_time = flight_time
 
    times = [round(total_time), round(flight_time), round(takeoff_time), round(landing_time)]
    return times

def print_solution(manager, routing, solution):
    """Prints solution on console."""
    indices = []
    index = routing.Start(0)
    plan_output = "\n"
    route_distances = []
    route_distance = 0
    while not routing.IsEnd(index):
        plan_output += f" {manager.IndexToNode(index)} ->"
        previous_index = index
        index = solution.Value(routing.NextVar(index))
        route_distances.append(routing.GetArcCostForVehicle(previous_index, index, 0))
        route_distance += route_distances[-1]
        indices.append(index)
    plan_output += f" {manager.IndexToNode(index)}\n"
    print(f"  distance: {route_distance}m")
    print(plan_output)
    
    return indices, route_distances

def main(hmodule, args):
    """Entry point of the program."""
    # Instantiate the data problem.
    data = create_data_model(args['source'], args['home'])

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
        indices, distances = print_solution(manager, routing, solution)
        sorted_coords = [data["coordinates"][i] for i in indices[:-1]]

        if args.get('winch', False):
            # NEW winch format: NAV_LAND + DO_SET_SERVO sequence
            generate_mission_winch(args, coords=sorted_coords)
        else:
            # Original format: kept unchanged for backward compatibility
            generate_mission(args, coords=sorted_coords)
            mission_times = estimate_missionTime(hmodule, distances, len(indices) - 1, args)
            print(f"estimated mission time: {mission_times[0]//60:02d}:{mission_times[0]%60:02d} (mins:secs)")
            print(f"           flight time: {mission_times[1]//60:02d}:{mission_times[1]%60:02d} (mins:secs)")
            print(f"              takeoffs: {mission_times[2]//60:02d}:{mission_times[2]%60:02d} (mins:secs)")
            print(f"              landings: {mission_times[3]//60:02d}:{mission_times[3]%60:02d} (mins:secs)")
            print(f"   max UAV flight time: {UAV_MAX_FTIME//60:02d}:{UAV_MAX_FTIME%60:02d} (mins:secs)")
            if mission_times[1] >= UAV_MAX_FTIME:
                print("ROUTE SIZE WARNING: UAV may end mission early with low battery")

        return sorted_coords
    else:
        raise Exception('no solution found, try again ...')