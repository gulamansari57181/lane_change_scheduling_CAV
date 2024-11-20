import traci
import traci.constants as tc
import matplotlib.pyplot as plt
import time

# Start SUMO as a TraCI server
sumo_cmd = ["sumo-gui", "-c", "Basic_Test.sumocfg"]
traci.start(sumo_cmd)

# Track lane changes, vehicle positions, safe distances, and lane change scheduling times
lane_change_info = {}
vehicle_positions = {}
lane_change_schedule_times = {}  # Dictionary to track lane change times in milliseconds
min_safe_distance = 10.0  # Safe distance in meters
max_acceleration = 2.5
min_deceleration = -2.5
minimum_speed = 2.0  # Minimum speed to prevent stopping on the road

# Initialize vehicles in distinct lanes and positions (Lane 0 and Lane 1)
def initialize_vehicles():
    vehicles = traci.vehicle.getIDList()
    lane_0_positions = 10
    lane_1_positions = 20
    initial_speed = 5  # Set an initial non-zero speed for all vehicles

    for idx, vehicle_id in enumerate(vehicles):
        if idx % 2 == 0:
            traci.vehicle.moveTo(vehicle_id, "E0_0", lane_0_positions)
            lane_0_positions += 10
        else:
            traci.vehicle.moveTo(vehicle_id, "E0_1", lane_1_positions)
            lane_1_positions += 10
        # Set an initial speed using setSpeed method in Python code
        traci.vehicle.setSpeed(vehicle_id, initial_speed)

# Function for Forward Connection (Algorithm 1)
def fFW(t0, p0, pi, ti, a):
    x0, v0 = p0
    xi, vi = pi

    # Check to prevent division by zero if vi is 0
    if vi == 0:
        return (x0, v0, t0)  # Return current state if target speed is zero

    tf = ti + (xi - x0) / vi
    return (x0 + v0 * (tf - t0) + 0.5 * a * (tf - t0)**2, v0 + a * (tf - t0), tf)

# Function for Minimum Predecessor Calculation (Algorithm 2)
def fMP(pCP, pTP):
    xMP, vMP = pCP
    tC = 0
    while True:
        tD = tC + 1
        if tD > 1000:
            break
        xMP, vMP, tC = fFW(tC, (xMP, vMP), pTP, tD, -1)
    return (xMP, vMP)

# Function for Single Lane Change Execution (Algorithm 3)
def fSLC(pCP, pTP, pSV, pTF, tf):
    xSV, vSV = pSV
    xMP, vMP = pCP
    xTF, vTF = pTF
    dSV = min_safe_distance
    dTF = min_safe_distance

    # Move towards safe distance if needed
    if xSV < xMP - dSV:
        pSV, vSV, tSV = fFW(0, pSV, (xMP - dSV, vMP), tf, 1)
    else:
        pSV, vSV, tSV = fFW(0, pSV, (xMP - dSV, vMP), tf, -1)
    
    # Check target follower distance for safe lane change
    if xTF + dTF > xSV:
        xF, vF, tF = fFW(0, pTF, (xSV - dTF, vTF), tf, -1)
    else:
        tF = 0
    
    # Compute lane change timing
    tauSV = max(tSV, tF)
    tauLC = tauSV + 2  # Lane change duration
    return ((pSV, vSV), tauLC)

# Simulation step loop with integrated lane change logic
step = 0
initialize_vehicles()

# Dictionary to store global information shared among vehicles
global_vehicle_info = {}

while step < 50:  # Set total steps as per simulation needs
    traci.simulationStep()

    # Update global vehicle information
    for vehicle_id in traci.vehicle.getIDList():
        global_vehicle_info[vehicle_id] = {
            "position": traci.vehicle.getPosition(vehicle_id),
            "speed": traci.vehicle.getSpeed(vehicle_id),
            "lane_id": traci.vehicle.getLaneID(vehicle_id)
        }

    for vehicle_id in traci.vehicle.getIDList():
        # Access each vehicle's position, speed, and lane information from global_vehicle_info
        local_info = global_vehicle_info[vehicle_id]
        x_position, y_position = local_info["position"]
        current_speed = max(local_info["speed"], minimum_speed)  # Ensure minimum speed
        lane_id = local_info["lane_id"]

        # Store vehicle position over time for plotting
        if vehicle_id not in vehicle_positions:
            vehicle_positions[vehicle_id] = []
        vehicle_positions[vehicle_id].append((step, x_position))

        if vehicle_id in lane_change_info:
            # Check if the lane change has completed
            if traci.vehicle.getLaneChangeState(vehicle_id, 1) == 0:
                start_time = lane_change_info[vehicle_id]['start_time']
                lane_change_duration = step - start_time
                print(f"Vehicle {vehicle_id} completed lane change in {lane_change_duration} steps.")
                del lane_change_info[vehicle_id]

        # Get leader vehicle and calculate necessary adjustments
        leader_info = traci.vehicle.getLeader(vehicle_id, 20)
        if leader_info:
            leader_vehicle_id, distance_to_leader = leader_info
            leader_position = global_vehicle_info[leader_vehicle_id]["position"][0]
            leader_speed = global_vehicle_info[leader_vehicle_id]["speed"]
            
            # Calculate minimum predecessor (MP) position based on leader
            current_position = (x_position, current_speed)
            leader_position_data = (leader_position, leader_speed)
            min_pred_pos, min_pred_speed = fMP(current_position, leader_position_data)

            # Check if the vehicle needs to adjust position relative to MP
            if distance_to_leader < min_safe_distance:
                # Capture the lane change start time for scheduling
                if vehicle_id not in lane_change_schedule_times:
                    lane_change_schedule_times[vehicle_id] = time.time() * 1000  # Record start time in ms

                # Calculate forward connection and single lane change if necessary
                (new_position, new_speed), lane_change_time = fSLC((x_position, current_speed), 
                                                                  (leader_position, leader_speed), 
                                                                  current_position, 
                                                                  (leader_position, leader_speed),  # placeholder for follower
                                                                  step)

                # Adjust speed based on acceleration/deceleration constraints
                new_speed = max(new_speed, minimum_speed)  # Ensure vehicle does not stop
                if new_speed - current_speed > max_acceleration:
                    new_speed = current_speed + max_acceleration
                elif current_speed - new_speed > -min_deceleration:
                    new_speed = current_speed - min_deceleration

                traci.vehicle.setSpeed(vehicle_id, new_speed)

                # Perform lane change to maintain safe distance if necessary
                if lane_id == "E0_0" and vehicle_id not in lane_change_info:
                    traci.vehicle.changeLane(vehicle_id, 1, 10)
                    lane_change_info[vehicle_id] = {'start_time': step, 'target_lane': 1}
                    print(f"Vehicle {vehicle_id} started lane change from lane 0 to lane 1 at step {step}.")
                elif lane_id == "E0_1" and vehicle_id not in lane_change_info:
                    traci.vehicle.changeLane(vehicle_id, 0, 10)
                    lane_change_info[vehicle_id] = {'start_time': step, 'target_lane': 0}
                    print(f"Vehicle {vehicle_id} started lane change from lane 1 to lane 0 at step {step}.")

    step += 1

# Calculate lane change scheduling times for plotting
lane_change_times = {
    vehicle_id: time.time() * 1000 - start_time  # Convert time to milliseconds
    for vehicle_id, start_time in lane_change_schedule_times.items()
}

# Plot lane change scheduling times
plt.figure(figsize=(10, 6))
vehicle_ids = list(lane_change_times.keys())
times_ms = list(lane_change_times.values())
plt.bar(vehicle_ids, times_ms, color='skyblue')
plt.title("Lane Change Scheduling Time for Vehicles")
plt.xlabel("Vehicle ID")
plt.ylabel("Scheduling Time (ms)")
plt.grid(True)
plt.show()

# Plot vehicle positions over time
plt.figure(figsize=(10, 6))
for vehicle_id, positions in vehicle_positions.items():
    if positions:  # Only plot if there are recorded positions
        times, positions_x = zip(*positions)
        plt.plot(times, positions_x, label=f"{vehicle_id}")

plt.title("Vehicle Positions Over Time")
plt.xlabel("Time (steps)")
plt.ylabel("Position (meters)")
plt.legend()
plt.grid(True)
plt.show()

# Close the TraCI connection
traci.close()
