# Lane Change Scheduling for Connected and Autonomous Vehicles

This project demonstrates the implementation of a lane change scheduling algorithm for Connected and Autonomous Vehicles (CAVs) using the **SUMO Traffic Simulator** and Python. The goal is to minimize total lane change scheduling time while ensuring safe and efficient traffic flow.

---

## Table of Contents
- [Features](#features)
- [Project Structure](#project-structure)
- [Dependencies](#dependencies)
- [Installation and Setup](#installation-and-setup)
- [Running the Simulation](#running-the-simulation)
- [Explanation of Code Files](#explanation-of-code-files)
- [Outputs and Visualizations](#outputs-and-visualizations)
- [Sample Terminal Output](#sample-terminal-output)
- [License](#license)
- [Future Improvements](#future-improvements)

---

## Features
- Implements a lane change scheduling algorithm using four core sub-algorithms:
  - Forward Trajectory Computation
  - Minimum Predecessor Calculation
  - Single Lane Change Execution
  - Multi-Vehicle Coordination
- Simulates traffic flow with multiple vehicles on a two-lane road.
- Real-time interaction with SUMO using the TraCI API.
- Generates visualizations:
  - Lane change scheduling times.
  - Vehicle positions over simulation time.

---

## Project Structure

```plaintext
|-- Basic_Test.net.xml        # Road network configuration
|-- Basic_Test.rou.xml        # Vehicle route and behavior configuration
|-- Basic_Test.sumocfg        # SUMO simulation configuration
|-- lane_change.py            # Python script implementing the scheduling algorithm
|-- outputs/                  # Directory for storing simulation outputs and plots
|-- README.md                 # Project documentation (this file)

```

---

## Dependencies

To run this project, ensure you have the following installed:

**1. SUMO (Simulation of Urban Mobility):**
   - Download and install SUMO from [SUMO Downloads](https://sumo.dlr.de/docs/Downloads.html).
   - Ensure SUMO binaries (`sumo` and `sumo-gui`) are added to your system’s PATH.

2. **Python** (Version 3.6 or later):
   - Install Python from [python.org](https://www.python.org/downloads/).

3. **Python Libraries**:
   ```bash
   pip install matplotlib traci
   
   
---

## Installation and Setup

To set up the project, follow the steps below:

1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/lane-change-scheduling.git
   
   cd lane-change-scheduling
   
   
 ---
 ## Running the Simulation

### Launching the Simulation

- The Python script `lane_change.py` starts the SUMO GUI and initializes the simulation.
- Vehicles are placed on a two-lane road as defined in the `Basic_Test.rou.xml`.
### Real-Time Interaction

The script uses the [TraCI API](https://sumo.dlr.de/docs/TraCI.html) to:
- Retrieve real-time data from SUMO.
- Apply the lane change scheduling algorithm.
- Send updated commands (e.g., speed adjustments, lane changes) back to SUMO.

### Generating Outputs

- Logs lane change events, reasons, and total scheduling time in the terminal.
- Plots for scheduling time and vehicle positions are saved in the `outputs/` directory.

## Explanation of Code Files

#### Varoiuos Components of the project and Dataflow
![Component Flow](https://github.com/gulamansari57181/lane_change_scheduling_CAV/blob/main/Flowcharts/Component_Flow.png?raw=true)

### `Basic_Test.net.xml`

Defines the road network:
- Two lanes (`E0_0` and `E0_1`) with lengths of 100 meters.
- Junctions (`J0` and `J1`) at the start and end of the road segment.
- Configures lane properties, including speed limits and lane geometries.

### `Basic_Test.rou.xml`

Configures the vehicles in the simulation:
- Defines six vehicles (`v_0` to `v_5`).
- Vehicle properties include:
  - Acceleration: 2.5 m/s²
  - Deceleration: 2.5 m/s²
  - Minimum gap: 10 meters.
- Specifies routes for all vehicles along the road (`E0`).

### `Basic_Test.sumocfg`

Combines the network and route files:
- Loads `Basic_Test.net.xml` and `Basic_Test.rou.xml`.
- Sets the simulation time to 100 seconds.

### `lane_change.py`

Implements the lane change scheduling algorithm using the following core functions:
- **fFW (Forward Trajectory Computation)**: Predicts future positions and velocities of vehicles.
- **fMP (Minimum Predecessor Calculation)**: Ensures safe gaps between vehicles during lane changes.
- **fSLC (Single Lane Change Execution)**: Calculates lane change timings and ensures smooth transitions.

Real-time interaction functions:
```python
traci.vehicle.getPosition(vehicle_id)
traci.vehicle.setSpeed(vehicle_id, new_speed)
traci.vehicle.changeLane(vehicle_id, lane_index, duration)


```

## Outputs and Visualizations

### Terminal Output

The simulation logs lane change events in the terminal, including:

- **Vehicle IDs**: The IDs of vehicles performing lane changes.
- **Reasons for Lane Changes**: The reason behind each lane change, such as maintaining a safe distance.
- **Total Lane Change Scheduling Time**: The total time taken for the lane change scheduling process.

#### Sample Terminal Output:

```text
Vehicle v_1 started lane change from lane 0 to lane 1 at step 4 due to: Maintaining safe distance from leader.
Vehicle v_3 started lane change from lane 0 to lane 1 at step 9 due to: Maintaining safe distance from leader.
Vehicle v_4 started lane change from lane 0 to lane 1 at step 17 due to: Maintaining safe distance from leader.
Total lane change scheduling time: 52480.02 ms

```

### Snapshots of the Output

Below is a snapshot of the visual output generated during the simulation:

#### Road Setup Visualization
![Road Setup Visualization](https://github.com/gulamansari57181/lane_change_scheduling_CAV/blob/main/Flowcharts/road_setup_visualization.png?raw=true)

#### SUMO Simulator Terminal Snapshot
![SUMO Simulator Terminal Snapshot](https://github.com/gulamansari57181/lane_change_scheduling_CAV/blob/main/Flowcharts/Sumo_Simulator_Terminal_Snapshot.png?raw=true)

#### Terminal Output After Scheduling of Vehicles
![Terminal Output](https://github.com/gulamansari57181/lane_change_scheduling_CAV/blob/main/Flowcharts/Terminal_Output.png?raw=true)

#### Lane Change Scheduling Time for Vehicles
![Lane Change Scheduling Time](https://github.com/gulamansari57181/lane_change_scheduling_CAV/blob/main/Flowcharts/Lane_Change_Time_Graph.png?raw=true)




## Future Improvements

There are several potential enhancements for the lane change scheduling algorithm:

- **Extend the Algorithm for Multi-Lane Highways**: 
  Improve the algorithm to handle more complex road networks with multiple lanes, allowing for better lane change decisions in diverse traffic scenarios.

- **Integrate Machine Learning for Predictive Lane Change Decisions**: 
  Implement machine learning techniques to predict lane changes based on real-time traffic data, optimizing lane change decisions for better efficiency and safety.

- **Simulate Larger Road Networks and Traffic Conditions**: 
  Expand the simul
   
   
