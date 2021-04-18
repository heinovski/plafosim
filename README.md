# Platoon Formation Simulator (PlaFoSim)

[![Build Status](https://ci.tkn.tu-berlin.de/api/badges/CCS/plafosim/status.svg)](https://ci.tkn.tu-berlin.de/CCS/plafosim)
[![Test Coverage](https://plafosim.de/coverage/coverage.svg)](https://ci.tkn.tu-berlin.de/CCS/plafosim)

[PlaFoSim](https://www.plafosim.de) - A simple (and flexible) simulator for platoon formation.

The idea of PlaFoSim is to simulate the process of forming platoons from individually driven vehicle.
While the main focus of the simulator is on the assignment process, simulation of advertisements and maneuvers is more abstract. That is V2V use an Unit Disc Model (UDM) and maneuvers are implemented by teleports.
<table>
<tr>
<td>Scenario</td>
<td>Advertisement</td>
<td>Assignment</td>
<td>Maneuver</td>
</tr>
<tr>
<td><img src="doc/scenario.png" /></td>
<td><img src="doc/advertisement.png" /></td>
<td><img src="doc/assignment.png" /></td>
<td><img src="doc/maneuver.png" /></td>
</tr>
</table>

This tool is in active development and has not been published yet.
For more information, contact Julian Heinovski ([heinovski@ccs-labs.org](mailto:heinovski@ccs-labs.org)).

---

## Quickstart

```./plafosim.py```

## Installation

- Install Python3 (tested with 3.6.9)
- Optionally install SUMO (tested with >= 1.1.0)
- Clone the repository
- Install the minimum requirements (for running a simulation):
```
pip3 install -r requirements.txt
```
- Install the optional requirements (for testing and CI):
```
pip3 install -r requirements.opt.txt
```

## Running a Simulation

You can use the simulator as module as well as from the command-line.
Currently, only command-line is thorougly tested and thus completely available though.

### Command-line

Use the following command to run a simulation with the default configuration:

```python3 plafosim.py```

### Advanced Simulation Control

You can use a variety of different parameters to customize the scenario and the simulation itself.
E.g., use the parameter `vehicles` to configure the number of vehicles in the simulation:

```python3 plafosim.py --vehicles 1000```

The available parameters are grouped into different categories:

```
- road network properties
- vehicle properties
- trip properties
- communication properties
- platoon properties
- formation properties
- infrastructure properties
- simulation properties
- gui properties
- result recording properties
```

You can see the complete list of available parameters in the help:

```python3 plafosim.py -h, --help```

### Examples

```
# Configure a 100km freeway with ramps at every 10km
python3 plafosim.py --road-length 100 --ramp-interval 10000

# Configure random (normally distributed) desired driving speed of 130km/h
python3 plafosim.py --random-driving-speed true --desired-speed 36

# Configure random trips for 500 vehicles
python3 plafosim.py --vehicles 500 --random-depart-position true --random-arrival-position true --depart-desired true

# Pre fill the freeway with 1000 vehicles
python3 plafosim.py --vehicles 1000 --pre fill true

# Configure 50% of the vehicles with Advanced Cruise Control (ACC) and a headway time of 1.5s
python3 plafosim.py --penetration 0.5 --acc-headway-time 1.5

# Enable a simple, distributed platoon formation algorithm [1] in order to form platoons every 30s
python3 plafosim.py --formation-algorithm speedposition --formation-strategy distributed --execution-interval 30
```

### Live GUI

You can get a very simple live GUI based on SUMO by using the parameter `gui`:

```python3 plafosim.py --gui true```

![](doc/gui.png)

More options for the live GUI can be found within the ``gui properties`` section of the help.

## Re-Playing a Simulation

The simulation writes a trace for every simulated vehicle to a trace file (default `results_vehicle_traces.csv`).
You can view it by using a corresponding script that is shipped within this repository:

```python3 scripts/play-vehicle-trace.py results_vehicle_traces.csv```

To see all options of this script, run:

```python3 scripts/play-vehicle-trace.py -h, --help```

## References

[1] Julian Heinovski and Falko Dressler, "Platoon Formation: Optimized Car to Platoon Assignment Strategies and Protocols," Proceedings of 10th IEEE Vehicular Networking Conference (VNC 2018), Taipei, Taiwan, December 2018.
