# Platoon Formation Simulator (PlaFoSim)

[PlaFoSim](https://www.plafosim.de) - A simple simulator for platoon formation.

[![Build Status](https://drone.tkn.tu-berlin.de/api/badges/CCS/plafosim/status.svg)](https://drone.tkn.tu-berlin.de/CCS/plafosim)

### Quickstart

```./plafosim.py```

## Installation

- install the dependencies listed below
- clone the repository

### Dependencies

- Python3 (tested with 3.6.9)
- opt. SUMO (tested with >= 1.2.0)

## Running a Simulation

You can use the simulator as module as well as from the command-line.
Currently, only command-line is available though.

### Command-line

Use the following script to run a simulation

```./plafosim.py```

### Advanced Simulation Control

You can use a variety of different parameters to customize your simulation.
E.g., use the parameter `vehicles` to configure the number of vehicles in the simulation

```./plafosim.py --vehicles 1000```

You can see the complete list of available parameters by running

```./plafosim.py -h```

### Live GUI

You can get a very simple live GUI based on SUMO by using the parameter `gui`

```./plafosim.py --gui true```

## Re-Playing a Simulation Run

The simulation writes a trace for every simulated vehicle to a trace file (default `results_vehicle_traces.csv`).
You can view it by using a corresponding script that is shipped within this repository

```./scripts/play-vehicle-trace.py results_vehicle_traces.csv sumocfg/freeway.sumo.cfg```

To see all options of this script, run

```./scripts/play-vehicle-trace.py -h```
