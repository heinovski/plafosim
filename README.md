# Platoon Formation Simulator (PlaFoSim)

[PlaFoSim](https://www.plafosim.de) - A simple (and flexible) simulator for platoon formation.

[![Build Status](https://drone.tkn.tu-berlin.de/api/badges/CCS/plafosim/status.svg)](https://drone.tkn.tu-berlin.de/CCS/plafosim)
[![Test Coverage](https://plafosim.de/coverage/coverage.svg)](https://drone.tkn.tu-berlin.de/CCS/plafosim)

This tool is in active development and has not been published yet.

### Quickstart

```./plafosim.py```

## Installation

- Install the dependencies listed below
- Clone the repository

### Dependencies

- Python3 (tested with 3.6.9)
- argparse (argusment parsing, tested with 1.4.0)
- pandas (mobility data handling, tested with 1.1.5)
- tqdm (progress bar, tested with 4.48.2)
- opt. SUMO (tested with >= 1.1.0)
- opt. matplotlib (tested with 3.3.0)
- opt. seaborn (tested with 0.10.1)
- opt. statsmodels (tested with 0.11.1)

## Running a Simulation

You can use the simulator as module as well as from the command-line.
Currently, only command-line is thorougly tested and thus completely available though.

### Command-line

Use the following script to run a simulation

```python3 plafosim.py```

### Advanced Simulation Control

You can use a variety of different parameters to customize your simulation.
E.g., use the parameter `vehicles` to configure the number of vehicles in the simulation

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

You can see the complete list of available parameters in the help by running

```python3 plafosim.py -h, --help```

### Live GUI

You can get a very simple live GUI based on SUMO by using the parameter `gui`

```python3 plafosim.py --gui true```

More options for the live GUI can be found within the ``gui properties`` section of the help.

## Re-Playing a Simulation Run

The simulation writes a trace for every simulated vehicle to a trace file (default `results_vehicle_traces.csv`).
You can view it by using a corresponding script that is shipped within this repository

```python3 scripts/play-vehicle-trace.py results_vehicle_traces.csv```

To see all options of this script, run

```python3 scripts/play-vehicle-trace.py -h, --help```
