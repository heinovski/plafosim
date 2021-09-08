# Platoon Formation Simulator (PlaFoSim)

[![Version](https://img.shields.io/badge/version-v0.12.0-blue)](CHANGELOG.md)
[![Build Status](https://ci.tkn.tu-berlin.de/api/badges/CCS/plafosim/status.svg)](https://ci.tkn.tu-berlin.de/CCS/plafosim)
[![Test Coverage](https://plafosim.de/coverage/coverage.svg)](https://ci.tkn.tu-berlin.de/CCS/plafosim)

[PlaFoSim](https://www.plafosim.de) - A simple (and flexible) simulator for platoon formation.

The idea of PlaFoSim is to simulate the process of forming platoons from individually driven vehicle.
While the main focus of the simulator is on the assignment process, simulation of advertisements and maneuvers is more abstract. That is V2V uses an Unit Disc Model (UDM) and maneuvers are implemented by teleports.

| Scenario | Advertisement | Assignment | Maneuver |
| -------- | ------------- | ---------- | -------- |
![Scenario](doc/scenario.png) | ![Advertisement](doc/advertisement.png) | ![Assignment](doc/assignment.png) | ![Maneuver](doc/maneuver.png) |

This tool is in active development and has not been published yet.
For more information, contact Julian Heinovski ([heinovski@ccs-labs.org](mailto:heinovski@ccs-labs.org)).

---

## Quickstart

```./plafosim.py```

## Installation

- Install Python3 (tested with 3.6.9, 3.7.10, 3.8.5)
- Optionally install SUMO (tested with 1.1.0, 1.4.0, 1.6.0, 1.8.0)
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

## Development

In order to contribute, please follow these steps:
- Make sure to install all dependencies
- Make your desired changes
- Submit a Pull Request (PR)

### Testing

When adding methods and functions, make sure to add corresponding unit tests for `py.test`.
The tests are located under `tests` and can be executed with `./scripts/run-pytest.sh`.
This will also generate a test coverage report.

### Validation

To validate the behavior of PlaFoSim, it is compared to SUMO 1.6.0 by means of simulation results (e.g., vehicle traces).
The corresponding scripts are located under `scripts` and executed withn CI/CD pipelines.
You can have a look at `.drone.yml` for details regarding the execution.

## References

[1] Julian Heinovski and Falko Dressler, "Platoon Formation: Optimized Car to Platoon Assignment Strategies and Protocols," Proceedings of 10th IEEE Vehicular Networking Conference (VNC 2018), Taipei, Taiwan, December 2018.

## License
```
# Copyright (c) 2020-2021 Julian Heinovski <heinovski@ccs-labs.org>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
```
