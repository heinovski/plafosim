#
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
#

import logging
import os
import random
import sys
import time
from collections import namedtuple
from timeit import default_timer as timer

import numpy as np
import pandas as pd
from tqdm import tqdm

from .cf_model import CF_Model
from .emissions import EmissionClass
from .gui import (
    add_gui_vehicle,
    close_gui,
    draw_infrastructures,
    draw_ramps,
    draw_road_end,
    gui_step,
    move_gui_vehicle,
    prune_vehicles,
    remove_gui_vehicle,
    start_gui,
)
from .infrastructure import Infrastructure
from .mobility import (
    compute_lane_changes,
    compute_new_speeds,
    get_crashed_vehicles,
    get_predecessors,
    is_gap_safe,
    lane_predecessors,
    update_position,
)
from .platoon_role import PlatoonRole
from .platooning_vehicle import PlatooningVehicle
from .statistics import (
    initialize_emission_traces,
    initialize_infrastructure_assignments,
    initialize_platoon_changes,
    initialize_platoon_formation,
    initialize_platoon_maneuvers,
    initialize_platoon_traces,
    initialize_platoon_trips,
    initialize_simulation_trace,
    initialize_vehicle_changes,
    initialize_vehicle_emissions,
    initialize_vehicle_platoon_changes,
    initialize_vehicle_platoon_traces,
    initialize_vehicle_traces,
    initialize_vehicle_trips,
    record_general_data_begin,
    record_general_data_end,
    record_platoon_change,
    record_simulation_trace,
    record_vehicle_change,
    record_vehicle_platoon_change,
)
from .util import addLoggingLevel
from .vehicle import Vehicle
from .vehicle_type import VehicleType

addLoggingLevel('TRACE', 5)
LOG = logging.getLogger(__name__)

# assumptions
# you just reach your arrival_position
# position is in the middle of the front bumper
# a vehicle ends at position + length
# crash detection does not work with step length greater than 1

# vehicle properties
_length = 4  # m # TODO make parameter
_max_speed = 55  # m/s # TODO make parameter
_max_acceleration = 2.5  # m/s # TODO make parameter
_max_deceleration = 15  # m/s # TODO make parameter
_min_gap = 2.5  # m # TODO make parameter
_desired_headway_time = 1.0  # s # TODO make parameter
# HBEFA3/PC_G_EU4 (a gasoline powered Euro norm 4 passenger car modeled using the HBEFA3 based model), default of SUMO
_emission_class = EmissionClass.PC_G_EU4.name  # TODO make parameter
vtype = VehicleType(
    "car",
    _length,
    _max_speed,
    _max_acceleration,
    _max_deceleration,
    _min_gap,
    _desired_headway_time,
    _emission_class,
)  # TODO support multiple vtypes
TV = namedtuple('TV', ['position', 'rear_position', 'lane'])

# vehicle data fram type collections
# TODO: extract to module or class later
CFModelDtype = pd.CategoricalDtype(list(CF_Model), ordered=True)
PlatoonRoleDtype = pd.CategoricalDtype(list(PlatoonRole), ordered=True)

# default values in SI units
DEFAULTS = {
    'road_length': 100 * 1000,  # km -> m
    'lanes': 3,
    'ramp_interval': 5 * 1000,  # km -> m
    'pre_fill': False,
    'vehicles': 100,
    'vehicle_density': -1,
    'max_speed': 55,  # FIXME not used currently
    'acc_headway_time': 1.0,
    'cacc_spacing': 5.0,
    'penetration_rate': 1.0,
    'random_depart_position': False,
    'random_depart_lane': False,
    'desired_speed': 36.0,
    'random_desired_speed': True,
    'speed_variation': 0.1,
    'min_desired_speed': 22.0,
    'max_desired_speed': 50.0,
    'random_depart_speed': False,
    'depart_desired': False,
    'depart_flow': False,
    'depart_method': 'interval',
    'depart_interval': 1,
    'depart_probability': 1.0,
    'depart_rate': 3600,
    'random_arrival_position': False,
    'minimum_trip_length': 0,
    'maximum_trip_length': -1 * 1000,  # km -> m
    'communication_range': 1000,
    'start_as_platoon': False,
    'reduced_air_drag': True,
    'maximum_teleport_distance': 2000,
    'maximum_approach_time': 60,
    'delay_teleports': True,
    'update_desired_speed': True,
    'formation_algorithm': None,
    'formation_strategy': 'distributed',  # TODO rename
    'execution_interval': 1,
    'infrastructures': 0,
    'step_length': 1,
    'max_step': 1 * 3600,  # h -> s
    'actions': True,
    'collisions': True,
    'random_seed': -1,
    'log_level': logging.WARNING,
    'progress': True,
    'gui': False,
    'gui_delay': 0,
    'gui_track_vehicle': -1,
    'sumo_config': 'sumocfg/freeway.sumo.cfg',
    'gui_play': True,
    'gui_start': 0,
    'draw_ramps': True,
    'draw_ramp_labels': True,
    'draw_road_end': True,
    'draw_road_end_label': True,
    'draw_infrastructures': True,
    'draw_infrastructure_labels': True,
    'result_base_filename': 'results',
    'record_simulation_trace': False,
    'record_end_trace': True,
    'record_vehicle_trips': False,
    'record_vehicle_emissions': False,
    'record_vehicle_traces': False,
    'record_vehicle_changes': False,
    'record_emission_traces': False,
    'record_platoon_trips': False,
    'record_platoon_maneuvers': False,
    'record_platoon_formation': False,
    'record_platoon_traces': False,
    'record_platoon_changes': False,
    'record_infrastructure_assignments': False,
    'record_prefilled': False,
}


def report_rough_braking(
    vdf: pd.DataFrame,
    new_speed: pd.Series,
    rough_factor: float = 0.5
) -> None:
    """
    Report about vehicle preforming rough braking maneuvers.

    Rough braking meaning more than rough_factor times max_deceleration.
    """
    brakers = (
        (vdf['speed'] - new_speed) > (vdf.max_deceleration * rough_factor)
    )
    if not brakers.any():
        return

    fields = ['lane', 'position', 'speed', 'new_speed', 'max_deceleration']
    brake_df = (
        vdf.assign(new_speed=new_speed)
        .loc[brakers, fields]
        .sort_values(["lane", "position"], ascending=False)
    )
    LOG.warning(
        "The following vehicles performed rough braking:\n%s",
        brake_df
    )


class Simulator:
    """A collection of parameters and information of the simulator."""

    def __init__(
            self,
            *,
            road_length: int = DEFAULTS['road_length'],
            number_of_lanes: int = DEFAULTS['lanes'],
            ramp_interval: int = DEFAULTS['ramp_interval'],
            pre_fill: bool = DEFAULTS['pre_fill'],
            number_of_vehicles: int = DEFAULTS['vehicles'],
            vehicle_density: float = DEFAULTS['vehicle_density'],
            max_speed: float = DEFAULTS['max_speed'],
            acc_headway_time: float = DEFAULTS['acc_headway_time'],
            cacc_spacing: float = DEFAULTS['cacc_spacing'],
            penetration_rate: float = DEFAULTS['penetration_rate'],
            random_depart_position: bool = DEFAULTS['random_depart_position'],
            random_depart_lane: bool = DEFAULTS['random_depart_lane'],
            desired_speed: float = DEFAULTS['desired_speed'],
            random_desired_speed: bool = DEFAULTS['random_desired_speed'],
            speed_variation: float = DEFAULTS['speed_variation'],
            min_desired_speed: float = DEFAULTS['min_desired_speed'],
            max_desired_speed: float = DEFAULTS['max_desired_speed'],
            random_depart_speed: bool = DEFAULTS['random_depart_speed'],
            depart_desired: bool = DEFAULTS['depart_desired'],
            depart_flow: bool = DEFAULTS['depart_flow'],
            depart_method: str = DEFAULTS['depart_method'],
            depart_interval: int = DEFAULTS['depart_interval'],
            depart_probability: float = DEFAULTS['depart_probability'],
            depart_rate: int = DEFAULTS['depart_rate'],
            random_arrival_position: bool = DEFAULTS['random_arrival_position'],
            minimum_trip_length: int = DEFAULTS['minimum_trip_length'],
            maximum_trip_length: int = DEFAULTS['maximum_trip_length'],
            communication_range: int = DEFAULTS['communication_range'],
            start_as_platoon: bool = DEFAULTS['start_as_platoon'],
            reduced_air_drag: bool = DEFAULTS['reduced_air_drag'],
            maximum_teleport_distance: int = DEFAULTS['maximum_teleport_distance'],
            maximum_approach_time: int = DEFAULTS['maximum_approach_time'],
            delay_teleports: bool = DEFAULTS['delay_teleports'],
            update_desired_speed: bool = DEFAULTS['update_desired_speed'],
            formation_algorithm: str = DEFAULTS['formation_algorithm'],
            formation_strategy: str = DEFAULTS['formation_strategy'],
            execution_interval: int = DEFAULTS['execution_interval'],
            number_of_infrastructures: int = DEFAULTS['infrastructures'],
            step_length: int = DEFAULTS['step_length'],
            max_step: int = DEFAULTS['max_step'],
            actions: bool = DEFAULTS['actions'],
            collisions: bool = DEFAULTS['collisions'],
            random_seed: int = DEFAULTS['random_seed'],
            log_level: int = DEFAULTS['log_level'],
            progress: bool = DEFAULTS['progress'],
            gui: bool = DEFAULTS['gui'],
            gui_delay: int = DEFAULTS['gui_delay'],
            gui_track_vehicle: int = DEFAULTS['gui_track_vehicle'],
            sumo_config: str = DEFAULTS['sumo_config'],
            gui_play: int = DEFAULTS['gui_play'],
            gui_start: int = DEFAULTS['gui_start'],
            draw_ramps: bool = DEFAULTS['draw_ramps'],
            draw_ramp_labels: bool = DEFAULTS['draw_ramp_labels'],
            draw_road_end: bool = DEFAULTS['draw_road_end'],
            draw_road_end_label: bool = DEFAULTS['draw_road_end_label'],
            draw_infrastructures: bool = DEFAULTS['draw_infrastructures'],
            draw_infrastructure_labels: bool = DEFAULTS['draw_infrastructure_labels'],
            result_base_filename: str = DEFAULTS['result_base_filename'],
            record_simulation_trace: bool = DEFAULTS['record_simulation_trace'],
            record_end_trace: bool = DEFAULTS['record_end_trace'],
            record_vehicle_trips: bool = DEFAULTS['record_vehicle_trips'],
            record_vehicle_emissions: bool = DEFAULTS['record_vehicle_emissions'],
            record_vehicle_traces: bool = DEFAULTS['record_vehicle_traces'],
            record_vehicle_changes: bool = DEFAULTS['record_vehicle_changes'],
            record_emission_traces: bool = DEFAULTS['record_emission_traces'],
            record_platoon_trips: bool = DEFAULTS['record_platoon_trips'],
            record_platoon_maneuvers: bool = DEFAULTS['record_platoon_maneuvers'],
            record_platoon_formation: bool = DEFAULTS['record_platoon_formation'],
            record_platoon_traces: bool = DEFAULTS['record_platoon_traces'],
            record_platoon_changes: bool = DEFAULTS['record_platoon_changes'],
            record_infrastructure_assignments: bool = DEFAULTS['record_infrastructure_assignments'],
            record_prefilled: bool = DEFAULTS['record_prefilled'],
            **kwargs: dict,
    ):
        """Initializes a simulator instance."""

        # TODO add custom filter that prepends the log entry with the step time
        logging.basicConfig(level=log_level, format="%(levelname)s [%(name)s]: %(message)s")

        # road network properties
        self._road_length = road_length  # the length of the road
        self._number_of_lanes = number_of_lanes  # the number of lanes
        self._ramp_interval = ramp_interval  # the distance between any two on-/off-ramps

        # vehicle properties
        self._vehicles = {}  # the list (dict) of vehicles in the simulation
        self._last_vehicle_id = -1  # the id of the last vehicle generated
        # the maximum number of vehicles
        if vehicle_density > 0:
            # override vehicles
            number_of_vehicles = int(vehicle_density * (self._road_length / 1000) * self._number_of_lanes)
        if vehicle_density == 0:
            sys.exit("ERROR: A vehicle density of 0 vehicles per lane per km does not make sense!")
        if number_of_vehicles <= 0:
            sys.exit("ERROR: A simulation with 0 vehicles does not make sense!")
        self._number_of_vehicles = number_of_vehicles
        self._max_speed = max_speed  # the maximum driving speed # FIXME not used currently
        self._acc_headway_time = acc_headway_time  # the headway time for ACC
        if acc_headway_time < 1.0:
            LOG.warning("Values for ACC headway time lower 1.0s are not recommended to avoid crashes!")
        self._cacc_spacing = cacc_spacing  # the constant spacing for CACC
        if cacc_spacing < 5.0:
            LOG.warning("Values for CACC spacing lower than 5.0m are not recommended to avoid crashes!")
        self._penetration_rate = penetration_rate  # the penetration rate of platooning vehicles

        # trip properties
        self._random_depart_position = random_depart_position  # whether to use random depart positions
        self._random_depart_lane = random_depart_lane  # whether to use random depart lanes
        self._desired_speed = desired_speed  # the desired driving speed
        self._random_desired_speed = random_desired_speed  # whether to use random desired driving speeds
        self._speed_variation = speed_variation  # the deviation from the desired driving speed
        self._min_desired_speed = min_desired_speed  # the minimum desired driving speed
        self._max_desired_speed = max_desired_speed  # the maximum desired driving speed
        self._random_depart_speed = random_depart_speed  # whether to use random departure speeds
        self._depart_desired = depart_desired  # whether to depart with the desired driving speed
        if random_depart_position and not depart_desired:
            sys.exit("ERROR: random-depart-position is only possible in conjunction with depart-desired!")
        self._depart_flow = depart_flow  # whether to spawn vehicles in a continuous flow
        if not depart_flow and depart_method == "number":
            sys.exit("ERROR: The depart method number can only be used in conjuction with a depart flow!")
        self._depart_method = depart_method  # the departure method to use
        if depart_interval < 1:
            sys.exit("ERROR: The depart interval has to be at least 1!")
        self._depart_interval = depart_interval  # the interval between two vehicle departures
        if depart_probability < 0 or depart_probability > 1:
            sys.exit("ERROR: The depart probability needs to be between 0 and 1!")
        self._depart_probability = depart_probability  # the departure probability
        if depart_rate <= 0:
            sys.exit("ERROR: The departure rate has to be at least 1 vehicle per hour!")
        self._depart_rate = depart_rate  # the departure rate
        self._vehicles_scheduled = 1
        self._random_arrival_position = random_arrival_position  # whether to use random arrival positions
        if minimum_trip_length > road_length:
            sys.exit("ERROR: Minimum trip length cannot be bigger than the length of the entire road!")
        self._minimum_trip_length = minimum_trip_length  # the minimum trip length
        if maximum_trip_length == -1 * 1000:
            self._maximum_trip_length = road_length
        else:
            if maximum_trip_length < minimum_trip_length:
                sys.exit("ERROR: Maximum trip length cannot be smaller than the minimum trip length!")
            if maximum_trip_length < ramp_interval:
                sys.exit("ERROR: Maximum trip length cannot be smaller than the ramp interval!")
            if not maximum_trip_length % ramp_interval == 0:
                sys.exit("ERROR: Maximum trip length has to be a multiple of the ramp interval!")
            self._maximum_trip_length = maximum_trip_length  # the maximum trip length

        # communication properties
        self._communication_range = communication_range  # the maximum communication range between two vehicles
        if communication_range == -1:
            self._communication_range = road_length
        if self._communication_range <= 0:
            sys.exit("ERROR: Communication range has to be > 0!")

        # platoon properties
        self._start_as_platoon = start_as_platoon  # whether vehicles start as one platoon
        if start_as_platoon:
            if penetration_rate < 1.0:
                sys.exit("ERROR: The penetration rate cannot be smaller than 1.0 when starting as one platoon!")
            if formation_algorithm:
                sys.exit("ERROR: A formation algorithm cannot be used when all starting as one platoon!")
            if not pre_fill:
                sys.exit("ERROR: start-as-platoon is only available when using prefill!")
            if depart_flow:
                sys.exit("ERROR: Vehicles can not spawn in a flow when starting as one platoon!")
            if random_depart_position:
                sys.exit("ERROR: Vehicles can not have random departure positions when starting as one platoon!")
            if random_depart_lane:
                sys.exit("ERROR: Vehicles can not have random departure lanes when starting as one platoon!")
        self._reduced_air_drag = reduced_air_drag  # whether the reduced air drag due to platooning should be considered in the emissions calculation
        if maximum_teleport_distance == -1:
            self._maximum_teleport_distance = road_length
            LOG.warning("No maximum teleport distance configured! The vehicle behavior may be unrealistic!")
        else:
            if maximum_teleport_distance >= ramp_interval:
                LOG.warning(f"A maximum teleport distance of {maximum_teleport_distance}m allows teleports beyond the next highway ramp! ")
            if maximum_teleport_distance >= minimum_trip_length and minimum_trip_length > 0:
                LOG.warning(f"A maximum teleport distance of {maximum_teleport_distance}m allows teleports beyond the minimum trip length!")
            self._maximum_teleport_distance = maximum_teleport_distance  # maximum teleport distance
        if maximum_approach_time == -1:
            LOG.warning("No maximum approach timeout configured! The vehicle behavior may be unrealistic!")
            self._maximum_appraoch_time = float('inf')
        else:
            self._maximum_appraoch_time = maximum_approach_time  # maximum approach time
        if not delay_teleports:
            LOG.warning("Teleports will be executed instantaneous! The vehicle behavior may be unrealistic!")
        self._delay_teleports = delay_teleports  # whether teleports during a join maneuver should be delayed by the approach time

        self._update_desired_speed = update_desired_speed  # whether to update the platoon's desired driving speed to the average speed of all members after the formation changed
        self._formation_algorithm = formation_algorithm  # the formation algorithm to use
        if formation_strategy == "centralized" and number_of_infrastructures <= 0:
            sys.exit("ERROR: When using a centralized strategy at least 1 infrastructure is needed!")
        self._formation_strategy = formation_strategy  # the formation strategy to use

        # formation properties
        self._execution_interval = execution_interval  # the interval between two iterations of a formation algorithm
        if execution_interval <= 0:
            sys.exit("ERROR: Execution interval has to be at least 1 second!")

        # infrastructure properties
        self._infrastructures = {}  # the list (dict) of infrastructures in the simulation

        # simulation properties
        self._step = 0  # the current simulation step in s
        assert(step_length > 0)
        if step_length != 1:
            LOG.warning("Values for step length other than 1s are not yet properly implemented and tested!")
        if step_length < 1.0:
            LOG.warning("Values for step length small than 1s are not recommended in order to avoid crashes!")
        self._step_length = step_length  # the length of a simulation step
        self._max_step = max_step
        self._running = False  # whether the simulation is running
        self._actions = actions  # whether to enable actions
        self._collisions = collisions  # whether to check for collisions
        if random_seed >= 0:
            LOG.info(f"Using random seed {random_seed}")
            random.seed(random_seed)
        self._progress = progress  # whether to enable the (simulation) progress bar

        # gui properties
        self._gui = gui  # whether to show a live sumo-gui
        if gui:
            if 'SUMO_HOME' not in os.environ:
                sys.exit("ERROR: Environment variable 'SUMO_HOME' was not declared!")
            tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
            sys.path.append(tools)

        self._gui_delay = gui_delay  # the delay in every simulation step for the gui
        self._gui_track_vehicle = gui_track_vehicle  # the id of a vehicle to track in the gui
        self._sumo_config = sumo_config  # the name of the SUMO config file
        self._gui_play = gui_play  # whether to start the simulation immediately
        if gui_start < 0:
            sys.exit("ERROR: GUI start time cannot be negative!")
        self._gui_start = gui_start  # the time when to connect to the GUI
        self._draw_ramps = draw_ramps  # whether to draw on-/off-ramps
        self._draw_ramp_labels = draw_ramp_labels  # whether to draw labels for on-/off-ramps
        self._draw_road_end = draw_road_end  # whether to draw the end of the road
        self._draw_road_end_label = draw_road_end_label  # whether to draw a label for the end of the road
        self._draw_infrastructures = draw_infrastructures  # whether to draw infrastructures
        self._draw_infrastructure_labels = draw_infrastructure_labels  # whether to draw labels for infrastructures

        # result recording properties
        self._result_base_filename = result_base_filename  # the base filename of the result files
        self._record_simulation_trace = record_simulation_trace  # whether to record a continuous simulation trace
        self._record_end_trace = record_end_trace  # whether to record another trace item at the trip end
        self._record_vehicle_trips = record_vehicle_trips  # whether to record vehicles trips
        self._record_vehicle_emissions = record_vehicle_emissions  # whether to record vehicle emissions
        self._record_vehicle_traces = record_vehicle_traces  # whether to record continuous vehicle traces
        self._record_vehicle_changes = record_vehicle_changes  # whether to record vehicle lane changes
        self._record_emission_traces = record_emission_traces  # whether to record continuous emission traces
        self._record_platoon_trips = record_platoon_trips  # whether to record platoon trips
        self._record_platoon_maneuvers = record_platoon_maneuvers  # whether to record platoon maneuvers
        self._record_platoon_formation = record_platoon_formation  # whether to record platoon formation
        self._record_platoon_traces = record_platoon_traces  # whether to record continuous platoon traces
        self._record_platoon_changes = record_platoon_changes  # whether to record platoon lane changes
        self._record_infrastructure_assignments = record_infrastructure_assignments  # whether to record infrastructure assignments
        if record_prefilled and not start_as_platoon:
            LOG.warning("Recording results for pre-filled vehicles is not recommended to avoid broken statistics!")
        self._record_prefilled = record_prefilled  # whether to record results for pre-filled vehicles

        # additional keyword arguments (e.g., for formation algorithms)
        self._kwargs = kwargs

        # statistics
        self._avg_number_vehicles = 0
        self._values_in_avg_number_vehicles = 0

        # TODO log generation parameters
        if pre_fill:
            self._generate_vehicles()

        self._generate_infrastructures(number_of_infrastructures)

    @property
    def road_length(self) -> int:
        """Returns the road length."""

        return self._road_length

    @property
    def number_of_lanes(self) -> int:
        """Returns the number of lanes."""

        return self._number_of_lanes

    @property
    def step_length(self) -> int:
        """Returns the length of a simulation step."""

        return self._step_length

    @property
    def step(self) -> int:
        """Returns the current simulation step."""

        return self._step

    def _call_vehicle_actions(self):
        """Triggers actions on all vehicles in the simulation."""

        for vehicle in self._vehicles.values():
            vehicle.action(self._step)

    def _call_infrastructure_actions(self):
        """Triggers actions on all infrastructures in the simulation."""

        for infrastructure in self._infrastructures.values():
            infrastructure.action(self._step)

    def _get_predecessor(self, vehicle: Vehicle, lane: int = -1) -> Vehicle:
        """
        Returns the preceding (i.e., front) vehicle for a given vehicle on a given lane.

        Parameters
        ----------
        vehicle : Vehicle
            The vehicle to consider
        lane : int, optional
            The lane to consider.
            A lane of -1 indicates the vehicle's current lane.
        """

        if lane == -1:
            # implicitly search on current lane of vehicle
            lane = vehicle._lane
        predecessor = None  # there is no predecessor so far
        candidates = {
            v._vid: v.rear_position for v in self._vehicles.values() if
            v._lane == lane and  # correct lane
            v._position >= vehicle._position  # in front this vehicle
        }
        # not this vehicle
        candidates.pop(vehicle._vid, None)
        # find candidate with smallest rear_position (min)
        # we do not check for collisions here because this method is also called within an update step
        if candidates:
            predecessor = self._vehicles[min(candidates, key=candidates.get)]
        return predecessor

    def _get_successor(self, vehicle: Vehicle, lane: int = -1) -> Vehicle:
        """
        Returns the succeeding (i.e., back) vehicle for a given vehicle on a given lane.

        Parameters
        ----------
        vehicle : Vehicle
            The vehicle to consider
        lane : int, optional
            The lane to consider.
            A lane of -1 indicates the vehicle's current lane.
        """

        if lane == -1:
            # implicitly search on current lane of vehicle
            lane = vehicle._lane
        successor = None  # there is no successor so far
        candidates = {
            v._vid: v._position for v in self._vehicles.values() if
            v._lane == lane and  # correct lane
            v._position <= vehicle._position  # behind this vehicle
        }
        # not this vehicle
        candidates.pop(vehicle._vid, None)
        # find candidate with largest_position (max)
        # we do not check for collisions here because this method is also called within an update step
        if candidates:
            successor = self._vehicles[max(candidates, key=candidates.get)]
        return successor

    def _get_predecessor_rear_position(self, vehicle: Vehicle, lane: int = -1) -> float:
        """
        Returns the rear position of the preceding (i.e., front) vehicle for a given vehicle on a given lane.

        Parameters
        ----------
        vehicle : Vehicle
            The vehicle to consider
        lane : int, optional
            The lane to consider.
            A lane of -1 indicates the vehicle's current lane.
        """

        p = self._get_predecessor(vehicle, lane)
        if not p:
            return -1
        else:
            return p.rear_position

    def _get_predecessor_speed(self, vehicle: Vehicle, lane: int = -1) -> float:
        """
        Returns the speed of the preceding (i.e., front) vehicle for a given vehicle on a given lane.

        Parameters
        ----------
        vehicle : Vehicle
            The vehicle to consider
        lane : int, optional
            The lane to consider.
            A lane of -1 indicates the vehicle's current lane.
        """

        p = self._get_predecessor(vehicle, lane)
        if not p:
            return -1
        else:
            return p._speed

    def _remove_arrived_vehicles(self, arrived_vehicles: list):
        """
        Removes arrived vehicles from the simulation.

        Parameters
        ----------
        arrived_vehicles : list
            The ids of arrived vehicles
        """

        for vid in arrived_vehicles:
            # call finish on arrived vehicle
            self._vehicles[vid].finish()
            # remove arrived vehicle from gui
            if self._gui and self._step >= self._gui_start:
                remove_gui_vehicle(vid)
            # remove from vehicles
            del self._vehicles[vid]

    @staticmethod
    def _check_collisions(vdf: pd.DataFrame):
        """
        Does collision checks for all vehicles in the simulation.

        Parameters
        ----------
        vdf : pandas.DataFrame
            The dataframe containing the vehicles as rows
            index: vid
            columns: [position, length, lane, ..]
        """

        if vdf.empty:
            return

        crashed_vehicles = get_crashed_vehicles(vdf)
        if crashed_vehicles:
            for v in crashed_vehicles:
                print(f"{v}: {vdf.at[v, 'position']}-{vdf.at[v, 'position']-vdf.at[v, 'length']},{vdf.at[v, 'lane']}")
            sys.exit(f"ERROR: There were collisions with the following vehicles {crashed_vehicles}!")

    @staticmethod
    def has_collision(vehicle1: TV, vehicle2: TV) -> bool:
        """
        Checks for a collision between two vehicles.

        Parameters
        ----------
        vehicle1 : TV(position, rear_position, lane)
        vehicle2 : TV(position, rear_position, lane)
        """
        assert(vehicle1 is not vehicle2)
        assert(vehicle1.lane == vehicle2.lane)
        return min(vehicle1.position, vehicle2.position) - max(vehicle1.rear_position, vehicle2.rear_position) >= 0

    def _generate_vehicles(self):
        """Adds pre-filled vehicles to the simulation."""

        LOG.info(f"Pre-filling the road network with {self._number_of_vehicles} vehicles")

        assert not self._vehicles
        assert self._last_vehicle_id == -1

        for vid in tqdm(range(0, self._number_of_vehicles), desc="Generated vehicles", disable=not self._progress):

            desired_speed = self._get_desired_speed()

            # TODO remove duplicated code
            if self._start_as_platoon:
                depart_time = 0
                depart_position = (self._number_of_vehicles - vid) * (vtype._length + self._cacc_spacing) - self._cacc_spacing
                depart_lane = 0

                if vid == 0:
                    # pick regular depart speed
                    depart_speed = self._get_depart_speed(desired_speed)
                else:
                    # always set speed to leader speed
                    depart_speed = self._vehicles[0]._depart_speed
            else:
                depart_time = -1  # since this is a pre-filled vehicle, we cannot say when it departed

                # always use desired speed for pre-fill vehicles
                depart_speed = desired_speed

                # assume we have a collision to check at least once
                collision = True
                while collision:
                    collision = False
                    # actual calculation of position and lane
                    # always use random position for pre-filled vehicle
                    # we do not consider depart interval here since this is supposed to be a snapshot from an earlier point of simulation
                    # make sure to also include the end of the road itself
                    # consider length, equal to departPos="base" in SUMO
                    depart_position = random.uniform(vtype._length, self._road_length)
                    # always use random lane for pre-filled vehicle
                    depart_lane = random.randrange(0, self._number_of_lanes, 1)

                    LOG.debug(f"Generated random depart position ({depart_position},{depart_lane}) for vehicle {vid}")

                    if not self._vehicles:
                        continue

                    # avoid a collision with an existing vehicle
                    for other_vehicle in self._vehicles.values():
                        if other_vehicle._lane != depart_lane:
                            # we do not care about other lanes
                            continue

                        # do we have a collision?
                        # avoid being inserted in between two platoon members by also considering the min gap
                        tv = TV(
                            depart_position + vtype._min_gap,  # front collider
                            depart_position - vtype._length,  # rear collider
                            depart_lane
                        )
                        otv = TV(
                            other_vehicle._position + other_vehicle.min_gap,  # front collider
                            other_vehicle.rear_position,  # rear collider
                            other_vehicle._lane
                        )

                        # do we have a "collision" (now or in the next step)?
                        collision = (
                            collision
                            or self.has_collision(tv, otv)
                            or self._is_insert_unsafe(depart_position, depart_speed, vtype, other_vehicle)
                        )

            arrival_position = self._get_arrival_position(depart_position, pre_fill=True)

            self._add_vehicle(
                vid,
                vtype,
                depart_position,
                arrival_position,
                desired_speed,
                depart_lane,
                depart_speed,
                depart_time,
            )

            LOG.debug(f"Generated vehicle {vid} at {depart_position}-{depart_position - vtype._length},{depart_lane} with {depart_speed}")

    def _get_desired_speed(self) -> float:
        """Returns a (random) depart speed."""

        if self._random_desired_speed:
            # normal distribution
            desired_speed = self._desired_speed * random.normalvariate(1.0, self._speed_variation)
            desired_speed = max(desired_speed, self._min_desired_speed)
            desired_speed = min(desired_speed, self._max_desired_speed)
        else:
            desired_speed = self._desired_speed

        return desired_speed

    def _get_depart_speed(self, desired_speed: float) -> float:
        """
        Returns a (random) depart speed.

        Parameters
        ----------
        desired_speed : float
            The desired speed to consider
        """

        if self._random_depart_speed:
            # make sure to also include the desired speed itself
            depart_speed = random.randrange(0, desired_speed + 1, 1)
        else:
            depart_speed = 0

        if self._depart_desired:
            depart_speed = desired_speed

        return depart_speed

    def _get_depart_position(self) -> int:
        """
        Returns a (random) depart position for a given depart position.

        This considers the ramp interval, road length, and minimum trip length.
        """

        # NOTE: this should only be called for non-pre-filled vehicles
        if self._random_depart_position:
            # set maximum theoretical depart position
            # make sure that the vehicles can drive for at least the minimum length of a trip
            # and at least for one ramp
            max_depart = self._road_length - max(self._minimum_trip_length, self._ramp_interval)
            if not self._random_arrival_position:
                min_depart = max(self._road_length - self._maximum_trip_length, 0)
                min_depart_ramp = min_depart - (self._ramp_interval + min_depart) % self._ramp_interval
            else:
                min_depart_ramp = 0
            max_depart_ramp = max_depart - (self._ramp_interval + max_depart) % self._ramp_interval
            assert(max_depart_ramp <= self._road_length)
            assert(max_depart_ramp >= 0)
            if max_depart_ramp == 0:
                # start at beginning
                depart_position = 0
            else:
                depart_position = random.randrange(min_depart_ramp, max_depart_ramp + 1, self._ramp_interval)
            assert(depart_position <= max_depart_ramp)
            if not self._random_arrival_position:
                assert(depart_position >= self._road_length - self._maximum_trip_length)
        else:
            # simply start at beginning
            depart_position = 0

        assert(depart_position >= 0)
        assert(depart_position <= self._road_length)

        return depart_position

    def _get_arrival_position(self, depart_position: int, pre_fill: bool = False) -> int:
        """
        Returns a (random) arrival position for a given depart position.

        This considers the ramp interval, road length, and minimum trip length.

        Parameters
        ----------
        depart_position : int
            The depart position to consider
        prefill : bool, optional
            Whether the trip is for a pre-filled vehicle
        """

        if self._random_arrival_position:
            # set minimum theoretical arrival position
            if pre_fill:
                # We cannot use the minimum trip time here,
                # since the pre-generation is supposed to produce a snapshot of a realistic simulation.
                # But we can assume that a vehicle has to drive at least 1m
                min_arrival = depart_position + 1
                max_arrival = min(depart_position + self._maximum_trip_length - self._ramp_interval, self._road_length)
            else:
                # make sure that the vehicles drive at least for the minimum length of a trip
                # and at least for one ramp
                min_arrival = depart_position + max(self._minimum_trip_length, self._ramp_interval)
                max_arrival = min(depart_position + self._maximum_trip_length, self._road_length)
            min_arrival_ramp = min_arrival + (self._ramp_interval - min_arrival) % self._ramp_interval
            max_arrival_ramp = max_arrival + (self._ramp_interval - max_arrival) % self._ramp_interval
            assert(min_arrival_ramp >= 0)
            assert(min_arrival_ramp <= self._road_length)
            assert(min_arrival_ramp <= max_arrival_ramp)
            assert(max_arrival_ramp <= self._road_length)
            if min_arrival % self._ramp_interval == 0:
                assert(min_arrival == min_arrival_ramp)
            if min_arrival_ramp == self._road_length:
                # avoid empty randrange
                # exit at end
                arrival_position = self._road_length
            else:
                # make sure to also include the end of the road itself
                arrival_position = random.randrange(min_arrival_ramp, max_arrival_ramp + 1, self._ramp_interval)
            assert(arrival_position >= min_arrival_ramp)
        else:
            # simply drive until the end
            arrival_position = self._road_length

        assert(arrival_position <= self._road_length)
        assert(arrival_position > depart_position)
        assert(arrival_position - depart_position <= self._maximum_trip_length)

        return arrival_position

    def _spawn_vehicles(self):
        """Spawns vehicles within the current step"""

        if not self._depart_flow and self._last_vehicle_id >= self._number_of_vehicles - 1:
            # limit the spawn by a maximum number of total vehicles
            LOG.debug(f"All {self._number_of_vehicles} vehicles have been spawned already")
            # clear scheduled vehicles
            self._vehicles_scheduled = 0
            return

        if self._random_depart_position:
            # enable spawning at multiple ramps
            num_spawn_ramps = int(self._road_length / self._ramp_interval)
        else:
            # spawn only at first ramp (i.e., beginning of the road)
            num_spawn_ramps = 1

        if self._depart_method == "interval":
            # spawn interval, similar to SUMO's flow param period
            self._spawn_vehicles_interval(self._depart_interval, num_spawn_ramps)
        elif self._depart_method == "rate":
            # spawn #vehicles per hour, similar to SUMO's flow param vehsPerHour
            depart_interval = 3600 / self._step_length / self._depart_rate
            self._spawn_vehicles_interval(depart_interval, num_spawn_ramps)
        elif self._depart_method == "number":
            # spawn #number vehicles, similar to SUMO's flow param number
            if len(self._vehicles) >= self._number_of_vehicles:
                # limit the flow by a maximum number of concurrent vehicles
                LOG.debug(f"Maximum number of vehicles ({self._number_of_vehicles}) is reached already")
                return
            depart_interval = self._max_step / self._number_of_vehicles
            self._spawn_vehicles_interval(depart_interval, num_spawn_ramps)
        elif self._depart_method == "probability":
            # spawn probability per time step, similar to SUMO's flow param probability
            for _ in range(num_spawn_ramps):
                if random.random() <= self._depart_probability:
                    self._spawn_vehicle()
        else:
            sys.exit("ERROR: Unknown depart method!")

    def _spawn_vehicles_interval(self, depart_interval: float, num_spawn_ramps: int):
        """
        Spawns vehicles within the current step based on the depart interval.
        Note that this does not automatically spawn a vehicle at step 0 (cf. SUMO's flow param begin).

        Parameters
        ----------
        depart_interval : float
            The (equally spaced) depart interval between vehicles
        num_spawn_ramps : int
            The number of depart ramps
        """

        # spawn rate per spawn ramp
        spawn_rate = self._step_length / depart_interval
        LOG.trace(f"The spawn rate per step per ramp is {spawn_rate}")
        # total spawn rate for all ramps
        spawn_rate = spawn_rate * num_spawn_ramps
        LOG.trace(f"The total spawn rate per step is {spawn_rate}")

        LOG.debug(f"Currently scheduled vehicles {self._vehicles_scheduled} ({self._step})")

        # spawn all vehicles
        num_vehicles = int(self._vehicles_scheduled)
        LOG.debug(f"I will spawn {num_vehicles} total vehicles now ({self._step})")
        for v in range(0, num_vehicles):
            self._spawn_vehicle()

        # update scheduled vehicles
        self._vehicles_scheduled -= num_vehicles  # remove spawned vehicles
        self._vehicles_scheduled += spawn_rate  # add spawn rate

    def _spawn_vehicle(self):
        """Spawns a vehicle within the simulation"""

        vid = self._last_vehicle_id + 1

        depart_time = self._step

        if self._random_depart_lane:
            depart_lane = random.randrange(0, self._number_of_lanes, 1)
        else:
            depart_lane = 0

        depart_position = self._get_depart_position()
        arrival_position = self._get_arrival_position(depart_position)
        depart_position += vtype._length  # equal to departPos="base" in SUMO

        desired_speed = self._get_desired_speed()

        depart_speed = self._get_depart_speed(desired_speed)

        LOG.debug(f"Spawning vehicle {vid} at {depart_position}-{depart_position - vtype._length},{depart_lane} with {depart_speed}")

        # TODO remove duplicated code
        # check whether the vehicle can actually be inserted safely
        # assume we have a collision to check at least once
        collision = bool(self._vehicles)
        LOG.debug(f"Checking for a collision with an existing vehicle for new vehicle {vid}")
        while collision:
            collision = False  # so far we do not have a collision
            # avoid a collision with an existing vehicle
            # check all vehicles
            for other_vehicle in self._vehicles.values():
                LOG.trace(f"Checking vehicle {other_vehicle._vid}")
                if other_vehicle._lane != depart_lane:
                    # we do not care about other lanes
                    continue

                # do we have a collision?
                # avoid being inserted in between two platoon members by also considering the min gap
                tv = TV(
                    depart_position + vtype._min_gap,  # front collider
                    depart_position - vtype._length,  # rear collider
                    depart_lane
                )
                otv = TV(
                    other_vehicle._position + other_vehicle.min_gap,  # front collider
                    other_vehicle.rear_position,  # rear collider
                    other_vehicle._lane
                )

                # do we have a "collision" (now or in the next step)?
                collision = (
                    collision
                    or self.has_collision(tv, otv)
                    or self._is_insert_unsafe(depart_position, depart_speed, vtype, other_vehicle)
                )

            if collision:
                # can we avoid the collision by switching the departure lane?
                if depart_lane == self._number_of_lanes - 1:
                    # reached maximum number of lanes already
                    LOG.warning(f"Could not further increase depart lane ({depart_lane}) for vehicle {vid}! You might want to reduce the number of vehicles to reduce the traffic. Vehicle {vid} will be delayed!")
                    # delay insertion of vehicle
                    # TODO add real delaying incl. depart time and time loss
                    self._vehicles_scheduled += 1
                    return  # do not insert this vehicle but also do not abort the simulation
                depart_lane = depart_lane + 1
                LOG.info(f"Increased depart lane for {vid} to avoid a collision (now lane {depart_lane})")
                # we need to check again

        vehicle = self._add_vehicle(
            vid,
            vtype,
            depart_position,
            arrival_position,
            desired_speed,
            depart_lane,
            depart_speed,
            depart_time,
        )

        if self._gui and self._step >= self._gui_start:
            add_gui_vehicle(vehicle, track=vehicle.vid == self._gui_track_vehicle)

        LOG.info(f"Spawned vehicle {vid} ({depart_position}-{vehicle.rear_position},{depart_lane})")

    def _is_insert_unsafe(self, depart_position, depart_speed, vtype, other_vehicle):
        # would it be unsafe to insert the vehicle?
        if other_vehicle._position <= depart_position:
            return not is_gap_safe(
                front_position=depart_position,
                front_speed=depart_speed,
                front_max_deceleration=vtype.max_deceleration,
                front_length=vtype.length,
                back_position=other_vehicle.position,
                back_speed=other_vehicle.speed,
                back_max_acceleration=other_vehicle.max_acceleration,
                step_length=self._step_length
            )
        else:
            return not is_gap_safe(
                front_position=other_vehicle.position,
                front_speed=other_vehicle.speed,
                front_max_deceleration=other_vehicle.max_deceleration,
                front_length=other_vehicle.length,
                back_position=depart_position,
                back_speed=depart_speed,
                back_max_acceleration=vtype.max_acceleration,
                step_length=self._step_length
            )

    def _add_vehicle(
            self,
            vid,
            vtype,
            depart_position,
            arrival_position,
            desired_speed,
            depart_lane,
            depart_speed,
            depart_time,
            communication_range: int = DEFAULTS['communication_range'],
    ):
        """
        Adds a vehicle to the simulation based on the given parameters.

        Parameters
        ----------
        vid : int
            The id of the vehicle
        vtype : VehicleType
            The vehicle type of the vehicle
        depart_position : int
            The depart position of the vehicle
        arrival_position : int
            The arrival position of the vehicle
        desired_speed : float
            The desired driving speed of the vehicle
        depart_lane : int
            The depart lane of the vehicle
        depart_speed : float
            The depart speed of the vehicle
        depart_time : int
            The depart time of the vehicle
        communication_range : int
            The maximum communication range of the vehicle
        """

        # choose vehicle "type" depending on the penetration rate
        if random.random() <= self._penetration_rate:
            vehicle = PlatooningVehicle(
                self,
                vid,
                vtype,
                depart_position,
                arrival_position,
                desired_speed,
                depart_lane,
                depart_speed,
                depart_time,
                self._communication_range,
                self._acc_headway_time,
                self._cacc_spacing,
                self._formation_algorithm if self._formation_strategy == "distributed" else None,
                self._execution_interval,
                **self._kwargs,
            )
        else:
            vehicle = Vehicle(
                self,
                vid,
                vtype,
                depart_position,
                arrival_position,
                desired_speed,
                depart_lane,
                depart_speed,
                depart_time,
                self._communication_range,
            )

        # add instance
        self._vehicles[vid] = vehicle
        self._last_vehicle_id = vid

        return vehicle

    def _generate_infrastructures(self, number_of_infrastructures: int):
        """
        Generates infrastructures for the simulation.

        Parameters
        ----------
        number_of_infrastructures : int
            The number of infrastructures to generate
        """

        if number_of_infrastructures <= 0:
            return

        placement_interval = self._road_length / number_of_infrastructures

        for iid in tqdm(range(0, number_of_infrastructures), desc="Generated infrastructures", disable=not self._progress):
            position = (iid + 0.5) * placement_interval

            infrastructure = Infrastructure(
                self,
                iid,
                position,
                self._formation_algorithm if self._formation_strategy == "centralized" else None,
                self._execution_interval,
                **self._kwargs,
            )
            self._infrastructures[iid] = infrastructure

            LOG.info(f"Generated infrastructure {infrastructure} at {position}")

    def _initialize_result_recording(self):
        """Creates output files for all (enabled) statistics and writes the headers."""

        # write some general information about the simulation
        record_general_data_begin(basename=self._result_base_filename, simulator=self)

        if self._record_vehicle_trips:
            # create output file for vehicle trips
            initialize_vehicle_trips(basename=self._result_base_filename)

        if self._record_vehicle_emissions:
            # create output file for vehicle emissions
            initialize_vehicle_emissions(basename=self._result_base_filename)

        if self._record_platoon_trips:
            # create output file for platoon trips
            initialize_platoon_trips(basename=self._result_base_filename)

        if self._record_platoon_maneuvers:
            # create output file for platoon maneuvers
            initialize_platoon_maneuvers(basename=self._result_base_filename)

        if self._record_platoon_formation:
            # create output file for platoon formation
            initialize_platoon_formation(basename=self._result_base_filename)

        if self._record_infrastructure_assignments:
            # create output file for infrastructure assignments
            initialize_infrastructure_assignments(basename=self._result_base_filename)

        # traces

        if self._record_vehicle_traces:
            # create output file for vehicle traces
            initialize_vehicle_traces(basename=self._result_base_filename)

        if self._record_vehicle_changes:
            # create output file for vehicle lane changes
            initialize_vehicle_changes(basename=self._result_base_filename)

        if self._record_emission_traces:
            # create output file for emission traces
            initialize_emission_traces(basename=self._result_base_filename)

        if self._record_platoon_traces:
            # create output file for vehicle platoon traces
            initialize_vehicle_platoon_traces(basename=self._result_base_filename)

            # create output file for platoon traces
            initialize_platoon_traces(basename=self._result_base_filename)

        if self._record_platoon_changes:
            # create output file for platoon lane changes
            initialize_platoon_changes(basename=self._result_base_filename)

            # create output file for vehicle_platoon lane changes
            initialize_vehicle_platoon_changes(basename=self._result_base_filename)

        if self._record_simulation_trace:
            # create output file for continuous simulation trace
            initialize_simulation_trace(basename=self._result_base_filename)

    def _initialize_prefilled_platoon(self):
        """
        Initializes all pre-filled vehicles as one platoon.
        """

        # we avoid the complicated join procedure and simply set all parameters
        leader = self._vehicles[0]
        leader._platoon_role = PlatoonRole.LEADER
        leader._platoon._formation = list(self._vehicles.values())
        if self._update_desired_speed:
            leader.platoon.update_desired_speed()
        leader.platoon.update_limits()
        leader._cf_target_speed = leader.platoon.desired_speed
        for vehicle in list(self._vehicles.values())[1:]:
            vehicle._platoon_role = PlatoonRole.FOLLOWER
            vehicle._cf_model = CF_Model.CACC
            vehicle._blocked_front = False
            vehicle._platoon = leader.platoon
            vehicle._cf_target_speed = vehicle._platoon.desired_speed

    def _initialize_gui(self):
        """Initializes the GUI."""

        # start gui
        start_gui(self._sumo_config, self._gui_play)

        # draw ramps
        if self._draw_ramps:
            draw_ramps(
                road_length=self._road_length,
                interval=self._ramp_interval,
                labels=self._draw_ramp_labels
            )

        # draw road end
        if self._draw_road_end:
            draw_road_end(
                length=self._road_length,
                label=self._draw_road_end_label
            )

        # draw infrastructures
        if self._draw_infrastructures:
            draw_infrastructures(
                infrastructures=self._infrastructures.values(),
                labels=self._draw_infrastructure_labels
            )

        # draw pre-filled vehicles
        for vehicle in self._vehicles.values():
            add_gui_vehicle(vehicle, track=vehicle.vid == self._gui_track_vehicle)

    def _update_gui(self):
        """Updates the GUI."""

        for vehicle in self._vehicles.values():
            # update vehicles
            move_gui_vehicle(vehicle)

        # remove vehicles not in simulator
        prune_vehicles(keep_vids=self._vehicles.keys())

        # sleep for visualization
        time.sleep(self._gui_delay)

    def run(self):
        """
        Main simulation method.
        Runs the simulation with the specified parameters until it is stopped.

        This is based on Krauss' multi lane traffic:
        laneChange();
        adjust();
        move();
        """

        if not self._running:
            self._running = True
        else:
            LOG.warning("Simulation is already running!")

        self._initialize_result_recording()

        if self._start_as_platoon:
            self._initialize_prefilled_platoon()

        progress_bar = tqdm(desc='Simulation progress', total=self._max_step, unit='step', disable=not self._progress)
        # let the simulator run
        while self._running:
            start_time = timer()

            if self._step >= self._max_step:
                self.stop("Reached step limit")
                continue

            # initialize the GUI
            if self._gui and self._step == self._gui_start:
                self._initialize_gui()

            # spawn vehicle based on given parameters
            self._spawn_vehicles()

            if self._vehicles:
                # update the GUI
                if self._gui and self._step >= self._gui_start:
                    self._update_gui()

                # call regular actions on vehicles
                self._call_vehicle_actions()
                # call regular actions on infrastructure
                self._call_infrastructure_actions()

                # BEGIN VECTORIZATION PART
                # TODO move upwards/get rid of it entirely
                # convert dict of vehicles to dataframe (temporary)
                vdf = self._get_vehicles_df()
                vdf = vdf.sort_values(["position", "lane"], ascending=False)

                # perform lane changes (for all vehicles)
                # update neighbor data (predecessor, successor, front)
                lane_changes = compute_lane_changes(
                    vdf=vdf,
                    max_lane=self._number_of_lanes - 1,
                    step_length=self._step_length,
                )
                vdf['old_lane'] = vdf['lane']
                vdf['lane'] = lane_changes['lane']
                vdf['lc_reason'] = lane_changes['reason']

                # record lane changes
                # TODO move to better location
                if self._record_vehicle_changes or self._record_platoon_changes:
                    self._record_lane_changes(vdf)

                # update neighbor data (predecessor, successor, front)
                predecessor_map = lane_predecessors(vdf, self._number_of_lanes - 1)
                predecessor = get_predecessors(
                    vdf=vdf,
                    predecessor_map=predecessor_map,
                    target_lane=vdf.lane,
                ).rename(columns=lambda col: "predecessor_" + col)

                # adjust speed (of all vehicles)
                new_speed = compute_new_speeds(
                    vdf.merge(predecessor, left_index=True, right_index=True),
                    step_length=self._step_length,
                )
                report_rough_braking(vdf, new_speed)
                vdf['old_speed'] = vdf['speed']
                vdf['speed'] = new_speed
                vdf['blocked_front'] = (
                    (vdf.speed < vdf.max_speed)
                    & ((vdf.speed - vdf.old_speed) <= 0)
                    & (vdf.cf_model != CF_Model.CACC)
                )

                # adjust positions (of all vehicles)
                vdf = update_position(vdf, self._step_length)

                # convert dataframe back to dict of vehicles
                self._write_back_vehicles_df(vdf)

                # get arrived vehicles
                arrived_vehicles = vdf[
                    (vdf.position >= vdf.arrival_position)
                ].index.values

                # remove arrived vehicles from dataframe
                vdf = vdf.drop(arrived_vehicles)

                # do collision check (for all vehicles)
                # without arrived vehicles
                if self._collisions:
                    self._check_collisions(vdf)

                # remove arrived vehicles from dict and do finish
                self._remove_arrived_vehicles(arrived_vehicles)

                # make sure that everything is correct
                assert(list(vdf.index).sort() == list(self._vehicles.keys()).sort())

                del vdf
                # END VECTORIZATION PART
            else:
                if self._vehicles_scheduled == 0:
                    self.stop("No more vehicles in the simulation")  # do we really want to exit here?

            end_time = timer()

            # record some periodic statistics
            run_time = end_time - start_time
            self._statistics(run_time)

            # a new step begins
            self._step += self._step_length
            progress_bar.update(self._step_length)
            if self._gui and self._step > self._gui_start:
                gui_step(target_step=self._step)

        # We reach this point only by setting self._running to False
        # which is only done by calling self.stop()

        self._finish()

        return self._step

    def _statistics(self, runtime: float):
        """Record some period statistics."""

        self._avg_number_vehicles = (
            (self._values_in_avg_number_vehicles * self._avg_number_vehicles + len(self._vehicles)) /
            (self._values_in_avg_number_vehicles + 1)
        )

        if self._record_simulation_trace:
            # write continuous simulation traces
            record_simulation_trace(basename=self._result_base_filename, simulator=self, runtime=runtime)

    def _record_lane_changes(self, vdf: pd.DataFrame):
        for row in vdf.query('lane != old_lane').itertuples():
            if row.cf_model != CF_Model.CACC:
                if self._record_vehicle_changes:
                    record_vehicle_change(
                        basename=self._result_base_filename,
                        step=self._step,
                        vid=row.Index,
                        position=row.position,
                        speed=row.speed,
                        source_lane=row.old_lane,
                        target_lane=row.lane,
                        reason=row.lc_reason,
                    )
                if self._record_platoon_changes and row.platoon_role == PlatoonRole.LEADER:
                    record_platoon_change(
                        basename=self._result_base_filename,
                        step=self._step,
                        leader=self._vehicles[row.Index],
                        source_lane=row.old_lane,
                        target_lane=row.lane,
                        reason=row.lc_reason,
                    )
            else:
                if self._record_platoon_changes:
                    record_vehicle_platoon_change(
                        basename=self._result_base_filename,
                        step=self._step,
                        member=self._vehicles[row.Index],
                        source_lane=row.old_lane,
                        target_lane=row.lane,
                        reason=row.lc_reason,
                    )

    def _get_vehicles_df(self) -> pd.DataFrame:
        """Returns a pandas dataframe from the internal data structure."""
        platoon_fields = [
            "leader_id",
            "platoon_id",
            "platoon_desired_speed",
            "platoon_max_speed",
            "platoon_max_acceleration",
            "platoon_max_deceleration",
            "platoon_position",
            "platoon_rear_position",
        ]

        def get_platoon_data(vehicle):
            data = {key: 1e15 for key in platoon_fields}
            data["leader_id"] = -1
            data["leader_id"] = -1
            data["platoon_role"] = None

            if not isinstance(vehicle, PlatooningVehicle):
                return data

            platoon = vehicle._platoon
            data["leader_id"] = platoon._formation[0].vid
            data["platoon_id"] = platoon._platoon_id
            data["platoon_role"] = vehicle._platoon_role
            data["platoon_desired_speed"] = platoon._desired_speed
            data["platoon_max_speed"] = platoon.max_speed
            data["platoon_max_acceleration"] = platoon.max_acceleration
            data["platoon_max_deceleration"] = platoon.max_deceleration
            data["platoon_position"] = platoon.position
            data["platoon_rear_position"] = platoon.rear_position
            return data

        fields = [
            "arrival_position",
            "cf_target_speed",
            "position",
            "lane",
            "speed",
            "cf_model",
            "vid",
        ]
        vtype_fields = [
            "length",
            "min_gap",
            "max_speed",
            "max_acceleration",
            "max_deceleration",
        ]

        if not self._vehicles:
            return pd.DataFrame({key: [] for key in fields + vtype_fields + platoon_fields + ["desired_headway_time", "acc_lambda"]}).drop(["cf_target_speed"], axis="columns")
        return (
            pd.DataFrame([
                dict(
                    **{key: vehicle.__dict__[f"_{key}"] for key in fields},
                    **{key: vehicle._vehicle_type.__dict__[f"_{key}"] for key in vtype_fields},
                    **get_platoon_data(vehicle),
                    desired_headway_time=vehicle.desired_headway_time,
                    acc_lambda=getattr(vehicle, "_acc_lambda", np.nan),
                    rear_position=vehicle.rear_position,
                )
                for vehicle in self._vehicles.values()
            ])
            .astype({"cf_model": CFModelDtype, "platoon_role": PlatoonRoleDtype})
            .set_index('vid')
            # compute effective limits
            # These computations may negatively impact performance, but will
            # eventually be removed from here anyway once the DataFrame stays
            # and computations/updates move to actions (like platoon updates).
            .assign(
                max_speed=lambda df: df[["max_speed", "cf_target_speed", "platoon_max_speed", "platoon_desired_speed"]].min(axis="columns"),
                max_acceleration=lambda df: df[["max_acceleration", "platoon_max_acceleration"]].min(axis="columns"),
                max_deceleration=lambda df: df[["max_deceleration", "platoon_max_deceleration"]].min(axis="columns"),
                rear_position=lambda df: df[["rear_position", "platoon_rear_position"]].min(axis="columns"),
            )
            .drop(["cf_target_speed"], axis="columns")
        )

    def _write_back_vehicles_df(self, vdf: pd.DataFrame):
        """
        Writes back the vehicle updates from a given pandas dataframe to the internal data structure.

        Parameters
        ----------
        vdf : pandas.DataFrame
            The dataframe containing the vehicles as rows
            index: vid
            columns: [position, length, lane, ..]
        """

        # make sure that everything is correct
        assert(list(vdf.index).sort() == list(self._vehicles.keys()).sort())

        for row in vdf.itertuples():
            # update all fields within the data that we updated with pandas
            vehicle = self._vehicles[row.Index]
            vehicle._position = row.position
            vehicle._speed = row.speed
            vehicle._acceleration = row.speed - row.old_speed
            vehicle._blocked_front = row.blocked_front
            vehicle._lane = row.lane

    def stop(self, msg: str):
        """
        Stops the simulation with the given message.

        Parameters
        ----------
        msg : str
            The message to show after stopping the simulation
        """

        self._running = False
        if self._progress:
            print(f"\n{msg}")

    def __str__(self) -> str:
        """Returns a str representation of a simulator instance."""

        sim_dict = self.__dict__.copy()
        sim_dict.pop('_vehicles')
        sim_dict.pop('_infrastructures')
        sim_dict.update({'current_number_of_vehicles': len(self._vehicles)})
        sim_dict.update({'current_number_of_infrastructures': len(self._infrastructures)})
        return str(sim_dict)

    def _finish(self):
        """Cleans up the simulation."""

        if self._running:
            LOG.warning("Finish called during simulation!")
            return

        # write some general information about the simulation
        record_general_data_end(basename=self._result_base_filename, simulator=self)

        # call finish on infrastructures
        for infrastructure in self._infrastructures.values():
            infrastructure.finish()

        # call finish on remaining vehicles?
        for vehicle in self._vehicles.values():
            # we do not want to write statistics for not finished vehicles
            # therefore, we do not call finish here
            if self._gui and self._step >= self._gui_start:
                remove_gui_vehicle(vehicle._vid)
            # remove from vehicles
            del vehicle

        if self._gui and self._step >= self._gui_start:
            close_gui()
