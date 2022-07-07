#
# Copyright (c) 2020-2022 Julian Heinovski <heinovski@ccs-labs.org>
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

import time
from typing import TYPE_CHECKING

from plafosim import __version__

from .formation_algorithm import FormationAlgorithm
from .util import rgb2hex

if TYPE_CHECKING:
    from .platooning_vehicle import PlatooningVehicle  # noqa 401
    from .simulator import Simulator  # noqa 401
    from .vehicle import Vehicle  # noqa 401


def record_general_data_begin(basename: str, simulator: 'Simulator'):
    assert basename
    from .simulator import Simulator
    assert isinstance(simulator, Simulator)
    with open(f'{basename}_general.out', 'w') as f:
        f.write(f"simulation start: {time.asctime(time.localtime(time.time()))}\n")
        f.write(f"version: {__version__}\n")
        f.write(f"parameters:\n{str(simulator)}\n")


def record_general_data_end(basename: str, simulator: 'Simulator'):
    assert basename
    from .simulator import Simulator
    assert isinstance(simulator, Simulator)
    with open(f'{basename}_general.out', 'a') as f:
        f.write(f"simulation end: {time.asctime(time.localtime(time.time()))}\n")
        f.write(f"average number of vehicles: {simulator._avg_number_vehicles}\n")


def initialize_vehicle_trips(basename: str):
    assert basename
    with open(f'{basename}_vehicle_trips.csv', 'w') as f:
        f.write(
            "id,"
            "vType,"
            "eClass,"
            "vClass,"
            "depart,"
            "departLane,"
            "departPos,"
            "departSpeed,"
            "departDelay,"
            "arrival,"
            "arrivalLane,"
            "arrivalPos,"
            "arrivalSpeed,"
            "duration,"
            "routeLength,"
            "timeLoss,"
            "desiredSpeed,"
            "expectedTravelTime,"
            "travelTimeRatio,"
            "avgDrivingSpeed,"
            "avgDeviationDesiredSpeed"
            "\n"
        )


def record_vehicle_trip(
    basename: str,
    vehicle: 'Vehicle',
    depart_delay: int,
    time_loss: int,
    expected_travel_time: float,
    travel_time_ratio: float,
    average_driving_speed: float,
    average_deviation_desired_speed: float,
):
    from .vehicle import Vehicle
    assert isinstance(vehicle, Vehicle)

    with open(f'{basename}_vehicle_trips.csv', 'a') as f:
        f.write(
            f"{vehicle._vid},"
            f"{vehicle._vehicle_type.name},"
            f"{vehicle._vehicle_type.emission_class.name},"
            f"{vehicle.__class__.__name__},"
            f"{vehicle._depart_time},"
            f"{vehicle._depart_lane},"
            f"{vehicle._depart_position},"
            f"{vehicle._depart_speed},"
            f"{vehicle._depart_delay},"
            f"{vehicle._simulator.step},"
            f"{vehicle._lane},"
            f"{vehicle._position},"
            f"{vehicle._speed},"
            f"{vehicle.travel_time},"
            f"{vehicle.travel_distance},"
            f"{time_loss},"
            f"{vehicle._desired_speed},"  # use explicit individual desired speed
            f"{expected_travel_time},"
            f"{travel_time_ratio},"
            f"{average_driving_speed},"
            f"{average_deviation_desired_speed}"
            "\n"
        )


def initialize_vehicle_emissions(basename: str):
    assert basename
    with open(f'{basename}_vehicle_emissions.csv', 'w') as f:
        f.write(
            "id,"
            "CO,"
            "CO2,"
            "HC,"
            "NOx,"
            "PMx,"
            "fuel"
            "\n"
        )


def record_vehicle_emission(basename: str, vehicle: 'Vehicle'):
    assert basename
    from .vehicle import Vehicle
    assert isinstance(vehicle, Vehicle)
    with open(f'{basename}_vehicle_emissions.csv', 'a') as f:
        # TODO log estimated emissions?
        f.write(
            f"{vehicle._vid},"
            f"{vehicle._emissions['CO']},"
            f"{vehicle._emissions['CO2']},"
            f"{vehicle._emissions['HC']},"
            f"{vehicle._emissions['NOx']},"
            f"{vehicle._emissions['PMx']},"
            f"{vehicle._emissions['fuel']}"
            "\n"
        )


def initialize_platoon_trips(basename: str):
    assert basename
    with open(f'{basename}_vehicle_platoon_trips.csv', 'w') as f:
        f.write(
            "id,"
            "timeInPlatoon,"
            "distanceInPlatoon,"
            "platoonTimeRatio,"
            "platoonDistanceRatio,"
            "numberOfPlatoons,"
            "timeUntilFirstPlatoon,"
            "distanceUntilFirstPlatoon"
            "\n"
        )


def record_platoon_trip(
    basename: str,
    vehicle: 'PlatooningVehicle',
    platoon_time_ratio: float,
    platoon_distance_ratio: float,
    time_until_first_platoon: float,
    distance_until_first_platoon: float,
):
    assert basename
    from .platooning_vehicle import PlatooningVehicle
    assert isinstance(vehicle, PlatooningVehicle)

    # TODO log savings from platoon?
    with open(f'{basename}_vehicle_platoon_trips.csv', 'a') as f:
        f.write(
            f"{vehicle._vid},"
            f"{vehicle._time_in_platoon},"
            f"{vehicle._distance_in_platoon},"
            f"{platoon_time_ratio},"
            f"{platoon_distance_ratio},"
            f"{vehicle._number_platoons},"
            f"{time_until_first_platoon},"
            f"{distance_until_first_platoon}"
            "\n"
        )


def initialize_platoon_maneuvers(basename: str):
    assert basename
    with open(f'{basename}_vehicle_platoon_maneuvers.csv', 'w') as f:
        f.write(
            "id,"
            "joinsAttempted,"
            "joinsSuccessful,"
            "joinsAborted,"
            "joinsAbortedFront,"
            "joinsAbortedArbitrary,"
            "joinsAbortedRoadBegin,"
            "joinsAbortedTripBegin,"
            "joinsAbortedRoadEnd,"
            "joinsAbortedTripEnd,"
            "joinsAbortedLeaderManeuver,"
            "joinsAbortedMaxSpeed,"
            "joinsAbortedTeleportThreshold,"
            "joinsAbortedApproaching,"
            "joinsAbortedNoSpace,"
            "joinsAbortedLeaveOther,"
            "joinsFront,"
            "joinsArbitrary,"
            "joinsBack,"
            "joinsTeleportPosition,"
            "joinsTeleportLane,"
            "joinsTeleportSpeed,"
            "joinsCorrectPosition,"
            "leavesAttempted,"
            "leavesSuccessful,"
            "leavesAborted,"
            "leavesFront,"
            "leavesArbitrary,"
            "leavesBack"
            "\n"
        )


def record_vehicle_platoon_maneuvers(basename: str, vehicle: 'PlatooningVehicle'):
    assert basename
    from .platooning_vehicle import PlatooningVehicle
    assert isinstance(vehicle, PlatooningVehicle)
    with open(f'{basename}_vehicle_platoon_maneuvers.csv', 'a') as f:
        f.write(
            f"{vehicle._vid},"
            f"{vehicle._joins_attempted},"
            f"{vehicle._joins_succesful},"
            f"{vehicle._joins_aborted},"
            f"{vehicle._joins_aborted_front},"
            f"{vehicle._joins_aborted_arbitrary},"
            f"{vehicle._joins_aborted_road_begin},"
            f"{vehicle._joins_aborted_trip_begin},"
            f"{vehicle._joins_aborted_road_end},"
            f"{vehicle._joins_aborted_trip_end},"
            f"{vehicle._joins_aborted_leader_maneuver},"
            f"{vehicle._joins_aborted_max_speed},"
            f"{vehicle._joins_aborted_teleport_threshold},"
            f"{vehicle._joins_aborted_approaching},"
            f"{vehicle._joins_aborted_no_space},"
            f"{vehicle._joins_aborted_leave_other},"
            f"{vehicle._joins_front},"
            f"{vehicle._joins_arbitrary},"
            f"{vehicle._joins_back},"
            f"{vehicle._joins_teleport_position},"
            f"{vehicle._joins_teleport_lane},"
            f"{vehicle._joins_teleport_speed},"
            f"{vehicle._joins_correct_position},"
            f"{vehicle._leaves_attempted},"
            f"{vehicle._leaves_successful},"
            f"{vehicle._leaves_aborted},"
            f"{vehicle._leaves_front},"
            f"{vehicle._leaves_arbitrary},"
            f"{vehicle._leaves_back}"
            "\n"
        )


def initialize_platoon_formation(basename: str):
    assert basename
    with open(f'{basename}_vehicle_platoon_formation.csv', 'w') as f:
        f.write(
            "id,"
            "formationIterations,"
            "candidatesFound,"
            "candidatesFoundAvg,"
            "candidatesFiltered,"
            "candidatesFilteredAvg,"
            "candidatesFilteredFollower,"
            "candidatesFilteredManeuver"
            "\n"
        )


def record_platoon_formation(
    basename: str,
    vehicle: 'PlatooningVehicle',
    candidates_found_avg: float,
    candidates_filtered_avg: float
):
    assert basename
    from .platooning_vehicle import PlatooningVehicle
    assert isinstance(vehicle, PlatooningVehicle)

    with open(f'{basename}_vehicle_platoon_formation.csv', 'a') as f:
        f.write(
            f"{vehicle._vid},"
            f"{vehicle._formation_iterations},"
            f"{vehicle._candidates_found},"
            f"{candidates_found_avg},"
            f"{vehicle._candidates_filtered},"
            f"{candidates_filtered_avg},"
            f"{vehicle._candidates_filtered_follower},"
            f"{vehicle._candidates_filtered_maneuver}"
            "\n"
        )


def initialize_infrastructure_assignments(basename: str):
    assert basename
    with open(f'{basename}_infrastructure_assignments.csv', 'w') as f:
        f.write(
            "id,"
            "assignmentsSolved,"
            "assignmentsNotSolvable,"
            "assignmentsSolvedOptimal,"
            "assignmentsSolvedFeasible,"
            "assignmentsNone,"
            "assignmentsSelf,"
            "assignmentsCandidateJoinedAlready,"
            "assignmentsVehicleBecameLeader,"
            "assignmentsSuccessful"
            "\n"
        )


def record_infrastructure_assignments(basename: str, iid: int, algorithm: FormationAlgorithm):
    assert basename
    assert isinstance(algorithm, FormationAlgorithm)
    with open(f'{basename}_infrastructure_assignments.csv', 'a') as f:
        f.write(
            f"{iid},"
            f"{algorithm._assignments_solved},"
            f"{algorithm._assignments_not_solvable},"
            f"{algorithm._assignments_solved_optimal},"
            f"{algorithm._assignments_solved_feasible},"
            f"{algorithm._assignments_none},"
            f"{algorithm._assignments_self},"
            f"{algorithm._assignments_candidate_joined_already},"
            f"{algorithm._assingments_vehicle_became_leader},"
            f"{algorithm._assignments_successful}"
            "\n"
        )

# traces


def initialize_vehicle_traces(basename: str):
    assert basename
    with open(f'{basename}_vehicle_traces.csv', 'w') as f:
        f.write(
            "step,"
            "id,"
            "position,"
            "lane,"
            "speed,"
            "blocked,"
            "duration,"
            "routeLength,"
            "desiredSpeed,"
            "cfTargetSpeed,"
            "cfModel,"
            "color"
            "\n"
        )


def record_vehicle_trace(basename: str, step: int, vehicle: 'Vehicle'):
    assert basename
    from .vehicle import Vehicle
    assert isinstance(vehicle, Vehicle)
    with open(f'{basename}_vehicle_traces.csv', 'a') as f:
        f.write(
            f"{step},"
            f"{vehicle._vid},"
            f"{vehicle._position},"
            f"{vehicle._lane},"
            f"{vehicle._speed},"
            f"{vehicle._blocked_front},"
            f"{vehicle.travel_time},"
            f"{vehicle.travel_distance},"
            f"{vehicle.desired_speed},"  # use potential platoon desired driving speed
            f"{vehicle._cf_target_speed},"
            f"{vehicle._cf_model.name},"
            f"{rgb2hex(vehicle.color)}"  # use potential platoon color
            "\n"
        )


def initialize_vehicle_changes(basename: str):
    assert basename
    with open(f'{basename}_vehicle_changes.csv', 'w') as f:
        f.write(
            "step,"
            "id,"
            "position,"
            "from,"
            "to,"
            "speed,"
            "reason"
            "\n"
        )


def record_vehicle_change(
    basename: str,
    step: int,
    vid: int,
    position: float,
    speed: float,
    source_lane: int,
    target_lane: int,
    reason: str,
):
    with open(f'{basename}_vehicle_changes.csv', 'a') as f:
        f.write(
            f"{step},"
            f"{vid},"
            f"{position},"
            f"{source_lane},"
            f"{target_lane},"
            f"{speed},"
            f"{reason}"
            "\n"
        )


def initialize_emission_traces(basename: str):
    assert basename
    with open(f'{basename}_emission_traces.csv', 'w') as f:
        f.write(
            "step,"
            "id,"
            "CO,"
            "CO2,"
            "HC,"
            "NOx,"
            "PMx,"
            "fuel"
            "\n"
        )


def record_emission_trace_prefix(basename: str, step: int, vid: int):
    assert basename
    with open(f'{basename}_emission_traces.csv', 'a') as f:
        f.write(
            f"{step},"
            f"{vid}"
        )


def record_emission_trace_value(basename: str, value: float):
    assert basename
    with open(f'{basename}_emission_traces.csv', 'a') as f:
        f.write(
            f",{value}"
        )


def record_emission_trace_suffix(basename: str):
    assert basename
    with open(f'{basename}_emission_traces.csv', 'a') as f:
        f.write("\n")


def initialize_vehicle_platoon_traces(basename: str):
    assert basename
    with open(f'{basename}_vehicle_platoon_traces.csv', 'w') as f:
        f.write(
            "step,"
            "id,"
            "platoon,"
            "platoonRole,"
            "platoonPosition"
            "\n"
        )


def record_vehicle_platoon_trace(basename: str, step: int, vehicle: 'PlatooningVehicle'):
    assert basename
    from .platooning_vehicle import PlatooningVehicle
    assert isinstance(vehicle, PlatooningVehicle)
    with open(f'{basename}_vehicle_platoon_traces.csv', 'a') as f:
        f.write(
            f"{step},"
            f"{vehicle._vid},"
            f"{vehicle._platoon.platoon_id},"
            f"{vehicle._platoon_role.name},"
            f"{vehicle._platoon.get_member_index(vehicle)}"
            "\n"
        )


def initialize_platoon_traces(basename: str):
    assert basename
    with open(f'{basename}_platoon_traces.csv', 'w') as f:
        f.write(
            "step,"
            "id,"
            "leader,"
            "position,"
            "rearPosition,"
            "lane,"
            "speed,"
            "size,"
            "length,"
            "desiredSpeed"
            "\n"
        )


def record_platoon_trace(basename: str, step: int, vehicle: 'PlatooningVehicle'):
    assert basename
    from .platooning_vehicle import PlatooningVehicle
    assert isinstance(vehicle, PlatooningVehicle)
    with open(f'{basename}_platoon_traces.csv', 'a') as f:
        f.write(
            f"{step},"
            f"{vehicle._platoon.platoon_id},"
            f"{vehicle._platoon.leader.vid},"
            f"{vehicle._platoon.position},"
            f"{vehicle._platoon.rear_position},"
            f"{vehicle._platoon.lane},"
            f"{vehicle._platoon.speed},"
            f"{vehicle._platoon.size},"
            f"{vehicle._platoon.length},"
            f"{vehicle._platoon.desired_speed}"
            "\n"
        )


def initialize_platoon_changes(basename: str):
    assert basename
    with open(f'{basename}_platoon_changes.csv', 'w') as f:
        f.write(
            "step,"
            "id,"
            "position,"
            "from,"
            "to,"
            "speed,"
            "reason"
            "\n"
        )


def record_platoon_change(
    basename: str,
    step: int,
    leader: 'PlatooningVehicle',
    source_lane: int,
    target_lane: int,
    reason: str,
):
    assert basename
    from .platooning_vehicle import PlatooningVehicle
    assert isinstance(leader, PlatooningVehicle)
    from .platoon_role import PlatoonRole
    assert leader.platoon_role == PlatoonRole.LEADER and leader.platoon.leader.vid == leader.vid
    with open(f'{basename}_platoon_changes.csv', 'a') as f:
        f.write(
            f"{step},"
            f"{leader.platoon.platoon_id},"
            f"{leader.position},"
            f"{source_lane},"
            f"{target_lane},"
            f"{leader.speed},"
            f"{reason}"
            "\n"
        )


def initialize_vehicle_platoon_changes(basename: str):
    assert basename
    with open(f'{basename}_vehicle_platoon_changes.csv', 'w') as f:
        f.write(
            "step,"
            "id,"
            "platoon,"
            "position,"
            "from,"
            "to,"
            "speed,"
            "reason"
            "\n"
        )


def record_vehicle_platoon_change(
    basename: str,
    step: int,
    member: 'PlatooningVehicle',
    source_lane: int,
    target_lane: int,
    reason: str,
):
    assert basename
    from .platooning_vehicle import PlatooningVehicle
    assert isinstance(member, PlatooningVehicle)
    with open(f'{basename}_vehicle_platoon_changes.csv', 'a') as f:
        f.write(
            f"{step},"
            f"{member.vid},"
            f"{member.platoon.platoon_id},"
            f"{member.position},"
            f"{source_lane},"
            f"{target_lane},"
            f"{member.speed},"
            f"{reason}"
            "\n"
        )


def initialize_simulation_trace(basename: str):
    assert basename
    with open(f'{basename}_simulation_trace.csv', 'w') as f:
        f.write(
            "step,"
            "numberOfVehicles,"
            "vehiclesInSpawnQueue,"
            "vehiclesSpawned,"
            "vehiclesArrived,"
            "executionTime"
            "\n"
        )


def record_simulation_trace(
        basename: str,
        step: int,
        vehicles_in_simulator: int,
        vehicles_in_queue: int,
        vehicles_spawned: int,
        vehicles_arrived: int,
        runtime: float
):
    assert basename
    with open(f'{basename}_simulation_trace.csv', 'a') as f:
        f.write(
            f"{step},"
            f"{vehicles_in_simulator},"
            f"{vehicles_in_queue},"
            f"{vehicles_spawned},"
            f"{vehicles_arrived},"
            f"{runtime}"
            "\n"
        )
