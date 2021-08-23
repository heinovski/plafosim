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
            "ccTargetSpeed,"
            "cfModel"
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


def initialize_simulation_trace(basename: str):
    assert basename
    with open(f'{basename}_simulation_trace.csv', 'w') as f:
        f.write(
            "step,"
            "numberOfVehicles,"
            "executionTime"
            "\n"
        )
