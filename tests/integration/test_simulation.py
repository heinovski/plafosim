#
# Copyright (c) 2020-2023 Julian Heinovski <heinovski@ccs-labs.org>
#
# SPDX-License-Identifier: GPL-3.0-or-later
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

import pytest

from plafosim.simulator import Simulator, vtype


def test_collision_detection():
    """
    Two vehicles (a slow and a fast one) are set up for a collision.
    The simulation should detect that and fail.
    """

    # create simulation environment
    s = Simulator(
        number_of_lanes=1,
        number_of_vehicles=2,
        penetration_rate=0,
        record_vehicle_traces=True,
        max_step=100,
    )

    # add first (slow) vehicle
    s._add_vehicle(
        vid=0,
        vtype=vtype,
        depart_position=15,
        arrival_position=s.road_length,
        desired_speed=0,
        depart_lane=0,
        depart_speed=vtype.max_deceleration,
        depart_time=0,
        communication_range=None,
    )

    # add second (fast) vehicle
    s._add_vehicle(
        vid=1,
        vtype=vtype,
        depart_position=10,
        arrival_position=s.road_length,
        desired_speed=20,
        depart_lane=0,
        depart_speed=20,
        depart_time=0,
        communication_range=None,
    )

    assert len(s._vehicles) == 2
    s._last_vehicle_id = 1

    # situation is clear at the beginning
    assert s._vehicles[0].position > s._vehicles[1].position

    # run the simulation to record the trace file
    with pytest.raises(SystemExit) as pytest_wrapped_e:
        s.run()
    assert pytest_wrapped_e.type == SystemExit
    assert pytest_wrapped_e.value.code == "ERROR: There were collisions with the following vehicles [0, 1]!"

    # TODO: check extreme collision cases in which one vehicle completely passes its successor. The check should still catch this.
