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
#from plafosim import Message

import pytest

from plafosim.vehicle import safe_speed

DESIRED_HEADWAY_TIME = 1.0  # "tau" in krauss model
MAX_DECELERATION = 10  # "b" in krauss model

SAFE_SPEED_TESTDATA = [
    # v_pred, v_veh, gap, expected
    (10, 10, 10, 10),
    (10, 10, 15, 12.5),
    (10, 10, 20, 15),
    (10, 10, 30, 20),
    (12, 10, 5, 8.66),
    (12, 10, 20, 15.810),
    (20, 20, 20, 20),
    (20, 20, 10, 16.666),
    (20, 20, 30, 23.333),
    (20, 10, 20, 20),
    (20, 10, 15, 18),
    (20, 10, 30, 24)
]


@pytest.mark.parametrize("v_pred,v_veh,gap,expected", SAFE_SPEED_TESTDATA)
def test_safe_speed(v_pred, v_veh, gap, expected):
    speed = safe_speed(
        speed_predecessor=v_pred,
        gap_to_predecessor=gap,
        desired_headway_time=DESIRED_HEADWAY_TIME,
        desired_gap=v_veh * DESIRED_HEADWAY_TIME,
    )
    assert speed == pytest.approx(expected)
