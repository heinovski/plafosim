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

import pandas as pd

from plafosim.util import (
    acceleration2speed,
    distance2speed,
    get_crashed_vehicles,
    speed2acceleration,
    speed2distance,
)


def test_speed2distance():
    assert(speed2distance(36.0) == 36.0)
    assert(speed2distance(26.0, 10) == 260.0)


def test_distance2speed():
    assert(distance2speed(275.0) == 275.0)
    assert(distance2speed(257.0, 10) == 25.7)


def test_acceleration2speed():
    assert(acceleration2speed(2.5) == 2.5)
    assert(acceleration2speed(5.2, 10) == 52.0)


def test_speed2acceleration():
    assert(speed2acceleration(36.0, 24.0) == -12.0)
    assert(speed2acceleration(32.0, 42.0, 10) == 1.0)


def test_get_crashed_vehicles():
    vehicle_dtypes = {
        'vid': int,
        'position': float,
        'length': float,
        'lane': int,
    }

    raw_no = [
        {
            'vid': 0,
            'position': 19,
            'length': 4,
            'lane': 2,
        },
        {
            'vid': 1,
            'position': 11.5,
            'length': 4,
            'lane': 1,
        },
        {
            'vid': 2,
            'position': 6.5,
            'length': 4,
            'lane': 2,
        },
        {
            'vid': 3,
            'position': 24,
            'length': 4,
            'lane': 1,
        }
    ]
    vehicles_no = pd.DataFrame(raw_no, columns=vehicle_dtypes.keys()).astype(vehicle_dtypes).set_index('vid')
    assert(not get_crashed_vehicles(vehicles_no))

    raw_yes = [
        {
            'vid': 0,
            'position': 19,
            'length': 4,
            'lane': 2,
        },
        {
            'vid': 1,
            'position': 20.5,
            'length': 4,
            'lane': 1,
        },
        {
            'vid': 2,
            'position': 16.5,
            'length': 4,
            'lane': 2,
        },
        {
            'vid': 3,
            'position': 24,
            'length': 4,
            'lane': 1,
        }
    ]
    vehicles_yes = pd.DataFrame(raw_yes, columns=vehicle_dtypes.keys()).astype(vehicle_dtypes).set_index('vid')
    assert(get_crashed_vehicles(vehicles_yes) == [0, 1, 2, 3])
