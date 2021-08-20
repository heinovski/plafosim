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

from plafosim.emission_class import EMISSION_FACTORS, EmissionClass
from plafosim.vehicle_type import VehicleType


def test_creation():
    name = "vtype"
    length = 4
    max_speed = 55
    max_acceleration = 2.5
    max_deceleration = 15.0
    min_gap = 1.0
    cc_headway_time = 1.0
    emission_class = EmissionClass.PC_G_EU4

    vtype = VehicleType(
        name,
        length,
        max_speed,
        max_acceleration,
        max_deceleration,
        min_gap,
        cc_headway_time,
        emission_class.name,
    )

    assert(vtype is not None)
    assert(vtype.name == name)
    assert(vtype.length == length)
    assert(vtype.max_speed == max_speed)
    assert(vtype.max_acceleration == max_acceleration)
    assert(vtype.max_deceleration == max_deceleration)
    assert(vtype.min_gap == min_gap)
    assert(vtype.cc_headway_time == cc_headway_time)
    assert(vtype.emission_class == emission_class)
    assert(vtype.emission_factors == EMISSION_FACTORS[emission_class.name])
    assert(str(vtype) == str(vtype.__dict__))
