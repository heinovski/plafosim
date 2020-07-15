import pytest

import sys
sys.path.append("..")

from src.plafosim.vehicle import VehicleType


def test_creation():
    name = "vtype"
    length = 4
    max_speed = 36
    max_acceleration = 2.5
    max_deceleration = 15

    vtype = VehicleType(name, length, max_speed, max_acceleration, max_deceleration)

    assert(vtype is not None)
    assert(vtype.name is name)
    assert(vtype.length is length)
    assert(vtype.max_speed is max_speed)
    assert(vtype.max_acceleration is max_acceleration)
    assert(vtype.max_deceleration is max_deceleration)
