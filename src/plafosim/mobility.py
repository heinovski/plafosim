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

from collections import namedtuple

import numpy as np
import pandas as pd

from .cf_model import CF_Model

# gap safety checks


def is_gap_safe(
    front_position,
    front_speed,
    front_max_acceleration,
    front_length,
    back_position,
    back_speed,
    back_max_acceleration,
    step_length,
):
    """
    Return whether the gap between the front and back vehicle is safe.

    Safe means:
    - the front vehicle can decelerate as hard as possible for one step
    - the back vehicle can accelerate as hard as possible for one step
    - the will not crash

    Assumes euclidian/non-ballistic position updates.
    """
    next_front_position = (
        front_position
        - front_length
        + (front_speed - front_max_acceleration * step_length) * step_length
    )
    next_back_position = (
        back_position
        + (back_speed + back_max_acceleration * step_length) * step_length
    )
    return next_front_position > next_back_position


# speed update components


def safe_speed_df(vdf: pd.DataFrame) -> pd.Series:
    """
    Compute the safe speed akkording to the Krauss model, DataFrame variant.
    """

    tau_b = (vdf.predecessor_speed + vdf.speed) / (2 * vdf.max_deceleration)
    return vdf.predecessor_speed + (
        (vdf.predecessor_gap - vdf.desired_gap)
        / (vdf.desired_headway_time + tau_b)
    )


def speed_cc_df(vdf: pd.DataFrame) -> pd.Series:
    """
    Compute new speed for CC vehicles, DataFrame variant.

    Basically just safe_speed, clamping is done outside this function.
    """

    # TODO: implement dawdling
    speed_safe = safe_speed_df(vdf)
    return speed_safe


def speed_acc_df(vdf: pd.DataFrame, step_length: float = 1.0) -> pd.Series:
    """
    Compute new speed for ACC vehicles, DataFrame variant.

    Taken from Segata's dissertation / plexe.
    Clamping is done outside this function.
    """

    acceleration = (
        vdf.predecessor_speed
        - vdf.speed
        - vdf.acc_lambda * (vdf.desired_gap - vdf.predecessor_gap)
    ) / vdf.desired_headway_time

    return vdf.speed + acceleration * step_length


def clamp_speed(
    new_speed,
    vdf,
    step_length,
):
    """
    Clamp (two-way limit) a new speed value to vehicle's maximum.
    """
    new_speed = np.min(
        [
            new_speed,
            vdf.max_speed,
            vdf.speed + vdf.max_acceleration * step_length,
        ],
        axis=0,
    )
    new_speed = np.max(
        [
            new_speed,
            vdf.speed - vdf.max_deceleration * step_length,
            np.zeros_like(new_speed),  # do not drive backwards
        ],
        axis=0,
    )
    assert not any(new_speed < 0)
    return pd.Series(new_speed, index=vdf.index)


def compute_new_speeds(
    vdf: pd.DataFrame,
    step_length: float = 1.0,
) -> pd.Series:
    """
    Compute the new speed for all vehicles in the simulation.
    """
    # FIXME: this won't work for "potential" new speed checks on other lanes!
    #        the current vdf buidling / predecessor selection relies on lanes,
    #        so potential predecessors on other lanes can't be checked (yet).

    # sort vehicles and determine their predecessors and copy the DataFrame
    # TODO: potentially move this out of this function
    vdf = vdf.sort_values(by="position", ascending=False, inplace=False)
    vids = vdf.reset_index("vid").set_index(vdf.index).groupby("lane")["vid"]
    pred_id = vids.shift(1, fill_value=-1)
    # derive a frame for predecessor data including a virtual -1 predecessor
    pred_data = (
        vdf[["speed", "position", "length"]]
        .append(
            # TODO: turn into constant or so
            pd.Series({"speed": 1e15, "position": 1e15, "length": 0}, name=-1)
        )
        .assign(rear_position=lambda df: df["position"] - df["length"])
    )

    # derive common data for all cf models
    # FIXME: ensure this is also valid as the desired_gap for ACC
    vdf["desired_gap"] = np.max(
        [
            vdf["min_gap"],
            # using own speed instead of predecessor speed (like krauss) here
            vdf["desired_headway_time"] * vdf["speed"],
        ],
        axis=0,
    )
    # extract concrete predecessor data
    # Note: use .values here to use corect index
    vdf["predecessor_speed"] = pred_data.loc[pred_id, "speed"].values
    vdf["predecessor_gap"] = (
        pred_data.loc[pred_id, "rear_position"].values - vdf["position"]
    )
    # Note:
    # we assume vehicles already have their max acceleration/deceleration
    # set/updated for individual driving or platooning.
    # I.e., platoons share max acceleration/deceleration.
    # We also assume that evary vehicle has speed and postion values of its
    # predecessor. If there is none, the values should just be large defaults.
    # The CF functions shouls be able to deal with this inherently.

    m_cc = vdf.cf_model == CF_Model.CC
    m_acc = vdf.cf_model == CF_Model.ACC
    m_cacc = vdf.cf_model == CF_Model.CACC

    # apply models
    # Start with a copy of the old speed to get the size and index right.
    # This should be faster than building and concatenating multiple series
    # and should avoid re-sorting the result -- but maybe benchmark this later
    new_speed = vdf.speed.copy()
    new_speed.loc[m_cc] = speed_cc_df(vdf.loc[m_cc])
    new_speed.loc[m_acc] = speed_acc_df(vdf.loc[m_acc])
    # TODO: apply clamping only to cc and acc vehicles (not cacc)
    new_speed.loc[m_cacc] = new_speed[vdf.loc[m_cacc].leader_id].values

    # clamp speed by common constraints
    new_speed = clamp_speed(new_speed, vdf, step_length)

    assert not any(np.isnan(new_speed))
    assert not any(new_speed > 1e14)
    assert not any(new_speed < 0)

    return pd.Series(new_speed, index=vdf.index)


# Stuff to remove
#
# Just temporary helpers for migration to fully vectorized code.
# Remove once no longer needd

SAFE_SPEED_DF = namedtuple(
    "SAFE_SPEED_DF",
    [
        "predecessor_speed",
        "speed",
        "max_deceleration",
        "predecessor_gap",
        "desired_gap",
        "desired_headway_time",
    ],
)

ACC_SPEED_DF = namedtuple(
    "ACC_SPEED_DF",
    [
        "predecessor_speed",
        "speed",
        "acc_lambda",
        "desired_gap",
        "predecessor_gap",
        "desired_headway_time",
    ],
)


FAKE_PREDECESSOR = namedtuple(
    "FAKE_PREDECESSOR",
    [
        "vid",
        "position",
        "speed",
        "length",
    ]
)


def safe_speed(
    speed_predecessor,
    speed_current,
    gap_to_predecessor,
    desired_headway_time,
    max_deceleration,
    desired_gap,
):
    data = SAFE_SPEED_DF(
        predecessor_speed=speed_predecessor,
        speed=speed_current,
        max_deceleration=max_deceleration,
        predecessor_gap=gap_to_predecessor,
        desired_gap=desired_gap,
        desired_headway_time=desired_headway_time,
    )
    return safe_speed_df(data)


def single_vehicle_new_speed(vehicle, predecessor, step_length):
    """
    Compute the new speed of a single vehicle object (DEPRECATED).

    Legacy function to use until furhter vectorization is done.
    The predecessor may be None.
    """
    vtype = vehicle._vehicle_type
    if predecessor is None:
        predecessor = FAKE_PREDECESSOR(
            vid=-1,
            position=1e15,
            speed=1e15,
            length=vtype.length,
        )
    gap_to_predecessor = (
        predecessor.position - predecessor.length - vehicle._position
    )
    desired_gap = max(
        vtype.min_gap, vehicle.desired_headway_time * vehicle._speed
    )
    if vehicle._cf_model == CF_Model.CC:
        new_speed = safe_speed(
            speed_predecessor=predecessor.speed,
            speed_current=vehicle._speed,
            gap_to_predecessor=gap_to_predecessor,
            desired_gap=desired_gap,
            max_deceleration=vtype.max_deceleration,
            desired_headway_time=vehicle.desired_headway_time,
        )
    elif vehicle._cf_model == CF_Model.ACC:
        new_speed = speed_acc_df(
            vdf=ACC_SPEED_DF(
                predecessor_speed=predecessor.speed,
                speed=vehicle._speed,
                acc_lambda=vehicle._acc_lambda,
                desired_gap=desired_gap,
                predecessor_gap=gap_to_predecessor,
                desired_headway_time=vehicle.desired_headway_time,
            ),
            step_length=step_length,
        )
    elif vehicle._cf_model == CF_Model.CACC:
        new_speed = vehicle._platoon._formation[0]._speed
    else:
        raise NotImplementedError(f"Unknown CF Model: {vehicle._cf_model}")

    new_speed = min(
        new_speed,
        vehicle.max_speed,
        vehicle._speed + vtype.max_acceleration * step_length,
    )
    new_speed = max(
        new_speed,
        vehicle._speed - vtype.max_deceleration * step_length,
        0,  # do not drive backwards
    )
    return new_speed
