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

from collections import namedtuple
from enum import Enum

import numpy as np
import pandas as pd

from .util import assert_index_equal

# misc constants
HIGHVAL = 1e15  # virtually infinite position or speed
HIGHRESULT = 1e13  # still virtually high value to compare against
DUMMY_PREDECESSOR = pd.Series(
    {"speed": HIGHVAL, "position": HIGHVAL, "length": 0, "rear_position": HIGHVAL}, name=-1
)
DUMMY_SUCCESSOR = pd.Series(
    {"speed": 0, "position": -HIGHVAL, "desired_headway_time": 0}, name=-1
)


class CF_Model(Enum):
    """
    Car Following models that vehicles can use for their mobility.
    """

    Human = 0  # safe speed
    ACC = 1  # fixed time gap
    CACC = 2  # small fixed distance


# gap safety checks


def is_gap_safe(
    front_position,
    front_speed,
    front_max_deceleration,
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
        + (front_speed - front_max_deceleration * step_length) * step_length
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

    See S. Krauss, Microscopic Modeling of Traffic Flow: Investigation of Collision Free Vehicle Dynamics, 1998.
    """

    tau_b = (vdf.predecessor_speed + vdf.speed) / (2 * vdf.max_deceleration)
    return vdf.predecessor_speed + (
        (vdf.predecessor_gap - vdf.desired_gap)
        / (vdf.desired_headway_time + tau_b)
    )


def speed_human_df(vdf: pd.DataFrame) -> pd.Series:
    """
    Compute new speed for human vehicles, DataFrame variant.

    Basically just safe_speed, clamping is done outside this function.
    """

    # TODO: implement dawdling
    speed_safe = safe_speed_df(vdf)
    return speed_safe


def speed_acc_df(vdf: pd.DataFrame, step_length: float = 1.0) -> pd.Series:
    """
    Compute new speed for ACC vehicles, DataFrame variant.

    Clamping is done outside this function.

    See Eq. 6.18 of R. Rajamani, Vehicle Dynamics and Control, 2nd. Springer, 2012.
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


def lane_predecessors(vdf, max_lane):
    """
    Find the current (potential) predecessor for each lane and each vehicle.

    This means: Which other vehicle would be the predecessor if a vehicle was
    on lane `column`.
    A predecessor id of -1 means there is no predecessor.

    Preconditions:
    - vdf.sorted_values(['position', 'lane'], ascending=False)
    """
    vdf = vdf.reset_index()
    return pd.DataFrame(
        {
            lane_nr: vdf.vid.where(vdf.lane == lane_nr)
            .ffill()
            .shift()
            .fillna(-1)
            .astype(int)
            for lane_nr in range(max_lane + 1)
        },
    ).set_index(vdf.vid)


def lane_successors(vdf, max_lane):
    """
    Find the current (potential) successors for each lane and each vehicle.

    This means: Which other vehicle would be the successor if a vehicle was
    on lane `column`.
    A successor id of -1 means there is no successor.

    Preconditions:
    - vdf.sorted_values(['position', 'lane'], ascending=False)
    """
    vdf = vdf.reset_index()
    return pd.DataFrame(
        {
            lane_nr: vdf.vid.where(vdf.lane == lane_nr)
            .bfill()
            .shift(-1)
            .fillna(-1)
            .astype(int)
            for lane_nr in range(max_lane + 1)
        },
    ).set_index(vdf.vid)


def get_successors(vdf, successor_map, target_lane) -> pd.DataFrame:
    """
    Return DataFrame of successors to the vehicles on a target lane.
    """
    # get id of (the one) successor on the target lane
    successor_ids = pd.Series(
        # pick one value per vehicle using numpy, which is fast and stable
        successor_map.values[np.arange(len(vdf)), target_lane],
        index=vdf.index,
    )
    # build data frame of successors, indexed by their predecessors
    # include a virtual predecessor with id -1
    return (
        vdf[["speed", "position", "desired_headway_time"]]
        .append(DUMMY_SUCCESSOR)
        .loc[successor_ids]
        .reset_index()
        .set_index(vdf.index)
    )


def get_predecessors(vdf, predecessor_map, target_lane) -> pd.DataFrame:
    """
    Return DataFrame of successors to the vehicles on a target lane.
    """
    # get id of (the one) successor on the target lane
    predecessor_ids = pd.Series(
        # pick one value per vehicle using numpy, which is fast and stable
        predecessor_map.values[np.arange(len(vdf)), target_lane],
        index=vdf.index,
    )
    # build data frame of successors, indexed by their predecessors
    # include a virtual predecessor with id -1
    return (
        vdf[["speed", "position", "length", "rear_position"]]
        .append(DUMMY_PREDECESSOR)
        .loc[predecessor_ids]
        .reset_index()
        .set_index(vdf.index)
    )


def compute_new_speeds(
    vdf: pd.DataFrame,
    step_length: float = 1.0,
) -> pd.Series:
    """
    Compute the new speed for all vehicles in the simulation.

    Can compute "potential" new speed for different target lanes.
    Assume vdf already contains predecessor and successor data.
    Just pass the right predecessor/successor data for different lanes.
    """

    # ensure no changes to vdf propagate out of this function
    vdf = vdf.copy()
    # derive common data for all cf models
    vdf["desired_gap"] = np.max(
        # FIXME: ensure this is also valid as the desired_gap for ACC
        # TODO: move desired_gap out of this function to avoid re-computation
        [
            vdf["min_gap"],
            # using own speed instead of predecessor speed (like krauss) here
            vdf["desired_headway_time"] * vdf["speed"],
        ],
        axis=0,
    )
    vdf["predecessor_gap"] = vdf["predecessor_rear_position"] - vdf["position"]

    # Note:
    # we assume vehicles already have their max acceleration/deceleration
    # set/updated for individual driving or platooning.
    # I.e., platoons share max acceleration/deceleration.
    # We also assume that evary vehicle has speed and postion values of its
    # predecessor. If there is none, the values should just be large defaults.
    # The CF functions shouls be able to deal with this inherently.

    m_human = vdf.cf_model == CF_Model.Human
    m_acc = vdf.cf_model == CF_Model.ACC
    m_cacc = vdf.cf_model == CF_Model.CACC

    # apply models
    # Start with a copy of the old speed to get the size and index right.
    # This should be faster than building and concatenating multiple series
    # and should avoid re-sorting the result -- but maybe benchmark this later
    new_speed = vdf.speed.copy()
    new_speed.loc[m_human] = speed_human_df(vdf.loc[m_human])
    new_speed.loc[m_acc] = speed_acc_df(vdf.loc[m_acc])
    # TODO: apply clamping only to human and acc vehicles (not cacc)
    new_speed.loc[m_cacc] = new_speed[vdf.loc[m_cacc].leader_id].values

    # clamp speed by common constraints
    new_speed = clamp_speed(new_speed, vdf, step_length)

    assert not any(np.isnan(new_speed))
    assert not any(new_speed > HIGHRESULT)
    assert not any(new_speed < 0)

    return pd.Series(new_speed, index=vdf.index)


def compute_lane_changes(vdf, max_lane, step_length=1.0):
    """
    Find desired and safe lane changes.

    1) compute all speed gains
    2) "apply" all speed gains
    3) compute all keep rights
    4) "apply" all keep rights

    This is based on Krauss' multi lane traffic:
    laneChange()
    congested = (v_safe < v_thresh) and (v^0_safe < v_thresh)
    favorable(right->left) = (v_safe < v_max) and (not congested)
    favorable(left->right) = (v_safe >= v_max) and (v^0_safe >= v_max)
    if ((favorable(i->j) or (rand < p_change)) and safe(i->j)) then change(i->j)
    for vehicles on the right lane:
    if (v > v^0_safe) and (not congested) then v <- v^0_safe
    """
    # TODO: check order of computation for performance
    # TODO: skip computations for CACC vehicles

    vdf_tmp = vdf.copy()
    m_cacc = vdf_tmp.cf_model == CF_Model.CACC

    ## SPEED GAIN

    # derive map of predecessrs and successors
    predecessor_map = lane_predecessors(vdf_tmp, max_lane)
    successor_map = lane_successors(vdf_tmp, max_lane)

    # derive lanes
    left_lane = (vdf_tmp.lane + 1).clip(upper=max_lane)

    # extract predecessor and successor data for potential maneuvers
    predecessor_left = get_predecessors(
        vdf=vdf_tmp,
        predecessor_map=predecessor_map,
        target_lane=left_lane,
    ).rename(columns=lambda col: "predecessor_" + col)
    predecessor_current = get_predecessors(
        vdf=vdf_tmp,
        predecessor_map=predecessor_map,
        target_lane=vdf_tmp.lane,
    ).rename(columns=lambda col: "predecessor_" + col)

    # predict speed for potential maneuvers
    speed_left = compute_new_speeds(
        vdf=vdf_tmp.join(predecessor_left, how='inner'),
        step_length=step_length,
    )
    speed_current = compute_new_speeds(
        vdf=vdf_tmp.join(predecessor_current, how='inner'),
        step_length=step_length,
    )

    # compute forwards gap in other lanes
    front_gap_left = (
        vdf_tmp["position"] + vdf_tmp["desired_headway_time"] * vdf_tmp["speed"]
        < predecessor_left["predecessor_rear_position"]
    )

    # compute backwards gaps in other lanes
    successor_left = get_successors(
        vdf=vdf_tmp, successor_map=successor_map, target_lane=left_lane
    )
    back_gap_left = (
        successor_left["position"]
        + successor_left["desired_headway_time"] * successor_left["speed"]
        < vdf_tmp["rear_position"]
    )

    # decide on maneuvers (speedGain first!)
    speed_gain = (
        (vdf_tmp.lane < max_lane)
        & (speed_left > speed_current)
        & back_gap_left
        & front_gap_left
        & ~m_cacc
    )

    # derive resulting lanes
    # apply speed gain maneuver
    vdf_tmp.loc[speed_gain, 'lane'] = left_lane
    vdf_tmp.loc[speed_gain, 'reason'] = "speedGain"
    # apply leader decisions
    vdf_tmp.loc[m_cacc, 'lane'] = vdf_tmp.loc[vdf_tmp.loc[m_cacc, 'leader_id'], 'lane'].values
    vdf_tmp.loc[m_cacc, 'reason'] = vdf_tmp.loc[vdf_tmp.loc[m_cacc, 'leader_id'], 'reason'].values

    ## KEEP RIGHT

    # derive map of predecessrs and successors
    predecessor_map = lane_predecessors(vdf_tmp, max_lane)
    successor_map = lane_successors(vdf_tmp, max_lane)

    # derive lanes
    right_lane = (vdf_tmp.lane - 1).clip(lower=0)

    # extract predecessor and successor data for potential maneuvers
    predecessor_right = get_predecessors(
        vdf=vdf_tmp,
        predecessor_map=predecessor_map,
        target_lane=right_lane,
    ).rename(columns=lambda col: "predecessor_" + col)
    # NOTE: for improved performance, we take the old predecessor_current values
    # The previous speed gains do not change the decision to perform a keep right.
    # They can only influence the time (i.e., in the current step or later) the keep right change is performed

    # predict speed for potential maneuvers
    speed_right = compute_new_speeds(
        vdf=vdf_tmp.join(predecessor_right, how='inner'),
        step_length=step_length,
    )
    # NOTE: for improved performance, we take the old predecessor_current values
    # The previous speed gains do not change the decision to perform a keep right.
    # They can only influence the time (i.e., in the current step or later) the keep right change is performed

    # compute forwards gap in other lanes
    front_gap_right = (
        vdf_tmp["position"] + vdf_tmp["desired_headway_time"] * vdf_tmp["speed"]
        < predecessor_right["predecessor_rear_position"]
    )
    # compute backwards gaps in other lanes
    successor_right = get_successors(
        vdf=vdf_tmp, successor_map=successor_map, target_lane=right_lane
    )
    back_gap_right = (
        successor_right["position"]
        + successor_right["desired_headway_time"] * successor_right["speed"]
        < vdf_tmp["rear_position"]
    )

    # decide on maneuvers (speedGain first!)
    keep_right = (
        (vdf_tmp.lane > 0)
        & (speed_right >= speed_current)
        & back_gap_right
        & front_gap_right
        & ~speed_gain
        & ~m_cacc
    )

    # apply speed gain maneuver
    vdf_tmp.loc[keep_right, 'lane'] = right_lane
    vdf_tmp.loc[keep_right, 'reason'] = "keepRight"
    # apply leader decisions
    vdf_tmp.loc[m_cacc, 'lane'] = vdf_tmp.loc[vdf_tmp.loc[m_cacc, 'leader_id'], 'lane'].values
    vdf_tmp.loc[m_cacc, 'reason'] = vdf_tmp.loc[vdf_tmp.loc[m_cacc, 'leader_id'], 'reason'].values

    # vehicls are only allowed to perform one lane change
    assert not (speed_gain & keep_right).any()

    return vdf_tmp[['lane', 'reason']]


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
    ],
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


# position update components


def clip_position(position: pd.Series, vdf: pd.DataFrame) -> pd.Series:
    """
    Returns the clipped positions (i.e., by arrival position) of vehicles within a pandas DataFrame.

    Parameters
    ----------
    position : pandas.Series
        The series containing the positions to be clipped
        index: vid
        columns: [position, length, lane, ..]
    vdf : pandas.DataFrame
        The dataframe containing the vehicles as rows
        index: vid
        columns: [position, length, lane, ..]
    """

    assert_index_equal(position, vdf)

    clipped_position = pd.Series(
        np.clip(
            position.values,
            0,
            vdf.arrival_position,  # do not move further than arrival position
        ),
        index=position.index,
    )
    assert_index_equal(clipped_position, position)

    return clipped_position


def update_position(vdf: pd.DataFrame, step_length: int) -> pd.DataFrame:
    """
    Updates the position of vehicles within a pandas DataFrame.

    This is based on Krauss' single lane traffic:
    adjust position (move)
    x(t + step_size) = x(t) + v(t)*step_size

    Parameters
    ----------
    vdf : pandas.DataFrame
        The dataframe containing the vehicles as rows
        index: vid
        columns: [position, length, lane, ..]
    step_length : int
        The length of the simulated step
    """

    position = vdf.position + (vdf.speed * step_length)
    vdf["position"] = clip_position(position, vdf)
    return vdf


def get_crashed_vehicles(vdf: pd.DataFrame) -> list:
    """
    Returns the list of crashed vehicles' ids

    Parameters
    ----------
    vdf : pandas.DataFrame
        The dataframe containing the vehicles as rows
        index: vid
        columns: [position, length, lane, ..]
    """

    # sort vehicles by their position
    vdf = vdf.sort_values("position", ascending=False)

    # add rear_position
    vdf["rear_position"] = vdf.position - vdf.length

    # filter out lanes with 1 vehicle
    lane_mask = vdf.groupby("lane").size() > 1
    lanes_to_keep = lane_mask[lane_mask].index.values
    if len(lanes_to_keep) == 0:
        return []
    vdf = vdf[vdf.lane.isin(list(lanes_to_keep))]

    # TODO should we just use the precessor / successor

    # calculate vehicles with a crash in their back
    groupby = vdf.groupby("lane")
    crash_in_back = (
        # the car behind me touches my back
        groupby["position"].shift(-1)
        >= groupby["rear_position"].shift(0)
    )
    # calculate vehicles with a crash in their front
    crash_in_front = (
        # I touch the back of the car in front of me
        groupby["position"].shift(0)
        >= groupby["rear_position"].shift(1)
    )

    # calculate vehicles with a crash in the back or the front
    # TODO one check might be enough
    crash = crash_in_back | crash_in_front

    return list(sorted(crash[crash].index.values))
