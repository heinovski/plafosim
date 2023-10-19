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

import numpy as np
import pandas as pd
from pytest import fixture

from plafosim.mobility import lane_predecessors

num_vehicles = 100
num_lanes = 3
mean_speed = 30
std_speed = 0.1 * mean_speed


@fixture(params=[1, 1337, 42424242])
def rng(request):
    return np.random.RandomState(seed=request.param)


@fixture
def random_vdf(rng):
    return pd.DataFrame(
        {
            "vid": range(num_vehicles),
            "position": rng.normal(mean_speed, std_speed, num_vehicles),
            "lane": rng.randint(0, num_lanes, size=num_vehicles),
        }
    )


def test_found_predecessors_are_valid(random_vdf):
    test_vdf = random_vdf.sort_values(["position", "lane"], ascending=False)

    predecessors = lane_predecessors(test_vdf, num_lanes - 1)

    # extended vdf with dummy predecessor at the very front
    x_vdf = (
        test_vdf.append(
            pd.Series({"position": 1e15, "lane": -1, "vid": -1}),
            ignore_index=True,
        )
        .astype({"vid": int})
        .set_index("vid")
    )

    for lane_nr in range(num_lanes):
        # predecessors on each potential lanes are in front of test vehicles
        assert (
            x_vdf.position.loc[predecessors[lane_nr]].values >= test_vdf.position
        ).all()

        # each vehicle is actually on the lane that is can be a predecessor for
        real_predecessors = predecessors[lane_nr][predecessors[lane_nr] > 0]
        assert (test_vdf.set_index("vid").loc[real_predecessors].lane == lane_nr).all()
