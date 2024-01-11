#
# Copyright (c) 2020-2024 Julian Heinovski <heinovski@ccs-labs.org>
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

from enum import Enum

# emission factors of the HBEFA3 model
# see https://sumo.dlr.de/docs/Models/Emissions/HBEFA3-based.html
# CO - the total carbon monoxide (CO) emission in mg
# CO2 - the total carbon dioxide (CO2) emission in mg
# HC - the total hydro carbon (HC) emission in mg
# NOx - the total nitrogen oxides (NO and NO2) emission in mg
# PMx - the total fine-particle (PMx) emission in mg
# fuel - the total fuel consumption emission in ml
EMISSION_FACTORS = {
    "PC_G_EU4": {
        "CO": [593.2, 19.32, 0.0, -73.25, 2.086, 0.0],
        "CO2": [9449, 938.4, 0.0, -467.1, 28.26, 0.0],
        "HC": [2.923, 0.1113, 0.0, -0.3476, 0.01032, 0.0],
        "NOx": [4.336, 0.4428, 0.0, -0.3204, 0.01371, 0.0],
        "PMx": [0.2375, 0.0245, 0.0, -0.03251, 0.001325, 0.0],
        "fuel": [3014, 299.3, 0.0, -149, 9.014, 0.0],
    }
}


class EmissionClass(Enum):
    """
    Emission class for combustion engines using the HBEFA3 model.

    Website: https://www.hbefa.net/
    """

    PC_G_EU4 = 0

    @property
    def emission_factors(self) -> dict:
        """
        Return the emission factors of the emission class.

        Returns
        -------
        dict : The emission factors of the emission class
        """

        return EMISSION_FACTORS[self.name]

    @property
    def is_diesel(self) -> bool:
        """
        Return whether the emission class is for a diesel engine.

        Returns
        -------
        bool : Whether the emission class is for a diesel engine
        """

        split = self.name.split("_")
        assert len(split) == 3
        return split[1] == "D"
