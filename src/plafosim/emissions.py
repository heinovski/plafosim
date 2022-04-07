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

from enum import Enum

# emission factors of the HBEFA3 model
EMISSION_FACTORS = {
    'PC_G_EU4': {
        'CO': [593.2, 19.32, 0.0, -73.25, 2.086, 0.0],
        'CO2': [9449, 938.4, 0.0, -467.1, 28.26, 0.0],
        'HC': [2.923, 0.1113, 0.0, -0.3476, 0.01032, 0.0],
        'NOx': [4.336, 0.4428, 0.0, -0.3204, 0.01371, 0.0],
        'PMx': [0.2375, 0.0245, 0.0, -0.03251, 0.001325, 0.0],
        'fuel': [3014, 299.3, 0.0, -149, 9.014, 0.0],
    }
}


class EmissionClass(Enum):
    """
    Emission class for the HBEFA3 model.
    """

    PC_G_EU4 = 0

    @property
    def emission_factors(self) -> dict:
        """Returns the emission factors of an emission class."""

        return EMISSION_FACTORS[self.name]

    @property
    def is_diesel(self) -> bool:
        """Returns whether an emission class is for a diesel engine."""

        split = self.name.split('_')
        assert len(split) == 3
        return split[1] == 'D'
