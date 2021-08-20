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


def test_emission_factors():

    cl = EmissionClass.PC_G_EU4
    assert(cl.emission_factors == EMISSION_FACTORS[cl.name])


def test_is_diesel():
    cl = EmissionClass.PC_G_EU4
    assert(not cl.is_diesel)
