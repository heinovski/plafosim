#!/bin/bash
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

ROOT=$(pwd)/$(dirname $0)/..

# NOTE: make sure to have pytest-cov installed as well
pytest -v --cov=plafosim --cov-report term-missing:skip-covered ${ROOT}/tests/unit_tests ${ROOT}/tests/integration
