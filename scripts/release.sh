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

set -e

VERSION=$(git describe | sed -E "s/^v(.*)$/\1/")
DIRNAME=.release

if [ "$(git branch --show-current)" != "master" ]
then
    echo "You are not on the master branch!"
    exit 1
fi

if [ -n "$(git status --untracked-files=no --porcelain)" ];
then
  echo "There are uncommitted changes in tracked files!"
  exit 1
fi

# create virtualenv
virtualenv $DIRNAME
# activate virtualenv
source $DIRNAME/bin/activate

# publish to testpypi
if [ -z "$(curl -v --silent https://test.pypi.org/simple/plafosim/ --stderr - | grep $VERSION)" ]
then
    echo "Publishing to TestPyPI..."
    poetry publish --build -r testpypi
    # wait for release on testpypi
    sleep 10s
else
    echo "TestPyPI already has version $VERSION."
fi

# install dependencies (HACK due to ortools)
echo "Installing dependencies (hack)..."
pip install plafosim
pip uninstall plafosim -y

# install from testpypi
echo "Installing from TestPyPI..."
pip install plafosim -i https://test.pypi.org/simple/
# check version
if [ "$(plafosim -V)" != "plafosim $VERSION" ]
then
    echo "Version mismatch from TestPyPI!"
    exit 1
fi
pip uninstall plafosim -y

# publish to pypi
if [ -z "$(curl -v --silent https://pypi.org/simple/plafosim/ --stderr - | grep $VERSION)" ]
then
    echo "Publishing to PyPI..."
    poetry publish --build
    # wait for release on pypi
    sleep 10s
else
    echo "PyPI already has version $VERSION."
fi

echo "Installing from PyPI..."

# wait for release on pypi
sleep 20s

pip install plafosim==$VERSION
# check version
if [ "$(plafosim -V)" != "plafosim $VERSION" ]
then
    echo "Version mismatch from PyPI!"
    exit 1
fi
pip uninstall plafosim -y
# clean up virtualenv
deactivate
rm -r $DIRNAME
