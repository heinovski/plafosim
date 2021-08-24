#!/bin/bash
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

set -e
set -o pipefail

ROOT=$(pwd)/$(dirname $0)/..

### CACC

experiment=cacc

# check correct sumo version
sumo --version | grep Version | head -n 1 | sed -E "s/.*([0-9]+\.[0-9]+\.[0-9]+)/\1/g" | xargs -i test "1.6.0" = "{}" || (echo "Incorrect SUMO version (expecting 1.6.0)!" && exit 1)

echo "simulator,real,user,sys" > ${experiment}_runtimes.csv

echo "Running PlaFoSim..."

/usr/bin/time --format="plafosim,%e,%U,%S" --output=${experiment}_runtimes.csv --append \
    $ROOT/plafosim.py \
    --acc-headway-time 1.5 \
    --cacc-spacing 5.0 \
    --collisions true \
    --depart-desired true \
    --depart-flow false \
    --depart-interval 3 \
    --depart-method interval \
    --desired-speed 36 \
    --lane-changes true \
    --lanes 4 \
    --max-speed 55 \
    --penetration 1 \
    --pre-fill true \
    --random-desired-speed false \
    --random-seed 1337 \
    --record-emission-traces true \
    --record-end-trace false \
    --record-prefilled true \
    --record-vehicle-changes true \
    --record-vehicle-emissions true \
    --record-vehicle-traces true \
    --record-vehicle-trips true \
    --reduced-air-drag false \
    --result-base-filename $experiment \
    --road-length 100 \
    --start-as-platoon true \
    --step-length 1 \
    --time-limit 1 \
    --update-desired-speed false \
    --vehicles 100 \
    2>&1 | tee ${experiment}_plafosim.log

echo "Running SUMO..."

/usr/bin/time --format="sumo,%e,%U,%S" --output=${experiment}_runtimes.csv --append \
    $ROOT/scripts/run-plexe-cacc.py \
    --experiment $experiment \
    --sumo-config sumocfg/freeway-cacc.sumo.cfg \
    --vehicles 100 \
    2>&1 | tee ${experiment}_sumo.log

echo "Converting results..."

$SUMO_HOME/tools/xml/xml2csv.py $experiment-trips.xml -o $experiment-trips.csv -s ','
$SUMO_HOME/tools/xml/xml2csv.py $experiment-emissions.xml -o $experiment-emissions.csv -s ','
$SUMO_HOME/tools/xml/xml2csv.py $experiment-traces.xml -o $experiment-traces.csv -s ','
$SUMO_HOME/tools/xml/xml2csv.py $experiment-changes.xml -o $experiment-changes.csv -s ','

echo "Comparing results..."

$ROOT/scripts/compare2sumo.py $experiment --vehicles 100 --desired-speed 36 --arrival-position 100000
