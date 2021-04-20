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

seed="$1"
ROOT=$(pwd)/$(dirname $0)/..

### ACC

experiment=acc

echo "simulator,real,user,sys" > runtimes_$experiment.csv

/usr/bin/time --format="plafosim,%e,%U,%S" --output=runtimes_$experiment.csv --append \
    $ROOT/plafosim.py \
    --lanes 4 \
    --collisions true \
    --lane-changes true \
    --vehicles 100 \
    --time-limit 1.1 \
    --road-length 100 \
    --max-speed 55 \
    --acc-headway-time 1.0 \
    --penetration 1 \
    --desired-speed 36 \
    --random-desired-speed true \
    --speed-variation 0.1 \
    --min-desired-speed 22 \
    --max-desired-speed 50 \
    --depart-flow false \
    --depart-method interval \
    --depart-time-interval 3 \
    --step-length 1 \
    --random-seed $(test -z "$seed" && echo -1 || echo $seed) \
    --result-base-filename $experiment \
    --record-end-trace false \
    --record-vehicle-trips true \
    --record-vehicle-emissions true \
    --record-vehicle-traces true \
    --record-vehicle-changes true \
    2>&1 | tee run_${experiment}_plafosim.log

# also change routes file
/usr/bin/time --format="sumo,%e,%U,%S" --output=runtimes_$experiment.csv --append \
    $SUMO_HOME/bin/sumo \
    -c $ROOT/sumocfg/freeway-$experiment.sumo.cfg \
    --fcd-output $experiment-traces.xml \
    --device.fcd.deterministic \
    --tripinfo-output $experiment-trips.xml \
    --emission-output $experiment-emissions.xml \
    --device.emissions.deterministic \
    --lanechange-output $experiment-changes.xml \
    --step-length 1 \
    $(test -z "$seed" && echo --random || echo --seed $seed) \
    2>&1 | tee run_${experiment}_sumo.log

$SUMO_HOME/tools/xml/xml2csv.py $experiment-trips.xml -o $experiment-trips.csv -s ','
$SUMO_HOME/tools/xml/xml2csv.py $experiment-emissions.xml -o $experiment-emissions.csv -s ','
$SUMO_HOME/tools/xml/xml2csv.py $experiment-traces.xml -o $experiment-traces.csv -s ','
$SUMO_HOME/tools/xml/xml2csv.py $experiment-changes.xml -o $experiment-changes.csv -s ','

$ROOT/scripts/compare2sumo.py $experiment --vehicles 100 --desired-speed 36 --arrival-position 100000
