#
# Copyright (c) 2020 Julian Heinovski <heinovski@ccs-labs.org>
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
#!/bin/bash

set -e

seed="$1"

### CACC

experiment=cacc

echo "simulator,real,user,sys" > runtimes_$experiment.csv

/usr/bin/time --format="plafosim,%e,%U,%S" --output=runtimes_$experiment.csv --append \
    ./plafosim.py \
    --lanes 4 \
    --collisions true \
    --lane-changes true \
    --vehicles 100 \
    --time-limit 1 \
    --road-length 100 \
    --max-speed 55 \
    --acc-headway-time 1.0 \
    --cacc-spacing 5.0 \
    --start-as-platoon true \
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
    2>&1 | tee run_${experiment}_plafosim.log
