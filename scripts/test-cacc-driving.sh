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
    --time-limit 100 \
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
    ---step-length 1 \
    --random-seed $(test -z "$seed" && echo -1 || echo $seed) \
    --result-base-filename $experiment \
    2>&1 | tee run_${experiment}_plafosim.log
