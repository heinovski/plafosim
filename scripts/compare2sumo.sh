#!/bin/bash

set -e

# also change freeway-static.rou.xml
echo "tool,real,user,sys" > runtimes.csv

/usr/bin/time --format="plafosim,%e,%U,%S" --output=runtimes.csv --append \
    ./plafosim.py \
    --lanes 4 \
    --collision true \
    --lane-changes true \
    --vehicles 100 \
    --time-limit 100 \
    --road-length 100 \
    --max-speed 55 \
    --penetration 0 \
    --desired-speed 36 \
    --random-desired-speed true \
    --speed-variation 0.1 \
    --min-desired-speed 22 \
    --max-desired-speed 50 \
    --depart-method interval \
    --depart-time-interval 3 \
    --step 1 \
    2>&1 | tee runlog_plafosim

/usr/bin/time --format="sumo,%e,%U,%S" --output=runtimes.csv --append \
    sumo \
    -c cfg/freeway-static.sumo.cfg \
    --fcd-output static-traces.xml \
    --device.fcd.deterministic \
    --tripinfo-output static-trips.xml \
    --emission-output static-emissions.xml \
    --device.emissions.deterministic \
    --lanechange-output static-changes.xml \
    --step-length 1 \
    2>&1 | tee runlog_sumo

~/src/software/sumo/tools/xml/xml2csv.py static-trips.xml -o static-trips.csv -s ','
~/src/software/sumo/tools/xml/xml2csv.py static-emissions.xml -o static-emissions.csv -s ','
~/src/software/sumo/tools/xml/xml2csv.py static-traces.xml -o static-traces.csv -s ','
~/src/software/sumo/tools/xml/xml2csv.py static-changes.xml -o static-changes.csv -s ','

./scripts/compare2sumo.py
