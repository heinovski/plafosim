#!/bin/bash

set -e

echo "tool,real,user,sys" > runtimes_human.csv

/usr/bin/time --format="plafosim,%e,%U,%S" --output=runtimes_human.csv --append \
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
    --result-base-filename human \
    2>&1 | tee runlog_plafosim

# also change routes file
/usr/bin/time --format="sumo,%e,%U,%S" --output=runtimes_human.csv --append \
    $SUMO_HOME/bin/sumo \
    -c sumocfg/freeway-human.sumo.cfg \
    --fcd-output human-traces.xml \
    --device.fcd.deterministic \
    --tripinfo-output human-trips.xml \
    --emission-output human-emissions.xml \
    --device.emissions.deterministic \
    --lanechange-output human-changes.xml \
    --step-length 1 \
    2>&1 | tee runlog_sumo

$SUMO_HOME/tools/xml/xml2csv.py human-trips.xml -o human-trips.csv -s ','
$SUMO_HOME/tools/xml/xml2csv.py human-emissions.xml -o human-emissions.csv -s ','
$SUMO_HOME/tools/xml/xml2csv.py human-traces.xml -o human-traces.csv -s ','
$SUMO_HOME/tools/xml/xml2csv.py human-changes.xml -o human-changes.csv -s ','

./scripts/compare2sumo.py
