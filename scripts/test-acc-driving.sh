#!/bin/bash

set -e

seed="$1"

### ACC

experiment=acc

echo "simulator,real,user,sys" > runtimes_$experiment.csv

/usr/bin/time --format="plafosim,%e,%U,%S" --output=runtimes_$experiment.csv --append \
    ./plafosim.py \
    --lanes 4 \
    --collision true \
    --lane-changes true \
    --vehicles 100 \
    --time-limit 100 \
    --road-length 100 \
    --max-speed 55 \
    --acc-headway-time 1.0 \
    --penetration 1 \
    --desired-speed 36 \
    --random-desired-speed true \
    --speed-variation 0.1 \
    --min-desired-speed 22 \
    --max-desired-speed 50 \
    --depart-method interval \
    --depart-time-interval 3 \
    --step 1 \
    --random-seed $(test -z "$seed" && echo -1 || echo $seed) \
    --result-base-filename $experiment \
    2>&1 | tee run_${experiment}_plafosim.log

# also change routes file
/usr/bin/time --format="sumo,%e,%U,%S" --output=runtimes_$experiment.csv --append \
    $SUMO_HOME/bin/sumo \
    -c sumocfg/freeway-$experiment.sumo.cfg \
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

./scripts/compare2sumo.py --experiment $experiment --desired-speed 36 --arrival-position 100000
