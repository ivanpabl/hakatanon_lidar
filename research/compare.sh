#!/bin/zsh
# Сравнение вариантов детектора: тревоги на 6 записях, на отрезках new_data и дальность при подъезде.
# research/compare.sh <tag> '<json kwargs>'
cd "$(dirname $0)/../tools"
TAG=$1; DET=$2
B=(doubleT_obstacle doubleT_platform roundT_doubleT roundT_pressureGate_roundT roundT_squareT_pressureGate_squareT squareT_platform_squareT_switch)
../venv/bin/python alarms.py --tag $TAG --det "$DET" --bags $B --workers 2 > ../runs/cmp_${TAG}_alarms.log 2>&1 &
../venv/bin/python alarms.py --tag nd_$TAG --det "$DET" --bags new_data --segments 8 --seglen 400 --workers 1 > ../runs/cmp_${TAG}_nd.log 2>&1 &
../venv/bin/python eval_approach.py --tag $TAG --det "$DET" --workers 2 > ../runs/cmp_${TAG}_approach.log 2>&1 &
wait
echo "== $TAG $DET"
grep -E "^ВСЕГО|^new_data|^doubleT_obstacle" ../runs/cmp_${TAG}_alarms.log ../runs/cmp_${TAG}_nd.log
sed -n '/испытаний/,$p' ../runs/cmp_${TAG}_approach.log
