#!/bin/zsh
# Вариант детектора -> строка таблицы (раздел 5.4 спеки): бэг организаторов, 5 пустых записей +
# doubleT_obstacle, вся new_data, подъезд (eval_approach), саморазметка new_data.
#   research/compare.sh <tag> '<json kwargs>'
#   FAST=1        -- только бэг организаторов (для сетки вариантов зоны, ~несколько минут)
#   ND_SEGMENTS=8 -- new_data отрезками по 400 кадров вместо всей записи (быстрее, для прикидки)
cd "$(dirname $0)/../tools"
TAG=$1; DET=${2:-'{}'}; PY=../venv/bin/python
B=(doubleT_obstacle doubleT_platform roundT_doubleT roundT_pressureGate_roundT roundT_squareT_pressureGate_squareT squareT_platform_squareT_switch)
$PY alarms.py --tag fo_$TAG --det "$DET" --bags cloud_with_fake_obj --workers 1 > ../runs/cmp_${TAG}_fo.log 2>&1 &
if [ -z "$FAST" ]; then
  ND=(); [ -n "$ND_SEGMENTS" ] && ND=(--segments $ND_SEGMENTS --seglen 400)
  $PY alarms.py --tag $TAG --det "$DET" --bags $B --workers 2 > ../runs/cmp_${TAG}_alarms.log 2>&1 &
  $PY alarms.py --tag nd_$TAG --det "$DET" --bags new_data $ND --workers 1 > ../runs/cmp_${TAG}_nd.log 2>&1 &
  $PY eval_approach.py --tag $TAG --det "$DET" --workers 2 > ../runs/cmp_${TAG}_approach.log 2>&1 &
fi
wait
$PY eval_fake_obj.py eval --tag fo_$TAG > ../runs/cmp_${TAG}_fo_eval.log 2>&1
[ -z "$FAST" ] && [ -f selflabel.py ] && $PY selflabel.py --tag nd_$TAG > ../runs/cmp_${TAG}_self.log 2>&1
echo "== $TAG $DET"
cat ../runs/cmp_${TAG}_fo_eval.log
[ -z "$FAST" ] && grep -E "^ПУСТЫЕ|^new_data|^doubleT_obstacle" ../runs/cmp_${TAG}_alarms.log ../runs/cmp_${TAG}_nd.log
[ -z "$FAST" ] && sed -n '/испытаний/,$p' ../runs/cmp_${TAG}_approach.log
[ -z "$FAST" ] && [ -f ../runs/cmp_${TAG}_self.log ] && tail -5 ../runs/cmp_${TAG}_self.log
$PY ../research/compare_row.py $TAG "$DET"
