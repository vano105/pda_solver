#!/bin/bash
# Замер двух методов на серии задач растущего размера.
#
#   sh build.sh
#   bash tools/bench.sh [число повторов]
#
# Для каждой задачи печатает значение критерия, медиану времени по повторам
# и отметку о том, прошла ли независимая проверка плана.

set -u
REPEATS=${1:-3}
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

# Задачи: объекты × интервалы. Генератор детерминирован по зерну.
CONFIGS="5:8 8:15 12:25 16:40 20:60"
SEEDS="1 2 3"

echo "повторов: $REPEATS, зёрна генератора: $SEEDS"
printf "%-14s %-8s %-14s %-10s %-9s\n" "задача" "метод" "критерий" "медиана,с" "проверка"

median() {
    sort -n | awk '{v[NR]=$1} END {print (NR%2) ? v[(NR+1)/2] : (v[NR/2]+v[NR/2+1])/2}'
}

for seed in $SEEDS; do
  for config in $CONFIGS; do
    nodes=${config%%:*}
    intervals=${config##*:}
    task="$WORK/t_${nodes}_${intervals}_s${seed}.xml"
    python3 tools/gen_task.py --nodes "$nodes" --intervals "$intervals" \
        --seed "$seed" -o "$task" > /dev/null

    for method in simplex dw; do
      times=""
      criterion=""
      checked=""
      for _ in $(seq "$REPEATS"); do
        start=$(date +%s.%N)
        out=$(./pda_solver "$task" --method "$method" --quiet --skip-validation \
              -o "$WORK/plan.bpmn" 2>&1)
        end=$(date +%s.%N)
        times="$times$(echo "$end - $start" | bc)
"
        criterion=$(echo "$out" | grep -oP 'критерия: \K[0-9.]+')
        checked=$(echo "$out" | grep -c 'выполнены')
      done
      printf "%-14s %-8s %-14s %-10s %-9s\n" \
          "${nodes}×${intervals} s$seed" "$method" "${criterion:-нет}" \
          "$(printf '%s' "$times" | median)" "${checked:-0}"
    done
  done
done