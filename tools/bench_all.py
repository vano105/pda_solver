#!/usr/bin/env python3
"""Прогон решателя на задачах научного руководителя и сводная таблица.

    python3 tools/bench_all.py --data путь/к/json --reference путь/к/bench_results.json

Скрипт конвертирует JSON-задачи в XML нужными окнами, запускает оба метода, собирает
размеры задачи, время и значение критерия и сверяет результат с эталоном.
Готовые XML сохраняются, поэтому повторный запуск не конвертирует заново.
"""

import argparse
import json
import os
import re
import subprocess
import sys
import time

# Что считаем: (файл JSON, число интервалов). None — весь горизонт.
WINDOWS = [
    ("pc10", 3), ("pc10", 8), ("pc10", 15), ("pc10", 30), ("pc10", 60),
    ("pc20", 10), ("pc30", 10),
]

# Готовые XML из examples/ прогоняются как есть.
PLAIN_TASKS = ["mini_task", "medium_task", "request"]

SOLVER = "./pda_solver"
CONVERTER = "tools/json_to_xml.py"


def run_solver(path, method, timeout):
    """Запуск решателя. Возвращает словарь с результатами либо отметку о таймауте."""
    command = [SOLVER, path, "--method", method, "--quiet", "--skip-validation",
               "-o", os.path.join("/tmp", "plan.bpmn")]
    environment = dict(os.environ, PDA_SIZES="1")

    start = time.perf_counter()
    try:
        done = subprocess.run(command, capture_output=True, text=True,
                              timeout=timeout, env=environment)
    except subprocess.TimeoutExpired:
        return {"status": "таймаут", "seconds": float(timeout)}
    elapsed = time.perf_counter() - start

    out = done.stdout + done.stderr
    result = {"status": "ок", "seconds": elapsed}

    match = re.search(r"критерия:\s*(-?[\d.]+)", out)
    result["objective"] = float(match.group(1)) if match else None
    result["checked"] = "выполнены" in out
    # Решатель мог вернуть допустимый план, не доказав его оптимальность.
    result["proven"] = "оптимальность не доказана" not in out

    match = re.search(r"переменных (\d+), ограничений (\d+)", out)
    if match:
        result["vars"] = int(match.group(1))
        result["cons"] = int(match.group(2))

    match = re.search(r"итераций: (\d+)", out)
    if match:
        result["iterations"] = int(match.group(1))

    match = re.search(r"главная задача: строк (\d+)", out)
    if match:
        result["master_rows"] = int(match.group(1))

    if result["objective"] is None or "не получено" in out:
        # Решатель отработал, но решения не нашёл: это не то же самое, что найденный
        # план, не прошедший проверку.
        result["status"] = "не решено"
        result["objective"] = None
    return result


def convert(data_dir, xml_dir, name, intervals):
    """Конвертация окна задачи; готовый файл повторно не создаётся."""
    source = os.path.join(data_dir, f"request_{name}SC.json")
    if not os.path.exists(source):
        return None

    suffix = str(intervals) if intervals else "full"
    target = os.path.join(xml_dir, f"{name}-{suffix}.xml")
    if os.path.exists(target):
        return target

    command = [sys.executable, CONVERTER, source, "-o", target]
    if intervals:
        command += ["--max-intervals", str(intervals)]
    subprocess.run(command, capture_output=True, text=True, check=True)
    return target


def load_reference(path):
    """Эталонные значения критерия из результатов научного руководителя."""
    if not path or not os.path.exists(path):
        return {}
    with open(path, encoding="utf-8") as handle:
        records = json.load(handle)

    # У научного руководителя мелкие задачи названы короче, чем файлы в examples/.
    aliases = {"mini": "mini_task", "medium": "medium_task"}

    reference = {}
    for record in records:
        if record.get("status") != "ok" or record.get("objective") is None:
            continue
        key = aliases.get(record["id"], record["id"])
        # Для каждой задачи берём значение метода MIP, он считается эталонным.
        if record.get("method") == "MIP" or key not in reference:
            reference[key] = record["objective"]
    return reference


def verdict(objective, expected):
    if objective is None or expected is None:
        return "—"
    tolerance = 1e-6 * max(1.0, abs(expected))
    return "совпадает" if abs(objective - expected) <= tolerance else "РАСХОЖДЕНИЕ"


def main():
    parser = argparse.ArgumentParser(description="Сводный прогон на данных руководителя")
    parser.add_argument("--data", default=".", help="каталог с request_pcXXSC.json")
    parser.add_argument("--xml-dir", default="/tmp/pda_xml", help="куда класть XML")
    parser.add_argument("--reference", default=None, help="bench_results.json для сверки")
    parser.add_argument("--timeout", type=float, default=120.0, help="лимит на один запуск, с")
    parser.add_argument("--methods", default="simplex,dwi", help="какие методы гонять: simplex, dw (блоки-объекты), dwi (блоки-интервалы)")
    parser.add_argument("--out", default=None, help="сохранить результаты в JSON")
    args = parser.parse_args()

    if not os.path.exists(SOLVER):
        sys.exit("Решатель не собран: выполните sh build.sh")

    os.makedirs(args.xml_dir, exist_ok=True)
    reference = load_reference(args.reference)
    if args.reference and not reference:
        print(f"ВНИМАНИЕ: эталоны не загружены из {args.reference} — "
              f"файл не найден либо пуст, колонка сверки будет пустой\n")
    methods = [m.strip() for m in args.methods.split(",") if m.strip()]

    tasks = []
    for name in PLAIN_TASKS:
        path = os.path.join("examples", f"{name}.xml")
        if os.path.exists(path):
            tasks.append((name, path))
    for name, intervals in WINDOWS:
        path = convert(args.data, args.xml_dir, name, intervals)
        if path:
            tasks.append((f"{name}-{intervals}", path))

    if not tasks:
        sys.exit("Задач не найдено: проверьте --data и каталог examples/")

    header = f"{'задача':<14}{'перем.':>8}{'огр.':>8}  {'метод':<9}{'критерий':>14}" \
             f"{'время,с':>10}{'итер.':>7}  итог"
    print(header)
    print("-" * len(header))

    collected = []
    for task_id, path in tasks:
        expected = reference.get(task_id)
        size = ""

        for method in methods:
            result = run_solver(path, method, args.timeout)
            result.update({"id": task_id, "method": method, "expected": expected})
            collected.append(result)

            if method == methods[0] and "vars" in result:
                size = (f"{result['vars']:>8}{result['cons']:>8}")

            status = result["status"]
            if status == "ок":
                mark = verdict(result.get("objective"), expected)
                if not result.get("proven", True):
                    mark += ", оптимальность не доказана"
                if not result.get("checked"):
                    mark = "ПЛАН НЕ ПРОШЁЛ ПРОВЕРКУ"
                value = f"{result['objective']:.1f}"
            else:
                mark = status.upper()
                value = "—"

            tail = mark
            if expected is not None and method == methods[0]:
                tail += f"   эталон {expected:.1f}"
            print(f"{task_id if method == methods[0] else '':<14}"
                  f"{size if method == methods[0] else '':<16}  "
                  f"{method:<9}{value:>14}{result['seconds']:>10.2f}"
                  f"{result.get('iterations', ''):>7}  {tail}")
            size = ""
        print()

    if args.out:
        with open(args.out, "w", encoding="utf-8") as handle:
            json.dump(collected, handle, ensure_ascii=False, indent=2)
        print(f"Результаты сохранены в {args.out}")


if __name__ == "__main__":
    main()