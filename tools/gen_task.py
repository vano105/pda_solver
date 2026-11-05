#!/usr/bin/env python3
"""Генератор тестовых задач для экспериментов с размерностью.

    python3 tools/gen_task.py --nodes 12 --intervals 20 -o examples/big.xml

Схема та же, что в примерах: периферийные объекты снимают данные и сбрасывают их
на центральный объект 1, который обрабатывает поток и выдаёт результат. Окна связи
раздаются псевдослучайно с фиксированным зерном, чтобы результаты воспроизводились.
"""

import argparse
import random


def generate(nodes: int, intervals: int, seed: int, density: float) -> str:
    rng = random.Random(seed)
    lines = ['<?xml version="1.0" encoding="utf-8"?>', '<XMLDocument version="1.0">', "  <task>"]

    lines += [
        "    <flows>",
        '      <type id="1" price="1" loss="1"/>',
        '      <type id="2" price="10" loss="1"/>',
        "    </flows>",
        '    <process>',
        '      <type id="1" time="0.004">',
        '        <input><type id="1" size="1"/></input>',
        '        <output><type id="2" size="1"/></output>',
        "      </type>",
        "    </process>",
        "    <transport>",
        '      <type id="1" time="0.05">',
        '        <input><type id="1" size="1"/></input>',
        '        <output><type id="1" size="1"/></output>',
        "      </type>",
        "    </transport>",
        "    <storage>",
        '      <type id="1">',
        '        <input><type id="1"/><type id="2"/></input>',
        "      </type>",
        "    </storage>",
    ]

    clock = 0.0
    for k in range(1, intervals + 1):
        duration = rng.choice([300, 600, 900, 1200, 1500])
        lines.append(
            f'    <struct id="{k}" time="{duration}" '
            f'start_time="{clock:g}" end_time="{clock + duration:g}">'
        )
        clock += duration

        lines.append('      <elem id="1" output_2="" process_1="" storage_1="500000"/>')
        for i in range(2, nodes + 1):
            volume = rng.choice([0, 0, 2000, 4000, 8000])
            attrs = f'input_1="{volume}" ' if volume else ""
            lines.append(f'      <elem id="{i}" {attrs}storage_1="30000"/>')

        for i in range(2, nodes + 1):
            if rng.random() < density:
                lines.append(f'      <link id1="{i}" id2="1" direction="forward" transport_1=""/>')
        lines.append("    </struct>")

    lines += [
        "    <selectors>",
        '      <selector id="1" sign="1">',
        '        <resultflow object="*" flow="2" interval="*"/>',
        "      </selector>",
        '      <selector id="2" sign="-1">',
        '        <lost object="*" flow="*" interval="*"/>',
        "      </selector>",
        "    </selectors>",
        '    <criterion sign="MAX">',
        '      <selector id="1"/>',
        '      <selector id="2"/>',
        "    </criterion>",
        "  </task>",
        "</XMLDocument>",
    ]
    return "\n".join(lines) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser(description="Генератор задач планирования")
    parser.add_argument("--nodes", type=int, default=8, help="число объектов, включая центральный")
    parser.add_argument("--intervals", type=int, default=10, help="число интервалов")
    parser.add_argument("--seed", type=int, default=1, help="зерно генератора")
    parser.add_argument("--density", type=float, default=0.5, help="доля объектов со связью")
    parser.add_argument("-o", "--output", default="task.xml")
    args = parser.parse_args()

    with open(args.output, "w", encoding="utf-8") as handle:
        handle.write(generate(args.nodes, args.intervals, args.seed, args.density))
    print(f"{args.output}: объектов {args.nodes}, интервалов {args.intervals}")


if __name__ == "__main__":
    main()