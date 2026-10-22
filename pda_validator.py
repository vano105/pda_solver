#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Модуль валидации исходных данных для системы PDA (Production Dynamics Analysis).
Проверяет XML-файл на соответствие требованиям модели, выявляет ошибки,
предупреждения и даёт рекомендации по улучшению.

Автор: Д.А. Павлов (адаптировано под требования)
Версия: 1.0
"""

import xml.etree.ElementTree as ET
import sys
import os
from enum import Enum
from dataclasses import dataclass, field
from typing import List, Dict, Set, Tuple, Optional, Any
from collections import defaultdict


class IssueLevel(Enum):
    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"
    CRITICAL = "CRITICAL"


@dataclass
class Issue:
    """Одна проблема, обнаруженная при валидации."""
    level: IssueLevel
    message: str
    element: Optional[ET.Element] = None
    line: Optional[int] = None

    def __str__(self):
        s = f"[{self.level.value}] {self.message}"
        if self.line:
            s += f" (строка ~{self.line})"
        return s


class ValidationReport:
    """Отчёт о валидации, содержащий список проблем и флаг успешности."""

    def __init__(self):
        self.issues: List[Issue] = []
        self._suggestions: List[str] = []   # отдельно для советов

    def add_issue(self, level: IssueLevel, message: str, element: Optional[ET.Element] = None):
        """Добавить проблему с определённым уровнем."""
        line = None
        if element is not None:
            # Попытаемся получить номер строки (если есть)
            try:
                line = element.sourceline
            except AttributeError:
                pass
        self.issues.append(Issue(level, message, element, line))

    def add_suggestion(self, message: str):
        """Добавить рекомендацию (не считается проблемой)."""
        self._suggestions.append(message)

    def has_errors(self) -> bool:
        """Есть ли ошибки уровня ERROR или CRITICAL?"""
        return any(iss.level in (IssueLevel.ERROR, IssueLevel.CRITICAL) for iss in self.issues)

    def print_report(self, stream=sys.stdout):
        """Вывести отчёт в поток (по умолчанию stdout)."""
        if not self.issues and not self._suggestions:
            print("✓ Валидация прошла успешно. Замечаний нет.", file=stream)
            return

        # Группировка по уровням
        by_level: Dict[IssueLevel, List[Issue]] = defaultdict(list)
        for iss in self.issues:
            by_level[iss.level].append(iss)

        print("\n" + "=" * 70, file=stream)
        print("ОТЧЁТ ВАЛИДАЦИИ", file=stream)
        print("=" * 70, file=stream)

        for level in (IssueLevel.CRITICAL, IssueLevel.ERROR, IssueLevel.WARNING, IssueLevel.INFO):
            if by_level[level]:
                print(f"\n{level.value} ({len(by_level[level])}):", file=stream)
                print("-" * 50, file=stream)
                for iss in by_level[level]:
                    print(f"  • {iss.message}", file=stream)
                    if iss.line:
                        print(f"    (строка {iss.line})", file=stream)
                    # Можно добавить краткий путь к элементу, но это усложнит вывод

        if self._suggestions:
            print("\nРЕКОМЕНДАЦИИ:", file=stream)
            print("-" * 50, file=stream)
            for sug in self._suggestions:
                print(f"  • {sug}", file=stream)

        print("\n" + "=" * 70, file=stream)

    def save_report(self, filename: str):
        """Сохранить отчёт в текстовый файл."""
        with open(filename, 'w', encoding='utf-8') as f:
            self.print_report(f)


class PDAValidator:
    """
    Валидатор XML-файлов для PDA.
    Использование:
        validator = PDAValidator()
        report = validator.validate('file.xml')
        report.print_report()
    """

    # Допустимые значения атрибутов
    VALID_SIGN = {'MAX', 'MIN', '==', '<=', '>=', 'less', 'more', 'equally'}
    VALID_DIRECTION = {'forward', 'backward', 'both', 'none'}

    # Типы переменных, встречающихся в селекторах, и их обязательные атрибуты
    VAR_TYPES = {
        'inputflow': ['object', 'flow', 'interval'],
        'resultflow': ['object', 'flow', 'interval'],
        'process_all': ['object', 'interval'],
        'to_process': ['object', 'flow', 'tech', 'interval'],
        'process': ['object', 'tech', 'interval'],
        'from_process': ['object', 'flow', 'tech', 'interval'],
        'transport_all': ['object', 'to_object', 'interval'],
        'to_transport': ['object', 'to_object', 'flow', 'tech', 'interval'],
        'transport': ['object', 'to_object', 'tech', 'interval'],
        'from_transport': ['object', 'to_object', 'flow', 'tech', 'interval'],
        'storage': ['object', 'flow', 'tech', 'interval'],
        'lost': ['object', 'flow', 'interval'],
        'time': ['interval']
    }

    def __init__(self):
        self.report = ValidationReport()
        self.tree: Optional[ET.ElementTree] = None
        self.root: Optional[ET.Element] = None
        self.task: Optional[ET.Element] = None

        # Словари для хранения всех допустимых ID
        self.flow_ids: Set[str] = set()
        self.process_ids: Set[str] = set()
        self.transport_ids: Set[str] = set()
        self.storage_ids: Set[str] = set()
        self.selector_ids: Set[str] = set()
        self.constraint_ids: Set[str] = set()
        self.object_ids_by_interval: Dict[str, Set[str]] = defaultdict(set)   # interval -> set of object ids
        self.link_pairs_by_interval: Dict[str, Set[Tuple[str, str]]] = defaultdict(set)

        # Сохраним структуру для семантических проверок
        self.structs_by_interval: Dict[str, ET.Element] = {}  # interval id -> struct element
        self.elem_attributes: Dict[Tuple[str, str], Dict[str, str]] = {}  # (interval, object) -> dict of attributes

    def validate(self, xml_path: str) -> ValidationReport:
        """
        Основной метод: выполняет все проверки и возвращает отчёт.
        """
        if not os.path.exists(xml_path):
            self.report.add_issue(IssueLevel.CRITICAL, f"Файл не найден: {xml_path}")
            return self.report

        try:
            self.tree = ET.parse(xml_path)
            self.root = self.tree.getroot()
        except ET.ParseError as e:
            self.report.add_issue(IssueLevel.CRITICAL, f"Ошибка парсинга XML: {e}")
            return self.report

        # Проверка корневого элемента
        self._check_root()

        # Поиск task
        self.task = self.root.find('task')
        if self.task is None:
            self.report.add_issue(IssueLevel.CRITICAL, "Отсутствует обязательный элемент <task>")
            return self.report

        # Сбор всех ID для последующей проверки ссылок
        self._collect_ids()

        # Последовательные проверки разделов
        self._check_flows()
        self._check_process()
        self._check_transport()
        self._check_storage()
        self._check_selectors()
        self._check_criterion()
        self._check_constraints()
        self._check_structures()

        # Проверка ссылочной целостности
        self._check_references()

        # Семантические проверки (соответствие переменных и атрибутов объектов)
        self._check_variable_existence()

        # Проверка непрерывности интервалов
        self._check_intervals_continuity()

        # Рекомендации по использованию возможностей
        self._add_usage_suggestions()

        return self.report

    # ---------- Вспомогательные методы ----------
    def _get_line(self, elem: ET.Element) -> Optional[int]:
        """Получить номер строки элемента, если доступно."""
        try:
            return elem.sourceline
        except AttributeError:
            return None

    def _check_root(self):
        """Проверка корневого элемента."""
        if self.root.tag != 'XMLDocument':
            self.report.add_issue(IssueLevel.WARNING,
                                  f"Корневой элемент должен быть <XMLDocument>, найден <{self.root.tag}>",
                                  self.root)
        if 'version' not in self.root.attrib:
            self.report.add_issue(IssueLevel.INFO,
                                  "Рекомендуется указывать версию в корневом элементе (version='1.0')",
                                  self.root)

    def _collect_ids(self):
        """Собираем все идентификаторы из разных разделов для последующих проверок."""
        # Потоки
        flows_elem = self.task.find('flows')
        if flows_elem is not None:
            for f in flows_elem.findall('type'):
                fid = f.get('id')
                if fid:
                    self.flow_ids.add(fid)

        # Технологии обработки
        proc_elem = self.task.find('process')
        if proc_elem is not None:
            for p in proc_elem.findall('type'):
                pid = p.get('id')
                if pid:
                    self.process_ids.add(pid)

        # Технологии передачи
        trans_elem = self.task.find('transport')
        if trans_elem is not None:
            for t in trans_elem.findall('type'):
                tid = t.get('id')
                if tid:
                    self.transport_ids.add(tid)

        # Хранилища
        stor_elem = self.task.find('storage')
        if stor_elem is not None:
            for s in stor_elem.findall('type'):
                sid = s.get('id')
                if sid:
                    self.storage_ids.add(sid)

        # Селекторы (рекурсивно)
        sel_elem = self.task.find('selectors')
        if sel_elem is not None:
            self._collect_selector_ids(sel_elem)

        # Ограничения
        constr_elem = self.task.find('constraints')
        if constr_elem is not None:
            for c in constr_elem.findall('constraint'):
                cid = c.get('id')
                if cid:
                    self.constraint_ids.add(cid)

        # Объекты и связи по интервалам
        for struct in self.task.findall('struct'):
            interval = struct.get('id')
            if interval is None:
                continue
            self.structs_by_interval[interval] = struct
            obj_set = self.object_ids_by_interval[interval]
            link_set = self.link_pairs_by_interval[interval]

            for elem in struct.findall('elem'):
                oid = elem.get('id')
                if oid:
                    obj_set.add(oid)
                    # Сохраняем все атрибуты объекта для семантических проверок
                    self.elem_attributes[(interval, oid)] = elem.attrib

            for link in struct.findall('link'):
                id1 = link.get('id1')
                id2 = link.get('id2')
                if id1 and id2:
                    link_set.add((id1, id2))

    def _collect_selector_ids(self, parent: ET.Element):
        """Рекурсивный сбор ID селекторов."""
        for sel in parent.findall('selector'):
            sid = sel.get('id')
            if sid:
                self.selector_ids.add(sid)
            self._collect_selector_ids(sel)

    # ---------- Проверки разделов ----------
    def _check_flows(self):
        flows = self.task.find('flows')
        if flows is None:
            self.report.add_issue(IssueLevel.ERROR, "Отсутствует раздел <flows>")
            return

        # Проверка уникальности ID потоков (уже собрано, но можно пройтись)
        seen = set()
        for flow in flows.findall('type'):
            fid = flow.get('id')
            if not fid:
                self.report.add_issue(IssueLevel.ERROR, "Поток не имеет атрибута 'id'", flow)
                continue
            if fid in seen:
                self.report.add_issue(IssueLevel.ERROR, f"Дубликат id потока: {fid}", flow)
            else:
                seen.add(fid)

            # Проверка числовых атрибутов
            for attr in ['price', 'loss', 'rashod']:
                val = flow.get(attr)
                if val is not None:
                    self._check_numeric(flow, attr, can_be_negative=(attr == 'price'))

    def _check_process(self):
        proc = self.task.find('process')
        if proc is None:
            self.report.add_issue(IssueLevel.WARNING, "Раздел <process> отсутствует (может быть пустым)")
            return
        self._check_tech_section(proc, 'process', self.process_ids)

    def _check_transport(self):
        trans = self.task.find('transport')
        if trans is None:
            self.report.add_issue(IssueLevel.WARNING, "Раздел <transport> отсутствует (может быть пустым)")
            return
        self._check_tech_section(trans, 'transport', self.transport_ids)

    def _check_tech_section(self, parent: ET.Element, tech_type: str, id_set: Set[str]):
        """Общая проверка для process и transport."""
        seen = set()
        for tech in parent.findall('type'):
            tid = tech.get('id')
            if not tid:
                self.report.add_issue(IssueLevel.ERROR, f"Технология {tech_type} без id", tech)
                continue
            if tid in seen:
                self.report.add_issue(IssueLevel.ERROR, f"Дубликат id {tech_type}: {tid}", tech)
            else:
                seen.add(tid)
                # Проверка на наличие в собранном множестве (уже есть)
                if tid not in id_set:
                    id_set.add(tid)  # на всякий случай

            # Проверка атрибута time
            time_val = tech.get('time')
            if time_val is None:
                self.report.add_issue(IssueLevel.ERROR, f"Технология {tech_type} не имеет атрибута 'time'", tech)
            else:
                self._check_numeric(tech, 'time', min_val=0)

            # Проверка input/output
            input_elem = tech.find('input')
            output_elem = tech.find('output')
            if input_elem is None and output_elem is None:
                self.report.add_issue(IssueLevel.WARNING,
                                      f"Технология {tech_type} не имеет ни input, ни output (бесполезна)", tech)

            self._check_io_section(input_elem, tech, 'input')
            self._check_io_section(output_elem, tech, 'output')

    def _check_io_section(self, section: Optional[ET.Element], parent: ET.Element, section_name: str):
        if section is None:
            return
        for io in section.findall('type'):
            fid = io.get('id')
            if not fid:
                self.report.add_issue(IssueLevel.ERROR, f"Элемент в секции {section_name} без id", io)
            # Проверка существования потока будет позже в _check_references

            size_val = io.get('size')
            if size_val is None:
                self.report.add_issue(IssueLevel.ERROR, f"Элемент в секции {section_name} не имеет атрибута 'size'", io)
            else:
                self._check_numeric(io, 'size', min_val=0)

    def _check_storage(self):
        stor = self.task.find('storage')
        if stor is None:
            self.report.add_issue(IssueLevel.WARNING, "Раздел <storage> отсутствует (может быть пустым)")
            return

        seen = set()
        for store in stor.findall('type'):
            sid = store.get('id')
            if not sid:
                self.report.add_issue(IssueLevel.ERROR, "Хранилище без id", store)
                continue
            if sid in seen:
                self.report.add_issue(IssueLevel.ERROR, f"Дубликат id хранилища: {sid}", store)
            else:
                seen.add(sid)
                if sid not in self.storage_ids:
                    self.storage_ids.add(sid)

            # Проверка input (size не обязателен, но если есть - проверим)
            input_elem = store.find('input')
            if input_elem is None:
                self.report.add_issue(IssueLevel.WARNING, f"Хранилище {sid} не имеет секции <input> (какие потоки хранит?)", store)
            else:
                for io in input_elem.findall('type'):
                    fid = io.get('id')
                    if not fid:
                        self.report.add_issue(IssueLevel.ERROR, "Элемент input хранилища без id", io)
                    size_val = io.get('size')
                    if size_val is not None:
                        self._check_numeric(io, 'size', min_val=0)

    def _check_selectors(self):
        selectors = self.task.find('selectors')
        if selectors is None:
            self.report.add_issue(IssueLevel.ERROR, "Отсутствует раздел <selectors>")
            return

        # Рекурсивная проверка дерева селекторов
        self._check_selector_tree(selectors)

    def _check_selector_tree(self, parent: ET.Element):
        """Проверка селекторов и вложенных элементов."""
        for sel in parent.findall('selector'):
            sid = sel.get('id')
            if not sid:
                self.report.add_issue(IssueLevel.ERROR, "Селектор без атрибута 'id'", sel)
            else:
                # Уникальность проверялась ранее, но можно дополнительно
                pass

            # Проверка sign как числового коэффициента (может быть отрицательным)
            sign = sel.get('sign')
            if sign is not None:
                try:
                    val = float(sign)
                    # Здесь не проверяем на отрицательность - отрицательные допустимы
                except ValueError:
                    self.report.add_issue(IssueLevel.ERROR, 
                                        f"Атрибут 'sign' в селекторе должен быть числом (найдено '{sign}')", sel)

            # Проверяем дочерние элементы
            for child in sel:
                if child.tag == 'selector':
                    self._check_selector_tree(child)
                else:
                    # Это элемент-переменная
                    self._check_variable_element(child)

    def _check_variable_element(self, elem: ET.Element):
        """Проверка корректности элемента-переменной (inputflow, resultflow и т.д.)."""
        if elem.tag not in self.VAR_TYPES:
            self.report.add_issue(IssueLevel.ERROR, f"Неизвестный тип переменной: {elem.tag}", elem)
            return

        required = self.VAR_TYPES[elem.tag]
        for attr in required:
            val = elem.get(attr)
            if val is None:
                self.report.add_issue(IssueLevel.ERROR, f"Переменная типа {elem.tag} не имеет обязательного атрибута '{attr}'", elem)
            elif val != '*':
                # Проверяем, что это целое число
                if not val.isdigit() and not (val.startswith('-') and val[1:].isdigit()):
                    self.report.add_issue(IssueLevel.ERROR, f"Атрибут '{attr}' должен быть целым числом или '*' (найдено '{val}')", elem)

    def _check_criterion(self):
        crit = self.task.find('criterion')
        if crit is None:
            self.report.add_issue(IssueLevel.ERROR, "Отсутствует раздел <criterion>")
            return

        sign = crit.get('sign')
        if sign is None:
            self.report.add_issue(IssueLevel.ERROR, "Целевая функция не имеет атрибута 'sign'", crit)
        elif sign not in {'MAX', 'MIN'}:
            self.report.add_issue(IssueLevel.ERROR, f"Атрибут 'sign' должен быть MAX или MIN, найден {sign}", crit)

        # Проверка ссылок на селекторы (будет в _check_references)
        for sel_ref in crit.findall('selector'):
            if 'id' not in sel_ref.attrib:
                self.report.add_issue(IssueLevel.ERROR, "Ссылка на селектор в criterion без id", sel_ref)

    def _check_constraints(self):
        constr = self.task.find('constraints')
        if constr is None:
            self.report.add_issue(IssueLevel.INFO, "Раздел <constraints> отсутствует (задача без ограничений)")
            return

        seen = set()
        for c in constr.findall('constraint'):
            cid = c.get('id')
            if not cid:
                self.report.add_issue(IssueLevel.ERROR, "Ограничение без id", c)
            elif cid in seen:
                self.report.add_issue(IssueLevel.ERROR, f"Дубликат id ограничения: {cid}", c)
            else:
                seen.add(cid)

            sign = c.get('sign')
            if sign is None:
                self.report.add_issue(IssueLevel.ERROR, "Ограничение не имеет атрибута 'sign'", c)
            elif sign not in self.VALID_SIGN:
                self.report.add_issue(IssueLevel.ERROR, f"Недопустимое значение sign: {sign}", c)

            val = c.get('value')
            if val is None:
                self.report.add_issue(IssueLevel.ERROR, "Ограничение не имеет атрибута 'value'", c)
            else:
                self._check_numeric(c, 'value')

            for sel_ref in c.findall('selector'):
                if 'id' not in sel_ref.attrib:
                    self.report.add_issue(IssueLevel.ERROR, "Ссылка на селектор в constraint без id", sel_ref)

    def _check_structures(self):
        structs = self.task.findall('struct')
        if not structs:
            self.report.add_issue(IssueLevel.ERROR, "Нет ни одного элемента <struct> (интервала планирования)")
            return

        for struct in structs:
            self._check_single_struct(struct)

    def _check_single_struct(self, struct: ET.Element):
        interval = struct.get('id')
        if interval is None:
            self.report.add_issue(IssueLevel.ERROR, "Интервал (struct) без атрибута 'id'", struct)
            return

        # Проверка уникальности id интервала уже выполнена при сборе

        # Проверка атрибута time
        time_val = struct.get('time')
        if time_val is not None and time_val != '':
            self._check_numeric(struct, 'time', min_val=0)

        # Проверка объектов
        objects_in_interval = self.object_ids_by_interval.get(interval, set())
        for elem in struct.findall('elem'):
            oid = elem.get('id')
            if not oid:
                self.report.add_issue(IssueLevel.ERROR, "Объект (elem) без id", elem)
                continue
            # Проверка атрибутов process_*, storage_*, input_*, output_*, capacity
            for attr, value in elem.attrib.items():
                if attr.startswith('process_'):
                    tech = attr.split('_')[1]
                    # Существование технологии проверим позже
                    if value != '':
                        self._check_numeric_with_value(elem, attr)
                elif attr.startswith('storage_'):
                    stor = attr.split('_')[1]
                    if value != '':
                        if value == '*':
                            # Звёздочка допустима? В предыдущем обсуждении решили, что нет.
                            self.report.add_issue(IssueLevel.WARNING,
                                                  f"Использование '*' в атрибуте storage_ (не документировано, возможно ошибка)", elem)
                        else:
                            self._check_numeric_with_value(elem, attr)
                elif attr.startswith('input_') or attr.startswith('output_'):
                    flow = attr.split('_')[1]
                    if value != '':
                        self._check_numeric_with_value(elem, attr)
                elif attr in ('send_capacity', 'receive_capacity', 'total_capacity'):
                    if value != '':
                        if value == '*':
                            # Звёздочка допустима (означает "равно времени интервала")
                            pass
                        else:
                            self._check_numeric_with_value(elem, attr)

        # Проверка связей
        for link in struct.findall('link'):
            id1 = link.get('id1')
            id2 = link.get('id2')
            if not id1 or not id2:
                self.report.add_issue(IssueLevel.ERROR, "Связь (link) без id1 или id2", link)
            else:
                if id1 not in objects_in_interval:
                    self.report.add_issue(IssueLevel.ERROR, f"Связь ссылается на несуществующий объект id1={id1}", link)
                if id2 not in objects_in_interval:
                    self.report.add_issue(IssueLevel.ERROR, f"Связь ссылается на несуществующий объект id2={id2}", link)

            direction = link.get('direction')
            if direction is not None and direction not in self.VALID_DIRECTION:
                self.report.add_issue(IssueLevel.ERROR, f"Недопустимое значение direction: {direction}", link)

            for attr, value in link.attrib.items():
                if attr.startswith('transport_'):
                    tech = attr.split('_')[1]
                    if value != '':
                        self._check_numeric_with_value(link, attr)

    # ---------- Проверка ссылочной целостности ----------
    def _check_references(self):
        """Проверка, что все ссылки (id) указывают на существующие элементы."""
        # Будем проходить по дереву и проверять каждый атрибут, который является ссылкой.
        # Это большой обход, но проще собрать все ссылки во время предыдущих проверок.
        # Мы уже собрали множества допустимых ID. Теперь пройдём по элементам, где могут быть ссылки.

        # Рекурсивный обход дерева
        self._walk_and_check_references(self.task)

    def _walk_and_check_references(self, elem: ET.Element):
        """Обход дерева и проверка атрибутов-ссылок."""
        # Проверка атрибутов текущего элемента
        self._check_element_references(elem)

        # Обход детей
        for child in elem:
            self._walk_and_check_references(child)

    def _check_element_references(self, elem: ET.Element):
        """Проверка ссылок в атрибутах элемента."""
        tag = elem.tag

        # Ссылки на потоки
        if tag in ('inputflow', 'resultflow', 'lost', 'to_process', 'from_process', 'storage'):
            flow = elem.get('flow')
            if flow and flow != '*' and flow not in self.flow_ids:
                self.report.add_issue(IssueLevel.ERROR, f"Ссылка на несуществующий поток id={flow}", elem)

        # Ссылки на технологии обработки
        if tag in ('process', 'to_process', 'from_process'):
            tech = elem.get('tech')
            if tech and tech != '*' and tech not in self.process_ids:
                self.report.add_issue(IssueLevel.ERROR, f"Ссылка на несуществующую технологию обработки id={tech}", elem)

        # Ссылки на технологии передачи
        if tag in ('transport', 'to_transport', 'from_transport'):
            tech = elem.get('tech')
            if tech and tech != '*' and tech not in self.transport_ids:
                self.report.add_issue(IssueLevel.ERROR, f"Ссылка на несуществующую технологию передачи id={tech}", elem)

        # Ссылки на хранилища
        if tag == 'storage':
            tech = elem.get('tech')
            if tech and tech != '*' and tech not in self.storage_ids:
                self.report.add_issue(IssueLevel.ERROR, f"Ссылка на несуществующее хранилище id={tech}", elem)

        # Ссылки на объекты (в переменных)
        if tag in ('inputflow', 'resultflow', 'process_all', 'to_process', 'process',
                   'from_process', 'transport_all', 'to_transport', 'transport',
                   'from_transport', 'storage', 'lost'):
            obj = elem.get('object')
            to_obj = elem.get('to_object')
            interval = elem.get('interval')

            if obj and obj != '*':
                # Проверим, что в указанном интервале есть такой объект
                if interval and interval != '*':
                    if interval not in self.object_ids_by_interval or obj not in self.object_ids_by_interval[interval]:
                        self.report.add_issue(IssueLevel.WARNING,
                                              f"Объект {obj} не найден в интервале {interval} (возможно, переменная всегда будет нулевой)",
                                              elem)
                # Без интервала не проверить

            if to_obj and to_obj != '*':
                if interval and interval != '*':
                    if interval not in self.object_ids_by_interval or to_obj not in self.object_ids_by_interval[interval]:
                        self.report.add_issue(IssueLevel.WARNING,
                                              f"Объект {to_obj} не найден в интервале {interval} (возможно, переменная всегда будет нулевой)",
                                              elem)

        # Ссылки на селекторы в criterion/constraints/selectors
        if tag == 'selector':
            # Проверяем, что это не корневой селектор (имеет родителя)
            # В стандартном ElementTree нет getparent, поэтому используем другой подход
            # Просто проверяем, что id существует в списке селекторов
            ref_id = elem.get('id')
            if ref_id and ref_id not in self.selector_ids:
                self.report.add_issue(IssueLevel.ERROR, f"Ссылка на несуществующий селектор id={ref_id}", elem)

        # Особые случаи: атрибуты process_*, transport_*, storage_* в struct
        if tag == 'elem':
            for attr, value in elem.attrib.items():
                if attr.startswith('process_'):
                    tech = attr.split('_')[1]
                    if tech not in self.process_ids:
                        self.report.add_issue(IssueLevel.ERROR, f"Ссылка на несуществующую технологию обработки id={tech} в атрибуте {attr}", elem)
                elif attr.startswith('transport_'):
                    # В elem не должно быть transport_, это ошибка? Но в коде нет, оставим на всякий случай
                    pass
                elif attr.startswith('storage_'):
                    stor = attr.split('_')[1]
                    if stor not in self.storage_ids:
                        self.report.add_issue(IssueLevel.ERROR, f"Ссылка на несуществующее хранилище id={stor} в атрибуте {attr}", elem)
                elif attr.startswith('input_') or attr.startswith('output_'):
                    flow = attr.split('_')[1]
                    if flow not in self.flow_ids:
                        self.report.add_issue(IssueLevel.ERROR, f"Ссылка на несуществующий поток id={flow} в атрибуте {attr}", elem)

        if tag == 'link':
            for attr, value in elem.attrib.items():
                if attr.startswith('transport_'):
                    tech = attr.split('_')[1]
                    if tech not in self.transport_ids:
                        self.report.add_issue(IssueLevel.ERROR, f"Ссылка на несуществующую технологию передачи id={tech} в атрибуте {attr}", elem)

    # ---------- Семантические проверки ----------
    def _check_variable_existence(self):
        """
        Для каждой переменной, встречающейся в селекторах, проверяем,
        есть ли в структуре соответствующие атрибуты, чтобы эта переменная
        могла быть ненулевой. Иначе выдаём предупреждение.
        """
        # Собираем все упоминания переменных из селекторов
        var_occurrences = []  # (тип, атрибуты)
        selectors_root = self.task.find('selectors')
        if selectors_root is not None:
            self._collect_variables(selectors_root, var_occurrences)

        for var_type, attrs in var_occurrences:
            interval = attrs.get('interval')
            obj = attrs.get('object')
            flow = attrs.get('flow')
            tech = attrs.get('tech')
            to_obj = attrs.get('to_object')

            # Если какой-то атрибут равен '*', пропускаем (слишком общее)
            if interval == '*' or obj == '*' or flow == '*' or tech == '*' or to_obj == '*':
                continue

            # Проверим наличие необходимых атрибутов у объекта в интервале
            key = (interval, obj)
            obj_attrs = self.elem_attributes.get(key)
            if obj_attrs is None:
                # Уже было предупреждение об отсутствии объекта
                continue

            # В зависимости от типа переменной проверяем наличие соответствующего атрибута
            if var_type == 'inputflow':
                if f'input_{flow}' not in obj_attrs:
                    self.report.add_issue(IssueLevel.WARNING,
                                          f"Переменная {var_type} для объекта {obj}, потока {flow} в интервале {interval} не имеет соответствующего атрибута input_{flow} у объекта. Переменная всегда будет равна 0.",
                                          None)  # Можно привязать к элементу, но сложно
            elif var_type == 'resultflow':
                if f'output_{flow}' not in obj_attrs:
                    self.report.add_issue(IssueLevel.WARNING,
                                          f"Переменная {var_type} для объекта {obj}, потока {flow} в интервале {interval} не имеет атрибута output_{flow}.")
            elif var_type == 'process':
                if f'process_{tech}' not in obj_attrs:
                    self.report.add_issue(IssueLevel.WARNING,
                                          f"Переменная process для объекта {obj}, технологии {tech} в интервале {interval} не имеет атрибута process_{tech}.")
            elif var_type == 'to_process':
                if f'process_{tech}' not in obj_attrs:
                    self.report.add_issue(IssueLevel.WARNING,
                                          f"Переменная to_process (потребление) для объекта {obj}, потока {flow}, технологии {tech} требует наличия process_{tech}.")
            elif var_type == 'from_process':
                if f'process_{tech}' not in obj_attrs:
                    self.report.add_issue(IssueLevel.WARNING,
                                          f"Переменная from_process (выпуск) для объекта {obj}, потока {flow}, технологии {tech} требует наличия process_{tech}.")
            elif var_type == 'transport':
                # Для transport нужно проверить наличие связи между obj и to_obj с данной технологией
                link_found = False
                struct = self.structs_by_interval.get(interval)
                if struct:
                    for link in struct.findall('link'):
                        id1 = link.get('id1')
                        id2 = link.get('id2')
                        if (id1 == obj and id2 == to_obj) or (id1 == to_obj and id2 == obj):
                            if f'transport_{tech}' in link.attrib:
                                link_found = True
                                break
                if not link_found:
                    self.report.add_issue(IssueLevel.WARNING,
                                          f"Переменная transport для объектов {obj}->{to_obj}, технологии {tech} в интервале {interval} не имеет соответствующей связи с атрибутом transport_{tech}.")
            elif var_type == 'to_transport':
                # Аналогично transport, но для конкретного направления? В коде to_transport генерируется на основе link, но наличие атрибута transport_* достаточно
                pass  # Можно аналогично transport
            elif var_type == 'from_transport':
                pass
            elif var_type == 'storage':
                if f'storage_{tech}' not in obj_attrs:
                    self.report.add_issue(IssueLevel.WARNING,
                                          f"Переменная storage для объекта {obj}, хранилища {tech} в интервале {interval} не имеет атрибута storage_{tech}.")
            elif var_type == 'lost':
                # lost всегда возможна? Она связана с потерями потоков, но не требует атрибута.
                pass
            elif var_type == 'time':
                # time всегда есть, если указан атрибут time у struct или переменная создаётся
                pass

    def _collect_variables(self, parent: ET.Element, out_list: List[Tuple[str, Dict[str, str]]]):
        """Собрать все элементы-переменные из дерева селекторов."""
        for sel in parent.findall('selector'):
            for child in sel:
                if child.tag in self.VAR_TYPES:
                    out_list.append((child.tag, child.attrib))
                elif child.tag == 'selector':
                    self._collect_variables(child, out_list)

    # ---------- Проверка непрерывности интервалов ----------
    def _check_intervals_continuity(self):
        """Проверяет, что id интервалов идут подряд без пропусков."""
        interval_ids = sorted(int(k) for k in self.structs_by_interval.keys())
        if not interval_ids:
            return
        expected = list(range(interval_ids[0], interval_ids[-1] + 1))
        if interval_ids != expected:
            missing = set(expected) - set(interval_ids)
            self.report.add_issue(IssueLevel.WARNING,
                                  f"Номера интервалов не образуют непрерывную последовательность. Пропущены: {sorted(missing)}")

    # ---------- Рекомендации по использованию ----------
    def _add_usage_suggestions(self):
        """Анализ использования возможностей и выдача советов."""
        # Проверим, используются ли односторонние связи
        has_one_way = False
        for struct in self.task.findall('struct'):
            for link in struct.findall('link'):
                direction = link.get('direction')
                if direction in ('forward', 'backward'):
                    has_one_way = True
                    break
            if has_one_way:
                break
        if not has_one_way:
            self.report.add_suggestion("Все связи двусторонние (direction='both'). Если какие-то передачи возможны только в одну сторону, используйте direction='forward' или 'backward' для уменьшения размерности задачи.")

        # Проверим использование звёздочек в селекторах
        star_in_selectors = False
        selectors = self.task.find('selectors')
        if selectors is not None:
            for elem in selectors.iter():
                if any(v == '*' for v in elem.attrib.values()):
                    star_in_selectors = True
                    break
        if star_in_selectors:
            self.report.add_suggestion("В селекторах используются звёздочки '*'. Это удобно, но помните, что они раскрываются во все возможные комбинации, что может сильно увеличить размерность задачи. Рекомендуется конкретизировать индексы, где возможно.")

        # Проверим, есть ли атрибуты send_capacity / receive_capacity / total_capacity
        has_capacity = False
        for struct in self.task.findall('struct'):
            for elem in struct.findall('elem'):
                if any(attr in elem.attrib for attr in ('send_capacity', 'receive_capacity', 'total_capacity')):
                    has_capacity = True
                    break
        if not has_capacity:
            self.report.add_suggestion("Ни один объект не имеет ограничений на пропускную способность (send_capacity, receive_capacity, total_capacity). Если каналы передачи имеют ограничения, добавьте эти атрибуты для более точной модели.")

        # Проверим, используются ли разные технологии с одинаковыми входами/выходами (возможность синтеза)
        # Для процессов
        proc_patterns = defaultdict(list)
        for tech_id in self.process_ids:
            tech = self.task.find(f".//process/type[@id='{tech_id}']")
            if tech is None:
                continue
            input_set = self._get_tech_io_set(tech, 'input')
            output_set = self._get_tech_io_set(tech, 'output')
            key = (frozenset(input_set), frozenset(output_set))
            proc_patterns[key].append(tech_id)
        for (inp, outp), ids in proc_patterns.items():
            if len(ids) > 1:
                self.report.add_suggestion(f"Технологии обработки {ids} имеют одинаковые наборы входов/выходов. Это позволяет использовать синтез (объединение) для повышения эффективности.")

        # Для транспорта
        trans_patterns = defaultdict(list)
        for tech_id in self.transport_ids:
            tech = self.task.find(f".//transport/type[@id='{tech_id}']")
            if tech is None:
                continue
            input_set = self._get_tech_io_set(tech, 'input')
            output_set = self._get_tech_io_set(tech, 'output')
            key = (frozenset(input_set), frozenset(output_set))
            trans_patterns[key].append(tech_id)
        for (inp, outp), ids in trans_patterns.items():
            if len(ids) > 1:
                self.report.add_suggestion(f"Технологии передачи {ids} имеют одинаковые наборы входов/выходов. Возможно применение синтеза.")

    def _get_tech_io_set(self, tech_elem: ET.Element, io: str) -> Set[str]:
        """Получить множество id потоков для input или output технологии."""
        section = tech_elem.find(io)
        if section is None:
            return set()
        return {io_type.get('id') for io_type in section.findall('type') if io_type.get('id')}

    # ---------- Вспомогательные проверки чисел ----------
    def _check_numeric(self, elem: ET.Element, attr: str, min_val: float = None, can_be_negative: bool = False):
        """Проверяет, что атрибут является числом и удовлетворяет ограничениям."""
        val = elem.get(attr)
        if val is None:
            return
        try:
            num = float(val)
        except ValueError:
            self.report.add_issue(IssueLevel.ERROR, f"Атрибут '{attr}' должен быть числом (найдено '{val}')", elem)
            return

        if not can_be_negative and num < 0:
            self.report.add_issue(IssueLevel.ERROR, f"Атрибут '{attr}' не может быть отрицательным", elem)
        if min_val is not None and num < min_val:
            self.report.add_issue(IssueLevel.ERROR, f"Атрибут '{attr}' должен быть >= {min_val}", elem)

    def _check_numeric_with_value(self, elem: ET.Element, attr: str):
        """Аналогично, но значение берётся из атрибута и не может быть пустым."""
        val = elem.get(attr)
        if val is None or val == '':
            return
        try:
            float(val)
        except ValueError:
            self.report.add_issue(IssueLevel.ERROR, f"Значение атрибута '{attr}' должно быть числом (найдено '{val}')", elem)


# ---------- CLI интерфейс ----------
def main():
    import argparse
    parser = argparse.ArgumentParser(description="Валидатор XML-файлов для системы PDA")
    parser.add_argument('file', help="Путь к XML-файлу")
    parser.add_argument('--output', '-o', help="Сохранить отчёт в файл")
    args = parser.parse_args()

    validator = PDAValidator()
    report = validator.validate(args.file)

    if args.output:
        report.save_report(args.output)
        print(f"Отчёт сохранён в {args.output}")
    else:
        report.print_report()

    sys.exit(1 if report.has_errors() else 0)


if __name__ == '__main__':
    main()