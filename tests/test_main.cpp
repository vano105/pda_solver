// Тесты решателя. Запуск: sh tests/run_tests.sh
//
// Проверяются четыре уровня:
//   1) симплекс-метод на задачах с известным ответом;
//   2) разбор входного XML;
//   3) модель целиком на задачах, решаемых вручную;
//   4) согласие двух методов и корректность найденного плана.

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "bpmn.h"
#include "check.h"
#include "dw.h"
#include "lp.h"
#include "parser.h"
#include "simplex.h"

namespace {

int failures = 0;
int checks = 0;
std::string currentTest;

void report(bool ok, const std::string& what) {
    ++checks;
    if (ok) return;
    ++failures;
    std::cout << "  ПРОВАЛ [" << currentTest << "] " << what << "\n";
}

void expectNear(double actual, double expected, const std::string& what,
                double tolerance = 1e-6) {
    const bool ok = std::fabs(actual - expected) <= tolerance * std::max(1.0, std::fabs(expected));
    if (!ok)
        std::cout << "  ПРОВАЛ [" << currentTest << "] " << what << ": получено " << actual
                  << ", ожидалось " << expected << "\n";
    ++checks;
    if (!ok) ++failures;
}

void expectTrue(bool value, const std::string& what) { report(value, what); }

void beginTest(const std::string& name) {
    currentTest = name;
    std::cout << "• " << name << "\n";
}

// Записывает временный XML и возвращает путь к нему.
std::string writeTemp(const std::string& name, const std::string& content) {
    const std::string path = "/tmp/pda_test_" + name + ".xml";
    std::ofstream out(path);
    out << content;
    return path;
}

// ---------------------------------------------------------------- симплекс

void testSimplexBasic() {
    beginTest("симплекс: задача с известным ответом");
    // max 3x + 5y  при  x <= 4, 2y <= 12, 3x + 2y <= 18  =>  x=2, y=6, цель 36
    pdaplan::LpProblem lp;
    lp.numVars = 2;
    lp.objective = {-3.0, -5.0};  // решатель минимизирует
    lp.inequalities.push_back({{{0, 1.0}}, 4.0});
    lp.inequalities.push_back({{{1, 2.0}}, 12.0});
    lp.inequalities.push_back({{{0, 3.0}, {1, 2.0}}, 18.0});

    const pdaplan::LpResult result = pdaplan::solveLp(lp);
    expectTrue(result.status == pdaplan::LpStatus::Optimal, "найден оптимум");
    expectNear(-result.objective, 36.0, "значение цели");
    expectNear(result.x[0], 2.0, "x");
    expectNear(result.x[1], 6.0, "y");
}

void testSimplexEquality() {
    beginTest("симплекс: равенства и двойственные оценки");
    // min 2x + 3y при x + y = 10, x <= 6  =>  x=6, y=4, цель 24
    pdaplan::LpProblem lp;
    lp.numVars = 2;
    lp.objective = {2.0, 3.0};
    lp.equalities.push_back({{{0, 1.0}, {1, 1.0}}, 10.0});
    lp.inequalities.push_back({{{0, 1.0}}, 6.0});

    const pdaplan::LpResult result = pdaplan::solveLp(lp);
    expectTrue(result.status == pdaplan::LpStatus::Optimal, "найден оптимум");
    expectNear(result.objective, 24.0, "значение цели");
    expectNear(result.x[0], 6.0, "x");
    expectTrue(result.duals.size() == 2, "двойственные оценки возвращены");
    // Оценка равенства равна стоимости лишней единицы правой части: это 3.
    expectNear(result.duals[1], 3.0, "оценка равенства");
}

void testSimplexInfeasible() {
    beginTest("симплекс: несовместные ограничения");
    pdaplan::LpProblem lp;
    lp.numVars = 1;
    lp.objective = {1.0};
    lp.equalities.push_back({{{0, 1.0}}, 5.0});
    lp.inequalities.push_back({{{0, 1.0}}, 2.0});

    const pdaplan::LpResult result = pdaplan::solveLp(lp);
    expectTrue(result.status == pdaplan::LpStatus::Infeasible, "несовместность обнаружена");
}

void testSimplexUnbounded() {
    beginTest("симплекс: неограниченная целевая функция");
    pdaplan::LpProblem lp;
    lp.numVars = 1;
    lp.objective = {-1.0};  // максимизируем x, ничем не ограниченный сверху

    const pdaplan::LpResult result = pdaplan::solveLp(lp);
    expectTrue(result.status == pdaplan::LpStatus::Unbounded, "неограниченность обнаружена");
}

// ---------------------------------------------------------------- разбор XML

const char* kTinyTask = R"(<?xml version="1.0" encoding="utf-8"?>
<XMLDocument version="1.0">
  <task>
    <flows><type id="1"/><type id="2"/></flows>
    <process>
      <type id="1" time="0.01">
        <input><type id="1" size="2"/></input>
        <output><type id="2" size="1"/></output>
      </type>
    </process>
    <transport>
      <type id="1" time="0.5">
        <input><type id="1" size="5"/></input>
        <output><type id="1" size="5"/></output>
      </type>
    </transport>
    <storage>
      <type id="1"><input><type id="1"/><type id="2"/></input></type>
    </storage>
    <struct id="1" time="100">
      <elem id="1" output_2="" process_1="" storage_1="1000"/>
      <elem id="2" input_1="300" storage_1="500"/>
      <link id1="2" id2="1" direction="forward" transport_1=""/>
    </struct>
    <selectors>
      <selector id="1" sign="1"><resultflow object="*" flow="2" interval="*"/></selector>
      <selector id="2" sign="-1"><lost object="*" flow="*" interval="*"/></selector>
    </selectors>
    <criterion sign="MAX"><selector id="1"/><selector id="2"/></criterion>
  </task>
</XMLDocument>)";

void testParser() {
    beginTest("разбор XML");
    const pdaplan::Task task = pdaplan::parseTask(writeTemp("tiny", kTinyTask));

    expectTrue(task.flows.size() == 2, "два типа потоков");
    expectTrue(task.intervals.size() == 1, "один интервал");
    expectNear(task.intervals[0].duration, 100.0, "длительность интервала");

    const pdaplan::ProcessTech& process = task.processes.at("1");
    expectNear(process.performance(), 200.0, "производительность обработки (2 ед / 0.01 с)");
    expectNear(process.yieldRatio(), 0.5, "коэффициент выхода (2 -> 1)");

    const pdaplan::TransportTech& transport = task.transports.at("1");
    expectNear(transport.performance(), 10.0, "производительность передачи (5 ед / 0.5 с)");
    expectNear(transport.timePerUnit(), 0.1, "удельное время передачи");

    const pdaplan::Node* sender = task.intervals[0].findNode("2");
    expectTrue(sender != nullptr, "объект-отправитель разобран");
    expectNear(sender->inputs.at("1"), 300.0, "входной объём");
    expectNear(sender->storages.at("1"), 500.0, "ёмкость ЗУ");

    const pdaplan::Node* receiver = task.intervals[0].findNode("1");
    expectTrue(receiver != nullptr && receiver->outputs.size() == 1, "потребитель разобран");
    expectTrue(receiver->processes.size() == 1, "технология обработки разрешена");

    expectTrue(task.criterion.sign == "MAX", "направление критерия");
    expectTrue(task.selectors.size() == 2, "селекторы разобраны");
}

void testLinkDirections() {
    beginTest("направления связи");
    pdaplan::Link link;
    link.id1 = "a";
    link.id2 = "b";

    link.direction = "forward";
    expectTrue(link.orderedPairs().size() == 1, "forward: одно направление");
    link.direction = "both";
    expectTrue(link.orderedPairs().size() == 2, "both: два направления");
    link.direction = "none";
    expectTrue(link.orderedPairs().empty(), "none: передача запрещена");
}

// ---------------------------------------------------------------- модель

void testTinyModel() {
    beginTest("модель: задача, решаемая вручную");
    const pdaplan::Task task = pdaplan::parseTask(writeTemp("tiny", kTinyTask));
    const pdaplan::Solution solution = pdaplan::buildAndSolve(task);

    expectTrue(solution.status == pdaplan::LpStatus::Optimal, "найден оптимум");

    // Объект 2 снял 300 единиц потока 1. Передача: 0.1 с на единицу, интервал 100 с,
    // значит передать можно не больше 1000 — ограничение не связывает, уходит всё 300.
    // Обработка: 0.005 с на единицу входа, 300 единиц это 1.5 с — тоже влезает.
    // Выход: 300 единиц входа дают 150 единиц потока 2. Критерий = 150.
    expectNear(solution.objective, 150.0, "значение критерия");
    expectNear(solution.value(pdaplan::transportVar("1", "2", "1", "1")), 300.0, "передано");
    expectNear(solution.value(pdaplan::processVar("1", "1", "1")), 300.0, "обработано");
    expectNear(solution.value(pdaplan::resultVar("1", "1", "2")), 150.0, "выдано потребителю");

    const auto problems = pdaplan::checkSolution(task, solution);
    expectTrue(problems.empty(), "проверка плана прошла");
}

void testTimeLimitBinds() {
    beginTest("модель: лимит времени ограничивает передачу");
    // Тот же пример, но интервал укорочен до 10 с: передать можно только 100 единиц.
    std::string xml = kTinyTask;
    const std::string from = "<struct id=\"1\" time=\"100\">";
    xml.replace(xml.find(from), from.size(), "<struct id=\"1\" time=\"10\">");

    const pdaplan::Task task = pdaplan::parseTask(writeTemp("short", xml));
    const pdaplan::Solution solution = pdaplan::buildAndSolve(task);

    expectNear(solution.value(pdaplan::transportVar("1", "2", "1", "1")), 100.0,
               "передача упирается в лимит времени (10 с / 0.1 с на единицу)");

    // 200 единиц не переданы: 500 влезает в ЗУ отправителя, значит потерь нет.
    const pdaplan::BalanceSummary balance = pdaplan::summarize(task, solution);
    expectNear(balance.input, 300.0, "поступило");
    expectNear(balance.delivered, 50.0, "доставлено (100 единиц входа -> 50 выхода)");
    expectNear(balance.lost, 0.0, "потерь нет");
    expectNear(balance.storedAtEnd, 200.0, "осталось в ЗУ отправителя");
    expectNear(balance.converted, 50.0, "объём, ушедший на сжатие при обработке");
    expectNear(balance.residual(), 0.0, "баланс сходится");
}

void testStorageOverflowCausesLoss() {
    beginTest("модель: нехватка ЗУ приводит к потерям");
    // Интервал 10 с (передать можно 100), ЗУ отправителя всего 50 единиц:
    // 300 - 100 - 50 = 150 единиц теряются.
    std::string xml = kTinyTask;
    const std::string fromTime = "<struct id=\"1\" time=\"100\">";
    xml.replace(xml.find(fromTime), fromTime.size(), "<struct id=\"1\" time=\"10\">");
    const std::string fromStorage = "input_1=\"300\" storage_1=\"500\"";
    xml.replace(xml.find(fromStorage), fromStorage.size(), "input_1=\"300\" storage_1=\"50\"");

    const pdaplan::Task task = pdaplan::parseTask(writeTemp("overflow", xml));
    const pdaplan::Solution solution = pdaplan::buildAndSolve(task);
    const pdaplan::BalanceSummary balance = pdaplan::summarize(task, solution);

    expectNear(balance.input, 300.0, "поступило");
    expectNear(balance.lost, 150.0, "потеряно");
    expectNear(balance.residual(), 0.0, "баланс сходится");
    expectTrue(pdaplan::checkSolution(task, solution).empty(), "проверка плана прошла");
}

// ---------------------------------------------------------------- два метода

void compareMethods(const std::string& path, const std::string& label) {
    beginTest("согласие методов: " + label);
    const pdaplan::Task task = pdaplan::parseTask(path);

    const pdaplan::Solution monolith = pdaplan::buildAndSolve(task);
    pdaplan::DwOptions options;
    pdaplan::DwStats stats;
    const pdaplan::Solution decomposed = pdaplan::solveDantzigWolfe(task, options, stats);

    expectTrue(monolith.status == pdaplan::LpStatus::Optimal, "монолит нашёл оптимум");
    expectTrue(decomposed.status == pdaplan::LpStatus::Optimal, "декомпозиция нашла оптимум");
    expectNear(decomposed.objective, monolith.objective, "критерии совпадают", 1e-6);

    expectTrue(pdaplan::checkSolution(task, monolith).empty(), "план монолита корректен");
    expectTrue(pdaplan::checkSolution(task, decomposed).empty(), "план декомпозиции корректен");

    // Баланс не зависит от метода.
    const pdaplan::BalanceSummary a = pdaplan::summarize(task, monolith);
    const pdaplan::BalanceSummary b = pdaplan::summarize(task, decomposed);
    expectNear(a.residual(), 0.0, "баланс монолита сходится");
    expectNear(b.residual(), 0.0, "баланс декомпозиции сходится");
    expectNear(b.input, a.input, "поступивший объём совпадает");
}

void testBpmnOutput() {
    beginTest("выгрузка плана в BPMN");
    const pdaplan::Task task = pdaplan::parseTask(writeTemp("tiny", kTinyTask));
    const pdaplan::Solution solution = pdaplan::buildAndSolve(task);

    auto chains = pdaplan::buildPlan(task, solution);
    expectTrue(!chains.empty(), "цепочки построены");

    size_t tasks = 0;
    for (const auto& chain : chains) tasks += chain.size();
    expectTrue(tasks > 0, "задачи плана построены");

    // Длительность задачи согласована с объёмом и производительностью технологии.
    for (const auto& chain : chains)
        for (const auto& item : chain)
            if (item.performance && *item.performance > 0)
                expectNear(item.duration, item.stream / *item.performance,
                           "длительность = объём / производительность");

    const std::string path = "/tmp/pda_test_plan.bpmn";
    pdaplan::writeBpmn(chains, path);
    std::ifstream file(path);
    expectTrue(file.good(), "файл записан");

    const std::string content((std::istreambuf_iterator<char>(file)),
                              std::istreambuf_iterator<char>());
    expectTrue(content.find("ltsm:props") != std::string::npos, "расширения ltsm на месте");
    expectTrue(content.find("bpmndi:BPMNDiagram") != std::string::npos, "раскладка схемы на месте");
}

}  // namespace

int main() {
    std::cout << "Тесты решателя\n\n";

    testSimplexBasic();
    testSimplexEquality();
    testSimplexInfeasible();
    testSimplexUnbounded();

    testParser();
    testLinkDirections();

    testTinyModel();
    testTimeLimitBinds();
    testStorageOverflowCausesLoss();

    compareMethods("examples/sample_task.xml", "sample_task");
    compareMethods("examples/tight_task.xml", "tight_task");

    testBpmnOutput();

    std::cout << "\nПроверок: " << checks << ", провалов: " << failures << "\n";
    return failures == 0 ? 0 : 1;
}