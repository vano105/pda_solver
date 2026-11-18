#include "dw.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <tuple>

namespace pdaplan {
namespace {

// Связующее ограничение: суммарный объём, принятый объектом to на интервале по
// технологии tech, равен сумме отправленного в его адрес всеми отправителями.
// Получателю не важно, кто именно прислал: в его балансе и в лимите времени приёма
// участвует только суммарный объём. Поэтому одна строка на (интервал, получатель,
// технология), а не на каждый канал — связующих ограничений получается в разы меньше.
struct Coupling {
    std::string interval;
    std::string to;
    std::string tech;

    bool operator<(const Coupling& other) const {
        return std::tie(interval, to, tech) < std::tie(other.interval, other.to, other.tech);
    }
};

// Строка взаимного исключения направлений канала: в обе стороны одновременно
// передавать нельзя, суммарная длительность не больше длины интервала. Такая строка
// охватывает оба конца канала, то есть два разных блока, поэтому она тоже связующая.
struct LinkRow {
    std::string interval;
    std::string nodeA;  // меньший из двух идентификаторов
    std::string nodeB;
    double duration = 0.0;

    bool operator<(const LinkRow& other) const {
        return std::tie(interval, nodeA, nodeB) <
               std::tie(other.interval, other.nodeA, other.nodeB);
    }
};

// Связующая строка разбиения по интервалам: остаток потока в запоминающем устройстве
// на границе между интервалом fromIndex и следующим. Это единственное, что связывает
// соседние интервалы: передачи целиком лежат внутри своего интервала.
struct CarryRow {
    int fromIndex = 0;
    std::string node;
    std::string flow;
    std::string tech;

    bool operator<(const CarryRow& other) const {
        return std::tie(fromIndex, node, flow, tech) <
               std::tie(other.fromIndex, other.node, other.flow, other.tech);
    }
};

// Подзадача одного объекта или одного интервала.
struct Block {
    std::string nodeId;
    std::vector<VarKey> keys;        // глобальный ключ каждого локального столбца
    std::vector<bool> incomingCopy;  // столбец — копия входящей передачи
    std::vector<double> objective;   // коэффициенты исходного критерия (на максимум)
    LpProblem lp;                    // собственные ограничения блока

    // Вхождения столбцов блока в связующие равенства: (номер связи, столбец, коэффициент).
    std::vector<std::tuple<int, int, double>> couplingEntries;
    // То же для строк взаимного исключения направлений канала.
    std::vector<std::tuple<int, int, double>> linkEntries;

    std::map<VarKey, int> index;

    int column(const VarKey& key, bool isIncoming) {
        auto it = index.find(key);
        if (it != index.end()) return it->second;
        const int id = static_cast<int>(keys.size());
        index[key] = id;
        keys.push_back(key);
        incomingCopy.push_back(isIncoming);
        objective.push_back(0.0);
        return id;
    }
};

// Сгенерированный план блока — крайняя точка его многогранника.
struct Column {
    int block = 0;
    std::vector<double> x;                         // значения переменных блока
    double objective = 0.0;                        // вклад в критерий (на максимум)
    std::vector<std::pair<int, double>> coupling;  // вклад в связующие равенства
    std::vector<std::pair<int, double>> links;     // вклад в строки каналов
};

bool matchesPattern(const std::string& pattern, const std::string& value) {
    return pattern == "*" || pattern == value;
}

// ---- Сбор списка связующих переменных ----

std::vector<Coupling> collectCouplings(const Task& task) {
    std::set<Coupling> unique;
    for (const Interval& interval : task.intervals) {
        for (const Link& link : interval.links) {
            for (const auto& [sender, receiver] : link.orderedPairs()) {
                if (!interval.findNode(sender) || !interval.findNode(receiver)) continue;
                for (const std::string& techId : link.transports) {
                    if (task.transports.count(techId) == 0) continue;
                    unique.insert({interval.id, receiver, techId});
                }
            }
        }
    }
    return std::vector<Coupling>(unique.begin(), unique.end());
}

// Сбор строк взаимного исключения: только двусторонние каналы, у односторонних
// достаточно лимита времени отправки у самого узла.
std::vector<LinkRow> collectLinkRows(const Task& task) {
    std::set<LinkRow> unique;
    for (const Interval& interval : task.intervals) {
        for (const Link& link : interval.links) {
            if (link.orderedPairs().size() < 2) continue;
            if (!interval.findNode(link.id1) || !interval.findNode(link.id2)) continue;
            if (link.transports.empty()) continue;
            const std::string& a = std::min(link.id1, link.id2);
            const std::string& b = std::max(link.id1, link.id2);
            unique.insert({interval.id, a, b, interval.duration});
        }
    }
    return std::vector<LinkRow>(unique.begin(), unique.end());
}

int linkRowIndex(const std::vector<LinkRow>& rows, const std::string& interval,
                 const std::string& one, const std::string& two) {
    const LinkRow key{interval, std::min(one, two), std::max(one, two), 0.0};
    auto it = std::lower_bound(rows.begin(), rows.end(), key);
    if (it == rows.end() || key < *it) return -1;
    return static_cast<int>(it - rows.begin());
}

std::vector<CarryRow> collectCarryRows(const Task& task) {
    std::set<CarryRow> unique;
    const int count = static_cast<int>(task.intervals.size());

    for (int idx = 0; idx + 1 < count; ++idx) {
        for (const Node& node : task.intervals[idx].nodes) {
            if (task.intervals[idx + 1].findNode(node.id) == nullptr) continue;
            for (const auto& [techId, capacity] : node.storages) {
                (void)capacity;
                auto st = task.storages.find(techId);
                if (st == task.storages.end()) continue;
                for (const std::string& flow : st->second.flows)
                    unique.insert({idx, node.id, flow, techId});
            }
        }
    }
    return std::vector<CarryRow>(unique.begin(), unique.end());
}

int carryRowIndex(const std::vector<CarryRow>& rows, int fromIndex, const std::string& node,
                  const std::string& flow, const std::string& tech) {
    const CarryRow key{fromIndex, node, flow, tech};
    auto it = std::lower_bound(rows.begin(), rows.end(), key);
    if (it == rows.end() || key < *it) return -1;
    return static_cast<int>(it - rows.begin());
}

// Номер связующего ограничения по (интервал, получатель, технология).
int couplingIndex(const std::vector<Coupling>& couplings, const std::string& interval,
                  const std::string& receiver, const std::string& tech) {
    const Coupling key{interval, receiver, tech};
    auto it = std::lower_bound(couplings.begin(), couplings.end(), key);
    if (it == couplings.end() || key < *it) return -1;
    return static_cast<int>(it - couplings.begin());
}

// ---- Построение одного блока ----

Block buildBlock(const Task& task, const std::string& nodeId,
                 const std::vector<Coupling>& couplings, const std::vector<LinkRow>& linkRows) {
    Block block;
    block.nodeId = nodeId;


    std::map<std::string, std::vector<VarKey>> carried;  // поток -> переменные хранения с прошлого интервала

    for (const Interval& interval : task.intervals) {
        const Node* node = interval.findNode(nodeId);
        if (node == nullptr) continue;
        const std::string& k = interval.id;

        std::map<std::string, std::map<int, double>> balance;  // поток -> строка
        std::map<std::string, double> constants;
        std::map<int, double> sendRow, recvRow, procRow;
        std::map<std::string, std::map<int, double>> localLinkRows;  // получатель -> строка

        auto addVar = [&](const std::string& flow, const VarKey& key, double coeff,
                          bool isIncoming = false) {
            balance[flow][block.column(key, isIncoming)] += coeff;
        };

        for (const auto& [flow, volume] : node->inputs) {
            constants[flow] += volume;
            balance[flow];
        }
        for (const auto& [flow, keys] : carried)
            for (const VarKey& key : keys) addVar(flow, key, 1.0);

        // Исходящие передачи: собственные переменные блока, каждая входит в связующее
        // ограничение своего получателя со знаком плюс.
        std::set<std::string> incomingTechs;
        std::set<std::string> incomingLinks;  // отправители, от которых возможен приём
        for (const Link& link : interval.links) {
            for (const auto& [sender, receiver] : link.orderedPairs()) {
                if (!interval.findNode(sender) || !interval.findNode(receiver)) continue;
                for (const std::string& techId : link.transports) {
                    auto tt = task.transports.find(techId);
                    if (tt == task.transports.end()) continue;

                    if (sender == nodeId) {
                        const int c = couplingIndex(couplings, k, receiver, techId);
                        const VarKey key = transportVar(k, sender, receiver, techId);
                        const int column = block.column(key, false);
                        balance[tt->second.flow][column] -= 1.0;
                        sendRow[column] = tt->second.timePerUnit();
                        block.couplingEntries.emplace_back(c, column, +1.0);

                        const int linkIndex = linkRowIndex(linkRows, k, sender, receiver);
                        if (linkIndex >= 0) {
                            // Двусторонний канал: ограничение охватывает оба блока.
                            block.linkEntries.emplace_back(linkIndex, column,
                                                           tt->second.timePerUnit());
                        } else {
                            // Односторонний канал: ограничение целиком внутри блока.
                            localLinkRows[receiver][column] = tt->second.timePerUnit();
                        }
                    }
                    if (receiver == nodeId) {
                        incomingTechs.insert(techId);
                        incomingLinks.insert(sender);
                    }
                }
            }
        }

        // Приём: одна переменная на технологию — суммарный принятый объём.
        for (const std::string& techId : incomingTechs) {
            const TransportTech& tech = task.transports.at(techId);
            const int c = couplingIndex(couplings, k, nodeId, techId);
            const VarKey key{"inflow", k, nodeId, "", tech.flow, techId};
            const int column = block.column(key, true);
            balance[tech.flow][column] += 1.0;
            recvRow[column] = tech.timePerUnit();
            block.couplingEntries.emplace_back(c, column, -1.0);
        }

        for (const std::string& techId : node->processes) {
            auto pt = task.processes.find(techId);
            if (pt == task.processes.end()) continue;
            const ProcessTech& tech = pt->second;
            const VarKey key = processVar(k, nodeId, techId);
            const int column = block.column(key, false);
            balance[tech.inFlow][column] -= 1.0;
            balance[tech.outFlow][column] += tech.yieldRatio();
            procRow[column] = tech.timePerUnit();
        }

        std::map<std::string, std::vector<VarKey>> nextCarried;
        for (const auto& [techId, capacity] : node->storages) {
            auto st = task.storages.find(techId);
            if (st == task.storages.end()) continue;

            std::map<int, double> capacityRow;
            for (const std::string& flow : st->second.flows) {
                const VarKey key = storageVar(k, nodeId, flow, techId);
                addVar(flow, key, -1.0);
                nextCarried[flow].push_back(key);
                capacityRow[block.column(key, false)] = 1.0;
            }
            if (std::isfinite(capacity) && !capacityRow.empty())
                block.lp.inequalities.push_back({capacityRow, capacity});
        }

        for (const std::string& flow : task.flows)
            addVar(flow, lostVar(k, nodeId, flow), -1.0);
        for (const std::string& flow : node->outputs)
            addVar(flow, resultVar(k, nodeId, flow), -1.0);

        for (const auto& [flow, row] : balance) {
            if (row.empty() && constants.count(flow) == 0) continue;
            double rhs = 0.0;
            auto it = constants.find(flow);
            if (it != constants.end()) rhs = -it->second;
            block.lp.equalities.push_back({row, rhs});
        }

        const double procLimit = node->totalCapacity.value_or(interval.duration);
        if (!procRow.empty()) block.lp.inequalities.push_back({procRow, procLimit});

        // Узловые лимиты отправки и приёма — только если заданы во входном файле.
        if (node->sendCapacity && !sendRow.empty())
            block.lp.inequalities.push_back({sendRow, *node->sendCapacity});

        // Приём ограничивается либо явным лимитом, либо суммарной пропускной способностью
        // входящих каналов. Вторая граница следует из ограничений каналов и потому не
        // отсекает допустимых решений, но делает многогранник блока ограниченным —
        // без этого подзадача получателя не имеет решения.
        if (!recvRow.empty()) {
            const double channels = static_cast<double>(incomingLinks.size());
            const double implied = channels * interval.duration;
            const double limit = node->receiveCapacity ? std::min(*node->receiveCapacity, implied)
                                                       : implied;
            block.lp.inequalities.push_back({recvRow, limit});
        }

        // Пропускная способность односторонних каналов.
        for (const auto& [receiver, row] : localLinkRows)
            if (!row.empty()) block.lp.inequalities.push_back({row, interval.duration});

        carried = std::move(nextCarried);
    }

    block.lp.numVars = static_cast<int>(block.keys.size());
    block.objective.resize(block.keys.size(), 0.0);
    return block;
}

// ---- Построение блока-интервала ----

// Блок содержит все переменные одного интервала. Остаток в памяти на входе блока —
// отдельная переменная, связанная равенством с остатком на выходе предыдущего блока.
Block buildIntervalBlock(const Task& task, int index, const std::vector<CarryRow>& carries) {
    const Interval& interval = task.intervals[index];
    const std::string& k = interval.id;

    Block block;
    block.nodeId = k;

    std::map<std::string, std::map<std::string, std::map<int, double>>> balance;  // узел -> поток
    std::map<std::string, std::map<std::string, double>> constants;
    std::map<std::string, std::map<int, double>> sendRow, recvRow, procRow;

    for (const Node& node : interval.nodes) {
        for (const auto& [flow, volume] : node.inputs) {
            constants[node.id][flow] += volume;
            balance[node.id][flow];
        }
    }

    // Передачи: обе стороны лежат в этом же блоке, связующих ограничений не нужно.
    for (const Link& link : interval.links) {
        std::map<int, double> linkRow;
        for (const auto& [sender, receiver] : link.orderedPairs()) {
            if (!interval.findNode(sender) || !interval.findNode(receiver)) continue;
            for (const std::string& techId : link.transports) {
                auto tt = task.transports.find(techId);
                if (tt == task.transports.end()) continue;
                const TransportTech& tech = tt->second;

                const int column = block.column(transportVar(k, sender, receiver, techId), false);
                balance[sender][tech.flow][column] -= 1.0;
                balance[receiver][tech.flow][column] += 1.0;
                sendRow[sender][column] = tech.timePerUnit();
                recvRow[receiver][column] = tech.timePerUnit();
                linkRow[column] = tech.timePerUnit();
            }
        }
        if (!linkRow.empty()) block.lp.inequalities.push_back({linkRow, interval.duration});
    }

    for (const Node& node : interval.nodes) {
        for (const std::string& techId : node.processes) {
            auto pt = task.processes.find(techId);
            if (pt == task.processes.end()) continue;
            const ProcessTech& tech = pt->second;

            const int column = block.column(processVar(k, node.id, techId), false);
            balance[node.id][tech.inFlow][column] -= 1.0;
            balance[node.id][tech.outFlow][column] += tech.yieldRatio();
            procRow[node.id][column] = tech.timePerUnit();
        }

        for (const auto& [techId, capacity] : node.storages) {
            auto st = task.storages.find(techId);
            if (st == task.storages.end()) continue;

            std::map<int, double> capacityRow, incomingRow;
            for (const std::string& flow : st->second.flows) {
                // Остаток на конец интервала.
                const int out = block.column(storageVar(k, node.id, flow, techId), false);
                balance[node.id][flow][out] -= 1.0;
                capacityRow[out] = 1.0;
                const int carryOut = carryRowIndex(carries, index, node.id, flow, techId);
                if (carryOut >= 0) block.couplingEntries.emplace_back(carryOut, out, +1.0);

                // Остаток, перенесённый с предыдущего интервала.
                const int carryIn = carryRowIndex(carries, index - 1, node.id, flow, techId);
                if (carryIn >= 0) {
                    const VarKey key{"carryin", k, node.id, "", flow, techId};
                    const int in = block.column(key, true);
                    balance[node.id][flow][in] += 1.0;
                    incomingRow[in] = 1.0;
                    block.couplingEntries.emplace_back(carryIn, in, -1.0);
                }
            }
            if (std::isfinite(capacity)) {
                if (!capacityRow.empty())
                    block.lp.inequalities.push_back({capacityRow, capacity});
                // Перенесённый остаток тоже не мог превышать ёмкость: без этой границы
                // многогранник блока не ограничен и подзадача не имеет решения.
                if (!incomingRow.empty())
                    block.lp.inequalities.push_back({incomingRow, capacity});
            }
        }

        for (const std::string& flow : task.flows)
            balance[node.id][flow][block.column(lostVar(k, node.id, flow), false)] -= 1.0;
        for (const std::string& flow : node.outputs)
            balance[node.id][flow][block.column(resultVar(k, node.id, flow), false)] -= 1.0;
    }

    for (const auto& [nodeId, flows] : balance) {
        for (const auto& [flow, row] : flows) {
            double rhs = 0.0;
            auto it = constants.find(nodeId);
            if (it != constants.end()) {
                auto jt = it->second.find(flow);
                if (jt != it->second.end()) rhs = -jt->second;
            }
            if (row.empty() && rhs == 0.0) continue;
            block.lp.equalities.push_back({row, rhs});
        }
    }

    for (const Node& node : interval.nodes) {
        const double procLimit = node.totalCapacity.value_or(interval.duration);
        if (auto it = procRow.find(node.id); it != procRow.end() && !it->second.empty())
            block.lp.inequalities.push_back({it->second, procLimit});
        if (node.sendCapacity)
            if (auto it = sendRow.find(node.id); it != sendRow.end() && !it->second.empty())
                block.lp.inequalities.push_back({it->second, *node.sendCapacity});
        if (node.receiveCapacity)
            if (auto it = recvRow.find(node.id); it != recvRow.end() && !it->second.empty())
                block.lp.inequalities.push_back({it->second, *node.receiveCapacity});
    }

    block.lp.numVars = static_cast<int>(block.keys.size());
    block.objective.resize(block.keys.size(), 0.0);
    return block;
}

// ---- Разнесение критерия по блокам ----

// Каждая переменная принадлежит ровно одному блоку: передача — отправителю,
// остальные — своему объекту.
void distributeObjective(const Task& task, std::vector<Block>& blocks) {
    for (const std::string& selectorId : task.criterion.selectorIds) {
        auto it = task.selectors.find(selectorId);
        if (it == task.selectors.end()) continue;

        // Повторяем раскрытие селекторов, но раскладываем результат по блокам.
        std::vector<std::pair<VarKey, double>> terms;
        std::function<void(const Selector&, double)> walk = [&](const Selector& selector,
                                                                double multiplier) {
            const double weight = multiplier * selector.sign;
            for (const VarRef& ref : selector.terms) {
                for (const Interval& interval : task.intervals) {
                    if (!matchesPattern(ref.at("interval"), interval.id)) continue;
                    for (const Node& node : interval.nodes) {
                        if (!matchesPattern(ref.at("object"), node.id)) continue;

                        if (ref.kind == "resultflow") {
                            for (const std::string& flow : node.outputs)
                                if (matchesPattern(ref.at("flow"), flow))
                                    terms.emplace_back(resultVar(interval.id, node.id, flow),
                                                       weight);
                        } else if (ref.kind == "lost") {
                            for (const std::string& flow : task.flows)
                                if (matchesPattern(ref.at("flow"), flow))
                                    terms.emplace_back(lostVar(interval.id, node.id, flow), weight);
                        } else if (ref.kind == "storage") {
                            for (const auto& [techId, capacity] : node.storages) {
                                (void)capacity;
                                auto st = task.storages.find(techId);
                                if (st == task.storages.end() ||
                                    !matchesPattern(ref.at("tech"), techId))
                                    continue;
                                for (const std::string& flow : st->second.flows)
                                    if (matchesPattern(ref.at("flow"), flow))
                                        terms.emplace_back(
                                            storageVar(interval.id, node.id, flow, techId), weight);
                            }
                        } else if (ref.kind == "process" || ref.kind == "process_all" ||
                                   ref.kind == "to_process" || ref.kind == "from_process") {
                            for (const std::string& techId : node.processes) {
                                auto pt = task.processes.find(techId);
                                if (pt == task.processes.end()) continue;
                                if (ref.kind != "process_all" &&
                                    !matchesPattern(ref.at("tech"), techId))
                                    continue;
                                const ProcessTech& tech = pt->second;
                                const VarKey key = processVar(interval.id, node.id, techId);
                                if (ref.kind == "to_process") {
                                    if (matchesPattern(ref.at("flow"), tech.inFlow))
                                        terms.emplace_back(key, weight);
                                } else if (ref.kind == "from_process") {
                                    if (matchesPattern(ref.at("flow"), tech.outFlow))
                                        terms.emplace_back(key, weight * tech.yieldRatio());
                                } else {
                                    // process и process_all — длительности обработки
                                    terms.emplace_back(key, weight * tech.timePerUnit());
                                }
                            }
                        } else if (ref.kind == "transport" || ref.kind == "transport_all" ||
                                   ref.kind == "to_transport" || ref.kind == "from_transport") {
                            for (const Link& link : interval.links) {
                                for (const auto& [sender, receiver] : link.orderedPairs()) {
                                    if (sender != node.id) continue;
                                    if (!matchesPattern(ref.at("to_object"), receiver)) continue;
                                    for (const std::string& techId : link.transports) {
                                        auto tt = task.transports.find(techId);
                                        if (tt == task.transports.end()) continue;
                                        if (ref.kind != "transport_all" &&
                                            !matchesPattern(ref.at("tech"), techId))
                                            continue;
                                        if ((ref.kind == "to_transport" ||
                                             ref.kind == "from_transport") &&
                                            !matchesPattern(ref.at("flow"), tt->second.flow))
                                            continue;
                                        // transport и transport_all — длительности передачи
                                        const double scale =
                                            (ref.kind == "transport" ||
                                             ref.kind == "transport_all")
                                                ? tt->second.timePerUnit()
                                                : 1.0;
                                        terms.emplace_back(
                                            transportVar(interval.id, sender, receiver, techId),
                                            weight * scale);
                                    }
                                }
                            }
                        }
                    }
                }
            }
            for (const Selector& child : selector.children) walk(child, weight);
        };
        walk(it->second, 1.0);

        // Переменная принадлежит ровно одному блоку, поэтому достаточно найти тот,
        // где она есть. Так код одинаково работает для обоих разбиений.
        for (const auto& [key, weight] : terms) {
            for (Block& block : blocks) {
                auto columnIt = block.index.find(key);
                if (columnIt == block.index.end()) continue;
                if (block.incomingCopy[columnIt->second]) break;  // копия не несёт критерия
                block.objective[columnIt->second] += weight;
                break;
            }
        }
    }
}

}  // namespace

Solution solveDantzigWolfe(const Task& task, const DwOptions& options, DwStats& stats) {
    std::vector<Block> blocks;
    int numCouplings = 0;
    int numLinkRows = 0;
    std::vector<double> linkLimits;  // правые части строк пропускной способности каналов

    if (options.byInterval) {
        // Блок — один интервал. Связующие строки: остатки в памяти на границах.
        const std::vector<CarryRow> carries = collectCarryRows(task);
        numCouplings = static_cast<int>(carries.size());
        for (int index = 0; index < static_cast<int>(task.intervals.size()); ++index)
            blocks.push_back(buildIntervalBlock(task, index, carries));
    } else {
        // Блок — один объект. Связующие строки: приём по каналам и их пропускная
        // способность.
        const std::vector<Coupling> couplings = collectCouplings(task);
        numCouplings = static_cast<int>(couplings.size());
        const std::vector<LinkRow> linkRows = collectLinkRows(task);
        numLinkRows = static_cast<int>(linkRows.size());
        for (const LinkRow& row : linkRows) linkLimits.push_back(row.duration);

        std::set<std::string> nodeIds;
        for (const Interval& interval : task.intervals)
            for (const Node& node : interval.nodes) nodeIds.insert(node.id);
        for (const std::string& nodeId : nodeIds)
            blocks.push_back(buildBlock(task, nodeId, couplings, linkRows));
    }
    distributeObjective(task, blocks);
    const int numBlocks = static_cast<int>(blocks.size());

    if (std::getenv("PDA_SIZES")) {
        size_t smallest = static_cast<size_t>(-1), largest = 0, rows = 0;
        for (const Block& block : blocks) {
            const size_t columns = static_cast<size_t>(block.lp.numVars);
            smallest = std::min(smallest, columns);
            largest = std::max(largest, columns);
            rows = std::max(rows, block.lp.equalities.size() + block.lp.inequalities.size());
        }
        std::fprintf(stderr,
                     "[Д-В] блоков %d; подзадача: переменных %zu-%zu, ограничений до %zu; "
                     "главная задача: строк %d (связующих %d + каналы %d + выпуклость %d)\n",
                     numBlocks, smallest, largest, rows,
                     numCouplings + numLinkRows + numBlocks, numCouplings, numLinkRows,
                     numBlocks);
    }

    stats.blocks = numBlocks;
    stats.couplings = numCouplings + numLinkRows;

    // Вклад плана блока в связующие равенства.
    auto couplingContribution = [&](int blockIndex, const std::vector<double>& x) {
        std::map<int, double> sums;
        for (const auto& [coupling, column, coeff] : blocks[blockIndex].couplingEntries)
            if (column < static_cast<int>(x.size()) && std::fabs(x[column]) > 1e-12)
                sums[coupling] += coeff * x[column];

        std::vector<std::pair<int, double>> result(sums.begin(), sums.end());
        return result;
    };

    auto linkContribution = [&](int blockIndex, const std::vector<double>& x) {
        std::map<int, double> sums;
        for (const auto& [row, column, coeff] : blocks[blockIndex].linkEntries)
            if (column < static_cast<int>(x.size()) && std::fabs(x[column]) > 1e-12)
                sums[row] += coeff * x[column];

        std::vector<std::pair<int, double>> result(sums.begin(), sums.end());
        return result;
    };

    auto blockObjective = [&](int blockIndex, const std::vector<double>& x) {
        double value = 0.0;
        for (size_t j = 0; j < x.size(); ++j) value += blocks[blockIndex].objective[j] * x[j];
        return value;
    };

    // Решение подзадачи блока при заданных ценах на связующие равенства.
    auto solveSubproblem = [&](int blockIndex, const std::vector<double>& prices,
                               const std::vector<double>& linkPrices) {
        Block& block = blocks[blockIndex];
        LpProblem lp = block.lp;
        lp.objective.assign(lp.numVars, 0.0);
        for (int j = 0; j < lp.numVars; ++j) lp.objective[j] = -block.objective[j];
        for (const auto& [coupling, column, coeff] : block.couplingEntries)
            lp.objective[column] -= prices[coupling] * coeff;
        for (const auto& [row, column, coeff] : block.linkEntries)
            lp.objective[column] -= linkPrices[row] * coeff;
        return solveLp(lp);
    };

    // ---- Стартовые столбцы ----

    // Тривиальный план блока: ничего не передавать и не обрабатывать, всё поступившее
    // списать в потери. Он допустим при любых данных и не даёт вклада в связующие
    // строки, поэтому главная задача допустима с первой итерации и обходится без
    // штрафных переменных.
    auto trivialPlan = [&](int b) {
        std::vector<double> x(blocks[b].keys.size(), 0.0);
        for (const Interval& interval : task.intervals)
            for (const Node& node : interval.nodes)
                for (const auto& [flow, volume] : node.inputs) {
                    auto it = blocks[b].index.find(lostVar(interval.id, node.id, flow));
                    if (it != blocks[b].index.end()) x[it->second] = volume;
                }
        return x;
    };

    // Согласованный стартовый план для разбиения по интервалам: прямой проход по
    // блокам. Подзадача каждого блока решается с остатком на входе, зафиксированным
    // по выходу предыдущего блока, и с малой премией за перенос остатка дальше.
    //
    // Без такого старта метод не сходится. Хранение само по себе не входит в критерий,
    // поэтому при нулевых ценах блок всегда предлагает нулевой перенос. Блок-получатель
    // охотно принял бы остаток, но связующее равенство обнуляет вес его столбца. Шаг
    // получается вырожденным, цены не растут, и генерация столбцов останавливается на
    // том, что доставимо без переноса между интервалами. Согласованный старт сразу даёт
    // главной задаче ненулевые веса и осмысленные цены.
    std::vector<std::vector<double>> seeded(numBlocks);
    bool haveSeed = false;

    if (options.byInterval && numBlocks > 1) {
        double reward = 1.0;
        for (int b = 0; b < numBlocks; ++b)
            for (double weight : blocks[b].objective) reward = std::max(reward, std::fabs(weight));
        const double bonus = 1e-4 * reward;  // только разрешает ничью в пользу переноса

        std::map<int, double> carry;  // связующая строка -> перенесённый объём
        haveSeed = true;
        for (int b = 0; b < numBlocks; ++b) {
            LpProblem lp = blocks[b].lp;
            lp.objective.assign(lp.numVars, 0.0);
            for (int j = 0; j < lp.numVars; ++j) lp.objective[j] = -blocks[b].objective[j];

            for (const auto& [row, column, coeff] : blocks[b].couplingEntries) {
                if (coeff > 0.0) {
                    lp.objective[column] -= bonus;  // премия за перенос остатка дальше
                } else {
                    LpRow fixed;  // вход зафиксирован выходом предыдущего блока
                    fixed.coeffs[column] = 1.0;
                    auto it = carry.find(row);
                    fixed.rhs = it == carry.end() ? 0.0 : it->second;
                    lp.equalities.push_back(fixed);
                }
            }

            const LpResult step = solveLp(lp);
            if (step.status != LpStatus::Optimal) {
                haveSeed = false;
                break;
            }
            seeded[b] = step.x;
            for (const auto& [row, column, coeff] : blocks[b].couplingEntries)
                if (coeff > 0.0 && column < static_cast<int>(step.x.size()))
                    carry[row] = step.x[column];
        }
    }

    // Согласованные столбцы идут первыми: на них настроен тёплый старт главной задачи.
    std::vector<Column> columns;
    for (int b = 0; b < numBlocks; ++b) {
        const std::vector<double> x = haveSeed ? seeded[b] : trivialPlan(b);
        columns.push_back({b, x, blockObjective(b, x), couplingContribution(b, x),
                           linkContribution(b, x)});
    }
    if (haveSeed)
        for (int b = 0; b < numBlocks; ++b) {
            const std::vector<double> x = trivialPlan(b);
            columns.push_back({b, x, blockObjective(b, x), couplingContribution(b, x),
                               linkContribution(b, x)});
        }
    stats.columnsGenerated = static_cast<int>(columns.size());

    std::vector<double> lambda;
    LpResult master;

    // Восстановление плана из весов: нужно и при оптимуме, и при остановке без
    // доказанной оптимальности — найденный план допустим в любом случае.
    auto recover = [&](LpStatus status, const std::string& message) {
        Solution solution;
        solution.status = status;
        solution.message = message;

        std::vector<std::vector<double>> blockValues(numBlocks);
        for (int b = 0; b < numBlocks; ++b) blockValues[b].assign(blocks[b].keys.size(), 0.0);

        for (size_t j = 0; j < lambda.size() && j < columns.size(); ++j) {
            if (lambda[j] <= 1e-12) continue;
            const Column& column = columns[j];
            for (size_t v = 0; v < column.x.size(); ++v)
                blockValues[column.block][v] += lambda[j] * column.x[v];
        }

        for (int b = 0; b < numBlocks; ++b) {
            for (size_t v = 0; v < blocks[b].keys.size(); ++v) {
                if (blocks[b].incomingCopy[v]) continue;  // копия, не собственная переменная
                const double value = blockValues[b][v];
                if (std::fabs(value) > 1e-9) solution.values[blocks[b].keys[v]] += value;
            }
        }

        double objective = 0.0;
        for (size_t j = 0; j < lambda.size() && j < columns.size(); ++j)
            objective += lambda[j] * columns[j].objective;
        solution.objective = objective;
        stats.masterObjective = objective;
        return solution;
    };

    // Остановка по отсутствию прогресса: на вырожденной главной задаче новые
    // столбцы входят с нулевым шагом, критерий не растёт, и продолжать бессмысленно.
    double bestObjective = -std::numeric_limits<double>::infinity();
    int stalled = 0;

    // Сглаживание двойственных оценок: главная задача вырождена, её цены скачут,
    // и блоки предлагают планы, которые входят с нулевым шагом. Усреднение с ценами
    // предыдущей итерации гасит скачки; оптимальность проверяется по точным ценам.
    std::vector<double> smoothPrices, smoothLinkPrices;
    const double alpha = 0.5;
    // Плато до сотни итераций встречается и у сходящихся задач.
    const int stallLimit = std::getenv("PDA_STALL") ? std::atoi(std::getenv("PDA_STALL")) : 120;

    for (int iteration = 1; iteration <= options.maxIterations; ++iteration) {
        stats.iterations = iteration;

        // ---- Главная задача ----
        // min  -Σ c_bp λ_bp
        // при  Σ (вклад в связь c) λ_bp = 0 для каждой связи c,  Σ_p λ_bp = 1 для каждого блока
        const int numColumns = static_cast<int>(columns.size());
        LpProblem lp;
        lp.numVars = numColumns;

        std::vector<LpRow> couplingRows(numCouplings);
        for (int c = 0; c < numCouplings; ++c) couplingRows[c].rhs = 0.0;
        for (int j = 0; j < numColumns; ++j)
            for (const auto& [coupling, value] : columns[j].coupling)
                couplingRows[coupling].coeffs[j] += value;

        // Масштабирование. Объёмы потоков достигают сотен тысяч, а условия
        // выпуклости имеют коэффициент единица: без выравнивания симплекс в плотной
        // таблице теряет точность и буксует на вырожденных шагах.
        std::vector<double> rowScale(numCouplings, 1.0);
        for (int c = 0; c < numCouplings; ++c) {
            double largest = 0.0;
            for (const auto& [column, value] : couplingRows[c].coeffs)
                largest = std::max(largest, std::fabs(value));
            if (largest > 0.0) rowScale[c] = largest;
            for (auto& [column, value] : couplingRows[c].coeffs) value /= rowScale[c];
        }
        for (const LpRow& row : couplingRows) lp.equalities.push_back(row);

        // Строки взаимного исключения направлений канала — неравенства <= длины интервала.
        std::vector<LpRow> linkLpRows(numLinkRows);
        std::vector<double> linkScale(numLinkRows, 1.0);
        for (int r = 0; r < numLinkRows; ++r) linkLpRows[r].rhs = linkLimits[r];
        for (int j = 0; j < numColumns; ++j)
            for (const auto& [row, value] : columns[j].links) linkLpRows[row].coeffs[j] += value;
        for (int r = 0; r < numLinkRows; ++r) {
            double largest = 0.0;
            for (const auto& [column, value] : linkLpRows[r].coeffs)
                largest = std::max(largest, std::fabs(value));
            if (largest > 0.0) linkScale[r] = largest;
            for (auto& [column, value] : linkLpRows[r].coeffs) value /= linkScale[r];
            linkLpRows[r].rhs /= linkScale[r];
            lp.inequalities.push_back(linkLpRows[r]);
        }

        double objScale = 1.0;
        for (const Column& column : columns)
            objScale = std::max(objScale, std::fabs(column.objective));
        lp.objective.assign(lp.numVars, 0.0);
        for (int j = 0; j < numColumns; ++j) lp.objective[j] = -columns[j].objective / objScale;

        for (int b = 0; b < numBlocks; ++b) {
            LpRow row;
            row.rhs = 1.0;
            for (int j = 0; j < numColumns; ++j)
                if (columns[j].block == b) row.coeffs[j] = 1.0;
            lp.equalities.push_back(row);
        }

        // Подсказываем допустимый стартовый базис: тривиальный план каждого блока
        // (столбцы 0..numBlocks-1) в строке его условия выпуклости. Связующие строки
        // при этом выполняются, потому что тривиальные планы ничего не передают.
        if (!std::getenv("PDA_NOWARM")) {
            lp.initialBasis.assign(numLinkRows + numCouplings + numBlocks, -1);
            for (int b = 0; b < numBlocks; ++b)
                lp.initialBasis[numLinkRows + numCouplings + b] = b;
        }

        master = solveLp(lp);
        if (master.status == LpStatus::IterationLimit) {
            return recover(LpStatus::IterationLimit,
                           "План допустим, но оптимальность не доказана: симплекс исчерпал "
                           "лимит шагов на вырожденной главной задаче");
        }
        if (master.status != LpStatus::Optimal) {
            Solution solution;
            solution.status = master.status;
            solution.message = "Главная задача: " + master.message;
            return solution;
        }

        lambda.assign(master.x.begin(), master.x.begin() + numColumns);

        if (std::getenv("PDA_DEBUG")) {
            std::vector<double> perBlock(numBlocks, 0.0);
            for (int j = 0; j < numColumns; ++j) perBlock[columns[j].block] += lambda[j];
            std::fprintf(stderr, "[Д-В] итерация %d: суммы весов по блокам:", iteration);
            for (double value : perBlock) std::fprintf(stderr, " %.6f", value);
            std::fprintf(stderr, "\n");

            double worst = 0.0;
            for (int c = 0; c < numCouplings; ++c) {
                double sum = 0.0;
                for (int j = 0; j < numColumns; ++j)
                    for (const auto& [coupling, value] : columns[j].coupling)
                        if (coupling == c) sum += lambda[j] * value;
                worst = std::max(worst, std::fabs(sum));
            }
            std::fprintf(stderr, "[Д-В] максимальная невязка связующих строк: %.6g\n", worst);
        }

        // Двойственные оценки: сначала связующие равенства, затем условия выпуклости.
        // Возвращаем их в исходный масштаб, чтобы подзадачи считались в тех же единицах.
        // Порядок строк в решателе: сначала неравенства (каналы), затем равенства
        // (приём и условия выпуклости).
        std::vector<double> linkPrices(numLinkRows, 0.0);
        for (int r = 0; r < numLinkRows; ++r)
            linkPrices[r] = master.duals[r] * objScale / linkScale[r];
        std::vector<double> prices(numCouplings, 0.0);
        for (int c = 0; c < numCouplings; ++c)
            prices[c] = master.duals[numLinkRows + c] * objScale / rowScale[c];
        std::vector<double> convexity(numBlocks, 0.0);
        for (int b = 0; b < numBlocks; ++b)
            convexity[b] = master.duals[numLinkRows + numCouplings + b] * objScale;

        if (smoothPrices.empty()) {
            smoothPrices = prices;
            smoothLinkPrices = linkPrices;
        } else {
            for (int c = 0; c < numCouplings; ++c)
                smoothPrices[c] = alpha * prices[c] + (1.0 - alpha) * smoothPrices[c];
            for (int r = 0; r < numLinkRows; ++r)
                smoothLinkPrices[r] = alpha * linkPrices[r] + (1.0 - alpha) * smoothLinkPrices[r];
        }

        // ---- Генерация столбцов ----
        // Сначала планы ищутся по сглаженным ценам. Если так ничего выгодного не
        // нашлось, это ещё не оптимум: проверяем точными ценами, иначе метод
        // останавливается раньше времени и выдаёт заниженный критерий.
        int added = 0;
        for (int pass = 0; pass < 2 && added == 0; ++pass) {
        const std::vector<double>& usedPrices = pass == 0 ? smoothPrices : prices;
        const std::vector<double>& usedLinkPrices = pass == 0 ? smoothLinkPrices : linkPrices;
        for (int b = 0; b < numBlocks; ++b) {
            const LpResult sub = solveSubproblem(b, usedPrices, usedLinkPrices);
            if (sub.status != LpStatus::Optimal) continue;

            // Выгодность столбца считается по точным ценам: сглаженные нужны только
            // чтобы выбрать, какой план предложить.
            double reduced = -blockObjective(b, sub.x) - convexity[b];
            for (const auto& [coupling, column, coeff] : blocks[b].couplingEntries)
                if (column < static_cast<int>(sub.x.size()))
                    reduced -= prices[coupling] * coeff * sub.x[column];
            for (const auto& [row, column, coeff] : blocks[b].linkEntries)
                if (column < static_cast<int>(sub.x.size()))
                    reduced -= linkPrices[row] * coeff * sub.x[column];
            if (reduced < -options.tolerance) {
                columns.push_back({b, sub.x, blockObjective(b, sub.x),
                                   couplingContribution(b, sub.x), linkContribution(b, sub.x)});
                ++added;
            }
        }
        }
        stats.columnsGenerated = static_cast<int>(columns.size());

        if (options.verbose) {
            double value = 0.0;
            for (int j = 0; j < numColumns; ++j) value += lambda[j] * columns[j].objective;
            std::cout << "  итерация " << iteration << ": столбцов " << columns.size()
                      << ", критерий " << value << ", добавлено " << added << "\n";
        }

        if (added == 0) break;

        double current = 0.0;
        for (int j = 0; j < numColumns; ++j) current += lambda[j] * columns[j].objective;
        if (current > bestObjective + 1e-6 * std::max(1.0, std::fabs(bestObjective))) {
            bestObjective = current;
            stalled = 0;
        } else if (++stalled >= stallLimit) {
            return recover(LpStatus::IterationLimit,
                           "План допустим, но оптимальность не доказана: " +
                               std::to_string(stallLimit) +
                               " итераций подряд без роста критерия");
        }
    }

    return recover(LpStatus::Optimal, "Оптимум найден (Данциг-Вульф)");
}

}  // namespace pdaplan