#include "lp.h"

#include <algorithm>
#include <cmath>
#include <functional>

namespace pdaplan {
namespace {

// Слагаемое раскрытого селектора: переменная (или константа) и коэффициент.
struct Term {
    bool isConstant = false;
    VarKey key;
    double coeff = 1.0;
};

bool matches(const std::string& pattern, const std::string& value) {
    return pattern == "*" || pattern == value;
}

// Накопитель строк ЛП с именованными переменными.
class Builder {
public:
    int var(const VarKey& key) {
        auto it = index_.find(key);
        if (it != index_.end()) return it->second;
        const int id = static_cast<int>(keys_.size());
        index_[key] = id;
        keys_.push_back(key);
        return id;
    }

    void addEq(const std::map<int, double>& row, double rhs) {
        if (!row.empty()) problem_.equalities.push_back({row, rhs});
    }

    void addUb(const std::map<int, double>& row, double rhs) {
        if (!row.empty()) problem_.inequalities.push_back({row, rhs});
    }

    LpProblem& problem() { return problem_; }
    const std::vector<VarKey>& keys() const { return keys_; }

private:
    std::map<VarKey, int> index_;
    std::vector<VarKey> keys_;
    LpProblem problem_;
};

std::vector<Term> expandVar(const Task& task, const VarRef& ref);

std::vector<Term> expandSelector(const Task& task, const Selector& selector, double multiplier) {
    const double weight = multiplier * selector.sign;
    std::vector<Term> terms;

    for (const VarRef& ref : selector.terms) {
        for (Term term : expandVar(task, ref)) {
            term.coeff *= weight;
            terms.push_back(term);
        }
    }
    for (const Selector& child : selector.children) {
        for (const Term& term : expandSelector(task, child, weight)) terms.push_back(term);
    }
    return terms;
}

std::vector<Term> selectorTerms(const Task& task, const std::vector<std::string>& ids) {
    std::vector<Term> terms;
    for (const std::string& id : ids) {
        auto it = task.selectors.find(id);
        if (it == task.selectors.end()) continue;
        for (const Term& term : expandSelector(task, it->second, 1.0)) terms.push_back(term);
    }
    return terms;
}

// Раскрывает элемент-переменную (со звёздочками) в список переменных ЛП.
std::vector<Term> expandVar(const Task& task, const VarRef& ref) {
    std::vector<Term> out;

    for (const Interval& interval : task.intervals) {
        if (!matches(ref.at("interval"), interval.id)) continue;
        const std::string& k = interval.id;

        if (ref.kind == "time") {
            out.push_back({true, {}, interval.duration});
            continue;
        }

        for (const Node& node : interval.nodes) {
            if (!matches(ref.at("object"), node.id)) continue;

            if (ref.kind == "inputflow") {
                for (const auto& [flow, volume] : node.inputs)
                    if (matches(ref.at("flow"), flow)) out.push_back({true, {}, volume});

            } else if (ref.kind == "resultflow") {
                for (const std::string& flow : node.outputs)
                    if (matches(ref.at("flow"), flow))
                        out.push_back({false, resultVar(k, node.id, flow), 1.0});

            } else if (ref.kind == "lost") {
                for (const std::string& flow : task.flows)
                    if (matches(ref.at("flow"), flow))
                        out.push_back({false, lostVar(k, node.id, flow), 1.0});

            } else if (ref.kind == "storage") {
                for (const auto& [techId, capacity] : node.storages) {
                    (void)capacity;
                    auto st = task.storages.find(techId);
                    if (st == task.storages.end() || !matches(ref.at("tech"), techId)) continue;
                    for (const std::string& flow : st->second.flows)
                        if (matches(ref.at("flow"), flow))
                            out.push_back({false, storageVar(k, node.id, flow, techId), 1.0});
                }

            } else if (ref.kind == "process" || ref.kind == "process_all" ||
                       ref.kind == "to_process" || ref.kind == "from_process") {
                for (const std::string& techId : node.processes) {
                    auto pt = task.processes.find(techId);
                    if (pt == task.processes.end()) continue;
                    if (ref.kind != "process_all" && !matches(ref.at("tech"), techId)) continue;
                    const ProcessTech& tech = pt->second;
                    const VarKey key = processVar(k, node.id, techId);

                    if (ref.kind == "to_process") {
                        if (matches(ref.at("flow"), tech.inFlow)) out.push_back({false, key, 1.0});
                    } else if (ref.kind == "from_process") {
                        if (matches(ref.at("flow"), tech.outFlow))
                            out.push_back({false, key, tech.yieldRatio()});
                    } else {
                        out.push_back({false, key, 1.0});
                    }
                }

            } else if (ref.kind == "transport" || ref.kind == "transport_all" ||
                       ref.kind == "to_transport" || ref.kind == "from_transport") {
                for (const Link& link : interval.links) {
                    for (const auto& [sender, receiver] : link.orderedPairs()) {
                        if (sender != node.id) continue;
                        if (!matches(ref.at("to_object"), receiver)) continue;
                        for (const std::string& techId : link.transports) {
                            auto tt = task.transports.find(techId);
                            if (tt == task.transports.end()) continue;
                            if (ref.kind != "transport_all" && !matches(ref.at("tech"), techId))
                                continue;
                            if ((ref.kind == "to_transport" || ref.kind == "from_transport") &&
                                !matches(ref.at("flow"), tt->second.flow))
                                continue;
                            out.push_back(
                                {false, transportVar(k, sender, receiver, techId), 1.0});
                        }
                    }
                }
            }
        }
    }
    return out;
}

// Собирает ограничения: баланс потоков, ёмкость ЗУ, лимиты времени.
void buildStructure(const Task& task, Builder& builder) {
    // (объект, поток) -> слагаемые, переносимые на следующий интервал
    std::map<std::pair<std::string, std::string>, std::vector<VarKey>> carried;

    for (const Interval& interval : task.intervals) {
        const std::string& k = interval.id;

        std::map<std::pair<std::string, std::string>, std::map<int, double>> balance;
        std::map<std::pair<std::string, std::string>, double> constants;
        std::map<std::string, std::map<int, double>> sendTime, recvTime, procTime;

        auto addVar = [&](const std::string& nodeId, const std::string& flow, const VarKey& key,
                          double coeff) {
            const int column = builder.var(key);
            balance[{nodeId, flow}][column] += coeff;
        };
        auto addConst = [&](const std::string& nodeId, const std::string& flow, double value) {
            constants[{nodeId, flow}] += value;
            balance[{nodeId, flow}];  // строка должна существовать
        };

        // Поступление извне и остаток с прошлого интервала.
        for (const Node& node : interval.nodes) {
            for (const auto& [flow, volume] : node.inputs) addConst(node.id, flow, volume);
            for (const std::string& flow : task.flows) {
                auto it = carried.find({node.id, flow});
                if (it == carried.end()) continue;
                for (const VarKey& key : it->second) addVar(node.id, flow, key, 1.0);
            }
        }

        // Передачи по связям.
        for (const Link& link : interval.links) {
            for (const auto& [sender, receiver] : link.orderedPairs()) {
                if (!interval.findNode(sender) || !interval.findNode(receiver)) continue;
                for (const std::string& techId : link.transports) {
                    auto tt = task.transports.find(techId);
                    if (tt == task.transports.end()) continue;
                    const TransportTech& tech = tt->second;

                    const VarKey key = transportVar(k, sender, receiver, techId);
                    const int column = builder.var(key);
                    addVar(sender, tech.flow, key, -1.0);
                    addVar(receiver, tech.flow, key, +1.0);
                    sendTime[sender][column] = tech.timePerUnit();
                    recvTime[receiver][column] = tech.timePerUnit();
                }
            }
        }

        // Обработка.
        for (const Node& node : interval.nodes) {
            for (const std::string& techId : node.processes) {
                auto pt = task.processes.find(techId);
                if (pt == task.processes.end()) continue;
                const ProcessTech& tech = pt->second;

                const VarKey key = processVar(k, node.id, techId);
                const int column = builder.var(key);
                addVar(node.id, tech.inFlow, key, -1.0);
                addVar(node.id, tech.outFlow, key, +tech.yieldRatio());
                procTime[node.id][column] = tech.timePerUnit();
            }
        }

        // Хранение, потери, выдача результата.
        std::map<std::pair<std::string, std::string>, std::vector<VarKey>> nextCarried;
        for (const Node& node : interval.nodes) {
            for (const auto& [techId, capacity] : node.storages) {
                auto st = task.storages.find(techId);
                if (st == task.storages.end()) continue;

                std::map<int, double> capacityRow;
                for (const std::string& flow : st->second.flows) {
                    const VarKey key = storageVar(k, node.id, flow, techId);
                    addVar(node.id, flow, key, -1.0);
                    nextCarried[{node.id, flow}].push_back(key);
                    capacityRow[builder.var(key)] = 1.0;
                }
                if (std::isfinite(capacity)) builder.addUb(capacityRow, capacity);
            }

            for (const std::string& flow : task.flows)
                addVar(node.id, flow, lostVar(k, node.id, flow), -1.0);
            for (const std::string& flow : node.outputs)
                addVar(node.id, flow, resultVar(k, node.id, flow), -1.0);
        }

        // Уравнения баланса.
        for (const auto& [cell, row] : balance) {
            auto it = constants.find(cell);
            const double rhs = it == constants.end() ? 0.0 : -it->second;
            builder.addEq(row, rhs);
        }

        // Лимиты времени на интервале.
        for (const Node& node : interval.nodes) {
            const double procLimit = node.totalCapacity.value_or(interval.duration);
            const double sendLimit = node.sendCapacity.value_or(interval.duration);
            const double recvLimit = node.receiveCapacity.value_or(interval.duration);

            if (auto it = procTime.find(node.id); it != procTime.end())
                builder.addUb(it->second, procLimit);
            if (auto it = sendTime.find(node.id); it != sendTime.end())
                builder.addUb(it->second, sendLimit);
            if (auto it = recvTime.find(node.id); it != recvTime.end())
                builder.addUb(it->second, recvLimit);
        }

        carried = std::move(nextCarried);
    }
}

void addUserConstraints(const Task& task, Builder& builder) {
    for (const Constraint& constraint : task.constraints) {
        std::map<int, double> row;
        double rhs = constraint.value;

        for (const Term& term : selectorTerms(task, constraint.selectorIds)) {
            if (term.isConstant) {
                rhs -= term.coeff;
                continue;
            }
            row[builder.var(term.key)] += term.coeff;
        }
        if (row.empty()) continue;

        const std::string& sign = constraint.sign;
        if (sign == "<=" || sign == "less") {
            builder.addUb(row, rhs);
        } else if (sign == ">=" || sign == "more") {
            std::map<int, double> negated;
            for (const auto& [column, coeff] : row) negated[column] = -coeff;
            builder.addUb(negated, -rhs);
        } else if (sign == "==" || sign == "equally") {
            builder.addEq(row, rhs);
        }
    }
}

}  // namespace

std::string VarKey::str() const {
    std::string result = kind + "[" + interval;
    for (const std::string* field : {&object, &object2, &flow, &tech})
        if (!field->empty()) result += "," + *field;
    return result + "]";
}

VarKey transportVar(const std::string& interval, const std::string& from, const std::string& to,
                    const std::string& tech) {
    return {"transport", interval, from, to, "", tech};
}
VarKey processVar(const std::string& interval, const std::string& object, const std::string& tech) {
    return {"process", interval, object, "", "", tech};
}
VarKey storageVar(const std::string& interval, const std::string& object, const std::string& flow,
                  const std::string& tech) {
    return {"storage", interval, object, "", flow, tech};
}
VarKey lostVar(const std::string& interval, const std::string& object, const std::string& flow) {
    return {"lost", interval, object, "", flow, ""};
}
VarKey resultVar(const std::string& interval, const std::string& object, const std::string& flow) {
    return {"resultflow", interval, object, "", flow, ""};
}

std::map<VarKey, double> Solution::nonzero(double eps) const {
    std::map<VarKey, double> result;
    for (const auto& [key, value] : values)
        if (std::fabs(value) > eps) result[key] = value;
    return result;
}

double Solution::value(const VarKey& key) const {
    auto it = values.find(key);
    return it == values.end() ? 0.0 : it->second;
}

Solution buildAndSolve(const Task& task) {
    Builder builder;
    buildStructure(task, builder);
    addUserConstraints(task, builder);

    // Целевая функция: взвешенная сумма раскрытых селекторов критерия.
    std::map<int, double> objectiveRow;
    for (const Term& term : selectorTerms(task, task.criterion.selectorIds)) {
        if (term.isConstant) continue;
        objectiveRow[builder.var(term.key)] += term.coeff;
    }

    const bool maximize = task.criterion.sign == "MAX" || task.criterion.sign == "max";
    LpProblem& problem = builder.problem();
    problem.numVars = static_cast<int>(builder.keys().size());
    problem.objective.assign(problem.numVars, 0.0);
    for (const auto& [column, coeff] : objectiveRow)
        problem.objective[column] = maximize ? -coeff : coeff;

    const LpResult result = solveLp(problem);

    Solution solution;
    solution.status = result.status;
    solution.message = result.message;
    solution.objective = maximize ? -result.objective : result.objective;
    for (size_t j = 0; j < builder.keys().size() && j < result.x.size(); ++j)
        solution.values[builder.keys()[j]] = result.x[j];
    return solution;
}

}  // namespace pdaplan
