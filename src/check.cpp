#include "check.h"

#include <cmath>
#include <sstream>

namespace pdaplan {
namespace {

constexpr double kTol = 1e-6;

std::string fmt(double value) {
    std::ostringstream out;
    out.setf(std::ios::fixed);
    out.precision(3);
    out << value;
    return out.str();
}

}  // namespace

std::vector<std::string> checkSolution(const Task& task, const Solution& solution) {
    std::vector<std::string> problems;
    std::map<std::pair<std::string, std::string>, double> stored;

    for (const Interval& interval : task.intervals) {
        const std::string& k = interval.id;

        std::map<std::pair<std::string, std::string>, double> incoming, outgoing;
        std::map<std::string, double> sendTime, recvTime, procTime;

        for (const Link& link : interval.links) {
            for (const auto& [sender, receiver] : link.orderedPairs()) {
                for (const std::string& techId : link.transports) {
                    auto tt = task.transports.find(techId);
                    if (tt == task.transports.end()) continue;
                    const double amount =
                        solution.value(transportVar(k, sender, receiver, techId));
                    if (amount <= 0.0) continue;

                    const TransportTech& tech = tt->second;
                    outgoing[{sender, tech.flow}] += amount;
                    incoming[{receiver, tech.flow}] += amount;
                    const double spent = amount * tech.timePerUnit();
                    sendTime[sender] += spent;
                    recvTime[receiver] += spent;
                }
            }
        }

        for (const Node& node : interval.nodes) {
            for (const std::string& techId : node.processes) {
                auto pt = task.processes.find(techId);
                if (pt == task.processes.end()) continue;
                const double amount = solution.value(processVar(k, node.id, techId));
                if (amount <= 0.0) continue;

                const ProcessTech& tech = pt->second;
                outgoing[{node.id, tech.inFlow}] += amount;
                incoming[{node.id, tech.outFlow}] += amount * tech.yieldRatio();
                procTime[node.id] += amount * tech.timePerUnit();
            }
        }

        std::map<std::pair<std::string, std::string>, double> nextStored;
        for (const Node& node : interval.nodes) {
            for (const auto& [techId, capacity] : node.storages) {
                auto st = task.storages.find(techId);
                if (st == task.storages.end()) continue;

                double total = 0.0;
                for (const std::string& flow : st->second.flows) {
                    const double amount = solution.value(storageVar(k, node.id, flow, techId));
                    total += amount;
                    nextStored[{node.id, flow}] += amount;
                }
                if (std::isfinite(capacity) && total > capacity + kTol) {
                    problems.push_back("интервал " + k + ", объект " + node.id + ": ЗУ " + techId +
                                       " переполнено (" + fmt(total) + " > " + fmt(capacity) + ")");
                }
            }
        }

        for (const Node& node : interval.nodes) {
            for (const std::string& flow : task.flows) {
                double left = incoming[{node.id, flow}] + stored[{node.id, flow}];
                auto in = node.inputs.find(flow);
                if (in != node.inputs.end()) left += in->second;

                double right = outgoing[{node.id, flow}] + nextStored[{node.id, flow}] +
                               solution.value(lostVar(k, node.id, flow));
                for (const std::string& out : node.outputs)
                    if (out == flow) right += solution.value(resultVar(k, node.id, flow));

                if (std::fabs(left - right) > kTol * std::max(1.0, std::fabs(left))) {
                    problems.push_back("интервал " + k + ", объект " + node.id + ", поток " + flow +
                                       ": баланс нарушен (" + fmt(left) + " != " + fmt(right) + ")");
                }
            }

            const struct {
                const char* name;
                double spent;
                double limit;
            } limits[] = {
                {"обработка", procTime[node.id], node.totalCapacity.value_or(interval.duration)},
                {"отправка", sendTime[node.id], node.sendCapacity.value_or(interval.duration)},
                {"приём", recvTime[node.id], node.receiveCapacity.value_or(interval.duration)},
            };
            for (const auto& item : limits) {
                if (item.spent > item.limit + kTol) {
                    problems.push_back("интервал " + k + ", объект " + node.id + ": " + item.name +
                                       " занимает " + fmt(item.spent) + " с при лимите " +
                                       fmt(item.limit) + " с");
                }
            }
        }

        stored = std::move(nextStored);
    }

    return problems;
}

}  // namespace pdaplan
