#pragma once

#include <limits>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace pdaplan {

struct ProcessTech {
    std::string id;
    double time = 1.0;
    std::string inFlow;
    double inSize = 1.0;
    std::string outFlow;
    double outSize = 1.0;

    double performance() const { return time > 0 ? inSize / time : 0.0; }
    double yieldRatio() const { return outSize / inSize; }
    double timePerUnit() const { return time / inSize; }
};

struct TransportTech {
    std::string id;
    double time = 1.0;
    std::string flow;
    double size = 1.0;

    double performance() const { return time > 0 ? size / time : 0.0; }
    double timePerUnit() const { return time / size; }
};

struct StorageTech {
    std::string id;
    std::vector<std::string> flows;
};

struct Node {
    std::string id;
    std::map<std::string, double> inputs;
    std::vector<std::string> outputs;
    std::vector<std::string> processes;
    std::map<std::string, double> storages;
    std::optional<double> sendCapacity;
    std::optional<double> receiveCapacity;
    std::optional<double> totalCapacity;
};

struct Link {
    std::string id1;
    std::string id2;
    std::string direction = "both";
    std::vector<std::string> transports;

    std::vector<std::pair<std::string, std::string>> orderedPairs() const {
        if (direction == "forward") return {{id1, id2}};
        if (direction == "backward") return {{id2, id1}};
        if (direction == "none") return {};
        return {{id1, id2}, {id2, id1}};
    }
};

struct Interval {
    std::string id;
    double duration = 0.0;
    std::optional<double> start;
    std::optional<double> end;
    std::vector<Node> nodes;
    std::vector<Link> links;

    const Node* findNode(const std::string& nodeId) const {
        for (const auto& node : nodes)
            if (node.id == nodeId) return &node;
        return nullptr;
    }
};

struct VarRef {
    std::string kind;
    std::map<std::string, std::string> idx;

    std::string at(const std::string& name) const {
        auto it = idx.find(name);
        return it == idx.end() ? "*" : it->second;
    }
};

struct Selector {
    std::string id;
    double sign = 1.0;
    std::vector<VarRef> terms;
    std::vector<Selector> children;
};

struct Constraint {
    std::string id;
    std::string sign = "<=";
    double value = 0.0;
    std::vector<std::string> selectorIds;
};

struct Criterion {
    std::string sign = "MAX";
    std::vector<std::string> selectorIds;
};

struct Task {
    std::vector<std::string> flows;
    std::map<std::string, ProcessTech> processes;
    std::map<std::string, TransportTech> transports;
    std::map<std::string, StorageTech> storages;
    std::vector<Interval> intervals;
    std::map<std::string, Selector> selectors;
    Criterion criterion;
    std::vector<Constraint> constraints;

    std::map<std::string, std::pair<double, double>> absoluteTimes() const {
        std::map<std::string, std::pair<double, double>> result;
        double cursor = 0.0;
        for (const auto& interval : intervals) {
            if (interval.start && interval.end) {
                result[interval.id] = {*interval.start, *interval.end};
                cursor = *interval.end;
            } else {
                result[interval.id] = {cursor, cursor + interval.duration};
                cursor += interval.duration;
            }
        }
        return result;
    }
};

constexpr double kInfCapacity = std::numeric_limits<double>::infinity();

}  // namespace pdaplan
