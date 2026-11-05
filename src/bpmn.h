#pragma once

#include <optional>
#include <string>
#include <vector>

#include "lp.h"
#include "model.h"

namespace pdaplan {

struct PlanTask {
    std::string name;
    std::string resource;
    double stream = 0.0;
    double duration = 0.0;
    std::optional<double> performance;
    double windowStart = 0.0;
    double windowEnd = 0.0;
    bool hasWindow = true;
    std::string id;
};

using Chain = std::vector<PlanTask>;

std::vector<Chain> buildPlan(const Task& task, const Solution& solution);

void writeBpmn(std::vector<Chain>& chains, const std::string& path);

}  // namespace pdaplan
