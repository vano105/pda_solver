#pragma once

#include <string>
#include <vector>

#include "lp.h"
#include "model.h"

namespace pdaplan {

std::vector<std::string> checkSolution(const Task& task, const Solution& solution);

}  // namespace pdaplan
