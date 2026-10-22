// Независимая проверка найденного плана: баланс потоков и лимиты времени.
#pragma once

#include <string>
#include <vector>

#include "lp.h"
#include "model.h"

namespace pdaplan {

// Возвращает список нарушений (пустой — план корректен).
std::vector<std::string> checkSolution(const Task& task, const Solution& solution);

}  // namespace pdaplan
