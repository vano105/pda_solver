// Формирование плана информационного взаимодействия в формате BPMN (litsam).
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "lp.h"
#include "model.h"

namespace pdaplan {

// Одно действие плана.
struct PlanTask {
    std::string name;
    std::string resource;                 // id объекта-исполнителя
    double stream = 0.0;                  // объём потока
    double duration = 0.0;                // время выполнения, с
    std::optional<double> performance;    // производительность (объём/с)
    double windowStart = 0.0;             // абсолютные границы интервала
    double windowEnd = 0.0;
    bool hasWindow = true;
    std::string id;                       // заполняется при записи
};

using Chain = std::vector<PlanTask>;

// Собирает цепочки задач по интервалам.
std::vector<Chain> buildPlan(const Task& task, const Solution& solution);

// Записывает план в BPMN-файл.
void writeBpmn(std::vector<Chain>& chains, const std::string& path);

}  // namespace pdaplan
