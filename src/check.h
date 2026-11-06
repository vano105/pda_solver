// Независимая проверка найденного плана: баланс потоков и лимиты времени.
#pragma once

#include <string>
#include <vector>

#include "lp.h"
#include "model.h"

namespace pdaplan {

// Возвращает список нарушений (пустой — план корректен).
std::vector<std::string> checkSolution(const Task& task, const Solution& solution);

// Итог плана: куда делись все поступившие данные.
struct BalanceSummary {
    double input = 0.0;        // поступило извне за весь горизонт
    double delivered = 0.0;    // выдано потребителю
    double storedAtEnd = 0.0;  // осталось в ЗУ на конец горизонта
    double lost = 0.0;         // потеряно
    // Изменение объёма при обработке: технология может сжимать поток (положительное
    // значение) или наращивать его (отрицательное). Без этой поправки баланс не сойдётся.
    double converted = 0.0;

    double accounted() const { return delivered + storedAtEnd + lost + converted; }
    // Расхождение с поступившим объёмом; должно быть нулевым.
    double residual() const { return input - accounted(); }
};

BalanceSummary summarize(const Task& task, const Solution& solution);

}  // namespace pdaplan