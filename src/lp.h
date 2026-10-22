// Сборка задачи ЛП по описанию задачи планирования.
#pragma once

#include <map>
#include <string>
#include <tuple>

#include "model.h"
#include "simplex.h"

namespace pdaplan {

// Ключ переменной ЛП. Пустые поля не используются для данного вида переменной.
struct VarKey {
    std::string kind;      // transport | process | storage | lost | resultflow
    std::string interval;  // номер интервала
    std::string object;    // объект-владелец
    std::string object2;   // получатель (для передачи)
    std::string flow;      // тип потока
    std::string tech;      // технология

    bool operator<(const VarKey& other) const {
        return std::tie(kind, interval, object, object2, flow, tech) <
               std::tie(other.kind, other.interval, other.object, other.object2, other.flow,
                        other.tech);
    }

    std::string str() const;
};

VarKey transportVar(const std::string& interval, const std::string& from, const std::string& to,
                    const std::string& tech);
VarKey processVar(const std::string& interval, const std::string& object, const std::string& tech);
VarKey storageVar(const std::string& interval, const std::string& object, const std::string& flow,
                  const std::string& tech);
VarKey lostVar(const std::string& interval, const std::string& object, const std::string& flow);
VarKey resultVar(const std::string& interval, const std::string& object, const std::string& flow);

struct Solution {
    LpStatus status = LpStatus::Infeasible;
    std::string message;
    double objective = 0.0;
    std::map<VarKey, double> values;

    std::map<VarKey, double> nonzero(double eps = 1e-9) const;
    double value(const VarKey& key) const;
};

Solution buildAndSolve(const Task& task);

}  // namespace pdaplan
