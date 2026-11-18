// Простой двухфазный симплекс-метод (плотный табличный вариант).
//
// Решает задачу вида
//     min c^T x,  A_eq x = b_eq,  A_ub x <= b_ub,  x >= 0.
//
// Реализация учебная: матрица хранится плотно, поэтому подходит для задач
// примерно до нескольких тысяч переменных. Для промышленных размеров
// сюда следует подставить HiGHS или другой внешний решатель.
#pragma once

#include <map>
#include <string>
#include <vector>

namespace pdaplan {

struct LpRow {
    std::map<int, double> coeffs;  // индекс переменной -> коэффициент
    double rhs = 0.0;
};

struct LpProblem {
    int numVars = 0;
    std::vector<double> objective;  // минимизируемый вектор c
    std::vector<LpRow> equalities;
    std::vector<LpRow> inequalities;  // вида <= rhs

    // Необязательный стартовый базис: для каждой строки номер столбца либо -1,
    // если выбор оставлен решателю. Порядок строк: сначала неравенства, затем
    // равенства. Если заданный базис допустим, первая фаза не выполняется —
    // это снимает вырожденный поиск допустимой точки там, где она уже известна.
    std::vector<int> initialBasis;
};

enum class LpStatus { Optimal, Infeasible, Unbounded, IterationLimit };

struct LpResult {
    LpStatus status = LpStatus::Infeasible;
    std::string message;
    double objective = 0.0;
    std::vector<double> x;
    // Двойственные оценки строк в порядке: сначала неравенства, затем равенства.
    // Нужны методу Данцига-Вульфа: это цены на связующие ограничения.
    std::vector<double> duals;
};

LpResult solveLp(const LpProblem& problem);

}  // namespace pdaplan