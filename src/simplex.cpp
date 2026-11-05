#include "simplex.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

namespace pdaplan {
namespace {

constexpr double kEps = 1e-9;
constexpr double kPivotEps = 1e-7;

// Симплекс-таблица: строки ограничений в канонической форме A x = b, x >= 0.
struct Tableau {
    std::vector<std::vector<double>> a;  // m x n
    std::vector<double> b;               // m
    std::vector<int> basis;              // базисный столбец каждой строки
    int m = 0;
    int n = 0;

    std::vector<std::vector<double>> a0;  // исходная матрица для рефакторизации
    std::vector<double> b0;

    // Пересчёт таблицы из исходной матрицы по текущему базису. Плотная таблица за
    // тысячи пивотов накапливает ошибку округления, и симплекс начинает буксовать
    // на мнимых улучшениях; рефакторизация возвращает точность.
    bool refactor() {
        std::vector<std::vector<double>> savedA = a;
        std::vector<double> savedB = b;
        const std::vector<int> wanted = basis;

        a = a0;
        b = b0;
        std::vector<bool> usedRow(m, false);

        for (int k = 0; k < m; ++k) {
            const int column = wanted[k];
            int bestRow = -1;
            double bestValue = 1e-9;
            for (int i = 0; i < m; ++i) {
                if (usedRow[i]) continue;
                const double value = std::fabs(a[i][column]);
                if (value > bestValue) {
                    bestValue = value;
                    bestRow = i;
                }
            }
            if (bestRow < 0) {  // базис вырожден — оставляем прежнюю таблицу
                a = std::move(savedA);
                b = std::move(savedB);
                basis = wanted;
                return false;
            }
            pivot(bestRow, column);
            usedRow[bestRow] = true;
        }
        return true;
    }

    void pivot(int row, int col) {
        const double factor = a[row][col];
        for (double& value : a[row]) value /= factor;
        b[row] /= factor;

        for (int i = 0; i < m; ++i) {
            if (i == row) continue;
            const double coeff = a[i][col];
            if (std::fabs(coeff) < kEps) continue;
            for (int j = 0; j < n; ++j) a[i][j] -= coeff * a[row][j];
            b[i] -= coeff * b[row];
        }
        basis[row] = col;
    }
};

// Одна фаза симплекса. allowed — какие столбцы разрешено вводить в базис.
// Возвращает false, если задача неограничена.
bool runPhase(Tableau& t, const std::vector<double>& cost,
              const std::vector<bool>& allowed, bool& hitLimit) {
    const long long limit = 2000LL * (t.m + t.n) + 100000;
    long long iteration = 0;

    // Допуск на приведённую стоимость масштабируется по целевой функции: при
    // коэффициентах порядка 10^5 абсолютный допуск 1e-9 тонет в шуме округления,
    // и симплекс бесконечно пивотирует на мнимых улучшениях.
    double costScale = 1.0;
    for (double value : cost) costScale = std::max(costScale, std::fabs(value));
    const double dualEps = 1e-9 * costScale;
    long long degenerate = 0;  // подряд идущие шаги с нулевым сдвигом
    hitLimit = false;

    while (true) {
        if (++iteration > limit) {
            hitLimit = true;
            if (getenv("PDA_DEBUG"))
                fprintf(stderr, "[simplex] лимит: m=%d n=%d итераций=%lld\n", t.m, t.n, iteration);
            return true;
        }
        // Правило Блэнда включается, как только появляется серия вырожденных шагов:
        // главная задача метода Данцига-Вульфа вырождена почти вся, и без этого
        // симплекс буксует на месте.
        const bool bland = getenv("PDA_BLAND") ? true : degenerate > t.m;

        // Приведённые стоимости: r_j = c_j - sum_i cB_i * a[i][j].
        int entering = -1;
        double best = -dualEps;
        for (int j = 0; j < t.n; ++j) {
            if (!allowed[j]) continue;
            double reduced = cost[j];
            for (int i = 0; i < t.m; ++i) {
                const double coeff = t.a[i][j];
                if (coeff != 0.0) reduced -= cost[t.basis[i]] * coeff;
            }
            if (reduced < best) {
                entering = j;
                best = reduced;
                if (bland) break;  // Блэнд: первый подходящий столбец
            }
        }
        if (entering < 0) return true;  // оптимум достигнут

        // Тест отношения: какая строка первой упрётся в ноль.
        int leaving = -1;
        double bestRatio = std::numeric_limits<double>::infinity();
        for (int i = 0; i < t.m; ++i) {
            const double coeff = t.a[i][entering];
            if (coeff <= kPivotEps) continue;
            const double ratio = t.b[i] / coeff;
            if (leaving < 0) {
                bestRatio = ratio;
                leaving = i;
                continue;
            }
            // Допуск относительный: при больших числах абсолютный тонет в шуме округления.
            const double tol = kEps * std::max(1.0, std::fabs(bestRatio));
            if (ratio < bestRatio - tol) {
                bestRatio = ratio;
                leaving = i;
            } else if (ratio <= bestRatio + tol && t.basis[i] < t.basis[leaving]) {
                bestRatio = std::min(bestRatio, ratio);
                leaving = i;  // правило Блэнда при равных отношениях
            }
        }
        if (leaving < 0) {
            if (getenv("PDA_DEBUG"))
                fprintf(stderr, "[simplex] неограничено: входит столбец %d, m=%d n=%d\n",
                        entering, t.m, t.n);
            return false;  // неограниченная задача
        }

        degenerate = bestRatio <= kEps ? degenerate + 1 : 0;
        t.pivot(leaving, entering);
        if (iteration % 200 == 0) t.refactor();
    }
}

}  // namespace

LpResult solveLp(const LpProblem& problem) {
    const int numVars = problem.numVars;
    const int numUb = static_cast<int>(problem.inequalities.size());
    const int numEq = static_cast<int>(problem.equalities.size());
    const int m = numUb + numEq;

    // Столбцы: исходные переменные, затем слэки неравенств, затем искусственные.
    const int slackOffset = numVars;
    const int artificialOffset = numVars + numUb;
    const int n = artificialOffset + m;

    Tableau t;
    t.m = m;
    t.n = n;
    t.a.assign(m, std::vector<double>(n, 0.0));
    t.b.assign(m, 0.0);
    t.basis.assign(m, -1);

    std::vector<bool> isArtificial(n, false);
    int artificialCount = 0;

    // Для двойственных оценок: единичный столбец каждой строки и признак смены знака.
    std::vector<int> identityColumn(m, -1);
    std::vector<bool> flipped(m, false);

    auto fillRow = [&](int row, const LpRow& source, int slackColumn) {
        for (const auto& [var, coeff] : source.coeffs) t.a[row][var] = coeff;
        t.b[row] = source.rhs;
        if (slackColumn >= 0) t.a[row][slackColumn] = 1.0;

        // Правая часть должна быть неотрицательной.
        if (t.b[row] < 0.0) {
            for (double& value : t.a[row]) value = -value;
            t.b[row] = -t.b[row];
            flipped[row] = true;
        }

        // Если слэк остался с коэффициентом +1, он и становится базисом.
        if (slackColumn >= 0 && t.a[row][slackColumn] > 0.0) {
            t.basis[row] = slackColumn;
            identityColumn[row] = slackColumn;
            return;
        }
        const int artificial = artificialOffset + artificialCount++;
        t.a[row][artificial] = 1.0;
        isArtificial[artificial] = true;
        t.basis[row] = artificial;
        identityColumn[row] = artificial;
    };

    for (int i = 0; i < numUb; ++i) fillRow(i, problem.inequalities[i], slackOffset + i);
    for (int i = 0; i < numEq; ++i) fillRow(numUb + i, problem.equalities[i], -1);

    t.a0 = t.a;
    t.b0 = t.b;

    // ---- Фаза 1: минимизируем сумму искусственных переменных ----
    std::vector<bool> allowed(n, true);
    bool hitLimit = false;

    if (artificialCount > 0) {
        std::vector<double> phase1Cost(n, 0.0);
        for (int j = 0; j < n; ++j)
            if (isArtificial[j]) phase1Cost[j] = 1.0;

        if (!runPhase(t, phase1Cost, allowed, hitLimit)) {
            return {LpStatus::Unbounded, "Фаза 1 неограничена (некорректная модель)", 0.0, {}, {}};
        }

        double residual = 0.0;
        for (int i = 0; i < m; ++i)
            if (isArtificial[t.basis[i]]) residual += t.b[i];
        if (residual > 1e-6) {
            return {LpStatus::Infeasible, "Ограничения несовместны", 0.0, {}, {}};
        }

        // Выводим искусственные переменные из базиса, где это возможно.
        for (int i = 0; i < m; ++i) {
            if (!isArtificial[t.basis[i]]) continue;
            for (int j = 0; j < artificialOffset; ++j) {
                if (std::fabs(t.a[i][j]) > kPivotEps) {
                    t.pivot(i, j);
                    break;
                }
            }
        }
        for (int j = artificialOffset; j < n; ++j) allowed[j] = false;
    }

    // ---- Фаза 2: исходная целевая функция ----
    std::vector<double> cost(n, 0.0);
    for (int j = 0; j < numVars && j < static_cast<int>(problem.objective.size()); ++j)
        cost[j] = problem.objective[j];

    if (!runPhase(t, cost, allowed, hitLimit)) {
        return {LpStatus::Unbounded, "Целевая функция не ограничена", 0.0, {}, {}};
    }

    std::vector<double> x(numVars, 0.0);
    for (int i = 0; i < m; ++i) {
        const int column = t.basis[i];
        if (column >= 0 && column < numVars) x[column] = t.b[i];
    }

    double objective = 0.0;
    for (int j = 0; j < numVars; ++j) objective += cost[j] * x[j];

    // Двойственная оценка строки i: y_i = -r_j, где j — единичный столбец этой строки,
    // r_j — его приведённая стоимость при целевой функции второй фазы.
    std::vector<double> duals(m, 0.0);
    for (int i = 0; i < m; ++i) {
        const int column = identityColumn[i];
        if (column < 0) continue;
        double reduced = cost[column];
        for (int row = 0; row < m; ++row) {
            const double coeff = t.a[row][column];
            if (coeff != 0.0) reduced -= cost[t.basis[row]] * coeff;
        }
        duals[i] = flipped[i] ? reduced : -reduced;
    }

    if (hitLimit) {
        return {LpStatus::IterationLimit, "Достигнут лимит итераций", objective, x, duals};
    }
    return {LpStatus::Optimal, "Оптимум найден", objective, x, duals};
}

}  // namespace pdaplan