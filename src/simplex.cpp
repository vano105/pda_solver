#include "simplex.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

// Ревизованный симплекс-метод с разреженным хранением столбцов.
//
// Матрица ограничений разрежена: в строке баланса участвуют лишь несколько величин,
// а остальные тысячи переменных входят с нулевым коэффициентом — на реальных данных
// заполнено около 0,1 % ячеек. Поэтому таблица целиком не хранится: столбцы лежат
// списками ненулевых элементов, а вместо таблицы ведётся обратная матрица базиса.
//
// Стоимость шага: O(m²) на пересчёт обратной матрицы и O(ненулевых) на оценку
// столбцов — вместо O(m · n) на пересчёт всей таблицы в плотной реализации.

namespace pdaplan {
namespace {

constexpr double kZero = 1e-12;
constexpr double kPivotEps = 1e-7;
constexpr double kFeasEps = 1e-7;

// Столбец матрицы: только ненулевые элементы.
struct Column {
    std::vector<int> rows;
    std::vector<double> values;

    void add(int row, double value) {
        rows.push_back(row);
        values.push_back(value);
    }
    size_t size() const { return rows.size(); }
};

// Базис: обратная матрица B⁻¹ и номера базисных столбцов.
class Basis {
public:
    Basis(int m, std::vector<int> columns) : m_(m), columns_(std::move(columns)) {
        inverse_.assign(static_cast<size_t>(m) * m, 0.0);
        for (int i = 0; i < m; ++i) inverse_[index(i, i)] = 1.0;
    }

    int column(int row) const { return columns_[row]; }
    const std::vector<int>& columnsView() const { return columns_; }

    // y = c_B · B⁻¹ — двойственные оценки строк.
    std::vector<double> prices(const std::vector<double>& cost) const {
        std::vector<double> y(m_, 0.0);
        for (int row = 0; row < m_; ++row) {
            const double weight = cost[columns_[row]];
            if (std::fabs(weight) < kZero) continue;
            const double* line = &inverse_[index(row, 0)];
            for (int j = 0; j < m_; ++j) y[j] += weight * line[j];
        }
        return y;
    }

    // d = B⁻¹ · a — как изменятся базисные переменные при вводе столбца a.
    std::vector<double> direction(const Column& a) const {
        std::vector<double> d(m_, 0.0);
        for (size_t k = 0; k < a.size(); ++k) {
            const int column = a.rows[k];
            const double value = a.values[k];
            if (std::fabs(value) < kZero) continue;
            for (int row = 0; row < m_; ++row) d[row] += value * inverse_[index(row, column)];
        }
        return d;
    }

    // Замена базисного столбца строки pivot на entering.
    void replace(int pivot, int entering, const std::vector<double>& d,
                 std::vector<double>& values) {
        const double lead = d[pivot];
        double* pivotLine = &inverse_[index(pivot, 0)];
        for (int j = 0; j < m_; ++j) pivotLine[j] /= lead;
        values[pivot] /= lead;

        for (int row = 0; row < m_; ++row) {
            if (row == pivot) continue;
            const double factor = d[row];
            if (std::fabs(factor) < kZero) continue;
            double* line = &inverse_[index(row, 0)];
            for (int j = 0; j < m_; ++j) line[j] -= factor * pivotLine[j];
            values[row] -= factor * values[pivot];
        }
        columns_[pivot] = entering;
    }

    // Пересборка обратной матрицы из исходных столбцов базиса. Произведение
    // элементарных преобразований накапливает ошибку округления, из-за чего на
    // вырожденных задачах метод начинает буксовать на мнимых улучшениях.
    bool refactor(const std::vector<Column>& columns) {
        std::vector<double> work(static_cast<size_t>(m_) * 2 * m_, 0.0);
        const int width = 2 * m_;
        auto at = [&](int row, int column) -> double& {
            return work[static_cast<size_t>(row) * width + column];
        };

        for (int column = 0; column < m_; ++column) {
            const Column& source = columns[columns_[column]];
            for (size_t k = 0; k < source.size(); ++k)
                at(source.rows[k], column) = source.values[k];
        }
        for (int row = 0; row < m_; ++row) at(row, m_ + row) = 1.0;

        for (int column = 0; column < m_; ++column) {
            int pivot = -1;
            double best = 1e-10;
            for (int row = column; row < m_; ++row) {
                const double value = std::fabs(at(row, column));
                if (value > best) {
                    best = value;
                    pivot = row;
                }
            }
            if (pivot < 0) return false;  // базис вырожден, оставляем прежнюю матрицу

            if (pivot != column)
                for (int j = 0; j < width; ++j) std::swap(at(column, j), at(pivot, j));

            const double lead = at(column, column);
            for (int j = 0; j < width; ++j) at(column, j) /= lead;

            for (int row = 0; row < m_; ++row) {
                if (row == column) continue;
                const double factor = at(row, column);
                if (std::fabs(factor) < kZero) continue;
                for (int j = 0; j < width; ++j) at(row, j) -= factor * at(column, j);
            }
        }

        for (int row = 0; row < m_; ++row)
            for (int j = 0; j < m_; ++j) inverse_[index(row, j)] = at(row, m_ + j);
        return true;
    }

    // Значения базисных переменных, пересчитанные из исходной правой части.
    std::vector<double> solve(const std::vector<double>& rhs) const {
        std::vector<double> x(m_, 0.0);
        for (int row = 0; row < m_; ++row) {
            const double* line = &inverse_[index(row, 0)];
            double sum = 0.0;
            for (int j = 0; j < m_; ++j) sum += line[j] * rhs[j];
            x[row] = sum;
        }
        return x;
    }

private:
    size_t index(int row, int column) const { return static_cast<size_t>(row) * m_ + column; }

    int m_;
    std::vector<int> columns_;
    std::vector<double> inverse_;
};

struct Phase {
    bool optimal = false;
    bool unbounded = false;
    bool limit = false;
};

// Одна фаза метода. allowed — какие столбцы разрешено вводить в базис.
Phase runPhase(const std::vector<Column>& columns, const std::vector<double>& rhs,
               const std::vector<double>& cost, const std::vector<bool>& allowed,
               Basis& basis, std::vector<double>& values) {
    const int m = static_cast<int>(rhs.size());
    const int n = static_cast<int>(columns.size());
    const long long limit = 50LL * (m + n) + 20000;

    double costScale = 1.0;
    for (double value : cost) costScale = std::max(costScale, std::fabs(value));
    const double dualEps = 1e-9 * costScale;

    std::vector<bool> inBasis(n, false);
    for (int row = 0; row < m; ++row) inBasis[basis.column(row)] = true;

    long long degenerate = 0;
    Phase phase;

    for (long long iteration = 1; iteration <= limit; ++iteration) {
        // Накопленная ошибка округления снимается пересборкой базиса. На больших
        // задачах пересборка дорога (O(m³)), там ограничиваемся пересчётом значений.
        if (iteration % 200 == 0) {
            if (m <= 2000) basis.refactor(columns);
            values = basis.solve(rhs);
        }

        const std::vector<double> y = basis.prices(cost);
        // Три режима выбора столбца: обычный (наибольшая приведённая стоимость),
        // нормированный при затяжной серии вырожденных шагов и правило Блэнда как
        // крайняя мера против зацикливания.
        const bool bland = std::getenv("PDA_BLAND") ? true : degenerate > 2 * m;
        const bool normalized = degenerate > m / 4 + 8;

        int entering = -1;
        double best = 0.0;
        for (int j = 0; j < n; ++j) {
            if (!allowed[j] || inBasis[j]) continue;
            double reduced = cost[j];
            const Column& column = columns[j];
            for (size_t k = 0; k < column.size(); ++k)
                reduced -= y[column.rows[k]] * column.values[k];
            if (reduced >= -dualEps) continue;

            if (bland) {  // правило Блэнда: первый подходящий столбец
                entering = j;
                break;
            }
            // При вырождении приведённая стоимость нормируется длиной столбца: иначе
            // метод предпочитает «длинные» столбцы, делает крошечные шаги и
            // зацикливается на главной задаче декомпозиции.
            double score = reduced;
            if (normalized) {
                double norm = 1.0;
                for (size_t k = 0; k < column.size(); ++k)
                    norm += column.values[k] * column.values[k];
                score = reduced / std::sqrt(norm);
            }
            if (score < best) {
                entering = j;
                best = score;
            }
        }
        if (entering < 0) {
            phase.optimal = true;
            if (std::getenv("PDA_TRACE"))
                std::fprintf(stderr, "[фаза] оптимум за %lld шагов (строк %d)\n", iteration, m);
            return phase;
        }

        const std::vector<double> d = basis.direction(columns[entering]);

        int pivot = -1;
        double bestRatio = 0.0;
        for (int row = 0; row < m; ++row) {
            if (d[row] <= kPivotEps) continue;
            const double ratio = values[row] / d[row];
            if (pivot < 0) {
                pivot = row;
                bestRatio = ratio;
                continue;
            }
            const double tol = 1e-9 * std::max(1.0, std::fabs(bestRatio));
            if (ratio < bestRatio - tol) {
                pivot = row;
                bestRatio = ratio;
            } else if (ratio <= bestRatio + tol && basis.column(row) < basis.column(pivot)) {
                pivot = row;
                bestRatio = std::min(bestRatio, ratio);
            }
        }
        if (pivot < 0) {
            phase.unbounded = true;
            return phase;
        }

        degenerate = bestRatio <= kFeasEps ? degenerate + 1 : 0;
        inBasis[basis.column(pivot)] = false;
        inBasis[entering] = true;
        basis.replace(pivot, entering, d, values);
    }

    phase.limit = true;
    if (std::getenv("PDA_TRACE"))
        std::fprintf(stderr, "[фаза] ЛИМИТ %lld шагов (строк %d, столбцов %d)\n", limit, m, n);
    return phase;
}

}  // namespace

LpResult solveLp(const LpProblem& problem) {
    const int numVars = problem.numVars;
    const int numUb = static_cast<int>(problem.inequalities.size());
    const int numEq = static_cast<int>(problem.equalities.size());
    const int m = numUb + numEq;

    if (m == 0) {
        for (int j = 0; j < numVars && j < static_cast<int>(problem.objective.size()); ++j)
            if (problem.objective[j] < 0.0)
                return {LpStatus::Unbounded, "Целевая функция не ограничена", 0.0, {}, {}};
        return {LpStatus::Optimal, "Оптимум найден", 0.0, std::vector<double>(numVars, 0.0), {}};
    }

    // Правая часть приводится к неотрицательной; знак строки запоминается для оценок.
    std::vector<double> rhs(m, 0.0);
    std::vector<bool> flipped(m, false);
    for (int i = 0; i < numUb; ++i) rhs[i] = problem.inequalities[i].rhs;
    for (int i = 0; i < numEq; ++i) rhs[numUb + i] = problem.equalities[i].rhs;
    for (int i = 0; i < m; ++i)
        if (rhs[i] < 0.0) {
            rhs[i] = -rhs[i];
            flipped[i] = true;
        }

    // Столбцы: исходные переменные, затем слэки неравенств, затем искусственные.
    std::vector<Column> columns(numVars);
    auto fill = [&](int row, const LpRow& source) {
        const double sign = flipped[row] ? -1.0 : 1.0;
        for (const auto& [var, value] : source.coeffs)
            if (var >= 0 && var < numVars && std::fabs(value) > kZero)
                columns[var].add(row, sign * value);
    };
    for (int i = 0; i < numUb; ++i) fill(i, problem.inequalities[i]);
    for (int i = 0; i < numEq; ++i) fill(numUb + i, problem.equalities[i]);

    std::vector<int> basisColumns(m, -1);
    for (int i = 0; i < numUb; ++i) {
        Column slack;
        slack.add(i, flipped[i] ? -1.0 : 1.0);
        columns.push_back(slack);
        // Слэк с коэффициентом +1 сразу становится базисным.
        if (!flipped[i]) basisColumns[i] = static_cast<int>(columns.size()) - 1;
    }

    const int artificialStart = static_cast<int>(columns.size());
    for (int i = 0; i < m; ++i) {
        if (basisColumns[i] >= 0) continue;
        Column artificial;
        artificial.add(i, 1.0);
        columns.push_back(artificial);
        basisColumns[i] = static_cast<int>(columns.size()) - 1;
    }
    const int n = static_cast<int>(columns.size());

    std::vector<bool> isArtificial(n, false);
    for (int j = artificialStart; j < n; ++j) isArtificial[j] = true;

    const std::vector<int> basisColumnsOriginal = basisColumns;

    if (std::getenv("PDA_SIZES")) {
        size_t nonzeros = 0;
        for (const Column& column : columns) nonzeros += column.size();
        std::fprintf(stderr, "[решатель] строк %d, столбцов %d, ненулевых %zu (%.3f %% ячеек)\n",
                     m, n, nonzeros, 100.0 * nonzeros / (static_cast<double>(m) * n));
    }

    // Стартовый базис, подсказанный вызывающей стороной.
    bool warmStart = false;
    if (static_cast<int>(problem.initialBasis.size()) == m) {
        std::vector<bool> taken(n, false);
        std::vector<int> candidate = basisColumns;
        bool usable = true;
        for (int row = 0; row < m && usable; ++row) {
            const int column = problem.initialBasis[row];
            if (column < 0 || column >= n || taken[column]) continue;
            taken[column] = true;
            candidate[row] = column;
        }
        if (usable) {
            basisColumns = candidate;
            warmStart = true;
        }
    }

    Basis basis(m, basisColumns);
    std::vector<double> values = rhs;  // единичный базис: x_B = b

    if (warmStart) {
        if (std::getenv("PDA_DEBUG"))
            std::fprintf(stderr, "[решатель] предложен стартовый базис, строк %d\n", m);
        if (basis.refactor(columns)) {
            values = basis.solve(rhs);
            bool feasible = true;
            for (int row = 0; row < m; ++row) {
                if (values[row] < -kFeasEps) feasible = false;
                if (isArtificial[basis.column(row)] && values[row] > kFeasEps) feasible = false;
            }
            if (std::getenv("PDA_DEBUG"))
                std::fprintf(stderr, "[решатель] стартовый базис %s\n",
                             feasible ? "принят" : "отвергнут (недопустим)");
            if (!feasible) {  // подсказка не подошла — возвращаемся к обычному старту
                basis = Basis(m, basisColumnsOriginal);
                values = rhs;
                warmStart = false;
            }
        } else {
            basis = Basis(m, basisColumnsOriginal);
            values = rhs;
            warmStart = false;
        }
    }

    // ---- Фаза 1: минимизируем сумму искусственных переменных ----
    std::vector<bool> allowed(n, true);
    bool hitLimit = false;

    // Фаза 1 выполняется всегда: с подсказанного базиса она завершается сразу,
    // но именно в ней искусственные переменные выводятся из базиса.
    if (artificialStart < n) {
        std::vector<double> phase1(n, 0.0);
        for (int j = artificialStart; j < n; ++j) phase1[j] = 1.0;

        // С допустимого стартового базиса цель первой фазы уже равна нулю, искать
        // нечего: сразу переходим к выводу искусственных переменных из базиса.
        if (!warmStart) {
            const Phase result = runPhase(columns, rhs, phase1, allowed, basis, values);
            if (result.unbounded)
                return {LpStatus::Unbounded, "Фаза 1 неограничена (некорректная модель)",
                        0.0, {}, {}};
            hitLimit = hitLimit || result.limit;
        }

        double residual = 0.0;
        for (int row = 0; row < m; ++row)
            if (isArtificial[basis.column(row)]) residual += std::max(0.0, values[row]);
        if (residual > 1e-6) {
            if (std::getenv("PDA_DEBUG"))
                std::fprintf(stderr,
                             "[решатель] несовместность: невязка фазы 1 %.6g, строк %d, "
                             "столбцов %d, лимит итераций %s\n",
                             residual, m, n, warmStart ? "старт был тёплым" : "возможен");
            return {LpStatus::Infeasible, "Ограничения несовместны", 0.0, {}, {}};
        }

        // Искусственные переменные выводятся из базиса: иначе во второй фазе они
        // снова получают ненулевые значения, и равенства перестают выполняться.
        std::vector<bool> inBasis(n, false);
        for (int row = 0; row < m; ++row) inBasis[basis.column(row)] = true;

        for (int row = 0; row < m; ++row) {
            if (!isArtificial[basis.column(row)]) continue;
            for (int j = 0; j < artificialStart; ++j) {
                if (inBasis[j]) continue;
                const std::vector<double> d = basis.direction(columns[j]);
                if (std::fabs(d[row]) <= kPivotEps) continue;
                inBasis[basis.column(row)] = false;
                inBasis[j] = true;
                basis.replace(row, j, d, values);
                break;
            }
        }

        for (int j = artificialStart; j < n; ++j) allowed[j] = false;
    }

    // ---- Фаза 2: исходная целевая функция ----
    std::vector<double> cost(n, 0.0);
    for (int j = 0; j < numVars && j < static_cast<int>(problem.objective.size()); ++j)
        cost[j] = problem.objective[j];

    const Phase result = runPhase(columns, rhs, cost, allowed, basis, values);
    if (result.unbounded)
        return {LpStatus::Unbounded, "Целевая функция не ограничена", 0.0, {}, {}};
    hitLimit = hitLimit || result.limit;

    std::vector<double> x(numVars, 0.0);
    for (int row = 0; row < m; ++row) {
        const int column = basis.column(row);
        if (column < numVars) x[column] = values[row];
    }
    for (double& value : x)
        if (value < 0.0 && value > -kFeasEps) value = 0.0;

    double objective = 0.0;
    for (int j = 0; j < numVars; ++j) objective += cost[j] * x[j];

    // Двойственные оценки строк; для перевёрнутых строк знак возвращается обратно.
    std::vector<double> duals = basis.prices(cost);
    for (int i = 0; i < m; ++i)
        if (flipped[i]) duals[i] = -duals[i];

    if (hitLimit)
        return {LpStatus::IterationLimit, "Достигнут лимит итераций", objective, x, duals};
    return {LpStatus::Optimal, "Оптимум найден", objective, x, duals};
}

}  // namespace pdaplan