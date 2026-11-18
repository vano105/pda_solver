// Точка входа: XML задачи -> проверка валидатором -> решение ЛП -> план в BPMN.
//
//   ./pda_solver examples/sample_task.xml -o plan.bpmn
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>

#include "bpmn.h"
#include "check.h"
#include "dw.h"
#include "lp.h"
#include "parser.h"

namespace {

struct Options {
    std::string input;
    std::string output = "plan.bpmn";
    std::string validator = "pda_validator.py";
    bool skipValidation = false;
    bool quiet = false;
    bool useDw = false;       // метод Данцига-Вульфа вместо монолитного симплекса
    bool dwByInterval = false;// разбиение по интервалам вместо разбиения по объектам
    bool verboseDw = false;   // печатать ход генерации столбцов
};

void usage(const char* program) {
    std::cout << "Использование: " << program
              << " <входной.xml> [-o план.bpmn] [--skip-validation] [--quiet]\n"
              << "               [--method simplex|dw|dwi] [--verbose]\n"
              << "               [--validator путь/к/pda_validator.py]\n";
}

bool parseOptions(int argc, char** argv, Options& options) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-o" && i + 1 < argc) {
            options.output = argv[++i];
        } else if (arg == "--validator" && i + 1 < argc) {
            options.validator = argv[++i];
        } else if (arg == "--method" && i + 1 < argc) {
            const std::string method = argv[++i];
            options.useDw = (method == "dw" || method == "dwi" || method == "dantzig-wolfe");
            options.dwByInterval = (method == "dwi");
        } else if (arg == "--verbose") {
            options.verboseDw = true;
        } else if (arg == "--skip-validation") {
            options.skipValidation = true;
        } else if (arg == "--quiet") {
            options.quiet = true;
        } else if (arg == "-h" || arg == "--help") {
            return false;
        } else if (options.input.empty()) {
            options.input = arg;
        }
    }
    return !options.input.empty();
}

// Запускает python-валидатор как внешний процесс.
bool runValidator(const Options& options) {
    const std::string command =
        "python3 " + options.validator + " " + options.input + " > /dev/null 2>&1";
    const int code = std::system(command.c_str());
    if (code == 0) {
        std::cout << "Валидация: ошибок нет.\n";
        return true;
    }
    std::cout << "Валидатор вернул код " << code
              << ". Подробности: python3 " << options.validator << " " << options.input << "\n";
    return false;
}

}  // namespace

int main(int argc, char** argv) {
    Options options;
    if (!parseOptions(argc, argv, options)) {
        usage(argv[0]);
        return 1;
    }

    if (!options.skipValidation && !runValidator(options)) {
        std::cout << "Во входном файле есть ошибки — решение не строится.\n";
        return 1;
    }

    try {
        const pdaplan::Task task = pdaplan::parseTask(options.input);
        std::cout << "Интервалов: " << task.intervals.size()
                  << ", потоков: " << task.flows.size()
                  << ", технологий обработки: " << task.processes.size()
                  << ", передачи: " << task.transports.size() << "\n";

        pdaplan::Solution solution;
        if (options.useDw) {
            pdaplan::DwOptions dwOptions;
            dwOptions.verbose = options.verboseDw;
            dwOptions.byInterval = options.dwByInterval;
            pdaplan::DwStats stats;
            solution = pdaplan::solveDantzigWolfe(task, dwOptions, stats);
            std::cout << "Метод: декомпозиция Данцига-Вульфа ("
                      << (options.dwByInterval ? "блоки — интервалы" : "блоки — объекты")
                      << ")\n"
                      << "Блоков: " << stats.blocks << ", связующих ограничений: "
                      << stats.couplings << ", итераций: " << stats.iterations
                      << ", сгенерировано столбцов: " << stats.columnsGenerated << "\n";
        } else {
            solution = pdaplan::buildAndSolve(task);
            std::cout << "Метод: монолитный симплекс\n";
        }
        std::cout << "Статус решателя: " << solution.message << "\n"
                  << "Значение критерия: " << std::fixed << std::setprecision(4)
                  << solution.objective << "\n";

        if (solution.status == pdaplan::LpStatus::IterationLimit && !solution.values.empty()) {
            // План есть и он допустим, не доказана только его оптимальность:
            // выводим его как обычно, но с предупреждением.
            std::cout << "ВНИМАНИЕ: оптимальность не доказана, план может быть хуже лучшего.\n";
        } else if (solution.status != pdaplan::LpStatus::Optimal) {
            std::cout << "Оптимальное решение не получено.\n";
            return 2;
        }

        if (!options.quiet) {
            std::cout << "\nНенулевые переменные:\n";
            for (const auto& [key, value] : solution.nonzero())
                std::cout << "  " << key.str() << " = " << value << "\n";
        }

        const pdaplan::BalanceSummary balance = pdaplan::summarize(task, solution);
        auto share = [&](double value) {
            return balance.input > 0 ? 100.0 * value / balance.input : 0.0;
        };
        std::cout << "\nБаланс данных:\n"
                  << "  поступило извне        " << std::setw(12) << balance.input << "\n"
                  << "  доставлено потребителю " << std::setw(12) << balance.delivered
                  << "  (" << std::setprecision(1) << share(balance.delivered) << " %)\n"
                  << std::setprecision(4)
                  << "  осталось в ЗУ          " << std::setw(12) << balance.storedAtEnd
                  << "  (" << std::setprecision(1) << share(balance.storedAtEnd) << " %)\n"
                  << std::setprecision(4)
                  << "  потеряно               " << std::setw(12) << balance.lost
                  << "  (" << std::setprecision(1) << share(balance.lost) << " %)\n"
                  << std::setprecision(4);
        if (std::fabs(balance.converted) > 1e-9)
            std::cout << "  изменение при обработке" << std::setw(12) << balance.converted
                      << "  (" << std::setprecision(1) << share(balance.converted) << " %)\n"
                      << std::setprecision(4);
        if (std::fabs(balance.residual()) > 1e-6 * std::max(1.0, balance.input))
            std::cout << "  ВНИМАНИЕ: расхождение " << balance.residual() << "\n";

        const auto problems = pdaplan::checkSolution(task, solution);
        if (problems.empty()) {
            std::cout << "Проверка плана: баланс потоков и ограничения по времени выполнены.\n";
        } else {
            std::cout << "\nПроверка плана выявила нарушения:\n";
            for (size_t i = 0; i < problems.size() && i < 20; ++i)
                std::cout << "  - " << problems[i] << "\n";
        }

        auto chains = pdaplan::buildPlan(task, solution);
        pdaplan::writeBpmn(chains, options.output);

        size_t total = 0;
        for (const auto& chain : chains) total += chain.size();
        std::cout << "\nПлан записан в " << options.output << ": цепочек " << chains.size()
                  << ", задач " << total << "\n";
    } catch (const std::exception& error) {
        std::cerr << "Ошибка: " << error.what() << "\n";
        return 3;
    }

    return 0;
}