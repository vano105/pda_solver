// Точка входа: XML задачи -> проверка валидатором -> решение ЛП -> план в BPMN.
//
//   ./pda_solver examples/sample_task.xml -o plan.bpmn
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>

#include "bpmn.h"
#include "check.h"
#include "lp.h"
#include "parser.h"

namespace {

struct Options {
    std::string input;
    std::string output = "plan.bpmn";
    std::string validator = "pda_validator.py";
    bool skipValidation = false;
    bool quiet = false;
};

void usage(const char* program) {
    std::cout << "Использование: " << program
              << " <входной.xml> [-o план.bpmn] [--skip-validation] [--quiet]\n"
              << "               [--validator путь/к/pda_validator.py]\n";
}

bool parseOptions(int argc, char** argv, Options& options) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-o" && i + 1 < argc) {
            options.output = argv[++i];
        } else if (arg == "--validator" && i + 1 < argc) {
            options.validator = argv[++i];
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

        const pdaplan::Solution solution = pdaplan::buildAndSolve(task);
        std::cout << "Статус решателя: " << solution.message << "\n"
                  << "Значение критерия: " << std::fixed << std::setprecision(4)
                  << solution.objective << "\n";

        if (solution.status != pdaplan::LpStatus::Optimal) {
            std::cout << "Оптимальное решение не получено.\n";
            return 2;
        }

        if (!options.quiet) {
            std::cout << "\nНенулевые переменные:\n";
            for (const auto& [key, value] : solution.nonzero())
                std::cout << "  " << key.str() << " = " << value << "\n";
        }

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
