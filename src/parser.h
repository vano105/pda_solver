// Разбор входного XML формата PDA.
#pragma once

#include <string>

#include "model.h"

namespace pdaplan {

// Читает файл задачи. Бросает std::runtime_error при структурных ошибках.
Task parseTask(const std::string& path);

}  // namespace pdaplan
