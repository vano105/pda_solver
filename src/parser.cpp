#include "parser.h"

#include <algorithm>
#include <cstdlib>
#include <stdexcept>

#include "tinyxml2.h"

namespace pdaplan {
namespace {

using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;

// Обязательные индексы переменных — совпадают с VAR_TYPES валидатора.
const std::map<std::string, std::vector<std::string>>& varTypes() {
    static const std::map<std::string, std::vector<std::string>> types = {
        {"inputflow", {"object", "flow", "interval"}},
        {"resultflow", {"object", "flow", "interval"}},
        {"process_all", {"object", "interval"}},
        {"to_process", {"object", "flow", "tech", "interval"}},
        {"process", {"object", "tech", "interval"}},
        {"from_process", {"object", "flow", "tech", "interval"}},
        {"transport_all", {"object", "to_object", "interval"}},
        {"to_transport", {"object", "to_object", "flow", "tech", "interval"}},
        {"transport", {"object", "to_object", "tech", "interval"}},
        {"from_transport", {"object", "to_object", "flow", "tech", "interval"}},
        {"storage", {"object", "flow", "tech", "interval"}},
        {"lost", {"object", "flow", "interval"}},
        {"time", {"interval"}},
    };
    return types;
}

std::string attr(const XMLElement* elem, const char* name, const std::string& fallback = "") {
    const char* value = elem->Attribute(name);
    return value ? std::string(value) : fallback;
}

double num(const std::string& text, double fallback = 0.0) {
    if (text.empty()) return fallback;
    try {
        return std::stod(text);
    } catch (const std::exception&) {
        return fallback;
    }
}

bool startsWith(const std::string& value, const std::string& prefix) {
    return value.size() >= prefix.size() && value.compare(0, prefix.size(), prefix) == 0;
}

void parseLibrary(const XMLElement* task, Task& out) {
    if (const XMLElement* flows = task->FirstChildElement("flows")) {
        for (const XMLElement* t = flows->FirstChildElement("type"); t;
             t = t->NextSiblingElement("type")) {
            out.flows.push_back(attr(t, "id"));
        }
    }

    if (const XMLElement* processes = task->FirstChildElement("process")) {
        for (const XMLElement* t = processes->FirstChildElement("type"); t;
             t = t->NextSiblingElement("type")) {
            const XMLElement* in = t->FirstChildElement("input");
            const XMLElement* out_elem = t->FirstChildElement("output");
            if (!in || !out_elem) continue;
            const XMLElement* inType = in->FirstChildElement("type");
            const XMLElement* outType = out_elem->FirstChildElement("type");
            if (!inType || !outType) continue;

            ProcessTech tech;
            tech.id = attr(t, "id");
            tech.time = num(attr(t, "time"), 1.0);
            tech.inFlow = attr(inType, "id");
            tech.inSize = num(attr(inType, "size"), 1.0);
            tech.outFlow = attr(outType, "id");
            tech.outSize = num(attr(outType, "size"), 1.0);
            out.processes[tech.id] = tech;
        }
    }

    if (const XMLElement* transports = task->FirstChildElement("transport")) {
        for (const XMLElement* t = transports->FirstChildElement("type"); t;
             t = t->NextSiblingElement("type")) {
            const XMLElement* in = t->FirstChildElement("input");
            if (!in) continue;
            const XMLElement* inType = in->FirstChildElement("type");
            if (!inType) continue;

            TransportTech tech;
            tech.id = attr(t, "id");
            tech.time = num(attr(t, "time"), 1.0);
            tech.flow = attr(inType, "id");
            tech.size = num(attr(inType, "size"), 1.0);
            out.transports[tech.id] = tech;
        }
    }

    if (const XMLElement* storages = task->FirstChildElement("storage")) {
        for (const XMLElement* s = storages->FirstChildElement("type"); s;
             s = s->NextSiblingElement("type")) {
            StorageTech tech;
            tech.id = attr(s, "id");
            if (const XMLElement* in = s->FirstChildElement("input")) {
                for (const XMLElement* t = in->FirstChildElement("type"); t;
                     t = t->NextSiblingElement("type")) {
                    tech.flows.push_back(attr(t, "id"));
                }
            }
            out.storages[tech.id] = tech;
        }
    }
}

void parseNode(const XMLElement* elem, Interval& interval) {
    Node node;
    node.id = attr(elem, "id");

    for (const tinyxml2::XMLAttribute* a = elem->FirstAttribute(); a; a = a->Next()) {
        const std::string name = a->Name();
        const std::string value = a->Value();
        if (name == "id") continue;

        if (startsWith(name, "input_")) {
            node.inputs[name.substr(6)] = num(value);
        } else if (startsWith(name, "output_")) {
            node.outputs.push_back(name.substr(7));
        } else if (startsWith(name, "process_")) {
            node.processes.push_back(name.substr(8));
        } else if (startsWith(name, "storage_")) {
            node.storages[name.substr(8)] = num(value, kInfCapacity);
        } else if (name == "send_capacity" || name == "receive_capacity" ||
                   name == "total_capacity") {
            // '*' означает "равно длительности интервала".
            const double limit = value == "*" ? interval.duration : num(value, kInfCapacity);
            if (name == "send_capacity") node.sendCapacity = limit;
            if (name == "receive_capacity") node.receiveCapacity = limit;
            if (name == "total_capacity") node.totalCapacity = limit;
        }
    }

    interval.nodes.push_back(std::move(node));
}

void parseStructs(const XMLElement* task, Task& out) {
    for (const XMLElement* s = task->FirstChildElement("struct"); s;
         s = s->NextSiblingElement("struct")) {
        Interval interval;
        interval.id = attr(s, "id");
        interval.duration = num(attr(s, "time"));
        if (s->Attribute("start_time")) interval.start = num(attr(s, "start_time"));
        if (s->Attribute("end_time")) interval.end = num(attr(s, "end_time"));

        for (const XMLElement* e = s->FirstChildElement("elem"); e;
             e = e->NextSiblingElement("elem")) {
            parseNode(e, interval);
        }

        for (const XMLElement* l = s->FirstChildElement("link"); l;
             l = l->NextSiblingElement("link")) {
            Link link;
            link.id1 = attr(l, "id1");
            link.id2 = attr(l, "id2");
            link.direction = attr(l, "direction", "both");
            for (const tinyxml2::XMLAttribute* a = l->FirstAttribute(); a; a = a->Next()) {
                const std::string name = a->Name();
                if (startsWith(name, "transport_")) link.transports.push_back(name.substr(10));
            }
            interval.links.push_back(std::move(link));
        }

        out.intervals.push_back(std::move(interval));
    }

    std::sort(out.intervals.begin(), out.intervals.end(),
              [](const Interval& a, const Interval& b) {
                  return std::atoi(a.id.c_str()) < std::atoi(b.id.c_str());
              });
}

Selector parseSelector(const XMLElement* elem) {
    Selector selector;
    selector.id = attr(elem, "id");
    selector.sign = num(attr(elem, "sign"), 1.0);

    for (const XMLElement* child = elem->FirstChildElement(); child;
         child = child->NextSiblingElement()) {
        const std::string tag = child->Name();
        if (tag == "selector") {
            selector.children.push_back(parseSelector(child));
            continue;
        }
        auto it = varTypes().find(tag);
        if (it == varTypes().end()) continue;

        VarRef ref;
        ref.kind = tag;
        for (const std::string& name : it->second) {
            ref.idx[name] = attr(child, name.c_str(), "*");
        }
        selector.terms.push_back(std::move(ref));
    }
    return selector;
}

void registerSelector(const Selector& selector, Task& out) {
    out.selectors[selector.id] = selector;
    for (const Selector& child : selector.children) registerSelector(child, out);
}

void parseObjective(const XMLElement* task, Task& out) {
    if (const XMLElement* selectors = task->FirstChildElement("selectors")) {
        for (const XMLElement* s = selectors->FirstChildElement("selector"); s;
             s = s->NextSiblingElement("selector")) {
            registerSelector(parseSelector(s), out);
        }
    }

    if (const XMLElement* crit = task->FirstChildElement("criterion")) {
        out.criterion.sign = attr(crit, "sign", "MAX");
        for (const XMLElement* r = crit->FirstChildElement("selector"); r;
             r = r->NextSiblingElement("selector")) {
            out.criterion.selectorIds.push_back(attr(r, "id"));
        }
    }

    if (const XMLElement* constraints = task->FirstChildElement("constraints")) {
        for (const XMLElement* c = constraints->FirstChildElement("constraint"); c;
             c = c->NextSiblingElement("constraint")) {
            Constraint constraint;
            constraint.id = attr(c, "id");
            constraint.sign = attr(c, "sign", "<=");
            constraint.value = num(attr(c, "value"));
            for (const XMLElement* r = c->FirstChildElement("selector"); r;
                 r = r->NextSiblingElement("selector")) {
                constraint.selectorIds.push_back(attr(r, "id"));
            }
            out.constraints.push_back(std::move(constraint));
        }
    }
}

}  // namespace

Task parseTask(const std::string& path) {
    XMLDocument doc;
    if (doc.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS) {
        throw std::runtime_error("Не удалось прочитать XML: " + path);
    }

    const XMLElement* root = doc.RootElement();
    if (!root) throw std::runtime_error("Пустой XML-документ");

    const XMLElement* task = std::string(root->Name()) == "XMLDocument"
                                 ? root->FirstChildElement("task")
                                 : root;
    if (!task) throw std::runtime_error("Не найден элемент <task>");

    Task result;
    parseLibrary(task, result);
    parseStructs(task, result);
    parseObjective(task, result);
    return result;
}

}  // namespace pdaplan
