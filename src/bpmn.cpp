#include "bpmn.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <sstream>

#include "tinyxml2.h"

namespace pdaplan {
namespace {

using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;

constexpr double kTaskW = 100.0;
constexpr double kTaskH = 80.0;
constexpr double kStepX = 160.0;
constexpr double kStepY = 120.0;
constexpr double kOriginX = 60.0;
constexpr double kOriginY = 60.0;

std::string fmt(double value) {
    std::ostringstream out;
    const double rounded = std::round(value * 1e6) / 1e6;
    if (std::fabs(rounded - std::round(rounded)) < 1e-9) {
        out << static_cast<long long>(std::llround(rounded));
    } else {
        out.precision(10);
        out << rounded;
    }
    return out.str();
}

struct Bounds {
    double x = 0.0, y = 0.0, w = 0.0, h = 0.0;
};

class IdGenerator {
public:
    std::string next(const std::string& prefix) {
        return "pda_" + prefix + "_" + std::to_string(++counter_);
    }

private:
    int counter_ = 1;
};

}  // namespace

std::vector<Chain> buildPlan(const Task& task, const Solution& solution) {
    const auto windows = task.absoluteTimes();
    const auto values = solution.nonzero();
    std::vector<Chain> chains;

    for (const Interval& interval : task.intervals) {
        const std::string& k = interval.id;
        const auto window = windows.at(k);
        Chain chain;

        auto push = [&](const std::string& name, const std::string& resource, double stream,
                        std::optional<double> performance) {
            PlanTask item;
            item.name = name;
            item.resource = resource;
            item.stream = stream;
            item.performance = performance;
            item.duration = performance && *performance > 0 ? stream / *performance : 0.0;
            item.windowStart = window.first;
            item.windowEnd = window.second;
            chain.push_back(std::move(item));
        };

        for (const Node& node : interval.nodes) {
            for (const auto& [flow, volume] : node.inputs) {
                if (volume <= 0.0) continue;
                push("Входной поток " + flow + " к исполнителю " + node.id + " на интервале " + k,
                     node.id, volume, std::nullopt);
            }
        }

        for (const auto& [key, amount] : values) {
            if (key.interval != k) continue;

            if (key.kind == "transport") {
                auto tt = task.transports.find(key.tech);
                if (tt == task.transports.end()) continue;
                push("Передача потока исполнителем " + key.object + " исполнителю " + key.object2 +
                         " способом " + key.tech + " на интервале " + k,
                     key.object, amount, tt->second.performance());

            } else if (key.kind == "process") {
                auto pt = task.processes.find(key.tech);
                if (pt == task.processes.end()) continue;
                push("Обработка потока исполнителем " + key.object + " способом " + key.tech +
                         " на интервале " + k,
                     key.object, amount, pt->second.performance());

            } else if (key.kind == "storage") {
                push("Хранение потока " + key.flow + " исполнителем " + key.object + " способом " +
                         key.tech + " на интервале " + k,
                     key.object, amount, std::nullopt);

            } else if (key.kind == "lost") {
                push("Потеря потока " + key.flow + " исполнителем " + key.object +
                         " на интервале " + k,
                     key.object, amount, std::nullopt);

            } else if (key.kind == "resultflow") {
                push("Выходной поток " + key.flow + " от исполнителя " + key.object +
                         " на интервале " + k,
                     key.object, amount, std::nullopt);
            }
        }

        if (!chain.empty()) chains.push_back(std::move(chain));
    }

    return chains;
}

void writeBpmn(std::vector<Chain>& chains, const std::string& path) {
    XMLDocument doc;
    doc.InsertFirstChild(doc.NewDeclaration());

    XMLElement* definitions = doc.NewElement("definitions");
    definitions->SetAttribute("xmlns", "http://www.omg.org/spec/BPMN/20100524/MODEL");
    definitions->SetAttribute("xmlns:bpmndi", "http://www.omg.org/spec/BPMN/20100524/DI");
    definitions->SetAttribute("xmlns:omgdc", "http://www.omg.org/spec/DD/20100524/DC");
    definitions->SetAttribute("xmlns:omgdi", "http://www.omg.org/spec/DD/20100524/DI");
    definitions->SetAttribute("xmlns:ltsm", "https://litsam.ru");
    definitions->SetAttribute("id", "pda_defs_1");
    definitions->SetAttribute("targetNamespace", "http://www.signavio.com/bpmn20");
    definitions->SetAttribute("typeLanguage", "http://www.w3.org/2001/XMLSchema");
    definitions->SetAttribute("expressionLanguage", "http://www.w3.org/1999/XPath");
    doc.InsertEndChild(definitions);

    IdGenerator ids;
    XMLElement* process = doc.NewElement("process");
    const std::string processId = ids.next("process");
    process->SetAttribute("id", processId.c_str());
    process->SetAttribute("name", "PROCESS");
    process->SetAttribute("isExecutable", "false");
    definitions->InsertEndChild(process);

    std::map<std::string, std::vector<std::string>> resources;
    std::map<std::string, Bounds> shapes;
    std::vector<std::pair<std::string, std::string>> flows;

    auto addTask = [&](const std::string& taskId, const std::string& name,
                       const std::string& resource, double stream, double duration,
                       std::optional<double> performance, bool hasWindow, double windowStart,
                       double windowEnd) {
        XMLElement* task = doc.NewElement("task");
        task->SetAttribute("id", taskId.c_str());
        task->SetAttribute("name", name.c_str());
        process->InsertEndChild(task);

        XMLElement* performer = doc.NewElement("performer");
        performer->SetAttribute("id", (taskId + "_RS_" + resource).c_str());
        task->InsertEndChild(performer);

        XMLElement* ref = doc.NewElement("resourceRef");
        ref->SetText(("RS_" + resource).c_str());
        performer->InsertEndChild(ref);
        resources.try_emplace(resource);

        if (performance) {
            const std::string paramId = "RS_" + resource + "_P_" + taskId;
            XMLElement* binding = doc.NewElement("resourceParameterBinding");
            binding->SetAttribute("id", (paramId + "_B").c_str());
            binding->SetAttribute("parameterRef", paramId.c_str());
            performer->InsertEndChild(binding);

            XMLElement* expression = doc.NewElement("formalExpression");
            expression->SetText(fmt(*performance).c_str());
            binding->InsertEndChild(expression);
            resources[resource].push_back(paramId);
        }

        XMLElement* extensions = doc.NewElement("extensionElements");
        task->InsertEndChild(extensions);

        XMLElement* props = doc.NewElement("ltsm:props");
        props->SetAttribute("stream", fmt(stream).c_str());
        props->SetAttribute("volume", fmt(duration).c_str());
        extensions->InsertEndChild(props);

        if (hasWindow) {
            for (const auto& [time, value] :
                 {std::make_pair(windowStart, "1"), std::make_pair(windowEnd, "0")}) {
                XMLElement* availability = doc.NewElement("ltsm:availability");
                availability->SetAttribute("availability_time", fmt(time).c_str());
                availability->SetAttribute("availability_value", value);
                availability->SetAttribute("res_id", ("RS_" + resource).c_str());
                extensions->InsertEndChild(availability);
            }
        }
    };

    const std::string startId = ids.next("start");
    XMLElement* start = doc.NewElement("startEvent");
    start->SetAttribute("id", startId.c_str());
    start->SetAttribute("name", "StartEvent");
    process->InsertEndChild(start);

    const std::string anchor =
        (!chains.empty() && !chains.front().empty()) ? chains.front().front().resource : "1";
    const std::string dummyStartId = ids.next("task");
    addTask(dummyStartId, "DummyAfterStart", anchor, 0.0, 0.0, 1.0, false, 0.0, 0.0);

    const std::string gwOpen = ids.next("gw");
    XMLElement* gateway = doc.NewElement("parallelGateway");
    gateway->SetAttribute("id", gwOpen.c_str());
    gateway->SetAttribute("name", "parallelGateway");
    gateway->SetAttribute("gatewayDirection", "Diverging");
    process->InsertEndChild(gateway);

    shapes[startId] = {kOriginX, kOriginY, 36.0, 36.0};
    shapes[dummyStartId] = {kOriginX + 116, kOriginY, kTaskW, kTaskH};
    shapes[gwOpen] = {kOriginX + 296, kOriginY + 10, 40.0, 40.0};
    flows.emplace_back(startId, dummyStartId);
    flows.emplace_back(dummyStartId, gwOpen);

    const std::string gwClose = "pda_gw_close";
    double width = kOriginX + 396;

    for (size_t row = 0; row < chains.size(); ++row) {
        const double y = kOriginY + static_cast<double>(row) * kStepY;
        std::string previous = gwOpen;

        for (size_t column = 0; column < chains[row].size(); ++column) {
            PlanTask& item = chains[row][column];
            item.id = ids.next("task");
            addTask(item.id, item.name, item.resource, item.stream, item.duration, item.performance,
                    true, item.windowStart, item.windowEnd);

            const double x = kOriginX + 396 + static_cast<double>(column) * kStepX;
            shapes[item.id] = {x, y, kTaskW, kTaskH};
            width = std::max(width, x + kStepX);
            flows.emplace_back(previous, item.id);
            previous = item.id;
        }
        flows.emplace_back(previous, gwClose);
    }

    XMLElement* closing = doc.NewElement("parallelGateway");
    closing->SetAttribute("id", gwClose.c_str());
    closing->SetAttribute("name", "parallelGateway_2");
    closing->SetAttribute("gatewayDirection", "Converging");
    process->InsertEndChild(closing);

    const std::string dummyEndId = ids.next("task");
    addTask(dummyEndId, "DummyBeforeEnd", anchor, 0.0, 0.0, 1.0, false, 0.0, 0.0);

    const std::string endId = ids.next("end");
    XMLElement* end = doc.NewElement("endEvent");
    end->SetAttribute("id", endId.c_str());
    end->SetAttribute("name", "EndEvent");
    process->InsertEndChild(end);

    shapes[gwClose] = {width + 40, kOriginY + 10, 40.0, 40.0};
    shapes[dummyEndId] = {width + 120, kOriginY, kTaskW, kTaskH};
    shapes[endId] = {width + 280, kOriginY, 36.0, 36.0};
    flows.emplace_back(gwClose, dummyEndId);
    flows.emplace_back(dummyEndId, endId);

    std::vector<std::tuple<std::string, std::string, std::string>> flowIds;
    for (const auto& [source, target] : flows) {
        const std::string flowId = ids.next("flow");
        XMLElement* flow = doc.NewElement("sequenceFlow");
        flow->SetAttribute("id", flowId.c_str());
        flow->SetAttribute("sourceRef", source.c_str());
        flow->SetAttribute("targetRef", target.c_str());
        process->InsertEndChild(flow);
        flowIds.emplace_back(flowId, source, target);
    }

    XMLElement* diagram = doc.NewElement("bpmndi:BPMNDiagram");
    diagram->SetAttribute("id", "Diagram_1");
    definitions->InsertEndChild(diagram);

    XMLElement* plane = doc.NewElement("bpmndi:BPMNPlane");
    plane->SetAttribute("id", "Plane_1");
    plane->SetAttribute("bpmnElement", processId.c_str());
    diagram->InsertEndChild(plane);

    for (const auto& [elementId, bounds] : shapes) {
        XMLElement* shape = doc.NewElement("bpmndi:BPMNShape");
        shape->SetAttribute("id", ("Shape_" + elementId).c_str());
        shape->SetAttribute("bpmnElement", elementId.c_str());
        plane->InsertEndChild(shape);

        XMLElement* box = doc.NewElement("omgdc:Bounds");
        box->SetAttribute("x", fmt(bounds.x).c_str());
        box->SetAttribute("y", fmt(bounds.y).c_str());
        box->SetAttribute("width", fmt(bounds.w).c_str());
        box->SetAttribute("height", fmt(bounds.h).c_str());
        shape->InsertEndChild(box);
    }

    for (const auto& [flowId, source, target] : flowIds) {
        auto from = shapes.find(source);
        auto to = shapes.find(target);
        if (from == shapes.end() || to == shapes.end()) continue;

        XMLElement* edge = doc.NewElement("bpmndi:BPMNEdge");
        edge->SetAttribute("id", ("Edge_" + flowId).c_str());
        edge->SetAttribute("bpmnElement", flowId.c_str());
        plane->InsertEndChild(edge);

        XMLElement* first = doc.NewElement("omgdi:waypoint");
        first->SetAttribute("x", fmt(from->second.x + from->second.w).c_str());
        first->SetAttribute("y", fmt(from->second.y + from->second.h / 2).c_str());
        edge->InsertEndChild(first);

        XMLElement* second = doc.NewElement("omgdi:waypoint");
        second->SetAttribute("x", fmt(to->second.x).c_str());
        second->SetAttribute("y", fmt(to->second.y + to->second.h / 2).c_str());
        edge->InsertEndChild(second);
    }

    for (const auto& [resourceId, params] : resources) {
        XMLElement* resource = doc.NewElement("resource");
        resource->SetAttribute("id", ("RS_" + resourceId).c_str());
        resource->SetAttribute("name", ("Исполнитель " + resourceId).c_str());
        definitions->InsertEndChild(resource);

        const char* names[] = {"name", "threads", "productivity"};
        const char* defaults[] = {"", "1", ""};
        for (int i = 0; i < 3; ++i) {
            XMLElement* param = doc.NewElement("resourceParameter");
            param->SetAttribute("id", ("RS_" + resourceId + "_P_" + std::to_string(i + 1)).c_str());
            param->SetAttribute("name", names[i]);
            param->SetAttribute("value", defaults[i]);
            resource->InsertEndChild(param);
        }
        for (const std::string& paramId : params) {
            XMLElement* param = doc.NewElement("resourceParameter");
            param->SetAttribute("id", paramId.c_str());
            param->SetAttribute("name", "performance");
            param->SetAttribute("type", "xsd:float");
            resource->InsertEndChild(param);
        }
    }

    if (doc.SaveFile(path.c_str()) != tinyxml2::XML_SUCCESS) {
        throw std::runtime_error("Не удалось записать файл: " + path);
    }
}

}  // namespace pdaplan
