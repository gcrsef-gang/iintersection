/**
 *  \file libiintersection.h
 *  
 *  \brief The lowest level of the iintersection project. Defines the Intersection
 *  data type and handles the wrapping of several simulation backends. 
 *
 *  \author Kai Vernooy, James Lian, and Arin Khare
 *  \date Feb, 2021
 */


#define SUMO_LIB

#ifndef LIBIINTERSECTION_H
#define LIBIINTERSECTION_H

#ifdef SUMO_LIB
// #include <sumo/main.h>
#endif


#include <algorithm>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "lib/pugixml/src/pugixml.hpp"


/**
 * \brief Entire iintersection namespace, with different backend interfaces
 * Contains input and solution classes, as well as ways to evaluate them
 */
namespace ii {


// Forward class declarations
class Intersection;
class SumoInterface;
class BackendsManager;
class IntersectionNode;


// Tracking current max node ID
static unsigned short int CURRENT_UUID_MAX;


// Global constants for the evaluation backends
static const std::size_t SIMTIME = 604800;  // Seconds of simulation time
enum class METRICS {SAFETY, EMISSIONS, EFFICIENCY};
enum class BACKENDS {SUMO, VISSIM, CITYFLOW};

enum class VEHICLETYPES {CAR, TRUCK, IDK};
const std::map<std::string, VEHICLETYPES> VEHICLETYPE_INDICES = {{"car", VEHICLETYPES::CAR}, {"truck", VEHICLETYPES::TRUCK}, {"idk", VEHICLETYPES::IDK}};

enum class JUNCTIONTYPES {PRIORITY, TRAFFIC_LIGHT, RIGHT_BEFORE_LEFT, UNREGULATED, PRIORITY_STOP, TRAFFIC_LIGHT_UNREGULATED, ALLWAY_STOP, ZIPPER, TRAFFIC_LIGHT_RIGHT_ON_RED};
const std::map<JUNCTIONTYPES, std::string> JUNCTIONTYPES_NAMES = {{JUNCTIONTYPES::PRIORITY, "priority"}, {JUNCTIONTYPES::TRAFFIC_LIGHT, "traffic_light"}, {JUNCTIONTYPES::RIGHT_BEFORE_LEFT, "right_before_left"}, {JUNCTIONTYPES::UNREGULATED, "unregulated"}, {JUNCTIONTYPES::PRIORITY_STOP, "priority_stop"}, {JUNCTIONTYPES::TRAFFIC_LIGHT_UNREGULATED, "traffic_light_unregulated"}, {JUNCTIONTYPES::ALLWAY_STOP, "allway_stop"}, {JUNCTIONTYPES::ZIPPER, "zipper"}, {JUNCTIONTYPES::TRAFFIC_LIGHT_RIGHT_ON_RED, "traffic_light_on_red"}};


// Intersection evaluation function type
typedef void (::ii::BackendsManager::*IntersectionEvalFunc)(::ii::Intersection*);


/**
 * \brief The base backend evaluation class - contains methods for retrieving all metrics
 * that will be derived for each backend-specific class. Created specifically for a generalized
 * evaluation function type, stored in the ::ii::Intersection::evaluations map.
 * 
 * Currently the parent to only ::ii::SumoInterface
 */
class BackendsManager
{
public:
    virtual void updateIntersectionEmissions(Intersection*) = 0;
    virtual void updateIntersectionSafety(Intersection*) = 0;
    virtual void updateIntersectionEfficiency(Intersection*) = 0;
};


class SumoInterface : public BackendsManager
{
public:
    SumoInterface(const SumoInterface&) = delete;
    SumoInterface& operator= (const SumoInterface&) = delete;

    static SumoInterface* GetInstance()
    {
        static SumoInterface instance;
        return &instance;
    }

    void rebuildNet(const Intersection*);
    void performSim(const std::size_t time);
    void updateIntersectionEmissions(Intersection*);
    void updateIntersectionSafety(Intersection*);
    void updateIntersectionEfficiency(Intersection*);

private:
    SumoInterface() {}
    // MSNet* net;
};


// SumoInterface SumoInterface::instance;


class Point3d
{
public:
    Point3d(short int x, short int y, short int z) : x_(x), y_(y), z_(z) {};
    Point3d(std::vector<short int> coords) : x_(coords[0]), y_(coords[1]), z_(coords[2]) {}

    short int x() {return this->x_;}
    short int y() {return this->y_;}
    short int z() {return this->z_;}

private:
    const short int x_, y_, z_;
};


class BezierCurve
{
public:
    BezierCurve() {}
    BezierCurve(IntersectionNode* s, IntersectionNode* e, std::vector<Point3d> handles) : s(s), e(e), handles(handles) {}

    std::vector<Point3d> rasterize();
    IntersectionNode* getStartNode() const {return this->s;}
    IntersectionNode* getEndNode() const {return this->e;}
    std::vector<Point3d> getHandles() const {return this->handles;}
    void updateHandles(std::vector<Point3d> handles) {handles = handles}

private:
    IntersectionNode* s;
    IntersectionNode* e;
    std::vector<Point3d> handles;
};


class Node
{
public:
    Node(Point3d loc) : loc(loc) {this->UUID = ++CURRENT_UUID_MAX;}

    Point3d* getLoc() {return &(this->loc);}
    unsigned short int getID() {return UUID;}

private:
    unsigned short int UUID;
    Point3d loc;

friend class Intersection;
};


class IntersectionNode : public Node
{
public:
    IntersectionNode(Point3d loc, JUNCTIONTYPES junctionType) : Node(loc), junctionType(junctionType) {}
    JUNCTIONTYPES getJunctionType() {return this->junctionType;}

private:
    JUNCTIONTYPES junctionType;
};


class Edge
{
public:
    Edge(Node* s, Node* e) : s(s), e(e) {};
    Node* getStartNode() const {return s;}
    Node* getEndNode() const {return e;}

private:
    Node* s; // starting node
    Node* e; // ending node
};


class IntersectionEdge : public Edge
{
public:
    IntersectionEdge(IntersectionNode* s, IntersectionNode* e, BezierCurve shape, short int numLanes, short int speedLimit, short int priority) : Edge(s, e), shape(shape), numlanes(numLanes), speedlimit(speedLimit), priority(priority) {}
    
    BezierCurve getShape() const {return this->shape;}
    short int getNumLanes() const {return this->numlanes;}
    short int getSpeedLimit() const {return this->speedlimit;}
    short int getPriority() const {return this->priority;}
    void updateHandles(std::vector<Point3d> handles) {shape.updateHandles(handles);}
    void setNumLanes(short int numLanes) {numlanes = numLanes;}
    void setSpeedLimit(short int speedLimit) {speedlimit = speedLimit;}
    void setPriority(short int priority) {priority = priority;}

private:
    BezierCurve shape;
    short int numlanes;
    short int speedlimit;
    short int priority;
};


class ScenarioEdge : public Edge
{
public:
    ScenarioEdge(Node* s, Node* e, std::map<VEHICLETYPES, short int> demand) : Edge(s, e), demand(demand) {}

    std::map<VEHICLETYPES, short int> getDemand() {return this->demand;}

private:
    std::map<VEHICLETYPES, short int> demand;
};


class IntersectionRoute
{
public:
    IntersectionRoute(std::vector<IntersectionNode*> nodeList, std::vector<IntersectionEdge> edgeList)
        : nodeList(nodeList), edgeList(edgeList) {}

    std::vector<IntersectionNode*> getNodeList() const {return this->nodeList;}
    std::vector<IntersectionEdge> getEdgeList() const {return this->edgeList;}

private:
    std::vector<IntersectionNode*> nodeList;
    std::vector<IntersectionEdge> edgeList;
};


class Intersection
{
public:
    Intersection(std::vector<IntersectionRoute*> routes) : routes(routes) {}

    void simulate(BACKENDS) const;
    void updateMetrics(BACKENDS);
    double getMetric(METRICS);
    IntersectionRoute* getRoutes() const;

    std::string getEdgeXML() const;
    std::string getNodeXML() const;

private:
    std::vector<IntersectionRoute*> routes;
    std::map<METRICS, double> currentMetrics;
    const static std::map<BACKENDS, std::map<METRICS, IntersectionEvalFunc> > evaluations;
    std::vector<IntersectionRoute*> routes;
};


const std::map<BACKENDS, std::map<METRICS, IntersectionEvalFunc> > Intersection::evaluations = 
{
    {
        BACKENDS::SUMO, {
            {METRICS::EFFICIENCY, static_cast<IntersectionEvalFunc>(&SumoInterface::updateIntersectionEfficiency)},
            {METRICS::SAFETY, static_cast<IntersectionEvalFunc>(&SumoInterface::updateIntersectionSafety)},
            {METRICS::EMISSIONS, static_cast<IntersectionEvalFunc>(&SumoInterface::updateIntersectionEmissions)}
        }
    }
};

// IntersectionEvalFunc func = SumoInterface::updateIntersectionEfficiency;

class IntersectionScenario
{
public:
    IntersectionScenario(std::vector<Node*> nodes, std::vector<ScenarioEdge> edges) : nodes(nodes), edges(edges) {}
    IntersectionScenario(std::string xmlFilePath);
    std::vector<Node*> getNodes() const {return this->nodes;}
    std::vector<ScenarioEdge> getEdges() const {return this->edges;}

private:
    std::vector<Node*> nodes;
    std::vector<ScenarioEdge> edges;
};



IntersectionScenario::IntersectionScenario(std::string xmlFilePath)
{
    // Load document.
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(xmlFilePath.c_str());
    if (!result)
    {
        std::cerr << "Could not parse XML." << std::endl;
        std::cerr << result.description() << std::endl;
    }

    pugi::xml_node xmlScenario = doc.child("scenario");

    // Append nodes to vector.
    pugi::xml_node nodesList = xmlScenario.child("nodes");
    std::map<std::string, Node*> nodeIDMap;
    for (pugi::xml_node xmlNode = nodesList.first_child(); xmlNode; xmlNode = xmlNode.next_sibling())
    {
        Node node({static_cast<short int>(xmlNode.attribute("x").as_int()), static_cast<short int>(xmlNode.attribute("y").as_int()), static_cast<short int>(xmlNode.attribute("z").as_int())});
        nodeIDMap[xmlNode.attribute("id").value()] = &node;
        nodes.push_back(&node);
    }

    // Append edges to vector.
    pugi::xml_node edgesList = xmlScenario.child("edges");
    for (pugi::xml_node xmlEdge = edgesList.first_child(); xmlEdge; xmlEdge = xmlEdge.next_sibling())
    {
        Node* s, *e;
        s = nodeIDMap[xmlEdge.attribute("from").value()];
        e = nodeIDMap[xmlEdge.attribute("to").value()];

        std::map<VEHICLETYPES, short int> demand;

        for (pugi::xml_attribute attr = xmlEdge.first_attribute(); attr; attr = attr.next_attribute())
        {
            std::string attrName = attr.name();
            int pos = attrName.find("_demand");

            // If attribute contains demand data.
            if (pos != std::string::npos)
            {
                VEHICLETYPES vehicleType = VEHICLETYPE_INDICES.at(attrName.substr(0, pos));
                short int vehicleDemand = attr.as_int();
                demand[vehicleType] = vehicleDemand;
            }
        }

        edges.push_back(ScenarioEdge(s, e, demand));
    }
}


void Intersection::simulate(BACKENDS back) const
{
    if (back == BACKENDS::SUMO)
    {
        SumoInterface::GetInstance()->rebuildNet(this);
        SumoInterface::GetInstance()->performSim(SIMTIME);
    }
}


void Intersection::updateMetrics(BACKENDS back)
{
    const std::map<METRICS, IntersectionEvalFunc> backendEvaluations = evaluations.at(back);

    for (auto it = backendEvaluations.begin(); it != backendEvaluations.end(); it++)
    {
        (SumoInterface::GetInstance()->*(it->second))(this);
    }
}


std::string Intersection::getNodeXML() const
{
    std::vector<IntersectionNode*> nodes;
    for (IntersectionRoute* route : routes)
    {
        for (IntersectionNode* node : route->getNodeList())
        {
            nodes.push_back(node);
        }
    }

    std::string xmlOutput = "<nodes>\n";
    std::stringstream nodeTag;

    for (int i = 0; i < nodes.size(); i++)
    {
        Point3d* nodeLoc = nodes[i]->getLoc();

        nodeTag << "\t<node ";
        nodeTag << "id=\"" << i << "\" ";
        nodeTag << "x=\"" << nodeLoc->x() << "\" ";
        nodeTag << "y=\"" << nodeLoc->y() << "\" ";
        nodeTag << "z=\"" << nodeLoc->z() << "\" ";
        nodeTag << "type=\"" << JUNCTIONTYPES_NAMES.at(nodes[i]->getJunctionType()) << "\"/>\n";

        xmlOutput += nodeTag.str();
        nodeTag.clear();
    }

    xmlOutput += "</nodes>";
    return xmlOutput;
}


std::string Intersection::getEdgeXML() const
{
    // Node IDs as they will be in the node file generated by getNodeXML.
    std::map<IntersectionNode*, int> sumoNodeIDs;
    std::vector<IntersectionEdge> edges;
    for (IntersectionRoute* route : routes)
    {
        for (IntersectionEdge edge : route->getEdgeList())
        {
            edges.push_back(edge);
        }
        std::vector<IntersectionNode*> routeNodes = route->getNodeList();
        for (int i = 0; i < routeNodes.size(); i++)
        {
            sumoNodeIDs.insert(std::pair<IntersectionNode*, int>(routeNodes[i], i));
        }
    }

    std::string xmlOutput = "<edges>\n";
    std::stringstream edgeTag;

    for (int i = 0; i < edges.size(); i++)
    {
        edgeTag << "\t<edge id=\"" << i << "e\" ";
        edgeTag << "from=\"" << sumoNodeIDs[(IntersectionNode*)edges[i].getStartNode()] << "\" ";
        edgeTag << "to=\"" << sumoNodeIDs[(IntersectionNode*)edges[i].getEndNode()] << "\" ";
        edgeTag << "priority=\"" << edges[i].getPriority() << "\" ";
        edgeTag << "numLanes=\"" << edges[i].getNumLanes() << "\" ";
        edgeTag << "speed=\"" << edges[i].getSpeedLimit() << "\"/>\n";

        xmlOutput += edgeTag.str();
        edgeTag.clear();
    }

    xmlOutput += "</edges>";
    return xmlOutput;
}


}


#endif