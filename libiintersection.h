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
#include <microsim/MSFrame.h>
#include <microsim/MSLane.h>
#include <microsim/MSVehicleControl.h>
#include <microsim/MSEventControl.h>
#include <microsim/MSEdge.h>
#include <microsim/MSNet.h>
#include <microsim/MSJunctionControl.h>
#include <utils/shapes/ShapeContainer.h>
#include <utils/vehicle/SUMORouteLoaderControl.h>
#include <utils/options/OptionsCont.h>
#include <microsim/MSEdgeControl.h>
#include <microsim/traffic_lights/MSTLLogicControl.h>
#endif


#include <algorithm>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include <assert.h>
#include <list>
#include <memory>

#include <pugixml/src/pugixml.hpp>


/**
 * \brief Entire iintersection namespace, with different backend interfaces
 * Contains input and solution classes, as well as ways to evaluate them
 */
namespace ii {


// Forward class declarations
class SumoInterface;
class BackendsManager;
class Point3d;
class Node;
class IntersectionNode;
class Edge;
class IntersectionEdge;
class IntersectionRoute;
class Intersection;
class IntersectionScenario;


// Tracking current max node ID
static unsigned short int CURRENT_NODE_UUID_MAX = -1;
static unsigned short int CURRENT_EDGE_UUID_MAX = -1;

static const short int BEZIER_SAMPLES = 500;
static const std::size_t SIMTIME_ = 604800;  // Seconds of simulation time


namespace METRICS {enum METRICS_ {SAFETY, EMISSIONS, EFFICIENCY};}
namespace BACKENDS {enum BACKENDS_ {SUMO, VISSIM, CITYFLOW, TRACI};}
namespace VEHICLETYPES {enum VEHICLETYPES_ {CAR, TRUCK};}
namespace JUNCTIONTYPES {enum JUNCTIONTYPES_ {PRIORITY, TRAFFIC_LIGHT, RIGHT_BEFORE_LEFT, UNREGULATED, PRIORITY_STOP, TRAFFIC_LIGHT_UNREGULATED, ALLWAY_STOP, TRAFFIC_LIGHT_RIGHT_ON_RED};}

const std::map<std::string, VEHICLETYPES::VEHICLETYPES_> VEHICLETYPES_INDICES = {{"car", VEHICLETYPES::CAR}, {"truck", VEHICLETYPES::TRUCK}};
const std::map<VEHICLETYPES::VEHICLETYPES_, std::string> VEHICLETYPES_NAMES = {{VEHICLETYPES::CAR, "car"}, {VEHICLETYPES::TRUCK, "truck"}};
const std::map<std::string, JUNCTIONTYPES::JUNCTIONTYPES_> JUNCTIONTYPES_INDICES = {{"priority", JUNCTIONTYPES::PRIORITY}, {"traffic_light", JUNCTIONTYPES::TRAFFIC_LIGHT}, {"right_before_left", JUNCTIONTYPES::RIGHT_BEFORE_LEFT}, {"unregulated", JUNCTIONTYPES::UNREGULATED}, {"priority_stop", JUNCTIONTYPES::PRIORITY_STOP}, {"traffic_light_unregulated", JUNCTIONTYPES::TRAFFIC_LIGHT_UNREGULATED}, {"allway_stop", JUNCTIONTYPES::ALLWAY_STOP}, {"traffic_light_on_red", JUNCTIONTYPES::TRAFFIC_LIGHT_RIGHT_ON_RED}};
const std::map<JUNCTIONTYPES::JUNCTIONTYPES_, std::string> JUNCTIONTYPES_NAMES = {{JUNCTIONTYPES::PRIORITY, "priority"}, {JUNCTIONTYPES::TRAFFIC_LIGHT, "traffic_light"}, {JUNCTIONTYPES::RIGHT_BEFORE_LEFT, "right_before_left"}, {JUNCTIONTYPES::UNREGULATED, "unregulated"}, {JUNCTIONTYPES::PRIORITY_STOP, "priority_stop"}, {JUNCTIONTYPES::TRAFFIC_LIGHT_UNREGULATED, "traffic_light_unregulated"}, {JUNCTIONTYPES::ALLWAY_STOP, "allway_stop"}, {JUNCTIONTYPES::TRAFFIC_LIGHT_RIGHT_ON_RED, "traffic_light_on_red"}};


std::map<JUNCTIONTYPES::JUNCTIONTYPES_, SumoXMLNodeType> SumoJunctionMap = {
    {JUNCTIONTYPES::PRIORITY, SumoXMLNodeType::PRIORITY},
    {JUNCTIONTYPES::TRAFFIC_LIGHT, SumoXMLNodeType::TRAFFIC_LIGHT},
    {JUNCTIONTYPES::RIGHT_BEFORE_LEFT, SumoXMLNodeType::RIGHT_BEFORE_LEFT},
    {JUNCTIONTYPES::UNREGULATED, SumoXMLNodeType::UNKNOWN}, // Idk if this is right?
    {JUNCTIONTYPES::PRIORITY_STOP, SumoXMLNodeType::PRIORITY_STOP},
    {JUNCTIONTYPES::TRAFFIC_LIGHT_UNREGULATED, SumoXMLNodeType::TRAFFIC_LIGHT_NOJUNCTION},
    {JUNCTIONTYPES::ALLWAY_STOP, SumoXMLNodeType::ALLWAY_STOP},
    {JUNCTIONTYPES::TRAFFIC_LIGHT_RIGHT_ON_RED, SumoXMLNodeType::TRAFFIC_LIGHT_RIGHT_ON_RED}
};


// Intersection evaluation function type
typedef void (::ii::BackendsManager::*IntersectionEvalFunc)(::ii::Intersection*);


/**
 * @brief The base backend evaluation class - contains methods for retrieving all metrics
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


class Point3d
{
public:
    Point3d() {}
    Point3d(short int x, short int y, short int z) : x_(x), y_(y), z_(z) {}
    Point3d(std::vector<short int> coords) : x_(coords[0]), y_(coords[1]), z_(coords[2]) {}

    short int x() {return this->x_;}
    short int y() {return this->y_;}
    short int z() {return this->z_;}

private:
    short int x_, y_, z_;
};


class SumoInterface : public BackendsManager
{
public:
    SumoInterface(const SumoInterface&) = delete;
    SumoInterface& operator= (const SumoInterface&) = delete;

    static SumoInterface* Get()
    {
        static SumoInterface instance;
        return &instance;
    }

    void performSim(const std::size_t time);
    void rebuildNet(Intersection*);
    void updateIntersectionEmissions(Intersection*) {};
    void updateIntersectionSafety(Intersection*) {};
    void updateIntersectionEfficiency(Intersection*);

private:
    SumoInterface()
    {
        this->constructNet();
    }


    void constructNet();
    MSNet* net;

    static Position Point3dToPosition(Point3d p) {return (Position(p.x() / 10.0, p.y() / 10.0, p.z() / 10.0));}

    MSEdgeControl* buildSumoEdges(std::vector<IntersectionEdge*>, std::vector<std::shared_ptr<IntersectionNode> >, MSJunctionControl*);
    MSEdge* buildSumoEdge(IntersectionEdge*);

    std::vector<MSLane*> buildSumoLanes(IntersectionEdge*, MSEdge*);
    MSLane* buildSumoLane(IntersectionEdge*, MSEdge*, int);

    MSJunctionControl* buildSumoJunctions(std::vector<std::shared_ptr<IntersectionNode> >);
    MSJunction* buildSumoJunction(std::shared_ptr<IntersectionNode>);

    std::map<std::shared_ptr<Node>, MSJunction*> nodeJunctionMap;
    std::vector<MSJunction> junctions;
    std::vector<MSEdge> edges;
};


class BezierCurve
{
public:
    BezierCurve() {}
    BezierCurve(std::shared_ptr<IntersectionNode> s, std::shared_ptr<IntersectionNode> e, std::vector<Point3d> handles)
        : s(s), e(e), handles(handles) {}

    std::vector<Point3d> rasterize(int resolution);

    std::vector<Point3d> getHandles() const {return this->handles;}
    std::shared_ptr<IntersectionNode> getStartNode() const {return this->s;}
    std::shared_ptr<IntersectionNode> getEndNode() const {return this->e;}
    void setHandles(std::vector<Point3d> handles_) {this->handles = handles_;}

private:
    Point3d evaluateParametric(double t);

    void setStartNode(std::shared_ptr<IntersectionNode> node) {s = node;}
    void setEndNode(std::shared_ptr<IntersectionNode> node) {e = node;}
    
    std::shared_ptr<IntersectionNode> s;
    std::shared_ptr<IntersectionNode> e;
    std::vector<Point3d> handles;

friend class IntersectionEdge;
};


class Node
{
public:
    Node(Point3d loc) : loc(loc) {this->UUID = ++CURRENT_NODE_UUID_MAX;}

    Point3d* getLoc() {return &(this->loc);}
    unsigned short int getID() const {return this->UUID;}

    void setID(unsigned short int ID) {this->UUID = ID;}

protected:
    unsigned short int UUID;

private:
    Point3d loc;

friend bool operator==(const Node& n1, const Node& n2) {return n1.getID() == n2.getID();}
};


typedef Node ScenarioNode;


class IntersectionNode : public Node
{
public:
    IntersectionNode(Point3d loc, JUNCTIONTYPES::JUNCTIONTYPES_ junctionType)
        : Node(loc), junctionType(junctionType) {this->referenceCount = 1;}

    JUNCTIONTYPES::JUNCTIONTYPES_ getJunctionType() const {return this->junctionType;}

private:
    JUNCTIONTYPES::JUNCTIONTYPES_ junctionType; 
    unsigned short int referenceCount;
};


class Edge
{
public:
    Edge() {}
    Edge(std::shared_ptr<Node> s, std::shared_ptr<Node> e) : s(s), e(e) {this->UUID = ++CURRENT_EDGE_UUID_MAX;};
    std::shared_ptr<Node> getStartNode() const {return s;}
    std::shared_ptr<Node> getEndNode() const {return e;}

    unsigned short int getId() const {return this->UUID;}

protected:
    unsigned short int UUID;

private:
    std::shared_ptr<Node> s; // starting node
    std::shared_ptr<Node> e; // ending node
};


class IntersectionEdge : public Edge
{
public:
    IntersectionEdge() {}
    IntersectionEdge(std::shared_ptr<IntersectionNode> s, std::shared_ptr<IntersectionNode> e, BezierCurve shape, short int numLanes, short int speedLimit, short int priority)
        : Edge(s, e), s(s), e(e), shape(shape), numlanes(numLanes), speedlimit(speedLimit), priority(priority) {}

    BezierCurve getShape() const {return this->shape;}
    short int getNumLanes() const {return this->numlanes;}
    short int getSpeedLimit() const {return this->speedlimit;}
    short int getPriority() const {return this->priority;}
    
    void setStartNode(std::shared_ptr<IntersectionNode> node) {s = node;}
    void setEndNode(std::shared_ptr<IntersectionNode> node) {e = node;}
    
    std::shared_ptr<IntersectionNode> getStartNode() const {return s;}
    std::shared_ptr<IntersectionNode> getEndNode() const {return e;}

    void setHandles(std::vector<Point3d> handles) {this->shape.setHandles(handles);}
    void setNumLanes(short int numLanes_) {this->numlanes = numLanes_;}
    void setSpeedLimit(short int speedLimit_) {this->speedlimit = speedLimit_;}
    void setPriority(short int priority_) {this->priority = priority_;}

private:
    std::shared_ptr<IntersectionNode> s;
    std::shared_ptr<IntersectionNode> e;

    BezierCurve shape;
    short int numlanes;
    short int speedlimit;
    short int priority;

friend bool operator==(const IntersectionEdge& e1, const IntersectionEdge& e2) {return e1.getStartNode() == e2.getStartNode() && e1.getEndNode() == e2.getStartNode();}
};


class ScenarioEdge : public Edge
{
public:
    ScenarioEdge() {}
    ScenarioEdge(std::shared_ptr<ScenarioNode> s, std::shared_ptr<ScenarioNode> e, std::map<VEHICLETYPES::VEHICLETYPES_, short int> demand) : Edge(s, e), demand(demand) {}

    std::map<VEHICLETYPES::VEHICLETYPES_, short int> getDemand() {return this->demand;}
private:
    std::map<VEHICLETYPES::VEHICLETYPES_, short int> demand;
};


class IntersectionRoute
{
public:
    IntersectionRoute() {}
    IntersectionRoute(std::vector<std::shared_ptr<IntersectionNode> > nodeList, std::vector<IntersectionEdge> edgeList)
        : nodeList(nodeList), edgeList(edgeList) {}

    std::vector<std::shared_ptr<IntersectionNode> > getNodeList() const {return this->nodeList;}
    std::vector<IntersectionEdge*> getEdgeList();

    void setNodeList(std::vector<std::shared_ptr<IntersectionNode> > nodelist) {nodeList = nodelist;}
    void setEdgeList(std::vector<IntersectionEdge> edgelist) {edgeList = edgelist;}

private:
    std::vector<std::shared_ptr<IntersectionNode> > nodeList;
    std::vector<IntersectionEdge> edgeList;
};


class Intersection
{
public:
    Intersection() {}
    Intersection(std::vector<IntersectionRoute> routes) : routes(routes), isValid(true) {}

    static Intersection fromSolFile(std::string solFilePath);
    static void Simulate(Intersection*, BACKENDS::BACKENDS_);

    void updateMetrics(BACKENDS::BACKENDS_ back, int safety = -1, double efficiency = -1.0, double emissions = -1.0);
    void markInvalid() {isValid = false;}
    double getMetric(METRICS::METRICS_);

    std::vector<IntersectionRoute*> getRoutes();
    std::vector<std::shared_ptr<IntersectionNode> > getUniqueNodes();
    std::vector<IntersectionEdge*> getUniqueEdges();
    
    std::string getEdgeXML();
    std::string getNodeXML();
    std::string getConXML();
    std::string getRouteXML(IntersectionScenario intersectionScenario);
    std::string getSolXML();

private:
    std::vector<IntersectionRoute> routes;
    bool isValid;
    
    std::map<METRICS::METRICS_, double> currentMetrics;
    const static std::map<BACKENDS::BACKENDS_, std::map<METRICS::METRICS_, IntersectionEvalFunc> > evaluations;
};


const std::map<BACKENDS::BACKENDS_, std::map<METRICS::METRICS_, IntersectionEvalFunc> > Intersection::evaluations = 
{
    {
        BACKENDS::SUMO, {
            {METRICS::EFFICIENCY, static_cast<IntersectionEvalFunc>(&SumoInterface::updateIntersectionEfficiency)},
            {METRICS::SAFETY, static_cast<IntersectionEvalFunc>(&SumoInterface::updateIntersectionSafety)},
            {METRICS::EMISSIONS, static_cast<IntersectionEvalFunc>(&SumoInterface::updateIntersectionEmissions)}
        }
    }
};


class IntersectionScenario
{
public:
    IntersectionScenario() {}
    IntersectionScenario(std::vector<std::shared_ptr<ScenarioNode> > nodes, std::vector<ScenarioEdge> edges) : nodes(nodes), edges(edges) {}
    IntersectionScenario(std::string xmlFilePath);

    std::vector<std::shared_ptr<ScenarioNode> > getNodes() const {return this->nodes;}
    std::vector<ScenarioEdge*> getEdges();

private:
    std::vector<std::shared_ptr<ScenarioNode> > nodes;
    std::vector<ScenarioEdge> edges;
};


std::vector<std::string> Split(std::string s, std::string delimiter)
{
    std::vector<std::string> list;
    std::string copiedString = s;
    size_t pos_start = 0;
    std::string token;
    
    while ((pos_start = copiedString.find(delimiter)) != std::string::npos) {
        token = copiedString.substr(0, pos_start);
        list.push_back(token);
        copiedString.erase(0, pos_start + delimiter.length());
    }

    list.push_back(copiedString);
    return list;
}


double GetDistance(Point3d a, Point3d b)
{
    return std::sqrt(std::pow(b.x() - a.x(), 2) + std::pow(b.y() - a.y(), 2) + std::pow(b.z() - a.z(), 2));
}


double LineDistance(std::vector<Point3d> points)
{
    double sum = 0;
    for (std::size_t i = 0; i < points.size() - 1; i++)
    {
        sum += GetDistance(points[i], points[i+1]);
    }
    return sum;
}


std::vector<Point3d> BezierCurve::rasterize(int resolution)
{
    resolution--;
    std::vector<Point3d> points;

    for (int i = 0; i <= resolution; i++)
    {
        double t = i / static_cast<double>(resolution);
        points.push_back(this->evaluateParametric(t));
    }

    return points;
}


unsigned int Choose(unsigned int n, unsigned int k)
{
    if (k > n) return 0;
    if (k * 2 > n) k = n-k;
    if (k == 0) return 1;

    unsigned int result = n;
    for (unsigned int i = 2; i <= k; ++i)
    {
        result *= (n - i + 1);
        result /= i;
    }

    return result;
}


Point3d BezierCurve::evaluateParametric(double t)
{
    double x = 0, y = 0, z = 0;
    std::size_t n = this->handles.size() + 1;

    for (std::size_t i = 0; i <= n; i++)
    {
        Point3d pnt(-1, -1, -1);

        // Assign pnt to the appropriate handle
        if (i == 0) {pnt = *s->getLoc();}
        else if (i == handles.size() + 1) {pnt = *e->getLoc();}
        else {pnt = handles[i - 1];}

        // determine next product in summation
        double bin = Choose(n, i) * std::pow((1.0 - t), (n - i)) * std::pow(t, i);

        // Add to current values
        x += bin * pnt.x();
        y += bin * pnt.y();
        z += bin * pnt.z();
    }

    return Point3d(x, y, z);
}


std::vector<IntersectionEdge*> IntersectionRoute::getEdgeList()
{
    std::vector<IntersectionEdge*> edgePtrs;
    
    for (IntersectionEdge& edge : edgeList)
    {
        edgePtrs.push_back(&edge);
    }
    
    return edgePtrs;
}


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
    std::map<std::string, std::shared_ptr<ScenarioNode> > nodeIDMap;

    for (pugi::xml_node xmlNode = nodesList.first_child(); xmlNode; xmlNode = xmlNode.next_sibling())
    {
        std::shared_ptr<ScenarioNode> node = std::make_shared<ScenarioNode>(std::vector<short int>{static_cast<short int>(xmlNode.attribute("x").as_int()), static_cast<short int>(xmlNode.attribute("y").as_int()), static_cast<short int>(xmlNode.attribute("z").as_int())});
        nodeIDMap[xmlNode.attribute("id").value()] = node;
        this->nodes.push_back(node);
    }


    // Append edges to vector.
    pugi::xml_node edgesList = xmlScenario.child("edges");
    for (pugi::xml_node xmlEdge = edgesList.first_child(); xmlEdge; xmlEdge = xmlEdge.next_sibling())
    {
        std::shared_ptr<ScenarioNode> s, e;
        s = nodeIDMap[xmlEdge.attribute("from").value()];
        e = nodeIDMap[xmlEdge.attribute("to").value()];

        std::map<VEHICLETYPES::VEHICLETYPES_, short int> demand;

        for (pugi::xml_attribute attr = xmlEdge.first_attribute(); attr; attr = attr.next_attribute())
        {
            std::string attrName = attr.name();
            std::size_t pos = attrName.find("_demand");

            // If attribute contains demand data.
            if (pos != std::string::npos)
            {
                VEHICLETYPES::VEHICLETYPES_ vehicleType = VEHICLETYPES_INDICES.at(attrName.substr(0, pos));
                short int vehicleDemand = attr.as_int();
                demand[vehicleType] = vehicleDemand;
            }
        }

        edges.push_back(ScenarioEdge(s, e, demand));
    }
}


std::vector<ScenarioEdge*> IntersectionScenario::getEdges()
{
    std::vector<ScenarioEdge*> edgePtrs;
    for (ScenarioEdge& edge : edges)
    {
        edgePtrs.push_back(&edge);
    }
    return edgePtrs;
}


Intersection Intersection::fromSolFile(std::string solFilePath)
{
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(solFilePath.c_str());

    if (!result)
    {
        std::cerr << "Could not parse XML." << std::endl;
        std::cerr << result.description() << std::endl;
    }

    pugi::xml_node xmlScenario = doc.child("scenario");

    // Append nodes to vector.
    pugi::xml_node nodesList = xmlScenario.child("nodes");
    std::vector<std::shared_ptr<IntersectionNode> > nodeVector;
    std::map<std::string, std::shared_ptr<IntersectionNode> > nodeIDMap;

    for (pugi::xml_node xmlNode = nodesList.first_child(); xmlNode; xmlNode = xmlNode.next_sibling())
    {
        std::string strtype = xmlNode.attribute("type").as_string();
        JUNCTIONTYPES::JUNCTIONTYPES_ type = JUNCTIONTYPES_INDICES.at(strtype);
        std::shared_ptr<IntersectionNode> node = std::make_shared<IntersectionNode>(Point3d(static_cast<short int>(xmlNode.attribute("x").as_int()), static_cast<short int>(xmlNode.attribute("y").as_int()), static_cast<short int>(xmlNode.attribute("z").as_int())), type);
        nodeIDMap[xmlNode.attribute("id").value()] = node;
        nodeVector.push_back(node);
    }

    std::vector<IntersectionEdge> edgeVector;
    std::map<std::string, IntersectionEdge> edgeIDMap;

    // Append edges to vector.
    pugi::xml_node edgesList = xmlScenario.child("edges");
    for (pugi::xml_node xmlEdge = edgesList.first_child(); xmlEdge; xmlEdge = xmlEdge.next_sibling())
    {
        std::shared_ptr<IntersectionNode> s, e;
        s = nodeIDMap[xmlEdge.attribute("from").value()];
        e = nodeIDMap[xmlEdge.attribute("to").value()];

        short int priority = static_cast<short int>(xmlEdge.attribute("priority").as_int());
        short int numLanes = static_cast<short int>(xmlEdge.attribute("numLanes").as_int());
        short int speedLimit = static_cast<short int>(xmlEdge.attribute("speed").as_int());
        BezierCurve handles;
        if (xmlEdge.attribute("handles").empty() == false) 
        {
            std::string handleString = xmlEdge.attribute("handles").as_string();
            std::vector<std::string> pointsList = Split(handleString, " ");
            std::vector<Point3d> points;
            for (std::string point : pointsList)
            {
                std::vector<short int> coords;
                for (std::string coord : Split(point, ","))
                {
                    coords.push_back(std::stoi(coord));
                }
                points.push_back(Point3d(coords));
            }
            handles = BezierCurve(s, e, points);
        }
        else
        {
            handles = BezierCurve(s, e, {});
        }

        IntersectionEdge edge = IntersectionEdge(s, e, handles, numLanes, speedLimit, priority);
        edgeIDMap[xmlEdge.attribute("id").value()] = edge;
        edgeVector.push_back(edge);
    }

    std::vector<IntersectionRoute> routes;

    // Append routes to vector.
    pugi::xml_node routesList = xmlScenario.child("routes");
    for (pugi::xml_node xmlRoute = routesList.first_child(); xmlRoute; xmlRoute = xmlRoute.next_sibling())
    {
        std::vector<std::string> edges = Split(xmlRoute.attribute("edges").as_string(), " ");
        std::vector<std::shared_ptr<IntersectionNode> > routeNodes;
        std::vector<IntersectionEdge> routeEdges;
        for (std::size_t i = 0; i < edges.size(); i++)
        {
            if (i == 0)
            {
                routeNodes.push_back(edgeIDMap[edges[i]].getStartNode());
                routeNodes.push_back(edgeIDMap[edges[i]].getEndNode());
            }
            else
            {
                routeNodes.push_back(edgeIDMap[edges[i]].getEndNode());
            }
            routeEdges.push_back(edgeIDMap[edges[i]]);
        }
        routes.push_back(IntersectionRoute(routeNodes, routeEdges));
    }
    return Intersection(routes);
}


void Intersection::Simulate(Intersection* int_, BACKENDS::BACKENDS_ back)
{
    if (back == BACKENDS::SUMO)
    {
        SumoInterface::Get()->rebuildNet(int_);
        SumoInterface::Get()->performSim(SIMTIME_);
    }
}


void Intersection::updateMetrics(BACKENDS::BACKENDS_ back, int safety, double efficiency, double emissions)
{
    if ( back == BACKENDS::TRACI )
    {
        this->currentMetrics[METRICS::SAFETY] = safety;
        this->currentMetrics[METRICS::EFFICIENCY] = efficiency;
        this->currentMetrics[METRICS::EMISSIONS] = emissions;
    } else
    {
        const std::map<METRICS::METRICS_, IntersectionEvalFunc> backendEvaluations = evaluations.at(back);
        for (auto it = backendEvaluations.begin(); it != backendEvaluations.end(); it++)
        {
            (SumoInterface::Get()->*(it->second))(this);
        }
    }
}


double Intersection::getMetric(METRICS::METRICS_ metric)
{
    return this->currentMetrics[metric];
}

std::vector<IntersectionRoute*> Intersection::getRoutes()
{
    std::vector<IntersectionRoute*> routePtrs;
    for (std::size_t i = 0; i < routes.size(); i++)
    {
        routePtrs.push_back(&routes[i]);
    }
    return routePtrs;
}


std::vector<std::shared_ptr<IntersectionNode> > Intersection::getUniqueNodes()
{
    std::vector<std::shared_ptr<IntersectionNode> > uniqueNodes;
    std::vector<short int> nodeIDs;

    for (IntersectionRoute& r : routes)
    {
        for (std::shared_ptr<IntersectionNode>& n : r.getNodeList())
        {
            if (std::find(nodeIDs.begin(), nodeIDs.end(), n->getID()) == nodeIDs.end()) {
                nodeIDs.push_back(n->getID());
                uniqueNodes.push_back(n);
            }
        }
    }

    return uniqueNodes;
}


std::vector<IntersectionEdge*> Intersection::getUniqueEdges()
{
    std::vector<IntersectionEdge*> uniqueEdges;

    for (IntersectionRoute& r : routes)
    {
        for (IntersectionEdge* edge : r.getEdgeList())
        {
            if (std::find(uniqueEdges.begin(), uniqueEdges.end(), edge) == uniqueEdges.end())
            {
                uniqueEdges.push_back(edge);
            }
        }
    }

    return uniqueEdges;
}


/**
 * Sumo XML writing methods. Utilizes PUGIXML to convert
 * Intersection to Sumo-compatible XML files and read Sumo
 * scenario files into IntersectionScenarios
 */

std::string Intersection::getNodeXML()
{
    std::vector<std::shared_ptr<IntersectionNode>> nodes = getUniqueNodes();
    std::string xmlOutput = "<nodes>\n";
    std::stringstream nodeTag;

    for (std::size_t i = 0; i < nodes.size(); i++)
    {
        Point3d* nodeLoc = nodes[i]->getLoc();

        nodeTag << "\t<node ";
        nodeTag << "id=\"" << nodes[i]->getID() << "\" ";
        nodeTag << "x=\"" << nodeLoc->x() / 10.0 << "\" ";
        nodeTag << "y=\"" << nodeLoc->y() / 10.0 << "\" ";
        nodeTag << "z=\"" << nodeLoc->z() / 10.0 << "\" ";        
        nodeTag << "shape=\""  << "\" ";
        nodeTag << "type=\"" << JUNCTIONTYPES_NAMES.at(nodes[i]->getJunctionType()) << "\"/>\n";
        xmlOutput += nodeTag.str();
        nodeTag.str(std::string());
    }

    xmlOutput += "</nodes>";
    return xmlOutput;
}


std::string Intersection::getEdgeXML()
{
    // Node IDs as they will be in the node file generated by getNodeXML.
    std::map<std::shared_ptr<IntersectionNode>, int> sumoNodeIDs;
    std::vector<IntersectionEdge*> edges = getUniqueEdges();
    std::vector<std::shared_ptr<IntersectionNode>> nodes = getUniqueNodes();
    for (std::size_t i = 0; i < nodes.size(); i++) {
        sumoNodeIDs[nodes[i]] = nodes[i]->getID();
    }

    // A map which just holds vectors of start and end ids, in order to make sure there are not duplicate edges
    std::map<short int, short int> connections;
    std::string xmlOutput = "<edges>\n";
    std::stringstream edgeTag;

    for (std::size_t i = 0; i < edges.size(); i++)
    {
        if (edges[i]->getStartNode()->getID() == edges[i]->getEndNode()->getID()) {
            continue;
        }
        if (connections[edges[i]->getStartNode()->getID()] == edges[i]->getEndNode()->getID()) {
            continue;
        }
        connections[edges[i]->getStartNode()->getID()] = edges[i]->getEndNode()->getID();
        edgeTag << "\t<edge id=\"" << i << "e\" ";
        // edgeTag << "from=\"" << sumoNodeIDs[edges[i]->getStartNode()] << "\" ";
        // edgeTag << "to=\"" << sumoNodeIDs[edges[i]->getEndNode()] << "\" ";
        edgeTag << "from=\"" << edges[i]->getStartNode()->getID() << "\" ";
        edgeTag << "to=\"" << edges[i]->getEndNode()->getID() << "\" ";
        edgeTag << "priority=\"" << edges[i]->getPriority() << "\" ";
        edgeTag << "numLanes=\"" << edges[i]->getNumLanes() << "\" ";
        edgeTag << "speed=\"" << edges[i]->getSpeedLimit() << "\" ";

        edgeTag << "shape=\"";
    
        std::vector<Point3d> sampledPoints = edges[i]->getShape().rasterize(BEZIER_SAMPLES);

        for (std::size_t p = 0; p < sampledPoints.size(); p++)
        {
            edgeTag << std::to_string(sampledPoints[p].x() / 10.0) << ","
                    << std::to_string(sampledPoints[p].y() / 10.0) << ","
                    << std::to_string(sampledPoints[p].z() / 10.0);

            if (p != sampledPoints.size() - 1) {edgeTag << " ";}
        }

        edgeTag << "\"/>\n";

        xmlOutput += edgeTag.str();
        edgeTag.str(std::string());
    }

    xmlOutput += "</edges>";
    return xmlOutput;
}

std::string Intersection::getRouteXML(IntersectionScenario intersectionScenario)
{
    std::stringstream xmlStream;

    // Open routes tag.
    xmlStream << "<routes>" << std::endl << "\t";
    // Defined vTypes for car and truck.
    xmlStream << R"(<vType id="car" vClass="passenger"/>)" << std::endl;
    xmlStream << "\t" << R"(<vType id="truck" vClass="truck" />)" << std::endl;

    std::vector<int> edgeStartIDList;
    std::vector<int> edgeEndIDList;
    std::vector<int> edgeIDList;
    std::map<short int, short int> connections;
    std::vector<IntersectionEdge*> edges = this->getUniqueEdges();
    for (int i = 0; i < edges.size(); i++) {
        if (edges[i]->getStartNode()->getID() == edges[i]->getEndNode()->getID()) {
            continue;
        }
        if (connections[edges[i]->getStartNode()->getID()] == edges[i]->getEndNode()->getID()) {
            continue;
        }
        connections[edges[i]->getStartNode()->getID()] = edges[i]->getEndNode()->getID();
        edgeStartIDList.push_back(edges[i]->getStartNode()->getID());
        edgeEndIDList.push_back(edges[i]->getEndNode()->getID());
        edgeIDList.push_back(i);
    }
    std::vector<ScenarioEdge*> scenarioEdges = intersectionScenario.getEdges();    
    std::vector<VEHICLETYPES::VEHICLETYPES_> vehicleTypes = {VEHICLETYPES::CAR, VEHICLETYPES::TRUCK};

    std::stringstream openFlowTag;
    std::stringstream routeTag;
    for (std::size_t i = 0; i < routes.size(); i++)
    {
        std::vector<IntersectionEdge*> routeEdges = routes[i].getEdgeList();
        std::map<VEHICLETYPES::VEHICLETYPES_, short int> demand;
        for (ScenarioEdge* scenarioEdge : scenarioEdges)
        {
            // If edge corresponds to current route.
            if (routeEdges[0]->getStartNode()->getID() == scenarioEdge->getStartNode()->getID() && routeEdges[routeEdges.size()-1]->getEndNode()->getID() == scenarioEdge->getEndNode()->getID())
            {
                demand = scenarioEdge->getDemand();
                break;
            }
        }

        // Create opening flow tag.
        for (VEHICLETYPES::VEHICLETYPES_ vehicleType : vehicleTypes) 
        {
            openFlowTag << "\t<flow ";
            // Ex. id="3v0" for the fourth flow and the first vehicle.
            openFlowTag << "id=\"flow_" << i << "v" << vehicleType << "\" ";
            openFlowTag << "begin=\"0.00\" ";
            // openFlowTag << "end=\"604800.00\" ";
            openFlowTag << "end=\"800.00\" ";
            openFlowTag << "vehsPerHour=\"" << demand[vehicleType] << "\" ";
            openFlowTag << "type=\"" << VEHICLETYPES_NAMES.at(vehicleType) << "\" ";
            openFlowTag << ">\n";

            xmlStream << openFlowTag.str();
            openFlowTag.str(std::string());

            // Create route tag within flow tag.
            routeTag << "\t\t<route edges=\"";
            for (std::size_t e = 0; e < routeEdges.size(); e++)
            {
                for (int f = 0; f < edgeStartIDList.size(); f++) {
                    if (edgeStartIDList[f] == routeEdges[e]->getStartNode()->getID() && edgeEndIDList[f] == routeEdges[e]->getEndNode()->getID()) {
                        // Add a space before the edge if it's not the first in the route.
                        if (e == 0) routeTag << edgeIDList[f] << "e";
                        else routeTag << " " << edgeIDList[f] << "e";
                        break;
                    }
                }
                // int id = edgeIDList.at({routeEdges[e]->getStartNode()->getID(), routeEdges[e]->getEndNode()->getID()});
            }
            routeTag << "\" />" << std::endl;
            xmlStream << routeTag.str();
            routeTag.str(std::string());

            xmlStream << "\t</flow>" << std::endl;
        }
        
    }

    xmlStream << "</routes>";
    return xmlStream.str();
}

// std::string Intersection::getConXML() {
//     std::string xmlOutput = "<connections>\n";

//     std::map<short int, short int> connectionMap;
//     std::map<short int, short int> edgeConnections;
//     std::vector<std::shared_ptr<IntersectionNode>> nodes = getUniqueNodes();
//     std::map<std::shared_ptr<IntersectionNode>, short int> nodeIDMap; 
//     std::vector<IntersectionEdge*> edges = getUniqueEdges();
//     std::map<IntersectionEdge*, short int> edgeIDMap; 
//     for (int i = 0; i < nodes.size(); i++) {
//         nodeIDMap[nodes[i]] = i;
//     }
//     for (int i = 0; i < edges.size(); i++) {
//         if (edges[i]->getStartNode() == edges[i]->getEndNode()) {
//             continue;
//         }
//         if (edgeConnections[edges[i]->getStartNode()->getID()] == edges[i]->getEndNode()->getID()) {
//             continue;
//         }
//         edgeConnections[edges[i]->getStartNode()->getID()] = edges[i]->getEndNode()->getID();
//         edgeIDMap[edges[i]] = i;
//     }
//     // for (IntersectionRoute* route : getRoutes()) {
//     std::vector<IntersectionRoute*> routeList = getRoutes();
//     for (int i = 0; i < routeList.size(); i++) {
//         IntersectionRoute* route = routeList[i];
//         std::vector<IntersectionEdge*> edgeList = route->getEdgeList();
//         for (int j = 0; j < edgeList.size()-1; j++) {
//             short int fromEdgeID = edgeIDMap[edgeList[j]];
//             short int toEdgeID = edgeIDMap[edgeList[j+1]];
//             if (connectionMap.find(fromEdgeID) != connectionMap.end()) {
//                 if (connectionMap[fromEdgeID] == toEdgeID) {
//                 continue;
//                 }
//             }
//             else {
//                 connectionMap[fromEdgeID] = toEdgeID;
//                 for (int k = 0; k < edgeList[j]->getNumLanes(); k++) {
//                     for (int l = 0; l < edgeList[j+1]->getNumLanes(); l++) {
//                         std::stringstream connectionText;
//                         connectionText << "\t<connection ";
//                         connectionText << "from=\"" << fromEdgeID << "e\" ";
//                         connectionText << "to=\"" << toEdgeID << "e\" ";
//                         connectionText << "fromLane=\"" << k << "\" ";
//                         connectionText << "toLane=\"" << l << "\" ";
//                         connectionText << "/>\n";
//                         xmlOutput += connectionText.str();
//                         connectionText.clear();
//                     }
//                 }
//             }
//         }    
//     }
//     xmlOutput += "</connections>";
//     return xmlOutput;

// }

std::string Intersection::getSolXML() 
{
    std::string xmlOutput = "<scenario> \n \t<nodes>\n";
    for (std::shared_ptr<IntersectionNode>& node : getUniqueNodes()) {
        std::stringstream nodeText;
        nodeText << "\t\t<node ";
        nodeText << "id=\"" << std::to_string(node->getID()) << "\" ";
        std::string x = std::to_string(node->getLoc()->x());
        std::string y = std::to_string(node->getLoc()->y());
        std::string z = std::to_string(node->getLoc()->z());
        nodeText << "x=\"" << x << "\" y=\"" << y << "\" z=\"" << z << "\" ";
        std::string type = JUNCTIONTYPES_NAMES.at(node->getJunctionType());
        nodeText << "type=\"" << type << "\" ";
        nodeText << "/>\n";
        
        xmlOutput += nodeText.str();
        nodeText.str(std::string());
    }

    xmlOutput += "\t</nodes>\n\n\t<edges>\n";

    std::vector<IntersectionEdge*> edgeList = getUniqueEdges();
    std::map<IntersectionEdge*, int> edgeIDMap;

    for (std::size_t i = 0; i < edgeList.size(); i++)
    {
        IntersectionEdge* edge = edgeList[i];
        edgeIDMap[edge] = i;

        std::stringstream edgeText;
        edgeText << "\t\t<edge ";
        edgeText << "id=\"" << std::to_string(i) << "\" ";
        // std::shared_ptr<IntersectionNode>& startNode = edge->getStartNode();
        // std::shared_ptr<IntersectionNode>& endNode = edge->getEndNode();
        edgeText << "from=\"" << std::to_string(edge->getStartNode()->getID()) << "\" ";
        edgeText << "to=\"" << std::to_string(edge->getEndNode()->getID()) << "\" ";
        edgeText << "priority=\"" << std::to_string(edge->getPriority()) << "\" ";
        edgeText << "numLanes=\"" << std::to_string(edge->getNumLanes()) << "\" ";
        edgeText << "speed=\"" << std::to_string(edge->getSpeedLimit()) << "\" ";

        std::string coordsText;
        std::vector<Point3d> points = edge->getShape().getHandles();
        if (points.size() > 0) {
            for (Point3d point : points) {
                coordsText += " ";
                std::string coordText = std::to_string(point.x()) + "," + std::to_string(point.y()) + "," + std::to_string(point.z());
                coordsText += coordText;
            }
            edgeText << "handles=\"" << coordsText.substr(1) << "\" ";
        }
        edgeText << "/>\n";

        xmlOutput += edgeText.str();
        edgeText.str(std::string());
    }
    
    xmlOutput += "\t</edges>\n\n\t<routes>\n";
     
    for (IntersectionRoute* route : this->getRoutes())
    {
        std::stringstream routeText;
        routeText << "\t\t<route ";
        
        std::string edgesText;
        for (IntersectionEdge* edge : route->getEdgeList())
        {
            edgesText += " ";
            edgesText += std::to_string(edgeIDMap[edge]);
        }
        routeText << "edges=\"" << edgesText.substr(1) << "\" ";
        routeText << "/>\n";

        xmlOutput += routeText.str();
        routeText.str(std::string());
    }

    xmlOutput += "\t</routes>\n</scenario>";
    return xmlOutput;
}


/**
 * All SUMO related backend helper methods
 */


void SumoInterface::performSim(const std::size_t time)
{
    net->simulate(0, SIMTIME);
}


void SumoInterface::rebuildNet(Intersection* iint)
{
    MSJunctionControl* junctionCtl = this->buildSumoJunctions(iint->getUniqueNodes());
    MSEdgeControl* edgeCtl = this->buildSumoEdges(iint->getUniqueEdges(), iint->getUniqueNodes(), junctionCtl);
    
    SUMORouteLoaderControl* routeLoaders = new SUMORouteLoaderControl(0);
    MSTLLogicControl* logicCtl = new MSTLLogicControl();

    this->net->closeBuilding(OptionsCont::getOptions(), edgeCtl, junctionCtl, routeLoaders, logicCtl, {0, 100}, {"test1.state", "test2.state"}, false, 1.05);
}


MSJunctionControl* SumoInterface::buildSumoJunctions(std::vector<std::shared_ptr<IntersectionNode> > iinodes)
{
    MSJunctionControl* ctl = new MSJunctionControl(); 
    
    for (std::shared_ptr<IntersectionNode> iinode : iinodes) 
    {
        ctl->add(std::to_string(iinode->getID()), this->buildSumoJunction(iinode));
    }

    return ctl;
}


MSJunction* SumoInterface::buildSumoJunction(std::shared_ptr<IntersectionNode> iinode)
{
    MSJunction* junc = new MSJunction(std::to_string(iinode->getID()), SumoJunctionMap[iinode->getJunctionType()], this->Point3dToPosition(*(iinode->getLoc())), std::vector<Position>{Point3dToPosition({0,0,0})}, "test");
    nodeJunctionMap[iinode] = junc;
    return junc;
}


MSEdgeControl* SumoInterface::buildSumoEdges(std::vector<IntersectionEdge*> iiedges, std::vector<std::shared_ptr<IntersectionNode> > iinodes, MSJunctionControl* junctionCtl)
{
    MSEdgeVector edges;
    for (IntersectionEdge* iiedge : iiedges)
    {
        MSEdge* edge = this->buildSumoEdge(iiedge);
        edges.push_back(edge);
    }

    MSEdgeControl* ctl = new MSEdgeControl(edges);
    return ctl; 
}


MSEdge* SumoInterface::buildSumoEdge(IntersectionEdge* iiedge)
{
    MSEdge* edge = new MSEdge(std::string(), iiedge->getStartNode()->getID(), SumoXMLEdgeFunc::UNKNOWN, "mr bob's street", "UNKNOWN", iiedge->getPriority(), LineDistance(iiedge->getShape().rasterize(BEZIER_SAMPLES)));    
    std::vector<MSLane*>* lanes = new std::vector<MSLane*> (this->buildSumoLanes(iiedge, edge));
    edge->initialize(lanes);
    edge->setJunctions(nodeJunctionMap[iiedge->getStartNode()], nodeJunctionMap[iiedge->getEndNode()]);
    MSEdge::dictionary(std::to_string(iiedge->getId()), edge);
    return edge;
}


std::vector<MSLane*> SumoInterface::buildSumoLanes(IntersectionEdge* iiedge, MSEdge* edge)
{
    int lanes = iiedge->getNumLanes();
    std::vector<MSLane*> laneVec;
    
    for (int i = 0; i < lanes; i++)
    {
        laneVec.push_back(this->buildSumoLane(iiedge, edge, i));
    }

    return laneVec;
}


MSLane* SumoInterface::buildSumoLane(IntersectionEdge* iiedge, MSEdge* edge, int id)
{
    PositionVector shape;
    for (Point3d p : iiedge->getShape().rasterize(BEZIER_SAMPLES))
    {
        shape.add(this->Point3dToPosition(p));
    }

    MSLane* lane = new MSLane(std::to_string(id), iiedge->getSpeedLimit(), static_cast<float>(LineDistance(iiedge->getShape().rasterize(BEZIER_SAMPLES))), edge, id, shape, -1, SVC_UNSPECIFIED, SVC_UNSPECIFIED, SVC_UNSPECIFIED, id, false, "UNSPECIFIED");
    MSLane::dictionary(std::to_string(id), lane);
    return lane;
}


void SumoInterface::constructNet()
{
    MSFrame::fillOptions();
    OptionsCont* options = &OptionsCont::getOptions();
    MSFrame::setMSGlobals(*options);
    MSLane::initRNGs(*options);
    // options->doRegister("no-duration-log", new Option_Bool(true));
    // options->doRegister("no-step-log", new Option_Bool(false));

    MSVehicleControl* vc = new MSVehicleControl();
    MSEventControl* ec = new MSEventControl();
    MSEventControl* end = new MSEventControl();
    MSEventControl* insertion = new MSEventControl();
    ShapeContainer* shape = new ShapeContainer();
    net = new MSNet(vc, ec, end, insertion, shape);
}


void SumoInterface::updateIntersectionEfficiency(Intersection* int_) {
    // this->net->getTravelTime();
}
}


#endif