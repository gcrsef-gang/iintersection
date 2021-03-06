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
#include <utils/options/OptionsCont.h>
#include <microsim/MSEdgeControl.h>
#endif


#include <algorithm>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include <list>

#include <pugixml/src/pugixml.hpp>


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
class IntersectionEdge;
class DataManager;
class Node;
class Point3d;
class IntersectionRoute;


// Tracking current max node ID
static unsigned short int CURRENT_NODE_UUID_MAX = 0;
static unsigned short int CURRENT_EDGE_UUID_MAX = 0;

static const short int BEZIER_SAMPLES = 500;
static const std::size_t SIMTIME_ = 604800;  // Seconds of simulation time


namespace METRICS {enum METRICS_ {SAFETY, EMISSIONS, EFFICIENCY};}
namespace BACKENDS {enum BACKENDS_ {SUMO, VISSIM, CITYFLOW};}
namespace VEHICLETYPES {enum VEHICLETYPES_ {CAR, TRUCK, IDK};}
namespace JUNCTIONTYPES {enum JUNCTIONTYPES_ {PRIORITY, TRAFFIC_LIGHT, RIGHT_BEFORE_LEFT, UNREGULATED, PRIORITY_STOP, TRAFFIC_LIGHT_UNREGULATED, ALLWAY_STOP, TRAFFIC_LIGHT_RIGHT_ON_RED};}

const std::map<std::string, VEHICLETYPES::VEHICLETYPES_> VEHICLETYPES_INDICES = {{"car", VEHICLETYPES::CAR}, {"truck", VEHICLETYPES::TRUCK}, {"idk", VEHICLETYPES::IDK}};
const std::map<JUNCTIONTYPES::JUNCTIONTYPES_, std::string> JUNCTIONTYPES_NAMES = {{JUNCTIONTYPES::PRIORITY, "priority"}, {JUNCTIONTYPES::TRAFFIC_LIGHT, "traffic_light"}, {JUNCTIONTYPES::RIGHT_BEFORE_LEFT, "right_before_left"}, {JUNCTIONTYPES::UNREGULATED, "unregulated"}, {JUNCTIONTYPES::PRIORITY_STOP, "priority_stop"}, {JUNCTIONTYPES::TRAFFIC_LIGHT_UNREGULATED, "traffic_light_unregulated"}, {JUNCTIONTYPES::ALLWAY_STOP, "allway_stop"}, {JUNCTIONTYPES::TRAFFIC_LIGHT_RIGHT_ON_RED, "traffic_light_on_red"}};


std::map<JUNCTIONTYPES::JUNCTIONTYPES_, SumoXMLNodeType> SumoJunctionMap = {
    // Fill this in later pls
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
    Point3d(short int x, short int y, short int z) : x_(x), y_(y), z_(z) {};
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

    MSEdgeControl* buildSumoEdges(std::vector<IntersectionEdge*>, std::vector<IntersectionNode*>, MSJunctionControl*);
    MSEdge* buildSumoEdge(IntersectionEdge*);

    std::vector<MSLane*> buildSumoLanes(IntersectionEdge*, MSEdge*);
    MSLane* buildSumoLane(IntersectionEdge*, MSEdge*, int);

    MSJunctionControl* buildSumoJunctions(std::vector<IntersectionNode*>);
    MSJunction* buildSumoJunction(IntersectionNode*);

    std::map<Node*, MSJunction*> nodeJunctionMap;
    std::vector<MSJunction> junctions;
    std::vector<MSEdge> edges;
};


class BezierCurve
{
public:
    BezierCurve() {}
    BezierCurve(IntersectionNode* s, IntersectionNode* e, std::vector<Point3d> handles) : s(s), e(e), handles(handles) {
        // assert((this->handles.size() == 1 || this->handles.size() == 2));
    }

    std::vector<Point3d> rasterize(int resolution);

    IntersectionNode* getStartNode() const {return this->s;}
    IntersectionNode* getEndNode() const {return this->e;}
    std::vector<Point3d> getHandles() const {return this->handles;}

    void setHandles(std::vector<Point3d> handles_) {this->handles = handles_;}

private:
    Point3d evaluateParametric(double t);

    void setStartNode(IntersectionNode*);
    void setEndNode(IntersectionNode*);
    IntersectionNode* s;
    IntersectionNode* e;
    std::vector<Point3d> handles;

friend class IntersectionEdge;
};


class Node
{
public:
    // Node() {}
    Node(Point3d loc) : loc(loc) {this->UUID = ++CURRENT_NODE_UUID_MAX;}
    Point3d* getLoc() {return &(this->loc);}
    unsigned short int getID() const {return this->UUID;}

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
    void addReference() {this->referenceCount++;}
    bool removeReference();

private:

    JUNCTIONTYPES::JUNCTIONTYPES_ junctionType; 
    unsigned short int referenceCount;
};


class Edge
{
public:
    Edge() {}
    Edge(Node* s, Node* e) : s(s), e(e) {this->UUID = ++CURRENT_EDGE_UUID_MAX;};
    Node* getStartNode() const {return s;}
    Node* getEndNode() const {return e;}

protected:
    unsigned short int UUID;

private:
    Node* s; // starting node
    Node* e; // ending node
};


class IntersectionEdge : public Edge
{
public:
    IntersectionEdge() {}
    IntersectionEdge(IntersectionNode* s, IntersectionNode* e, BezierCurve shape, short int numLanes, short int speedLimit, short int priority)
        : Edge(s, e), s(s), e(e), shape(shape), numlanes(numLanes), speedlimit(speedLimit), priority(priority) {}

    BezierCurve getShape() const {return this->shape;}
    short int getNumLanes() const {return this->numlanes;}
    short int getSpeedLimit() const {return this->speedlimit;}
    short int getPriority() const {return this->priority;}
    
    
    IntersectionNode* getStartNode() const {return s;}
    IntersectionNode* getEndNode() const {return e;}


    void setStartNode(IntersectionNode*);
    void setEndNode(IntersectionNode*);

    void setHandles(std::vector<Point3d> handles) {this->shape.setHandles(handles);}
    void setNumLanes(short int numLanes_) {this->numlanes = numLanes_;}
    void setSpeedLimit(short int speedLimit_) {this->speedlimit = speedLimit_;}
    void setPriority(short int priority_) {this->priority = priority_;}

private:
    IntersectionNode* s;
    IntersectionNode* e;

    BezierCurve shape;
    short int numlanes;
    short int speedlimit;
    short int priority;

friend bool operator==(const IntersectionEdge& e1, const IntersectionEdge& e2) {return e1.getStartNode() == e2.getStartNode() && e1.getEndNode() == e2.getStartNode();}

// bit of a janky fix pls dont pay attention
friend bool operator==(const IntersectionEdge* e1, const IntersectionEdge& e2) {return e1->getStartNode() == e2.getStartNode() && e1->getEndNode() == e2.getStartNode();}
};


class ScenarioEdge : public Edge
{
public:
    ScenarioEdge() {}
    ScenarioEdge(ScenarioNode* s, ScenarioNode* e, std::map<VEHICLETYPES::VEHICLETYPES_, short int> demand) : Edge(s, e), demand(demand) {}

    std::map<VEHICLETYPES::VEHICLETYPES_, short int> getDemand() {return this->demand;}

private:
    std::map<VEHICLETYPES::VEHICLETYPES_, short int> demand;
};


class IntersectionRoute
{
public:
    IntersectionRoute() {}
    IntersectionRoute(std::vector<IntersectionNode*> nodeList, std::vector<IntersectionEdge> edgeList)
        : nodeList(nodeList), edgeList(edgeList) {}

    std::vector<IntersectionNode*> getNodeList() {return this->nodeList;}
    std::vector<IntersectionEdge*> getEdgeList()
    {
        std::vector<IntersectionEdge*> vec;
        for (int i = 0; i < edgeList.size(); i++) {
            vec.push_back(&(edgeList.at(i)));
        }
        return vec;
    }
    
    void setNodeList(std::vector<IntersectionNode*> nodelist) {nodeList = nodelist;}
    void setEdgeList(std::vector<IntersectionEdge> edgelist) {edgeList = edgelist;}

private:
    std::vector<IntersectionNode*> nodeList;
    std::vector<IntersectionEdge> edgeList;
};


class Intersection
{
public:
    Intersection() {}
    Intersection(std::vector<IntersectionRoute> routes) : routes(routes) {}
    ~Intersection()
    {
        for (IntersectionNode* node : this->getUniqueNodes()) {delete node;}
    }

    static void Simulate(Intersection*, BACKENDS::BACKENDS_);
    void updateMetrics(BACKENDS::BACKENDS_);
    double getMetric(METRICS::METRICS_);

    std::vector<IntersectionRoute>* getRoutes() {return &routes;};
    std::vector<IntersectionNode*> getUniqueNodes();
    std::vector<IntersectionEdge*> getUniqueEdges();

    std::string getEdgeXML() const;
    std::string getNodeXML() const;

private:
    std::vector<IntersectionRoute> routes;
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
    IntersectionScenario(std::vector<ScenarioNode*> nodes, std::vector<ScenarioEdge> edges) : nodes(nodes), edges(edges) {}
    IntersectionScenario(std::string xmlFilePath);

    ~IntersectionScenario();
    std::vector<ScenarioNode*> getNodes() const {return this->nodes;}
    std::vector<ScenarioEdge> getEdges() const {return this->edges;}

private:
    std::vector<ScenarioNode*> nodes;
    std::vector<ScenarioEdge> edges;
};


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
    for (int i = 2; i <= k; ++i)
    {
        result *= (n - i + 1);
        result /= i;
    }

    return result;
}


double GetDistance(Point3d a, Point3d b)
{
    return std::sqrt(std::pow(b.x() - a.x(), 2) + std::pow(b.y() - a.y(), 2) + std::pow(b.z() - a.z(), 2));
}


double LineDistance(std::vector<Point3d> points)
{
    double sum = 0;
    for (int i = 0; i < points.size() - 1; i++)
    {
        sum += GetDistance(points[i], points[i+1]);
    }
    return sum;
}


Point3d BezierCurve::evaluateParametric(double t)
{
    double x = 0, y = 0, z = 0;
    int n = this->handles.size() + 1;

    for (int i = 0; i <= n; i++)
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


bool IntersectionNode::removeReference()
{
    this->referenceCount--;
    if (referenceCount == 1)
    {
        delete this;
        return true;
    }

    return false;
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
    std::map<std::string, ScenarioNode*> nodeIDMap;
    for (pugi::xml_node xmlNode = nodesList.first_child(); xmlNode; xmlNode = xmlNode.next_sibling())
    {
        ScenarioNode* node = new ScenarioNode({static_cast<short int>(xmlNode.attribute("x").as_int()), static_cast<short int>(xmlNode.attribute("y").as_int()), static_cast<short int>(xmlNode.attribute("z").as_int())});
        nodeIDMap[xmlNode.attribute("id").value()] = node;
        this->nodes.push_back(node);
    }


    // Append edges to vector.
    pugi::xml_node edgesList = xmlScenario.child("edges");
    for (pugi::xml_node xmlEdge = edgesList.first_child(); xmlEdge; xmlEdge = xmlEdge.next_sibling())
    {
        ScenarioNode* s, *e;
        s = nodeIDMap[xmlEdge.attribute("from").value()];
        e = nodeIDMap[xmlEdge.attribute("to").value()];

        std::map<VEHICLETYPES::VEHICLETYPES_, short int> demand;

        for (pugi::xml_attribute attr = xmlEdge.first_attribute(); attr; attr = attr.next_attribute())
        {
            std::string attrName = attr.name();
            int pos = attrName.find("_demand");

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


void Intersection::Simulate(Intersection* int_, BACKENDS::BACKENDS_ back)
{
    if (back == BACKENDS::SUMO)
    {
        SumoInterface::Get()->rebuildNet(int_);
        SumoInterface::Get()->performSim(SIMTIME_);
    }
}


void Intersection::updateMetrics(BACKENDS::BACKENDS_ back)
{
    const std::map<METRICS::METRICS_, IntersectionEvalFunc> backendEvaluations = evaluations.at(back);
    for (auto it = backendEvaluations.begin(); it != backendEvaluations.end(); it++)
    {
        (SumoInterface::Get()->*(it->second))(this);
    }
}


double Intersection::getMetric(METRICS::METRICS_ metric)
{
    return this->currentMetrics[metric];
}


/**
 * Sumo XML writing methods. Utilizes PUGIXML to convert
 * Intersection to Sumo-compatible XML files and read Sumo
 * scenario files into IntersectionScenarios
 */

std::string Intersection::getNodeXML() const
{
    std::vector<IntersectionNode*> nodes;
    for (IntersectionRoute route : routes)
    {
        for (IntersectionNode* node : route.getNodeList())
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

    for (IntersectionRoute route : routes)
    {
        edges.insert(edges.end(), route.getEdgeList().begin(), route.getEdgeList().end());
        std::vector<IntersectionNode*> routeNodes = route.getNodeList();

        for (int i = 0; i < routeNodes.size(); i++)
        {
            sumoNodeIDs[routeNodes[i]] = i;
        }
    }

    std::string xmlOutput = "<edges>\n";
    std::stringstream edgeTag;

    for (int i = 0; i < edges.size(); i++)
    {
        edgeTag << "\t<edge id=\"" << i << "e\" ";
        edgeTag << "from=\"" << sumoNodeIDs[static_cast<IntersectionNode*>(edges[i].getStartNode())] << "\" ";
        edgeTag << "to=\"" << sumoNodeIDs[static_cast<IntersectionNode*>(edges[i].getEndNode())] << "\" ";
        edgeTag << "priority=\"" << edges[i].getPriority() << "\" ";
        edgeTag << "numLanes=\"" << edges[i].getNumLanes() << "\" ";
        edgeTag << "speed=\"" << edges[i].getSpeedLimit() << "\" ";

        edgeTag << "shape=\"";
    
        std::vector<Point3d> sampledPoints = edges[i].getShape().rasterize(BEZIER_SAMPLES);

        for (int p = 0; p < sampledPoints.size(); p++)
        {
            edgeTag << std::to_string(sampledPoints[p].x()) << ","
                    << std::to_string(sampledPoints[p].y()) << ","
                    << std::to_string(sampledPoints[p].z());

            if (p != sampledPoints.size() - 1) {edgeTag << " ";}
        }

        edgeTag << "\"/>\n";

        xmlOutput += edgeTag.str();
        edgeTag.clear();
    }

    xmlOutput += "</edges>";
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
}


MSJunctionControl* SumoInterface::buildSumoJunctions(std::vector<IntersectionNode*> iinodes)
{
    MSJunctionControl* ctl = new MSJunctionControl(); 
    
    for (IntersectionNode* iinode : iinodes) 
    {
        ctl->add(std::to_string(iinode->getID()), this->buildSumoJunction(iinode));
    }

    return ctl;
}


MSJunction* SumoInterface::buildSumoJunction(IntersectionNode* iinode)
{
    MSJunction* junc = new MSJunction(std::to_string(iinode->getID()), SumoJunctionMap[iinode->getJunctionType()], this->Point3dToPosition(*(iinode->getLoc())), std::vector<Position>{Point3dToPosition({0,0,0})}, "test");
    nodeJunctionMap[iinode] = junc;
    return junc;
}


MSEdgeControl* SumoInterface::buildSumoEdges(std::vector<IntersectionEdge*> iiedges, std::vector<IntersectionNode*> iinodes, MSJunctionControl* junctionCtl)
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
    MSEdge* edge = new MSEdge(std::string(""), iiedge->getStartNode()->getID(), SumoXMLEdgeFunc::UNKNOWN, "mr bob's street", "UNKNOWN", iiedge->getPriority(), LineDistance(iiedge->getShape().rasterize(BEZIER_SAMPLES)));    
    std::vector<MSLane*> lanes = this->buildSumoLanes(iiedge, edge);
    edge->initialize(&lanes);
    edge->setJunctions(nodeJunctionMap[iiedge->getStartNode()], nodeJunctionMap[iiedge->getEndNode()]);
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