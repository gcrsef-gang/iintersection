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

#ifndef LIBIINTERSECTION.H
#define LIBIINTERSECTION.H

#ifdef SUMO_LIB
// #include <sumo/main.h>
#endif


#include <map>
#include <vector>


/**
 * \brief Entire iintersection namespace, with different backend interfaces
 * Contains input and solution classes, as well as ways to evaluate them
 */
namespace ii {


// forward class declarations
class Intersection;
class SumoInterface;

static unsigned short int CURRENT_UUID_MAX;
static const std::size_t SIMTIME;

const enum class METRICS {SAFETY, EMISSIONS, EFFICIENCY};
const enum class BACKENDS {SUMO, VISSIM, CITYFLOW};
const enum class VEHICLETYPES {CAR, TRUCK, IDK};

// i need some help with these values, what are we actually doing here?
const enum class JUNCTIONTYPE {TRAFFICLIGHT, SCENARIONODE};

typedef void (::ii::BackendsManager::*IntersectionEvalFunc)(const ::ii::Intersection*);


class BackendsManager
{
public:
    void updateIntersectionEmissions(const Intersection*);
    virtual void updateIntersectionSafety(const Intersection*) = 0;
    virtual void updateIntersectionEfficiency(const Intersection*) = 0;
};


class SumoInterface : public BackendsManager
{
public:
    void rebuildNet(const Intersection*);
    void performSim(const std::size_t time);
    static SumoInterface* getInstance();

private:
    // MSNet* net;

friend class Intersection;
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
    const short int x_, y_, z_;
};


class BezierCurve
{
public:
    BezierCurve(IntersectionNode* s, IntersectionNode* e, std::vector<Point3d> handles) : s(s), e(e), handles(handles) {}
    std::vector<Point3d> rasterize();
    IntersectionNode* getStartNode() const {return this->s;}
    IntersectionNode* getEndNode() const {return this->e;}
    std::vector<Point3d> getHandles() const {return this->handles;}


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
    IntersectionNode(Point3d loc, JUNCTIONTYPE junctionType) : Node(loc), junctionType(junctionType) {}
    JUNCTIONTYPE getJunctionType() {return this->junctionType;}
private:
    JUNCTIONTYPE junctionType;
};


class Edge
{
public:
    Edge(Node* s, Node* e) : s(s), e(e) {};
    unsigned short int getStartNode() const {return s->getID();}
    unsigned short int getEndNode() const {return e->getID();}

private:
    Node* s; // starting node
    Node* e; // ending node
};


class IntersectionEdge : public Edge
{
public:
    IntersectionEdge(IntersectionNode* s, IntersectionNode* e, BezierCurve shape, short int numLanes, short int speedLimit) : Edge(s, e), shape(shape) {}
    BezierCurve getShape() const {return (shape);}

private:
    BezierCurve shape;
};


class ScenarioEdge : public Edge {
public:
    ScenarioEdge(Node* s, Node* e, std::map<VEHICLETYPES, short int> demand) : Edge(s, e), demand(demand) {}
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
    std::vector<IntersectionRoute*> routes;

private:
    std::map<METRICS, double> currentMetrics;
    const static std::map<BACKENDS, std::map<METRICS, IntersectionEvalFunc> > evaluations;
};


// const std::map<BACKENDS, std::map<METRICS, IntersectionEvalFunc> > Intersection::evaluations = 
// {
//     {
//         BACKENDS::SUMO, {
//             {METRICS::EFFICIENCY, BackendsManager::updateIntersectionEfficiency}
//             // {METRICS::SAFETY, SumoInterface::updateIntersectionSafety},
//             // {METRICS::EMISSIONS, SumoInterface::updateIntersectionEmissions}
//         }
//     }
// };

// IntersectionEvalFunc evals = &::ii::BackendsManager::updateIntersectionEmissions;


class IntersectionScenario
{
public:
    IntersectionScenario(std::vector<Node*> nodes, std::vector<ScenarioEdge> edges) : nodes(nodes), edges(edges) {}
    std::vector<Node*> getNodes() const {return this->nodes;}
    std::vector<ScenarioEdge> getEdges() const {return this->edges;}

private:
    std::vector<Node*> nodes;
    std::vector<ScenarioEdge> edges;
};





void Intersection::simulate(BACKENDS back) const
{
    if (back == BACKENDS::SUMO)
    {
        SumoInterface::getInstance()->rebuildNet(this);
        SumoInterface::getInstance()->performSim(SIMTIME);
    }
}


void Intersection::updateMetrics(BACKENDS back)
{
    const std::map<METRICS, IntersectionEvalFunc> backendEvaluations = evaluations.at(back);

    for (auto it = backendEvaluations.begin(); it != backendEvaluations.end(); it++)
    {
        IntersectionEvalFunc func = (it->second);
        (SumoInterface::getInstance()->*func)(this);
    }
}


}


#endif