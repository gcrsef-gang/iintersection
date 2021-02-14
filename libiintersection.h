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



class Intersection;


static short int CURRENT_UUID_MAX;
const enum class METRICS {SAFETY, EMISSIONS, EFFICIENCY};
const enum class BACKENDS {SUMO, VISSIM, CITYFLOW};
const enum class VEHICLETYPES {CAR, TRUCK, IDK};

// i need some help with these values, what are we actually doing here?
const enum class JUNCTIONTYPE {TRAFFICLIGHT, WHATEVER};


class SumoInterface
{
public:
    void rebuildNet(Intersection*);
    void performSim(std::size_t time);
    void updateIntersectionEmissions(Intersection*);
    void updateIntersectionSafety(Intersection*);
    void updateIntersectionEfficiency(Intersection*);

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


template<class T>
class BezierCurve
{
public:
    BezierCurve(std::vector<Point3d> handles) {}
    std::vector<Point3d> rasterize();

private:
};


class Node
{
public:
    Node(Point3d) {this->UUID = ++CURRENT_UUID_MAX;}
    Point3d* getLoc() {return &(this->loc);}
private:
    int UUID;
    Point3d loc;
friend class Intersection;
};


class IntersectionNode : public Node
{
public:
    JUNCTIONTYPE getJunctionType() {return this->junctionType;}
private:
    JUNCTIONTYPE junctionType;
};


class Edge
{
public:
    Edge(Node* s, Node* e) : s(s), e(e) {};

private:
    Node* s; // starting node
    Node* e; // ending node
};


class IntersectionEdge : public Edge
{
public:
    IntersectionEdge(IntersectionNode* s, IntersectionNode* e, BezierCurve<IntersectionNode*> shape) : Edge(s, e), shape(shape) {}
private:
    BezierCurve<IntersectionNode* > shape;
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
    const static std::map<BACKENDS, std::map<METRICS, void(::ii::SumoInterface::*)(Intersection*)> > evaluations;
};


const std::map<BACKENDS, std::map<METRICS, void(::ii::SumoInterface::*)(Intersection*)> > Intersection::evaluations = 
{
    {
        BACKENDS::SUMO, {
            {METRICS::EFFICIENCY, ::ii::SumoInterface::updateIntersectionEfficiency},
            {METRICS::SAFETY, ::ii::SumoInterface::updateIntersectionSafety},
            {METRICS::EMISSIONS, ::ii::SumoInterface::updateIntersectionEmissions}
        }
    }
};


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

}

#endif