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


const enum class METRICS {SAFETY, EMISSIONS, EFFICIENCY};
const enum class BACKENDS {SUMO, VISSIM, CITYFLOW};
const enum class JUNCTIONTYPE {TRAFFICLIGHT, WHATEVER};


class SumoInterface
{
public:
    void rebuildNet(Intersection*);
    void performSim();
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
    Point3d(short int x, short int y, short int z) : x(x), y(y), z(z) {};
private:
    const short int x, y, z;
};


template<class T>
class BezierCurve
{
public:
    BezierCurve(T s, T e, std::vector<Point3d> handles);
    std::vector<Point3d> rasterize();
};


class NodeData {
public:
    Point3d* getLoc() {return &(this->loc);}
private:
    Point3d loc;
};


class ScenarioNodeData : public NodeData {
public:
    short int getTrafficFlow() {return this->trafficFlow;}
private:
    short int trafficFlow;
};


class IntersectionNodeData {
public:
    JUNCTIONTYPE getJunctionType() {return this->junctionType;}
private:
    JUNCTIONTYPE junctionType;
};


template<class T>
class Node
{
public:
    T getData() const {return this->data;}

private:
    int UUID;
    const T data;

friend class Intersection;
};


class Edge {
    // not implemented yet
};


class IntersectionEdge : public Edge
{
private:
    BezierCurve<IntersectionNodeData* > shape;
    // UUID 
};


class IntersectionRoute
{
    // implement route here
public:
    std::vector<Node<IntersectionNodeData>* > nodeList;
    std::vector<IntersectionEdge> edgeList;
};


class Intersection
{
public:
    void simulate(BACKENDS) const;
    void updateMetrics(BACKENDS);
    std::vector<IntersectionRoute*> routes;

private:
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



template<class T>
class Graph
{
    // not implemented yet
};


class IntersectionScenario : public Graph<Node<ScenarioNodeData> >
{
    // implement input graph here
};

}

#endif