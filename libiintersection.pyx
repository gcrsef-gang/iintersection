# disutils: langauge=c++

from libcpp.vector cimport vector
from libcpp.map cimport map

# Defining pointers since Cython's [] syntax doesn't work sometimes
ctypedef Node* nodepointer
ctypedef IntersectionRoute* intersectionroutepointer
ctypedef IntersectionNode* intersectionnodepointer

cdef extern from "libiintersection.h" namespace "ii":
    cdef cppclass IntersectionScenario:
        IntersectionScenario(vector[Node*] nodes, vector[ScenarioEdge] edges)
        vector[Node*] getNodes()
        vector[ScenarioEdge] getEdges()
    cdef cppclass Intersection:
        void simulate(BACKENDS)
        void updateMetrics(BACKENDS)
        vector[IntersectionRoute*] routes

    cdef enum METRICS:
        SAFETY,
        EMISSIONS, 
        EFFICIENCY 
    cdef enum BACKENDS:
        SUMO,
        VISSIM,
        CITYFLOW
    cdef enum JUNCTIONTYPE:
        TRAFFICLIGHT
        WHATEVER
    cdef enum VEHICLETYPES:
        CAR 
        TRUCK
        IDK

    cdef cppclass IntersectionRoute:
        IntersectionRoute(vector[intersectionnodepointer] nodeList, vector[IntersectionEdge] edgeList)
        vector[intersectionnodepointer] getNodeList()
        vector[IntersectionEdge] getEdgeList()

    cdef cppclass Node:
        Node(Point3d)
        Point3d* getLoc()
    cdef cppclass Point3d:
        Point3d(short int x, short int y, short int z)
    cdef cppclass IntersectionNode:
        IntersectionNode()
        JUNCTIONTYPE getJunctionType()
    cdef cppclass ScenarioEdge:
        ScenarioEdge(Node* s, Node* e, map[VEHICLETYPES, short int] demand)
    cdef cppclass IntersectionEdge:
        pass

cdef class PyIntersectionScenario:
    cdef IntersectionScenario* c_intersectionscenario
    
    def __cinit__(self, vector[Node*] nodes, vector[ScenarioEdge] edges):
        self.c_intersectionscenario = IntersectionScenario(vector[nodepointer], vector[ScenarioEdge])
    
    def getNodes(self):
        return self.c_intersectionscenario.getNodes()
    
    def getEdges(self):
        return self.c_intersectionscenario.getEdges()

    def __dealloc__(self):
        del self.c_intersectionscenario

cdef class PyIntersection:
    cdef Intersection* c_intersection

    def __cinit__(self, vector[IntersectionRoute*] routes):
        self.c_intersection = Intersection(vector[intersectionroutepointer])

    def simulate(self, backend):
        self.c_intersection.simulate(backend)

    def updateMetrics(self, backend):
        self.c_intersection.updateMetrics(backend)

    def getMetric(metric):
        return self.c_intersection.getMetric(metric)
    
    def __dealloc__(self):
        del self.c_intersection

cdef class PyIntersectionRoute:
    cdef IntersectionRoute* c_intersectionroute

    def __cinit__(self, vector[IntersectionNode*] nodeList, vector[IntersectionEdge] edgeList):
        self.c_intersectionroute = IntersectionRoute(vector[intersectionnodepointer], vector[IntersectionEdge])

    def getNodeList(self):
        return self.c_intersectionroute.getNodeList()

    def getEdgeList(self):
        return self.c_intersectionroute.getEdgeList()
    
    def __dealloc__(self):
        del self.c_intersectionroute