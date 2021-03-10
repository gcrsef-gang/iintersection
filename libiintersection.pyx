# distutils: language = c++
# distutils: sources = lib/pugixml/src/pugixml.cpp

from libcpp.map cimport map as map_
from libcpp.memory cimport shared_ptr, make_shared
from libcpp.string cimport string
from libcpp.utility cimport pair
from libcpp.vector cimport vector
from cython.operator cimport dereference as deref
from cython.operator cimport postincrement


# Defining pointers since Cython's [] syntax doesn't work sometimes
ctypedef IntersectionRoute* intersectionroutepointer
ctypedef IntersectionEdge* intersectionedgepointer
ctypedef ScenarioEdge* scenarioedgepointer

cdef extern from "<vector>" namespace "std":
    cdef cppclass size_t

cdef extern from "libiintersection.h" namespace "ii::METRICS":
    cdef enum METRICS_:
        SAFETY, EMISSIONS, EFFICIENCY

cdef extern from "libiintersection.h" namespace "ii::BACKENDS":
    cdef enum BACKENDS_:
        SUMO, VISSIM, CITYFLOW

cdef extern from "libiintersection.h" namespace "ii::VEHICLETYPES":
    cdef enum VEHICLETYPES_:
        CAR, TRUCK, IDK

cdef extern from "libiintersection.h" namespace "ii::JUNCTIONTYPES":
    cdef enum JUNCTIONTYPES_:
        PRIORITY, TRAFFIC_LIGHT, RIGHT_BEFORE_LEFT, UNREGULATED, PRIORITY_STOP, TRAFFIC_LIGHT_UNREGULATED, ALLWAY_STOP, TRAFFIC_LIGHT_RIGHT_ON_RED

cdef extern from "libiintersection.h" namespace "ii":

    cdef map_[JUNCTIONTYPES_, string] JUNCTIONTYPES_NAMES
    cdef map_[string, VEHICLETYPES_] VEHICLETYPES_INDICES


    cdef cppclass Point3d:
        Point3d()
        Point3d(short int x, short int y, short int z) except +

        short int x()
        short int y()
        short int z()


    cdef cppclass ScenarioNode:
        ScenarioNode(Point3d) except +

        Point3d* getLoc()
        unsigned short int getID()

        void setID(unsigned short int ID)


    cdef cppclass IntersectionNode:
        IntersectionNode(Point3d loc, JUNCTIONTYPES_ junctionType) except +

        JUNCTIONTYPES_ getJunctionType()
        Point3d* getLoc()
        unsigned short int getID()

        void setID(unsigned short int ID)


    cdef cppclass BezierCurve:
        BezierCurve()
        BezierCurve(shared_ptr[IntersectionNode] s, shared_ptr[IntersectionNode] e, vector[Point3d] handles) except +

        vector[Point3d] rasterize()

        vector[Point3d] getHandles()
        shared_ptr[IntersectionNode] getStartNode()
        shared_ptr[IntersectionNode] getEndNode()
        void setHandles(vector[Point3d] handles_)


    cdef cppclass IntersectionEdge:
        IntersectionEdge()
        IntersectionEdge(shared_ptr[IntersectionNode] s, shared_ptr[IntersectionNode] e, BezierCurve shape, short int numLanes, short int speedLimit, short int priority) except +

        BezierCurve getShape()
        short int getNumLanes()
        short int getSpeedLimit()
        short int getPriority()

        shared_ptr[IntersectionNode] getStartNode()
        shared_ptr[IntersectionNode] getEndNode()

        void setHandles(vector[Point3d] handles)
        void setSpeedLimit(short int speedLimit)
        void setNumLanes(short int numLanes)
        void setPriority(short int priority)


    cdef cppclass ScenarioEdge:
        ScenarioEdge()
        ScenarioEdge(shared_ptr[ScenarioNode] s, shared_ptr[ScenarioNode] e, map_[VEHICLETYPES_, short int] demand) except +

        map_[VEHICLETYPES_, short int] getDemand()
        shared_ptr[ScenarioNode] getStartNode()
        shared_ptr[ScenarioNode] getEndNode()


    cdef cppclass IntersectionRoute:
        IntersectionRoute()
        IntersectionRoute(vector[shared_ptr[IntersectionNode]] nodeList, vector[IntersectionEdge] edgeList) except +

        vector[shared_ptr[IntersectionNode]] getNodeList()
        vector[intersectionedgepointer] getEdgeList()

        void setNodeList(vector[shared_ptr[IntersectionNode]] nodelist)
        void setEdgeList(vector[IntersectionEdge] edgelist)


    cdef cppclass Intersection:
        Intersection()
        Intersection(vector[IntersectionRoute]) except +

        @staticmethod
        void Simulate(Intersection*, BACKENDS_)
        @staticmethod
        Intersection fromSolFile(string solFilePath)

        vector[intersectionroutepointer] getRoutes()
        vector[shared_ptr[IntersectionNode]] getUniqueNodes()
        vector[intersectionedgepointer] getUniqueEdges()

        void updateMetrics(BACKENDS_)
        void markInvalid()
        double getMetric(METRICS_)

        string getNodeXML()
        string getEdgeXML() 
        string getRouteXML(IntersectionScenario intersectionScenario)
        string getSolXML()


    cdef cppclass IntersectionScenario:
        IntersectionScenario()
        IntersectionScenario(string xmlFilePath) except +
        
        vector[shared_ptr[ScenarioNode]] getNodes()
        vector[scenarioedgepointer] getEdges()


PY_METRICS = {"safety": 0, "emissions": 1, "efficiency": 2}
PY_BACKENDS = {"sumo": 0, "vissim": 1, "cityflow": 2, "traci": 3}
PY_VEHICLETYPES = {"car": 0, "truck": 1}
PY_JUNCTIONTYPES = {"priority": 0, "traffic_light": 1, "right_before_left": 2, "unregulated": 3, "priority_stop": 4, "traffic_light_unregulated": 5, "allway_stop": 6, "traffic_light_on_red": 7}


cdef class PyIntersectionScenario:
    cdef IntersectionScenario c_intersectionscenario

    def __cinit__(self, str file_path):
        self.c_intersectionscenario = IntersectionScenario(file_path.encode("utf-8"))

    def getNodes(self):
        cdef vector[shared_ptr[ScenarioNode]] c_nodes = self.c_intersectionscenario.getNodes()
        cdef list nodes = []
        cdef shared_ptr[ScenarioNode] node
        for node in c_nodes:
            nodes.append(PyScenarioNodePointer.fromCppPointer(node))
        return nodes
    
    def getEdges(self):
        cdef vector[scenarioedgepointer] c_edges = self.c_intersectionscenario.getEdges()
        cdef list edges = []
        cdef scenarioedgepointer edge
        for edge in c_edges:
            edges.append(PyScenarioEdgePointer.fromCppPointer(edge))
        return edges


cdef class PyIntersection:
    cdef Intersection c_intersection

    def __cinit__(self, routes=None, xmlFilePath=None):
        cdef vector[IntersectionRoute] c_routes
        cdef PyIntersectionRoute route
        if routes:
            for route in routes:
                c_routes.push_back(route.c_intersectionroute)
        self.c_intersection = Intersection(c_routes)

    @staticmethod
    def Simulate(self, PyIntersection intersection, int backend):
        Intersection.Simulate(&intersection.c_intersection, <BACKENDS_>backend)

    @staticmethod
    def fromSolFile(str solFilePath):
        cdef PyIntersection intersection = PyIntersection()
        intersection.c_intersection = Intersection.fromSolFile(solFilePath.encode())
        return intersection

    def getUniqueNodes(self):
        cdef vector[shared_ptr[IntersectionNode]] c_unique_nodes = self.c_intersection.getUniqueNodes()
        cdef list unique_nodes = []
        cdef shared_ptr[IntersectionNode] node
        for node in c_unique_nodes:
            unique_nodes.append(PyIntersectionNodePointer.fromCppPointer(node))
        return unique_nodes

    def getUniqueEdges(self):
        cdef vector[intersectionedgepointer] c_unique_edges = self.c_intersection.getUniqueEdges()
        cdef list unique_edges = []
        cdef intersectionedgepointer edge
        for edge in c_unique_edges:
            unique_edges.append(PyIntersectionEdgePointer.fromCppPointer(edge))
        return unique_edges

    def updateMetrics(self, int backend):
        self.c_intersection.updateMetrics(<BACKENDS_>backend)

    def markInvalid(self):
        self.c_intersection.markInvalid()

    def getMetric(self, int metric):
        return self.c_intersection.getMetric(<METRICS_>metric)

    def getNodeXML(self):
        return self.c_intersection.getNodeXML().decode('utf-8')

    def getEdgeXML(self):
        return self.c_intersection.getEdgeXML().decode('utf-8')

    def getRouteXML(self, PyIntersectionScenario pyintersectionscenario):
        return self.c_intersection.getRouteXML(pyintersectionscenario.c_intersectionscenario).decode('utf-8')

    def getSolXML(self):
        return self.c_intersection.getSolXML().decode('utf-8')

    def getRoutes(self):
        cdef vector[intersectionroutepointer] c_routes = self.c_intersection.getRoutes()
        cdef list routes = []
        cdef intersectionroutepointer route
        for route in c_routes:
            routes.append(PyIntersectionRoutePointer.fromCppPointer(route))
        return routes


cdef class PyIntersectionRoute:
    cdef IntersectionRoute c_intersectionroute

    def __cinit__(self, list nodes, list edges):
        cdef vector[shared_ptr[IntersectionNode]] c_nodes
        cdef PyIntersectionNodePointer node
        for node in nodes:
            c_nodes.push_back(node.c_intersectionnodepointer)

        cdef PyIntersectionEdge edge
        cdef PyIntersectionEdgePointer edgepointer
        cdef vector[IntersectionEdge] cyedges
        
        for i in range(len(edges)):
            if isinstance(edges[i], PyIntersectionEdgePointer):
                for edgepointer in [edges[i]]:
                    cyedges.push_back(deref(edgepointer.c_intersectionedgepointer))
            else:
                for edge in [edges[i]]:
                    cyedges.push_back(edge.c_intersectionedge)
        self.c_intersectionroute = IntersectionRoute(c_nodes, cyedges)

    def getNodeList(self):
        cdef vector[shared_ptr[IntersectionNode]] c_nodes = self.c_intersectionroute.getNodeList()
        cdef list nodes = []
        cdef shared_ptr[IntersectionNode] node
        for node in c_nodes:
            nodes.append(PyIntersectionNodePointer.fromCppPointer(node))
        return nodes

    def getEdgeList(self):
        cdef vector[intersectionedgepointer] c_edges = self.c_intersectionroute.getEdgeList()
        cdef list edges = []
        cdef intersectionedgepointer edge
        for edge in c_edges:
            edges.append(PyIntersectionEdgePointer.fromCppPointer(edge))
        return edges

    def setNodeList(self, list nodeList):
        cdef vector[shared_ptr[IntersectionNode]] c_nodes
        cdef PyIntersectionNodePointer node
        for node in nodeList:
            c_nodes.push_back(node.c_intersectionnodepointer)
        self.c_intersectionroute.setNodeList(c_nodes)

    def setEdgeList(self, list edgeList):
        cdef vector[IntersectionEdge] c_edges
        cdef PyIntersectionEdge edge
        for edge in edgeList:
            c_edges.push_back(edge.c_intersectionedge)
        self.c_intersectionroute.setEdgeList(c_edges)


cdef class PyBezierCurve:
    cdef BezierCurve c_beziercurve

    def __cinit__(self, PyIntersectionNodePointer s, PyIntersectionNodePointer e, list handles):

        cdef vector[Point3d] c_handles
        for handle in handles:
            c_handles.push_back(Point3d(handle[0], handle[1], handle[2]))
        self.c_beziercurve = BezierCurve(s.c_intersectionnodepointer, e.c_intersectionnodepointer, c_handles)

    @staticmethod
    cdef PyBezierCurve fromCppObject(BezierCurve bezier_curve):
        cdef shared_ptr[IntersectionNode] node_ptr = bezier_curve.getStartNode()
        cdef PyIntersectionNodePointer s = PyIntersectionNodePointer.fromCppPointer(node_ptr)
        node_ptr = bezier_curve.getEndNode()
        cdef PyIntersectionNodePointer e = PyIntersectionNodePointer.fromCppPointer(node_ptr)

        cdef list handles = []
        cdef Point3d handle
        cdef vector[Point3d] c_handles = bezier_curve.getHandles()
        for handle in c_handles:
            handles.append((handle.x(), handle.y(), handle.z()))

        return PyBezierCurve(s, e, handles)

    def getStartNode(self):
        return PyIntersectionNodePointer.fromCppPointer(self.c_beziercurve.getStartNode())

    def getEndNode(self):
        return PyIntersectionNodePointer.fromCppPointer(self.c_beziercurve.getEndNode())

    def getHandles(self):
        cdef vector[Point3d] c_handles = self.c_beziercurve.getHandles()
        cdef list handles = []
        cdef Point3d handle
        for handle in c_handles:
            handles.append((handle.x(), handle.y(), handle.z()))
        return handles


cdef class PyIntersectionEdge:

    cdef IntersectionEdge c_intersectionedge

    def __cinit__(self, PyIntersectionNodePointer s=None, PyIntersectionNodePointer e=None,
            PyBezierCurve shape=None, short int numLanes=-1, short int speedLimit=-1,
            short int priority=-1):

        if (s is None and e is None and shape is None and numLanes == -1 and speedLimit == -1
                and priority == -1):
            # Being called from `fromCppObject`
            return

        self.c_intersectionedge = IntersectionEdge(s.c_intersectionnodepointer,
            e.c_intersectionnodepointer, shape.c_beziercurve, numLanes, speedLimit, priority)

    @staticmethod
    cdef PyIntersectionEdge fromCppObject(IntersectionEdge c_edge):
        cdef PyIntersectionEdge edge = PyIntersectionEdge()
        edge.c_intersectionedge = c_edge
        return edge

    def getShape(self):
        cdef BezierCurve c_bezier_curve = self.c_intersectionedge.getShape()
        return PyBezierCurve.fromCppObject(c_bezier_curve)

    def getNumLanes(self):
        return self.c_intersectionedge.getNumLanes()

    def getSpeedLimit(self):
        return self.c_intersectionedge.getSpeedLimit()

    def getPriority(self):
        return self.c_intersectionedge.getPriority()

    def getStartNode(self):
        cdef shared_ptr[IntersectionNode] c_node = self.c_intersectionedge.getStartNode()
        return PyIntersectionNodePointer.fromCppPointer(c_node)

    def getEndNode(self):
        cdef shared_ptr[IntersectionNode] c_node = self.c_intersectionedge.getEndNode()
        return PyIntersectionNodePointer.fromCppPointer(c_node)

    def setHandles(self, handles):
        cdef vector[Point3d] cyhandles
        for handle in handles:
            cyhandles.push_back(Point3d(handle[0], handle[1], handle[2]))
        self.c_intersectionedge.setHandles(cyhandles)

    def setNumLanes(self, numlanes):
        self.c_intersectionedge.setNumLanes(numlanes)

    def setSpeedLimit(self, speedlimit):
        self.c_intersectionedge.setSpeedLimit(speedlimit)

    def setPriority(self, priority):
        self.c_intersectionedge.setPriority(priority)

    def __eq__(self, other):

        cdef PyIntersectionEdge other_
        cdef PyIntersectionEdgePointer other_ptr

        cdef vector[Point3d] self_handles = self.c_intersectionedge.getShape().getHandles()
        cdef vector[Point3d] other_handles
        cdef Point3d self_handle, other_handle
        cdef size_t i

        if isinstance(other, PyIntersectionEdge):
            other_ = other
            other_handles = other_.c_intersectionedge.getShape().getHandles()
        elif isinstance(other, PyIntersectionEdgePointer):
            other_ptr = other
            other_handles = deref(other_ptr.c_intersectionedgepointer).getShape().getHandles()

        if self_handles.size() != other_handles.size():
            return False

        for i in range(self_handles.size()):
            self_handle = self_handles[i]
            other_handle = other_handles[i]
            if (self_handle.x() != other_handle.x() or self_handle.y() != other_handle.y() 
                    or self_handle.z() != other_handle.z()):
                return False

        return True


cdef class PyIntersectionEdgePointer:

    cdef intersectionedgepointer c_intersectionedgepointer

    def __cinit__(self):
        self.c_intersectionedgepointer = NULL

    @staticmethod
    cdef PyIntersectionEdgePointer fromCppPointer(intersectionedgepointer edge_ptr):
        cdef PyIntersectionEdgePointer intersection_edge = PyIntersectionEdgePointer()
        intersection_edge.c_intersectionedgepointer = edge_ptr
        return intersection_edge

    def getShape(self):
        cdef BezierCurve c_bezier_curve = deref(self.c_intersectionedgepointer).getShape()
        return PyBezierCurve.fromCppObject(c_bezier_curve)

    def getStartNode(self):
        cdef shared_ptr[IntersectionNode] c_node = deref(self.c_intersectionedgepointer).getStartNode()
        return PyIntersectionNodePointer.fromCppPointer(c_node)

    def getEndNode(self):
        cdef shared_ptr[IntersectionNode] c_node = deref(self.c_intersectionedgepointer).getEndNode()
        return PyIntersectionNodePointer.fromCppPointer(c_node)

    def getNumLanes(self):
        return deref(self.c_intersectionedgepointer).getNumLanes()

    def getSpeedLimit(self):
        return deref(self.c_intersectionedgepointer).getSpeedLimit()
        
    def getPriority(self):
        return deref(self.c_intersectionedgepointer).getPriority()

    def setHandles(self, handles):
        cdef vector[Point3d] cyhandles
        for handle in handles:
            cyhandles.push_back(Point3d(handle[0], handle[1], handle[2]))
        deref(self.c_intersectionedgepointer).setHandles(cyhandles)

    def setNumLanes(self, numlanes):
        deref(self.c_intersectionedgepointer).setNumLanes(numlanes)

    def setSpeedLimit(self, speedlimit):
        deref(self.c_intersectionedgepointer).setSpeedLimit(speedlimit)

    def setPriority(self, priority):
        deref(self.c_intersectionedgepointer).setPriority(priority)

    def __eq__(self, other):
        cdef PyIntersectionEdge other_
        cdef PyIntersectionEdgePointer other_ptr

        cdef vector[Point3d] self_handles = deref(self.c_intersectionedgepointer).getShape().getHandles()
        cdef vector[Point3d] other_handles
        cdef Point3d self_handle, other_handle
        cdef size_t i

        if isinstance(other, PyIntersectionEdge):
            other_ = other
            other_handles = other_.c_intersectionedge.getShape().getHandles()
        elif isinstance(other, PyIntersectionEdgePointer):
            other_ptr = other
            other_handles = deref(other_ptr.c_intersectionedgepointer).getShape().getHandles()

        if self_handles.size() != other_handles.size():
            return False

        for i in range(self_handles.size()):
            self_handle = self_handles[i]
            other_handle = other_handles[i]
            if (self_handle.x() != other_handle.x() or self_handle.y() != other_handle.y() 
                    or self_handle.z() != other_handle.z()):
                return False

        return True

cdef class PyScenarioEdgePointer:

    cdef scenarioedgepointer c_scenarioedgepointer

    def __cinit__(self):
        self.c_scenarioedgepointer = NULL

    @staticmethod
    cdef PyScenarioEdgePointer fromCppPointer(scenarioedgepointer edge_ptr):
        cdef PyScenarioEdgePointer scenario_edge = PyScenarioEdgePointer()
        scenario_edge.c_scenarioedgepointer = edge_ptr
        return scenario_edge

    def getStartNode(self):
        cdef shared_ptr[ScenarioNode] c_node = deref(self.c_scenarioedgepointer).getStartNode()
        return PyScenarioNodePointer.fromCppPointer(c_node)

    def getEndNode(self):
        cdef shared_ptr[ScenarioNode] c_node = deref(self.c_scenarioedgepointer).getEndNode()
        return PyScenarioNodePointer.fromCppPointer(c_node)

    def getDemand(self):
        cdef map_[VEHICLETYPES_, short int] c_demand = deref(self.c_scenarioedgepointer).getDemand()
        cdef dict demand = {}
        cdef map_[VEHICLETYPES_, short int].iterator it = c_demand.begin()
        while it != c_demand.end():
            demand[<int>(deref(it).first)] = deref(it).first
            postincrement(it)
        return demand


cdef class PyScenarioNodePointer:
    cdef shared_ptr[ScenarioNode] c_scenarionodepointer 
    
    def __cinit__(self):
        pass

    @staticmethod
    cdef PyScenarioNodePointer fromCppPointer(shared_ptr[ScenarioNode] c_node):
        cdef PyScenarioNodePointer node = PyScenarioNodePointer()
        node.c_scenarionodepointer = c_node
        return node

    def getLoc(self):
        loc = deref(self.c_scenarionodepointer).getLoc()
        pytuple = (loc.x(), loc.y(), loc.z())
        return pytuple

    def setID(self, int id_):
        deref(self.c_scenarionodepointer).setID(id_)

    def getID(self):
        return deref(self.c_scenarionodepointer).getID()


cdef class PyIntersectionNodePointer:
    cdef shared_ptr[IntersectionNode] c_intersectionnodepointer

    def __cinit__(self, tuple loc=None, int junctionType=-1):
        if loc is not None and junctionType != -1:
            self.c_intersectionnodepointer = make_shared[IntersectionNode](Point3d(loc[0], loc[1], loc[2]), <JUNCTIONTYPES_>junctionType)

    @staticmethod
    cdef PyIntersectionNodePointer fromCppPointer(shared_ptr[IntersectionNode] c_node):
        cdef PyIntersectionNodePointer node = PyIntersectionNodePointer()
        node.c_intersectionnodepointer = c_node
        return node

    @staticmethod
    def fromScenarioNode(PyScenarioNodePointer sce_node):
        cdef PyIntersectionNodePointer int_node = PyIntersectionNodePointer(sce_node.getLoc(), UNREGULATED)
        deref(int_node.c_intersectionnodepointer).setID(sce_node.getID())
        return int_node

    def getJunctionType(self):
        return deref(self.c_intersectionnodepointer).getJunctionType()

    def getLoc(self):
        loc = deref(self.c_intersectionnodepointer).getLoc()
        pytuple = (loc.x(), loc.y(), loc.z())
        return pytuple

    def getID(self):
        return deref(self.c_intersectionnodepointer).getID()

    def setID(self, int id_):
        deref(self.c_intersectionnodepointer).setID(id_)

    def __eq__(self, other):
        return self.getID() == other.getID()

    def __hash__(self):
        return hash((self.getLoc(), self.getID()))


cdef class PyIntersectionRoutePointer:
    cdef intersectionroutepointer c_intersectionroutepointer

    def __cinit__(self):
        self.c_intersectionroutepointer = NULL

    @staticmethod
    cdef PyIntersectionRoutePointer fromCppPointer(IntersectionRoute* c_route):
        cdef PyIntersectionRoutePointer route = PyIntersectionRoutePointer()
        route.c_intersectionroutepointer = c_route
        return route

    def getNodeList(self):
        cdef vector[shared_ptr[IntersectionNode]] c_nodes = deref(self.c_intersectionroutepointer).getNodeList()
        nodes = []
        cdef shared_ptr[IntersectionNode] node
        for node in c_nodes:
            nodes.append(PyIntersectionNodePointer.fromCppPointer(node))
        return nodes

    def getEdgeList(self):
        cdef vector[intersectionedgepointer] c_edges = deref(self.c_intersectionroutepointer).getEdgeList()
        edges = []
        cdef intersectionedgepointer edge
        for edge in c_edges:
            edges.append(PyIntersectionEdgePointer.fromCppPointer(edge))
        return edges

    def setNodeList(self, list nodeList):
        cdef vector[shared_ptr[IntersectionNode]] c_nodes
        cdef PyIntersectionNodePointer node
        for node in nodeList:
            c_nodes.push_back(node.c_intersectionnodepointer)
        deref(self.c_intersectionroutepointer).setNodeList(c_nodes)

    def setEdgeList(self, list edgeList):
        cdef vector[IntersectionEdge] c_edges
        cdef PyIntersectionEdge edge
        cdef PyIntersectionEdgePointer edgepointer
        for i in range(len(edgeList)):
            if isinstance(edgeList[i], PyIntersectionEdgePointer):
                for edgepointer in [edgeList[i]]:
                    c_edges.push_back(deref(edgepointer.c_intersectionedgepointer))
            else:
                for edge in [edgeList[i]]:
                    c_edges.push_back(edge.c_intersectionedge)
        deref(self.c_intersectionroutepointer).setEdgeList(c_edges)