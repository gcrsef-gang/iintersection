# distutils: language = c++
# distutils: sources = lib/pugixml/src/pugixml.cpp

from libcpp.vector cimport vector
from libcpp.map cimport map as map_
from libcpp.utility cimport pair
from libcpp.string import string
from cython.operator cimport dereference as deref
from cython.operator cimport postincrement


# Defining pointers since Cython's [] syntax doesn't work sometimes
ctypedef Node* nodepointer
ctypedef IntersectionRoute* intersectionroutepointer
ctypedef IntersectionNode* intersectionnodepointer


cdef extern from "libiintersection.h" namespace "ii":
    cdef cppclass DataManager:
        @staticmethod
        intersectionnodepointer createIntersectionNode(Point3d loc, JUNCTIONTYPES junctiontype)

    cdef cppclass IntersectionScenario:
        IntersectionScenario()
        IntersectionScenario(vector[Node] nodes, vector[ScenarioEdge] edges) except +
        IntersectionScenario(char* xmlFilePath)
        vector[Node] getNodes()
        vector[ScenarioEdge] getEdges()
        
    cdef cppclass Intersection:
        Intersection()
        Intersection(vector[IntersectionRoute]) except +
        void simulate(BACKENDS)
        void updateMetrics(BACKENDS)
        double getMetric(METRICS)

        char* getNodeXML()
        char* getEdgeXML() 
        vector[intersectionroutepointer] getRoutes()

    cdef enum METRICS:
        SAFETY,
        EMISSIONS, 
        EFFICIENCY 
    cdef enum BACKENDS:
        SUMO,
        VISSIM,
        CITYFLOW
    cdef enum JUNCTIONTYPES:
        PRIORITY,
        TRAFFIC_LIGHT,
        RIGHT_BEFORE_LEFT,
        UNREGULATED,
        PRIORITY_STOP,
        TRAFFIC_LIGHT_UNREGULATED,
        ALLWAY_STOP,
        ZIPPER,
        TRAFFIC_LIGHT_RIGHT_ON_RED
    cdef enum VEHICLETYPES:
        CAR,
        TRUCK,
        IDK

    cdef map_[JUNCTIONTYPES, char*] JUNCTIONTYPES_NAMES
    cdef map_[char*, VEHICLETYPES] VEHICLETYPES_INDICES

    cdef cppclass IntersectionRoute:
        IntersectionRoute()
        IntersectionRoute(vector[intersectionnodepointer] nodeList, vector[IntersectionEdge] edgeList) except +
        vector[intersectionnodepointer] getNodeList()
        vector[IntersectionEdge] getEdgeList()
        void setNodeList(vector[intersectionnodepointer] nodelist)
        void setEdgeList(vector[IntersectionEdge] edgelist)

    cdef cppclass Node:
        Node()
        Node(Point3d)
        Point3d* getLoc()
        int getID()

    cdef cppclass Point3d:
        Point3d()
        Point3d(short int x, short int y, short int z) except +
        Point3d(vector[short int] coords) except +
        short int x()
        short int y()
        short int z()

    cdef cppclass IntersectionNode(Node):
        IntersectionNode()
        IntersectionNode(Point3d loc, JUNCTIONTYPES junctionType) except +
        JUNCTIONTYPES getJunctionType()
        void addReference()
        void removeReference()

    cdef cppclass Edge:
        Edge()
        Edge(nodepointer s, nodepointer e) except +
        nodepointer getStartNode()
        nodepointer getEndNode()

    cdef cppclass ScenarioEdge(Edge):
        ScenarioEdge()
        ScenarioEdge(nodepointer s, nodepointer e, map_[VEHICLETYPES, short int] demand) except +
        map_[VEHICLETYPES, short int] getDemand()

    cdef cppclass BezierCurve:
        BezierCurve()
        BezierCurve(intersectionnodepointer s, intersectionnodepointer e, vector[Point3d] handles) except +
        vector[Point3d] rasterize()
        intersectionnodepointer getStartNode()
        intersectionnodepointer getEndNode()
        vector[Point3d] getHandles()

    cdef cppclass IntersectionEdge(Edge):
        IntersectionEdge()
        IntersectionEdge(intersectionnodepointer s, intersectionnodepointer e, BezierCurve shape, short int numLanes, short int speedLimit, short int priority) except +
        BezierCurve getShape()
        short int getNumLanes()
        short int getSpeedLimit()
        short int getPriority()
        void setStartNode(intersectionnodepointer s)
        void setEndNode(intersectionnodepointer e)
        void updateHandles(vector[Point3d] handles)
        void setNumLanes(short int numLanes)
        void setSpeedLimit(short int speedLimit)
        void setPriority(short int priority)


PY_METRICS = {"safety": 0, "emissions": 1, "efficiency": 2}
PY_BACKENDS = {"sumo": 0, "vissim": 1, "cityflow": 2}
PY_VEHICLETYPES = {"car": 0, "truck": 1, "idk": 2}
PY_JUNCTIONTYPES = {"priority": 0, "traffic_light": 1, "right_before_left": 2, "unregulated": 3, "priority_stop": 4, "traffic_light_unregulated": 5, "allway_stop": 6, "zipper": 7, "traffic_light_on_red": 8}


cdef class PyIntersectionScenario:
    cdef IntersectionScenario c_intersectionscenario

    def __cinit__(self, list pynodes=[], list pyedges=[]):

        if pynodes == [] and pyedges == []:
            # Being called from `fromCppObject`
            return

        cdef vector[Node] nodes
        cdef PyNode node
        for node in pynodes:
            nodes.push_back(node.c_node)

        cdef PyScenarioEdge edge
        cdef vector[ScenarioEdge] edges
        for edge in pyedges:
            edges.push_back(edge.c_scenarioedge)
        self.c_intersectionscenario = IntersectionScenario(nodes, edges)

    @staticmethod
    cdef PyIntersectionScenario fromCppObject(IntersectionScenario c_intersectionscenario):
        cdef PyIntersectionScenario scenario = PyIntersectionScenario()
        scenario.c_intersectionscenario = c_intersectionscenario
        return scenario

    @staticmethod
    def fromXML(str xmlFilePath):
        cdef IntersectionScenario c_intersectionscenario = IntersectionScenario(xmlFilePath.encode("utf-8"))
        return PyIntersectionScenario.fromCppObject(c_intersectionscenario)

    def getNodes(self):
        cdef vector[Node] nodevector = self.c_intersectionscenario.getNodes()
        cdef Node node
        pyvector = []
        for node in nodevector:
            loc = node.getLoc()
            pyvector.append(PyNode(loc.x(), loc.y(), loc.z()))
        return pyvector
    
    def getEdges(self):
        cdef vector[ScenarioEdge] edgevector = self.c_intersectionscenario.getEdges()
        pyvector = []
        for edge in edgevector:
            pyvector.append(PyScenarioEdge.fromCppObject(edge))
        return pyvector


cdef class PyIntersection:
    cdef Intersection c_intersection

    def __cinit__(self, pyroutes):
        cdef vector[IntersectionRoute] routes
        cdef PyIntersectionRoute route
        for route in pyroutes:
            routes.push_back(route.c_intersectionroute)
        self.c_intersection = Intersection(routes)

    def simulate(self, int backend):
        self.c_intersection.simulate(<BACKENDS>backend)

    def updateMetrics(self, int backend):
        self.c_intersection.updateMetrics(<BACKENDS>backend)

    def getMetric(self, int metric):
        return self.c_intersection.getMetric(<METRICS>metric)

    def getNodeXML(self):
        return self.c_intersection.getNodeXML()

    def getEdgeXML(self):
        return self.c_intersection.getEdgeXML()

    def getRoutes(self):
        cdef vector[intersectionroutepointer] routesvector = self.c_intersection.getRoutes()
        pyvector = []
        for route in routesvector:
            pyvector.append(PyIntersectionRoutePointer.fromCppPointer(route))
        return pyvector


cdef class PyIntersectionRoute:
    cdef IntersectionRoute c_intersectionroute

    def __cinit__(self, list pynodes, list pyedges):
        cdef vector[intersectionnodepointer] node_ptrs
        cdef PyIntersectionNodePointer node
        for node in pynodes:
            node_ptrs.push_back(node.c_intersectionnodepointer)

        cdef PyIntersectionEdge edge
        cdef vector[IntersectionEdge] edges
        for edge in pyedges:
            edges.push_back(edge.c_intersectionedge)
        self.c_intersectionroute = IntersectionRoute(node_ptrs, edges)

    def getNodeList(self):
        cdef vector[intersectionnodepointer] intersection_node_vector = self.c_intersectionroute.getNodeList()
        intersection_nodes = []
        for intersectionnode_ptr in intersection_node_vector:
            intersection_nodes.append(PyIntersectionNodePointer.fromCppPointer(intersectionnode_ptr))
        return intersection_nodes

    def getEdgeList(self):
        cdef vector[IntersectionEdge] intersectionedgevector = self.c_intersectionroute.getEdgeList()
        pyvector = []
        for intersectionedge in intersectionedgevector:
            pyvector.append(PyIntersectionEdge.fromCppObject(intersectionedge))
        return pyvector

    def setNodeList(self, nodelist):
        cdef vector[intersectionnodepointer] cyvector
        cdef PyIntersectionNodePointer node
        for node in nodelist:
            cyvector.push_back(node.c_intersectionnodepointer)
        self.c_intersectionroute.setNodeList(cyvector)

    def setEdgeList(self, edgelist):
        cdef vector[IntersectionEdge] cyvector
        cdef PyIntersectionEdge edge
        for edge in edgelist:
            cyvector.push_back(edge.c_intersectionedge)
        self.c_intersectionroute.setEdgeList(cyvector)

cdef class PyNode:
    cdef Node c_node

    def __cinit__(self, tuple coords):

        self.c_node = Node(Point3d(coords[0], coords[1], coords[2]))

    def getLoc(self):
        cdef Point3d loc = deref(self.c_node.getLoc())
        return loc.x(), loc.y(), loc.z()

    def getID(self):
        return self.c_node.getID()


cdef class PyBezierCurve:
    cdef BezierCurve c_beziercurve

    def __cinit__(self, s, e, list pyhandles):
        
        cdef PyIntersectionNodePointer startnode = s
        cdef intersectionnodepointer start_node_ptr = startnode.c_intersectionnodepointer

        cdef PyIntersectionNodePointer endnode = e
        cdef intersectionnodepointer end_node_ptr = endnode.c_intersectionnodepointer

        cdef vector[Point3d] handles
        cdef handle
        for handle in pyhandles:
            handles.push_back(Point3d(handle[0], handle[1], handle[2]))
        self.c_beziercurve = BezierCurve(start_node_ptr, end_node_ptr, handles)

    @staticmethod
    cdef PyBezierCurve fromCppObject(BezierCurve bezier_curve):

        cdef intersectionnodepointer node_ptr = bezier_curve.getStartNode()
        cdef PyIntersectionNodePointer s = PyIntersectionNodePointer.fromCppPointer(node_ptr)
        node_ptr = bezier_curve.getEndNode()
        cdef PyIntersectionNodePointer e = PyIntersectionNodePointer.fromCppPointer(node_ptr)

        cdef list handles = []
        cdef Point3d handle
        for handle in bezier_curve.getHandles():
            handles.append((handle.x(), handle.y(), handle.z()))

        return PyBezierCurve(s, e, handles)

    def rasterize(self):
        cdef vector[Point3d] point3dvector = self.c_beziercurve.rasterize()
        pyvector = []
        for point3d in point3dvector:
            pointtuple = (point3d.x(), point3d.y(), point3d.z())
            pyvector.append(pointtuple)
        return pyvector

    def getStartNode(self):
        return PyIntersectionNodePointer.fromCppPointer(self.c_beziercurve.getStartNode())
    
    def getEndNode(self):
        return PyIntersectionNodePointer.fromCppPointer(self.c_beziercurve.getEndNode())

    def getHandles(self):
        cdef vector[Point3d] point3dvector = self.c_beziercurve.getHandles()
        pyvector = []
        for point3d in point3dvector:
            pointtuple = (point3d.x(), point3d.y(), point3d.z())
            pyvector.append(pointtuple)
        return pyvector


cdef class PyEdge:

    cdef Edge c_edge 

    def __cinit__(self, *args, **kwargs):
        # Not meant to be instantiated directly.
        pass

    def getStartNode(self):
        cdef nodepointer startnode = self.c_edge.getStartNode()
        return PyNodePointer.fromCppPointer(startnode)

    def getEndNode(self):
        cdef nodepointer endnode = self.c_edge.getEndNode()
        return PyNodePointer.fromCppPointer(endnode)


cdef class PyScenarioEdge(PyEdge):

    cdef ScenarioEdge c_scenarioedge

    def __cinit__(self, PyNodePointer s=None, PyNodePointer e=None, dict demand={}):

        if s is None and e is None and not demand:
            return

        cdef map_[VEHICLETYPES, short int] c_demand
        for key, value in demand.items():
            c_demand[<VEHICLETYPES>VEHICLETYPES_INDICES[key]] = <short int>value
        self.c_scenarioedge = ScenarioEdge(s, e, c_demand)
        self.c_edge = self.c_scenarioedge

    @staticmethod
    cdef PyScenarioEdge fromCppObject(ScenarioEdge c_scenarioedge):
        cdef PyScenarioEdge scenario_edge = PyScenarioEdge()
        scenario_edge.c_scenarioedge = c_scenarioedge
        scenario_edge.c_edge = c_scenarioedge
        return scenario_edge

    def getDemand(self):
        cdef map_[VEHICLETYPES, short int] c_demand = self.c_scenarioedge.getDemand()
        cdef dict demand = {}
        cdef map_[VEHICLETYPES, short int].iterator it = c_demand.begin()
        while it != c_demand.end():
            demand[<int>(deref(it).first)] = deref(it).first
            postincrement(it)
        return demand


cdef class PyIntersectionEdge(PyEdge):

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
        self.c_edge = self.c_intersectionedge

    @staticmethod
    cdef PyIntersectionEdge fromCppObject(IntersectionEdge intersectionedge):
        cdef PyIntersectionEdge pyintersectionedge = PyIntersectionEdge()
        pyintersectionedge.c_intersectionedge = intersectionedge
        return pyintersectionedge

    def getShape(self):
        cdef BezierCurve c_bezier_curve = self.c_intersectionedge.getShape()
        return PyBezierCurve.fromCppObject(c_bezier_curve)

    def getNumLanes(self):
        return self.c_intersectionedge.getNumLanes()

    def getSpeedLimit(self):
        return self.c_intersectionedge.getSpeedLimit()

    def getPriority(self):
        return self.c_intersectionedge.getPriority()

    def setStartNode(self, intersectionnode_pointer):
        cdef PyIntersectionNodePointer intersection_node_pointer = intersectionnode_pointer
        self.c_intersectionedge.setStartNode(intersection_node_pointer.c_intersectionnodepointer)

    def setEndNode(self, intersectionnode_pointer):
        cdef PyIntersectionNodePointer intersection_node_pointer = intersectionnode_pointer
        self.c_intersectionedge.setEndNode(intersection_node_pointer.c_intersectionnodepointer)

    def updateHandles(self, handles):
        cdef vector[Point3d] cyhandles
        for handle in handles:
            cyhandles.push_back(Point3d(handle[0], handle[1], handle[2]))
        self.c_intersectionedge.updateHandles(cyhandles)

    def setNumLanes(self, numlanes):
        self.c_intersectionedge.setNumLanes(numlanes)

    def setSpeedLimit(self, speedlimit):
        self.c_intersectionedge.setSpeedLimit(speedlimit)

    def setPriority(self, priority):
        self.c_intersectionedge.setPriority(priority)


cdef class PyNodePointer:
    cdef nodepointer c_nodepointer 
    
    def __cinit__(self):
        self.c_nodepointer = NULL

    @staticmethod
    cdef PyNodePointer fromCppPointer(nodepointer node_ptr):
        cdef PyNodePointer pynodepointer = PyNodePointer()
        pynodepointer.c_nodepointer = node_ptr
        return pynodepointer

    def getLoc(self):
        locvector = deref(self.c_nodepointer).getLoc()
        pytuple = (locvector.x(), locvector.y(), locvector.z())
        return pytuple

    def getID(self):
        return deref(self.c_nodepointer).getID()

    def __dealloc__(self):
        if self.c_nodepointer is not NULL:
            del self.c_nodepointer


cdef class PyIntersectionNodePointer(PyNodePointer):
    cdef intersectionnodepointer c_intersectionnodepointer

    def __cinit__(self, point3d=None, junctiontype=None):
        if point3d != None:
            self.c_intersectionnodepointer = DataManager.createIntersectionNode(Point3d(point3d), junctiontype)
        else:
            self.c_intersectionnodepointer = NULL

    @staticmethod
    cdef PyIntersectionNodePointer fromCppPointer(intersectionnodepointer i_node_ptr):
        cdef PyIntersectionNodePointer pyintersectionnodepointer = PyIntersectionNodePointer()
        pyintersectionnodepointer.c_intersectionnodepointer = i_node_ptr
        return pyintersectionnodepointer

    def getJunctionType(self):
        return deref(self.c_intersectionnodepointer).getJunctionType()

    def addReference(self):
        deref(self.c_intersectionnodepointer).addReference()

    def removeReference(self):
        deref(self.c_intersectionnodepointer).removeReference()

    def __dealloc__(self):
        if self.c_intersectionnodepointer is not NULL:
            del self.c_intersectionnodepointer


cdef class PyIntersectionRoutePointer():
    cdef intersectionroutepointer c_intersectionroutepointer

    def __cinit__(self):
        self.c_intersectionroutepointer = NULL

    @staticmethod
    cdef PyIntersectionRoute fromCppPointer(IntersectionRoute* route_ptr):
        cdef PyIntersectionRoutePointer pyintersectionroutepointer = PyIntersectionRoutePointer()
        pyintersectionroutepointer.c_intersectionroutepointer = route_ptr
        return pyintersectionroutepointer

    def getNodeList(self):
        cdef vector[intersectionnodepointer] c_nodes = deref(self.c_intersectionroutepointer).getNodeList()
        nodes = []
        cdef nodepointer node_ptr
        for node_ptr in c_nodes:
            nodes.append(PyNodePointer.fromCppPointer(node_ptr))
        return nodes

    def getEdgeList(self):
        cdef vector[IntersectionEdge] cyvector = deref(self.c_intersectionroutepointer).getEdgeList()
        pyvector = []
        for edge in cyvector:
            pyvector.append(PyIntersectionEdge.fromCppObject(edge))
        return pyvector

    def __dealloc__(self):
        if self.c_intersectionroutepointer is not NULL:
            del self.c_intersectionroutepointer