# distutils: language = c++
# distutils: sources = lib/pugixml/src/pugixml.cpp

from libcpp.vector cimport vector
from libcpp.map cimport map as map_
from libcpp.utility cimport pair
from libcpp.string import string
from cython.operator cimport dereference as deref, postincrement


# Defining pointers since Cython's [] syntax doesn't work sometimes
ctypedef Node* nodepointer
ctypedef IntersectionRoute* intersectionroutepointer
ctypedef IntersectionNode* intersectionnodepointer


cdef extern from "libiintersection.h" namespace "ii":
    cdef cppclass IntersectionScenario:
        IntersectionScenario()
        IntersectionScenario(vector[nodepointer] nodes, vector[ScenarioEdge] edges) except +
        IntersectionScenario(char* xmlFilePath)
        vector[nodepointer] getNodes()
        vector[ScenarioEdge] getEdges()
        
    cdef cppclass Intersection:
        Intersection()
        Intersection(vector[intersectionroutepointer]) except +
        void simulate(BACKENDS)
        void updateMetrics(BACKENDS)
        double getMetric(METRICS)

        char* getNodeXML()
        char* getEdgeXML() 
        vector[intersectionroutepointer] routes

    cdef enum METRICS:
        SAFETY,
        EMISSIONS, 
        EFFICIENCY 
    cdef enum BACKENDS:
        SUMO,
        VISSIM,
        CITYFLOW
    cdef enum JUNCTIONTYPE:
        TRAFFICLIGHT,
        WHATEVER
    cdef enum VEHICLETYPES:
        CAR,
        TRUCK,
        IDK

    cdef cppclass IntersectionRoute:
        IntersectionRoute()
        IntersectionRoute(vector[intersectionnodepointer] nodeList, vector[IntersectionEdge] edgeList) except +
        vector[intersectionnodepointer] getNodeList()
        vector[IntersectionEdge] getEdgeList()

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
        IntersectionNode(Point3d loc, JUNCTIONTYPE junctionType) except +
        JUNCTIONTYPE getJunctionType()

    cdef cppclass Edge:
        Edge()
        Edge(nodepointer s, nodepointer e) except +
        int getStartNode()
        int getEndNode()

    cdef cppclass ScenarioEdge(Edge):
        ScenarioEdge()
        ScenarioEdge(Node* s, Node* e, map[VEHICLETYPES, short int] demand) except +
        map_[VEHICLETYPES, short int] getDemand()

    cdef cppclass BezierCurve:
        BezierCurve()
        BezierCurve(intersectionnodepointer s, intersectionnodepointer e, vector[Point3d] handles) except +
        vector[Point3d] rasterize()
        int getStartNode()
        int getEndNode()
        vector[Point3d] getHandles()

    cdef cppclass IntersectionEdge(Edge):
        IntersectionEdge()
        IntersectionEdge(intersectionnodepointer s, intersectionnodepointer e, BezierCurve shape, short int numLanes, short int speedLimit, short int priority) except +
        BezierCurve getShape()
        short int getNumLanes()
        short int getSpeedLimit()
        short int getPriority()


cdef class PyIntersectionScenario:
    cdef IntersectionScenario c_intersectionscenario

    def __cinit__(self, list pynodes=[], list pyedges=[]):

        if pynodes == [] and pyedges == []:
            # Being called from `fromCppObject`
            return

        cdef vector[nodepointer] nodes
        for node in pynodes:
            if isinstance(node, PyNode):
                nodes.push_back(&(node.c_node))
            elif isinstance(node, PyNodePointer):
                nodes.push_back(node.c_nodepointer)
            else:
                # Without raising the following TypeError, if one was to input the incorrect type
                # of `pynodes` no error would be raised, but instead `nodes` would be empty, 
                # leading to logic errors that will be much harder to debug than a simple TypeError.
                raise TypeError

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
        cdef IntersectionScenario c_intersectionscenario(xmlFilePath)
        return PyIntersectionScenario.fromCppObject(c_intersectionscenario)

    def getNodes(self):
        cdef vector[Node*] nodevector = self.c_intersectionscenario.getNodes()
        cdef nodepointer node_ptr
        pyvector = []
        for node_ptr in nodevector:
            pyvector.append(PyNodePointer.fromCppPointer(node_ptr))
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
        cdef vector[intersectionroutepointer] routes
        for route in pyroutes:
            if isinstance(route, PyIntersection):
                routes.push_back(&(route.c_intersectionroute))
            elif isinstance(route, PyIntersectionPointer):
                routes.push_back(route.c_intersectionroutepointer)
            else:
                raise TypeError
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
        return self.c_intersection.getEdgeXML()\
    
    def getRoutes(self):
        cdef vector[intersectionroutepointer] routesvector = self.c_intersection.getRoutes()
        pyvector = []
        for route in routesvector:
            pyvector.append(PyIntersectionRoutePointer.fromCppPointer(route))
        return pyvector


cdef class PyIntersectionRoute:
    cdef IntersectionRoute c_intersectionroute

    def __cinit__(self, list pynodes, list pyedges):
        cdef vector[intersectionnodepointer] node_ptr_vector
        for node in pynodes:
            if isinstance(node, PyIntersectionNode):
                node_ptr_vector.push_back(&(node.c_intersectionnode))
            elif isinstance(node, PyIntersectionNodePointer):
                node_ptr_vector.push_back(node.c_intersectionnodepointer)
            else:
                raise TypeError

        cdef PyIntersectionEdge edge
        cdef vector[IntersectionEdge] edge_vector
        for edge in pyedges:
            edge_vector.push_back(edge.c_intersectionedge)
        self.c_intersectionroute = IntersectionRoute(node_vector, edge_vector)

    def getNodeList(self):
        cdef vector[intersectionnodepointer] intersection_node_vector = self.c_intersectionroute.getNodeList()
        intersection_nodes = []
        for intersectionnode_ptr in intersection_node_vector:
            intersection_nodes.append(PyIntersectionNodePointer.fromCppPointer(intersectionnodepointer))
        return intersection_nodes

    def getEdgeList(self):
        cdef vector[IntersectionEdge] intersectionedgevector = self.c_intersectionroute.getEdgeList()
        pyvector = []
        for intersectionedge in intersectionedgevector:
            pyvector.append(PyIntersectionEdge.fromCppObject(intersectionedge))
        return pyvector

cdef class PyNode:
    """
    Parameters
    ----------
    loc: tuple of int
        The coordinates of the node's location.
    """

    cdef Node c_node

    def __cinit__(self, *args, **kwargs):
        if isinstance(self, PyIntersectionNode):
            return

        if args:
            self.c_node = args[0]
        else:
            try:
                self.c_node = kwargs["loc"]
            except KeyError:
                raise TypeError
    
    def getLoc(self):
        cdef Point3d loc = self.c_node.getLoc()
        return loc.x(), loc.y(), loc.z()

    def getID(self):
        return self.c_node.getID()


cdef class PyIntersectionNode(PyNode):
    cdef IntersectionNode c_intersectionnode

    def __cinit__(self, tuple coords, int junctiontype):
        self.c_intersectionnode = IntersectionNode(Point3d(*coords), <JUNCTIONTYPE>junctiontype)

    def getJunctionType(self):
        return <int>self.c_intersectionnode.getJunctionType()


cdef class PyBezierCurve:
    cdef BezierCurve c_beziercurve

    def __cinit__(self, s, e, list pyhandles):
        cdef intersectionnodepointer start_node_ptr
        if isinstance(s, PyIntersectionNodePointer):
            start_node_ptr = s.c_intersectionnodepointer
        elif isinstance(s, PyIntersectionNode):
            start_node_ptr = &(s.c_intersectionnode)
        else:
            raise TypeError

        cdef intersectionnodepointer end_node_ptr
        if isinstance(e, PyIntersectionNodePointer):
            end_node_ptr = e.c_intersectionnodepointer
        elif isinstance(e, PyIntersectionNode):
            end_node_ptr = &(e.c_intersectionnode)
        else:
            raise TypeError

        cdef vector[Point3d] handles
        cdef int handle
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
    """
    Parameters
    ----------
    s: PyNode or PyNodePointer
        The starting or "from" node of the edge.
    e: PyNode or PyNodePointer
        The ending or "to" node of the edge.
    """

    cdef Edge c_edge 

    def __cinit__(self, *args, **kwargs):

        if isinstance(self, PyIntersectionEdge) or isinstance(self, PyScenarioEdge):
            return

        cdef nodepointer s_ptr
        if len(args):
            s = args[0]
        else:
            try:
                s = kwargs["s"]
            except KeyError:
                raise TypeError
        if isinstance(s, PyNode):
            s_ptr = &(s.c_node)
        elif isinstance(s, PyNodePointer):
            s_ptr = s.c_nodepointer
        else:
            raise TypeError

        cdef nodepointer e_ptr
        if len(args) > 1:
            e = args[1]
        else:
            try:
                e = kwargs["e"]
            except KeyError:
                raise TypeError
        if isinstance(e, PyNode):
            e_ptr = &(e.c_node)
        elif isinstance(e, PyNodePointer):
            e_ptr = e.c_nodepointer
        else:
            raise TypeError

        self.c_edge = Edge(s_ptr, e_ptr)

    def getStartNode(self):
        return PyNodePointer.fromCppPointer(self.c_scenarioedge.getStartNode())

    def getEndNode(self):
        return PyNodePointer.fromCppPointer(self.c_scenarioedge.getEndNode())


cdef class PyScenarioEdge(PyEdge):
    cdef ScenarioEdge c_scenarioedge

    def __cinit__(self, s=None, e=None, dict demand={}):

        if s is None and e is None and not demand:
            return

        cdef nodepointer s_ptr
        if isinstance(s, PyNode):
            s_ptr = &(s.c_node)
        elif isinstance(s, PyNodePointer):
            s_ptr = s.c_nodepointer
        else:
            raise TypeError

        cdef nodepointer e_ptr
        if isinstance(e, PyNode):
            e_ptr = &(e.c_node)
        elif isinstance(e, PyNodePointer):
            e_ptr = e.c_nodepointer

        cdef map_[VEHICLETYPES, short int] c_demand
        for vehicletype in demand.keys():
            c_demand[<VEHICLETYPES>vehicletype] = demand[vehicletype]    
        self.c_scenarioedge = ScenarioEdge(s_ptr, e_ptr, c_demand)

    @staticmethod
    cdef PyScenarioEdge fromCppObject(ScenarioEdge c_scenarioedge):
        cdef PyScenarioEdge scenario_edge = PyScenarioEdge()
        scenario_edge.c_scenarioedge = c_scenarioedge
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

    def __cinit__(self, s=None, e=None, PyBezierCurve shape=None, short int numLanes=-1,
                  short int speedLimit=-1, short int priority=-1):

        if (s is None and e is None and shape is None and numLanes == -1 and speedLimit == -1
                and priority == -1):
            return

        cdef intersectionnodepointer s_ptr
        if isinstance(s, PyIntersectionNode):
            s_ptr = &(s.c_intersectionnode)
        if isinstance(s, PyIntersectionNodePointer):
            s_ptr = s.c_intersectionnodepointer
        else:
            raise TypeError

        cdef intersectionnodepointer e_ptr
        if isinstance(e, PyIntersectionNode):
            e_ptr = &(e.c_intersectionnode)
        if isinstance(e, PyIntersectionNodePointer):
            e_ptr = e.c_intersectionnodepointer
        else:
            raise TypeError

        cdef BezierCurve bezier_curve = (<PyBezierCurve?>shape).c_beziercurve
        self.c_intersectionedge = IntersectionEdge(s_ptr, e_ptr, bezier_curve, numLanes, speedLimit, priority)

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

cdef class PyNodePointer:
    cdef nodepointer c_nodepointer 
    
    def __cinit__(self):
        self.c_nodepointer = NULL

    @staticmethod
    cdef PyNodePointer fromCppPointer(nodepointer node_ptr):
        cdef PyNodePointer pynodepointer = PyNodePointer()
        pynodepointer.c_nodepointer = pointer
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

    def __cinit__(self):
        self.c_intersectionnodepointer = NULL

    @staticmethod
    cdef PyIntersectionNodePointer fromCppPointer(intersectionnodepointer i_node_ptr):
        cdef PyIntersectionNodePointer pyintersectionnodepointer = PyIntersectionNodePointer()
        pyintersectionnodepointer.c_intersectionnodepointer = i_node_ptr
        return pyintersectionnodepointer

    def getJunctionType(self):
        return deref(self.c_intersectionnodepointer).getJunctionType()

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
        return pyvector

    def getEdgeList(self):
        cdef vector[IntersectionEdge] cyvector = deref(self.c_intersectionroutepointer).getEdgeList()
        pyvector = []
        for edge in cyvector:
            pyvector.append(PyIntersectionEdge.fromCppObject(edge))
        return pyvector

    def __dealloc__(self):
        if self.c_intersectionroutepointer is not NULL:
            del self.c_intersectionroutepointer