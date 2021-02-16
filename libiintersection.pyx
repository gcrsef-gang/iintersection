#disutil language=c++

from libcpp.vector cimport vector
from libcpp.map cimport map
from libcpp.utility cimport pair

# Defining pointers since Cython's [] syntax doesn't work sometimes
ctypedef Node* nodepointer
ctypedef IntersectionRoute* intersectionroutepointer
ctypedef IntersectionNode* intersectionnodepointer

cdef extern from "libiintersection.h" namespace "ii":
    cdef cppclass IntersectionScenario:
        IntersectionScenario()
        IntersectionScenario(vector[nodepointer] nodes, vector[ScenarioEdge] edges) except +
        vector[nodepointer] getNodes()
        vector[ScenarioEdge] getEdges()
    cdef cppclass Intersection:
        Intersection()
        Intersection(vector[intersectionroutepointer]) except +
        void simulate(BACKENDS)
        void updateMetrics(BACKENDS)
        double getMetric(METRICS)
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
        map[VEHICLETYPES, short int] getDemand()
    cdef cppclass BezierCurve:
        BezierCurve()
        BezierCurve(intersectionnodepointer s, intersectionnodepointer e, vector[Point3d] handles) except +
        vector[Point3d] rasterize()
        int getStartNode()
        int getEndNode()
        vector[Point3d] getHandles()
    cdef cppclass IntersectionEdge(Edge):
        IntersectionEdge()
        IntersectionEdge(intersectionnodepointer s, intersectionnodepointer e, BezierCurve shape, short int numLanes, short int speedLimit) except +
        BezierCurve getShape()
        short int getNumLanes()
        short int getSpeedLimit()

cdef class PyIntersectionScenario:
    cdef IntersectionScenario c_intersectionscenario
    
    def __cinit__(self, pynodes, pyedges):
        cdef PyNode node
        cdef vector[nodepointer] nodes
        for node in pynodes:
            nodes.push_back(&(node.c_node))

        cdef PyScenarioEdge edge
        cdef vector[ScenarioEdge] edges
        for edge in pyedges:
            edges.push_back(edge.c_scenarioedge)
        self.c_intersectionscenario = IntersectionScenario(nodes, edges)
    
    def getNodes(self):
        nodevector = self.c_intersectionscenario.getNodes()
        pyvector = []
        for node in nodevector:
            loc3d = node.getLoc()
            cylocs = (loc3d.x(), loc3d.y(), loc3d.z())
            pyvector.append(PyNode(cylocs))
        return pyvector
    
    def getEdges(self):
        edgevector = self.c_intersectionscenario.getEdges()
        pyvector = []
        for edge in edgevector:
            startnode = edge.getStartNode()
            endnode = edge.getEndNode()
            demand = edge.getDemand()
            pyvector.append(PyScenarioEdge(startnode, endnode, demand))
        return pyvector

cdef class PyIntersection:
    cdef Intersection c_intersection

    def __cinit__(self, pyroutes):
        cdef PyIntersectionRoute route
        cdef vector[intersectionroutepointer] routes
        for route in pyroutes:
            routes.push_back(&(route.c_intersectionroute))
        self.c_intersection = Intersection(routes)

    def simulate(self, int backend):
        self.c_intersection.simulate(<BACKENDS>backend)

    def updateMetrics(self, int backend):
        self.c_intersection.updateMetrics(<BACKENDS>backend)

    def getMetric(self, int metric):
        return self.c_intersection.getMetric(<METRICS>metric)

    @property
    def routes(self):
        routesvector = self.c_intersection.routes
        pyvector = []
        for route in routesvector:
            nodeList = route.getNodeList()
            pynodelist = []
            for intersectionnode in nodeList:
                loc3d = intersectionnode.getLoc()
                cylocs = (loc3d.x(), loc3d.y(), loc3d.z())
                pynodelist.append(PyNode(cylocs))
            edgeList = route.getEdgeList()
            pyedgelist = []
            for intersectionedge in edgeList:
                startnode = intersectionedge.getStartNode()
                endnode = intersectionedge.getEndNode()
                shape = intersectionedge.getShape()
                handles = shape.getHandles()
                pyhandles = []
                for handle in handles:
                    pyhandles.append((handle.x(), handle.y(), handle.z()))
                pyshape = PyBezierCurve(shape.getStartNode(), shape.getEndNode(), pyhandles)
                numlanes = intersectionedge.getNumLanes()
                speedlimit = intersectionedge.getSpeedLimit()
                pyedgelist.append(PyIntersectionEdge(startnode, endnode, pyshape, numlanes, speedlimit))
            pyvector.append(PyIntersectionRoute(pynodelist, pyedgelist))
        return pyvector

cdef class PyIntersectionRoute:
    cdef IntersectionRoute c_intersectionroute

    def __cinit__(self, pynodeList, pyedgeList):
        cdef PyIntersectionNode node
        cdef vector[intersectionnodepointer] nodeList
        for node in pynodeList:
            nodeList.push_back(&(node.c_intersectionnode))

        cdef PyIntersectionEdge edge
        cdef vector[IntersectionEdge] edgeList
        for edge in pyedgeList:
            edgeList.push_back(edge.c_intersectionedge)
        self.c_intersectionroute = IntersectionRoute(nodeList, edgeList)

    def getNodeList(self):
        intersectionnodevector = self.c_intersectionroute.getNodeList()
        pyvector = []
        for intersectionnode in intersectionnodevector:
            loc3d = intersectionnode.getLoc()
            cylocs = (loc3d.x(), loc3d.y(), loc3d.z())
            cyjunctiontype = intersectionnode.getJunctionType()
            pyvector.append(PyIntersectionNode(cylocs, cyjunctiontype))
        return pyvector

    def getEdgeList(self):
        intersectionedgevector = self.c_intersectionroute.getEdgeList()
        pyvector = []
        for intersectionedge in intersectionedgevector:
            startnode = intersectionedge.getStartNode()
            endnode = intersectionedge.getEndNode()
            shape = intersectionedge.getShape()
            handles = shape.getHandles()
            pyhandles = []
            for handle in handles:
                pyhandles.append((handle.x(), handle.y(), handle.z()))
            pyshape = PyBezierCurve(shape.getStartNode(), shape.getEndNode(), pyhandles)
            numlanes = intersectionedge.getNumLanes()
            speedlimit = intersectionedge.getSpeedLimit()
            pyvector.append(PyIntersectionEdge(startnode, endnode, pyshape, numlanes, speedlimit))
        return pyvector

cdef class PyNode:
    cdef Node c_node

    def __cinit__(self, vector[short int] tuple):
        if type(self) is PyNode:
            self.c_node = Node(Point3d(tuple))
    
    def getLoc(self):
        locvector = self.c_node.getLoc()
        pytuple = (locvector.x(), locvector.y(), locvector.z())
        return pytuple

    def getId(self):
        return self.c_node.getID()

cdef class PyIntersectionNode(PyNode):
    cdef IntersectionNode c_intersectionnode

    def __cinit__(self, vector[short int] tuple, int junctiontype):
        self.c_intersectionnode = IntersectionNode(Point3d(tuple), <JUNCTIONTYPE>junctiontype)

    def getJunctionType(self):
        junction = self.c_intersectionnode.getJunctionType()
        print(f"Type of getJunctionType return value: {type(junction)}")
        return junction

cdef class PyBezierCurve():
    cdef BezierCurve c_beziercurve

    def __cinit__(self, s, e, pyhandles):
        cdef startnode = s.c_intersectionnode
        cdef Point3d startloc = Point3d(startnode.getLoc())
        cdef startjunctiontype = startnode.getJunctionType()
        cdef IntersectionNode cystartnode = IntersectionNode(startloc, startjunctiontype)

        cdef endnode = s.c_intersectionnode
        cdef Point3d endloc = Point3d(endnode.getLoc())
        cdef endjunctiontype = endnode.getJunctionType()
        cdef IntersectionNode cyendnode = IntersectionNode(endloc, endjunctiontype)

        cdef vector[Point3d] handles
        for tuple_ in pyhandles:
            handles.push_back(Point3d(tuple_))
        self.c_beziercurve = BezierCurve(&cystartnode, &cyendnode, handles)

    def rasterize(self):
        point3dvector = self.c_beziercurve.rasterize()
        pyvector = []
        for point3d in point3dvector:
            pointtuple = (point3d.x(), point3d.y(), point3d.z())
            pyvector.append(pointtuple)
        return pyvector

    def getStartNode(self):
        return self.c_beziercurve.getStartNode()
    
    def getEndNode(self):
        return self.c_beziercurve.getEndNode()

    def getHandles(self):
        point3dvector = self.c_beziercurve.getHandles()
        pyvector = []
        for point3d in point3dvector:
            pointtuple = (point3d.x(), point3d.y(), point3d.z())
            pyvector.append(pointtuple)
        return pyvector

cdef class PyEdge():
    cdef Edge c_edge 

    def __cinit__(self, s, e):
        cdef startnode = s.c_node
        cdef Point3d startloc = Point3d(startnode.getLoc())
        cdef Node cystartnode = Node(startloc)

        cdef endnode = e.c_node
        cdef Point3d endloc = Point3d(endnode.getLoc())
        cdef Node cyendnode = Node(endloc)
        self.c_edge = Edge(&cystartnode, &cyendnode)

    def getStartNode(self):
        return self.c_scenarioedge.getStartNode()

    def getEndNode(self):
        return  self.c_scenarioedge.getEndNode()

cdef class PyScenarioEdge(PyEdge):
    cdef ScenarioEdge c_scenarioedge

    def __cinit__(self, s, e, demand):
        cdef startnode = s.c_node
        cdef Point3d startloc = Point3d(startnode.getLoc())
        cdef Node cystartnode = Node(startloc)

        cdef endnode = e.c_node
        cdef Point3d endloc = Point3d(endnode.getLoc())
        cdef Node cyendnode = Node(endloc)  
        cdef map[VEHICLETYPES, short int] cydemand
        for vehicletype in demand.keys():
            cydemand[<VEHICLETYPES>vehicletype] = demand[vehicletype]    
        self.c_scenarioedge = ScenarioEdge(&cystartnode, &cyendnode, cydemand)


    def getDemand(self):
        return self.c_scenarioedge.getDemand()

cdef class PyIntersectionEdge(PyEdge):
    cdef IntersectionEdge c_intersectionedge

    def __cinit__(self, s, e, PyBezierCurve shape, short int numLanes, short int speedLimit):
        cdef startnode = s.c_intersectionnode
        cdef Point3d startloc = Point3d(startnode.getLoc())
        cdef startjunctiontype = startnode.getJunctionType()
        cdef IntersectionNode cystartnode = IntersectionNode(startloc, startjunctiontype)

        cdef endnode = s.c_intersectionnode
        cdef Point3d endloc = Point3d(endnode.getLoc())
        cdef endjunctiontype = endnode.getJunctionType()
        cdef IntersectionNode cyendnode = IntersectionNode(endloc, endjunctiontype)
        self.c_intersectionedge = IntersectionEdge(&cystartnode, &cyendnode, shape.c_beziercurve, numLanes, speedLimit)

    def getShape(self):
        cybeziercurve = self.c_intersectionedge.getShape()
        intersectionstartnode = cybeziercurve.getStartNode()
        intersectionendnode = cybeziercurve.getEndNode()
        handles = cybeziercurve.getHandles()
        pyhandles = []
        for handle in handles:
            pyhandles.append((handle.x(), handle.y(), handle.z()))
        return PyBezierCurve(intersectionstartnode, intersectionendnode, pyhandles)

    def getNumLanes(self):
        return self.c_intersectionedge.getNumLanes()

    def getSpeedLimit(self):
        return self.c_intersectionedge.getSpeedLimit()