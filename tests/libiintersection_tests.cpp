#include "../libiintersection.h"
#include <iostream>

using namespace std;
using namespace ii;


Intersection getIntersection() {
    IntersectionNode* n1 = new IntersectionNode(Point3d(-1, 0, 0), JUNCTIONTYPES::UNREGULATED);
    IntersectionNode* n2 = new IntersectionNode(Point3d(1, 0, 0), JUNCTIONTYPES::UNREGULATED);
    IntersectionNode* n3 = new IntersectionNode(Point3d(0, 0, 0), JUNCTIONTYPES::UNREGULATED);
    IntersectionNode* n4 = new IntersectionNode(Point3d(0, 1, 0), JUNCTIONTYPES::UNREGULATED);
    IntersectionNode* n5 = new IntersectionNode(Point3d(0, -1, 0), JUNCTIONTYPES::UNREGULATED);


    IntersectionEdge e1 = IntersectionEdge(n1, n3, BezierCurve(n1, n3, {}), 1, 45, 1);
    IntersectionEdge e2 = IntersectionEdge(n3, n2, BezierCurve(n3, n2, {}), 1, 45, 2);
    IntersectionEdge e3 = IntersectionEdge(n4, n3, BezierCurve(n4, n3, {}), 1, 45, 3);
    IntersectionEdge e4 = IntersectionEdge(n3, n5, BezierCurve(n3, n5, {}), 1, 45, 4);
    
    e1.setPriority(1);
    e2.setPriority(1);
    e3.setPriority(1);
    e4.setPriority(1);

    e1.setNumLanes(1);
    e2.setNumLanes(1);
    e3.setNumLanes(1);
    e4.setNumLanes(1);

    e1.setSpeedLimit(50);
    e2.setSpeedLimit(50);
    e3.setSpeedLimit(50);
    e4.setSpeedLimit(50);

    IntersectionRoute r1 = IntersectionRoute({n1, n3, n2}, {e1, e2});
    IntersectionRoute r2 = IntersectionRoute({n4, n3, n5}, {e3, e4});

    return Intersection({r1, r2});
}


int main() {
    Intersection int_ = getIntersection();
    Intersection int_2 = getIntersection();

    Intersection::Simulate(&int_, BACKENDS::SUMO);
    int_.getMetric(METRICS::EFFICIENCY);
}