#include "../libiintersection.h"
#include <iostream>

using namespace std;
using namespace ii;


Intersection getIntersection() {
    IntersectionNode* n1 = GLOBALDATA->createIntersectionNode(Point3d(-1, 0, 0), JUNCTIONTYPES::UNREGULATED);
    IntersectionNode* n2 = GLOBALDATA->createIntersectionNode(Point3d(1, 0, 0), JUNCTIONTYPES::UNREGULATED);
    IntersectionNode* n3 = GLOBALDATA->createIntersectionNode(Point3d(0, 0, 0), JUNCTIONTYPES::UNREGULATED);
    IntersectionNode* n4 = GLOBALDATA->createIntersectionNode(Point3d(0, 1, 0), JUNCTIONTYPES::UNREGULATED);
    IntersectionNode* n5 = GLOBALDATA->createIntersectionNode(Point3d(0, -1, 0), JUNCTIONTYPES::UNREGULATED);


    IntersectionEdge e1 = IntersectionEdge(n1, n3, BezierCurve(n1, n3, {}), 1, 45, 1);
    IntersectionEdge e2 = IntersectionEdge(n3, n2, BezierCurve(n3, n2, {}), 1, 45, 2);
    IntersectionEdge e3 = IntersectionEdge(n4, n3, BezierCurve(n4, n3, {}), 1, 45, 3);
    IntersectionEdge e4 = IntersectionEdge(n3, n5, BezierCurve(n3, n5, {}), 1, 45, 4);
    
    
    IntersectionRoute r1 = IntersectionRoute({n1, n3, n2}, {e1, e2});
    IntersectionRoute r2 = IntersectionRoute({n4, n3, n5}, {e3, e4});

    return Intersection({r1, r2});
}


int main() {
    Intersection int_ = getIntersection();
    int_.simulate(BACKENDS::SUMO);
    int_.getMetric(METRICS::EFFICIENCY);
}