#include "../libiintersection.h"
#include <iostream>

using namespace std;
using namespace ii;


int main() {
    // IntersectionScenario int_("monte-carlo.xml");
    // for (int i = 0; i < int_.getNodes().size(); i++) {
    //     cout << int_.getNodes()[i].getID() << endl; 
    // }
    Point3d p1(500, 1000, 500);
    Point3d p2(3500, 1500, 0);
    Point3d h1(1500, 1700, 800);
    Point3d h2(2500, 1900, -200);

    IntersectionNode* n1 = GLOBALDATA->createIntersectionNode(p1, JUNCTIONTYPES::UNREGULATED);
    IntersectionNode* n2 = GLOBALDATA->createIntersectionNode(p2, JUNCTIONTYPES::UNREGULATED);

    BezierCurve curve(n1, n2, {h1, h2});

    std::vector<Point3d> points = curve.rasterize(200);

    for (Point3d p : points) {
        cout << p.x() << "\t" << p.y() << "\t" << p.z() << endl;
    }
}