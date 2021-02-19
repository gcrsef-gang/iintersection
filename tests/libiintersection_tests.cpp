#include "../libiintersection.h"
#include <iostream>

using namespace std;
using namespace ii;


int main() {
    IntersectionScenario int_("monte-carlo.xml");
    for (int i = 0; i < int_.getNodes().size(); i++) {
        cout << int_.getNodes()[i]->getID() << endl; 
    }
}