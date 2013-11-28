#include <limits>
#include <vector>
#include <iostream>
#include "Net.h"
#include "oaDesignDB.h"

using namespace std;
using namespace oa;

Net_t::Net_t(const vector<oaPoint> &points, oaUInt4 id, NetType_t type, \
        oaString portName)
    : vector<oaPoint>(points), _id(id), _type(type), _portName(portName)
{
    vector<oaPoint>::const_iterator it;
    oaInt4 xmin, xmax, ymin, ymax;
    xmin = ymin = numeric_limits<oaInt4>::max(); 
    xmax = ymax = numeric_limits<oaInt4>::min();
    for (it = points.begin(); it != points.end(); ++it) {
        xmin = (it->x() < xmin) ? it->x() : xmin;
        xmax = (it->x() > xmax) ? it->x() : xmax; 
        ymin = (it->y() < ymin) ? it->y() : ymin;
        ymax = (it->y() > ymax) ? it->y() : ymax; 
    } 
    _bbox.set(oaPoint(xmin, ymin), oaPoint(xmax, ymax));
#ifdef DEBUG
    cout << "Bounding box for net: " << _id << " is: (";
    cout << _bbox.lowerLeft().x() << ", " << _bbox.lowerLeft().y() << ") ";
    cout << "(" << _bbox.upperRight().x() << ", " << _bbox.upperRight().y() << ")";
    cout << endl;
#endif
}
