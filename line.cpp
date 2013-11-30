#include <iostream>
#include "line.h"
#include "oaDesignDB.h"

using namespace std;
using namespace oa;

oaBoolean 
line_t::contains(const oaPoint &point) {
    if (this->first.x() == this->second.x()) {
        if (point.x() == this->first.x()) {
            oaCoord ypos = point.y();
            if (this->first.y() <= ypos && ypos <= this->second.y()) {
                return true;
            }
        }
    }
    else if (this->first.y() == this->second.y()) {
        if (point.y() == this->first.y()) {
            oaCoord xpos = point.x();
            if (this->first.x() <= xpos && xpos <= this->second.x()) {
                return true;
            }
        }
    }
    else {
        cerr << "The line is neither vertical nor horizontal!" << endl;
    }
    return false;
}
