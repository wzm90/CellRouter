#include <iostream>
#include <map>
#include "oaDesignDB.h"
#include "EndPoint.h"

using namespace oa;
using namespace std;

EndPoint_t::EndPoint_t(oaCoord x, oaCoord y, oaInt4 id)
    : _escapePoints(), _hlines(), _vlines(), _netID(id)
{
    _orient = BOTH;
    _noEscape = false;
    _escapePoints.push_back(oaPoint(x, y));
}

void
EndPoint_t::addHline(const line_t &newline)
{
    LineSet_t::iterator it = _hlines.find(newline.first.y());
    if (it != _hlines.end()) {
#ifdef DEBUG
        cerr << "Should not enter here!" << endl;
#endif
        // replace with a longer hline
        if ((it->second).first.x() > newline.first.x()) {
            (it->second).first.x() = newline.first.x();
        }
        if ((it->second).second.x() < newline.second.x()) {
            (it->second).second.x() = newline.second.x();
        }
    }
    else {
        _hlines.insert(LineSet_t::value_type(newline.first.y(), newline));
    }
}


void
EndPoint_t::addVline(const line_t &newline)
{
    LineSet_t::iterator it = _vlines.find(newline.first.x());
    if (it != _vlines.end()) {
#ifdef DEBUG
        cerr << "Should not enter here!" << endl;
#endif
        // replace with a longer vline
        if ((it->second).first.y() > newline.first.y()) {
            (it->second).first.y() = newline.first.y();
        }
        if ((it->second).second.y() < newline.second.y()) {
            (it->second).second.y() = newline.second.y();
        }
    }
    else {
        _vlines.insert(LineSet_t::value_type(newline.first.x(), newline));
    }
}

bool
EndPoint_t::isIntersect(const line_t &line) const 
{
    if (line.first.x() == line.second.x()) {
        // vertical line

        // iterate all the horizontal lines of the Endpoint
        // whose y is within the y range of line, if line.x()
        // is within the x range of one horizontal line, then
        // we find a intersection
        
        LineSet_t::const_iterator lineIter, low, high;
        low = _hlines.lower_bound(line.first.y());
        high = _hlines.upper_bound(line.second.y());
        oaCoord xpos = line.first.x();
        for (lineIter = low; lineIter != high; ++lineIter) {
            if (lineIter->second.first.x() <= xpos && \
                    xpos <= lineIter->second.second.x()) {
                return true;
            }
        }
        return false;
    }
    else if (line.first.y() == line.second.y()) {
        // horizontal line

        // iterate all the vertical lines of the Endpoint
        // whose x is within the x range of line, if line.y()
        // is within the y range of one vertical line, then
        // we find a intersection

        LineSet_t::const_iterator lineIter, low, high;
        low = _vlines.lower_bound(line.first.x());
        high = _vlines.upper_bound(line.second.x());
        oaCoord ypos = line.first.y();
        for (lineIter = low; lineIter != high; ++lineIter) {
            if (lineIter->second.first.y() <= ypos && \
                    ypos <= lineIter->second.second.y()) {
                return true;
            }
        }
        return false;
    }
    else {
#ifdef DEBUG
        cerr << "The line is neither vertical line or horizontal line.";
        cerr << endl;
        exit(1);
#endif
        return false;
    }
}

bool
EndPoint_t::onEscapeLines(const oaPoint &point, Orient_t orient) const
{
    LineSet_t::const_iterator lineIter;

    switch (orient) {
    case BOTH:
        // fall through
    case HORIZONTAL:
        lineIter = _hlines.find(point.y());
        if (lineIter != _hlines.end()) {
            // check if point is lies on this line
            if ((lineIter->second.first.x() <= point.x()) && \
                    (point.x() <= lineIter->second.second.x())) {
                return true;
            }
        }
        if (HORIZONTAL == orient) {
            // let case BOTH pass through
            break;
        }
    case VERTICAL:
        lineIter = _vlines.find(point.x());
        if (lineIter != _vlines.end()) {
            // check if point lies on this line
            if ((lineIter->second.first.y() <= point.y()) && \
                    (point.y() <= lineIter->second.second.y())) {
                return true;
            }
        }
        break;
    default:
        cerr << "Invalid orient!" << endl;
        exit(1);
    }

    return false;
}
