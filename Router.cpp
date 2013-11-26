#include <vector>
#include <algorithm>
#include <iostream>
#include "Router.h"
#include "EndPoint.h"

using namespace oa;
using namespace std;

static const oa::oaLayerNum CONTACT = 7;
static const oa::oaLayerNum METAL1 = 8;
static const oa::oaLayerNum VIA1 = 11;
static const oa::oaLayerNum METAL2 = 12;

class Comparator {
public:
    Comparator(const oaPoint &point) : center(point.x(), point.y()) {}
    bool operator()(const oaPoint &lhs, const oaPoint &rhs);
private:
    oaPoint center;
};

bool
Comparator::operator()(const oaPoint &lhs, const oaPoint &rhs)
{
    oaInt8 deltaX = (lhs.x() - center.x());
    deltaX = deltaX * deltaX;
    oaInt8 deltaY = (lhs.y() - center.y());
    deltaY = deltaY * deltaY;

    oaInt8 distance1 = deltaX + deltaY;

    deltaX = (rhs.x() - center.x());
    deltaX = deltaX * deltaX;
    deltaY = (rhs.y() - center.y());
    deltaY = deltaY * deltaY;

    oaInt8 distance2 = deltaX + deltaY;

    return (distance1 < distance2);
}

Router_t::Router_t(oaDesign *design, oaTech *tech, ifstream &file1,\
        ifstream &file2)
    :_design(design), _tech(tech), _nets(file1), _designRule(file2)
{
    // get bounding box of VDD rail and VSS rail
    oaBlock *block = _design->getTopBlock();
    
    oaLayerHeader *m1LayerHeader;
    m1LayerHeader = oaLayerHeader::find(block, 8);
    if (NULL == m1LayerHeader) {
        cerr << "Cannot open metal1 layer.\n";
        exit(1);
    }

    // get positions of VDD and VSS rails
    oaIter<oaLPPHeader> LPPHeaderIter(m1LayerHeader->getLPPHeaders());
    while (oaLPPHeader *LPPHeader = LPPHeaderIter.getNext()) {
        oaIter<oaShape> shapeIter(LPPHeader->getShapes());
        while (oaShape *shape = shapeIter.getNext()) {
            if (shape->getType() == oacRectType) {
                oaBox bbox;
                shape->getBBox(bbox);
                if (bbox.bottom() < 0) {
                    // vss rail
                    shape->getBBox(_VSSBox);
                } else {
                    // vdd rail
                    shape->getBBox(_VDDBox);
                }
            }
        }
    }
#ifdef DEBUG
    cout << "VDD rail position: ";
    cout << "(" << _VDDBox.left() << " " << _VDDBox.bottom() << ")";
    cout << " (" << _VDDBox.right() << " " << _VDDBox.top() << ")";
    cout << endl;
    
    cout << "VSS rail position: ";
    cout << "(" << _VSSBox.left() << " " << _VSSBox.bottom() << ")";
    cout << " (" << _VSSBox.right() << " " << _VSSBox.top() << ")";
    cout << endl;
#endif
    // initialize obstacles
    oaBox routeRegionBox(_VDDBox.left(), _VSSBox.top(), _VSSBox.right(), \
            _VDDBox.bottom());

    addObstacle(METAL1, routeRegionBox);
    addObstacle(METAL2, routeRegionBox);
    // add all contacts as obstacles
    net_iterator netIter;
    // contacts in VDD net
    for (netIter = _nets.begin(VDD); netIter != _nets.end(VDD); ++netIter) {
        Net_t::const_iterator citer;
        for (citer = netIter->begin(); citer != netIter->end(); ++citer) {
            oaPoint upperRight(citer->x() + _designRule.viaWidth(), \
                    citer->y() + _designRule.viaHeight());

            oaBox contactBBox(*citer, upperRight);
            addObstacle(METAL1, contactBBox);
            addObstacle(METAL2, contactBBox);
        }
    }
    
    // contacts in VSS net
    for (netIter = _nets.begin(VSS); netIter != _nets.end(VSS); ++netIter) {
        Net_t::const_iterator citer;
        for (citer = netIter->begin(); citer != netIter->end(); ++citer) {
            oaPoint upperRight(citer->x() + _designRule.viaWidth(), \
                    citer->y() + _designRule.viaHeight());

            oaBox contactBBox(*citer, upperRight);
            addObstacle(METAL1, contactBBox);
            addObstacle(METAL2, contactBBox);
        }
    }
    
    // contacts in IO net
    for (netIter = _nets.begin(IO); netIter != _nets.end(IO); ++netIter) {
        Net_t::const_iterator citer;
        for (citer = netIter->begin(); citer != netIter->end(); ++citer) {
            oaPoint upperRight(citer->x() + _designRule.viaWidth(), \
                    citer->y() + _designRule.viaHeight());

            oaBox contactBBox(*citer, upperRight);
            addObstacle(METAL1, contactBBox);
            addObstacle(METAL2, contactBBox);
        }
    }

    // contacts in S net
    for (netIter = _nets.begin(S); netIter != _nets.end(S); ++netIter) {
        Net_t::const_iterator citer;
        for (citer = netIter->begin(); citer != netIter->end(); ++citer) {
            oaPoint upperRight(citer->x() + _designRule.viaWidth(), \
                    citer->y() + _designRule.viaHeight());

            oaBox contactBBox(*citer, upperRight);
            addObstacle(METAL1, contactBBox);
            addObstacle(METAL2, contactBBox);
        }
    }


    // create metal2 layer and via1 layer if any of them does not exist
    oaLayer * layer;

    // check if via1 layer is in the database
    layer =  oaLayer::find(_tech, "via1");
    if (layer == NULL) {
        cout << "Creating via1 layer\n";
        oaPhysicalLayer::create(_tech, "via1", 11, oacMetalMaterial, 11);
    }

    // check if metal2 layer is in the database
    layer =  oaLayer::find(_tech, "metal2");
    if (layer == NULL) {
        cout << "Creating metal2 layer\n";
        oaPhysicalLayer::create(_tech, "metal2", 12, oacMetalMaterial, 12);
    }
}

bool
Router_t::route()
{
    net_iterator netIter;
    bool result = true;

    // route VDD
    for (netIter = _nets.begin(VDD); netIter != _nets.end(VDD); ++netIter) {
        Net_t net = *netIter;
        bool oneResult = routeOneNet(VDD, net);
        result = oneResult && result;
    }
    // route VSS
    cout << "VSS net:" << endl;
    for (netIter = _nets.begin(VSS); netIter != _nets.end(VSS); ++netIter) {
        Net_t net = *netIter;
        bool oneResult = routeOneNet(VSS, net);
        result = oneResult && result;
    }
    // route S
    cout << "S net:" << endl;
    for (netIter = _nets.begin(S); netIter != _nets.end(S); ++netIter) {
        Net_t net = *netIter;
        bool oneResult = routeOneNet(S, net);
        result = oneResult && result;
    }
    // route IO
    cout << "IO net:" << endl;
    for (netIter = _nets.begin(IO); netIter != _nets.end(IO); ++netIter) {
        Net_t net = *netIter;
        bool oneResult = routeOneNet(IO, net);
        result = oneResult && result;
    }

    return result;
}

bool
Router_t::routeOneNet(NetType_t type, Net_t &net)
{
    switch (type) {
    case VDD:
        return routeVDD(net);
    case VSS:
        return routeVSS(net);
    case S:
        return routeSignal(net);
    case IO:
        return routeIO(net);
    default:
        cerr << "Unknow NetType_t detected..." << endl;
        exit(1);
    }
}

bool
Router_t::routeVDD(Net_t &net)
{
    return true;
}


bool
Router_t::routeVSS(Net_t &net)
{
    return true;
}


bool
Router_t::routeSignal(Net_t &net)
{
    if (net.size() > 1) {
        /*
        sort(net.begin(), net.end(), compx);        
        oaCoord y_val = net[net.size() / 2].y();
        oaCoord xmin = _VDDBox.right(); 
        oaCoord xmax = _VDDBox.left();
        Net_t::const_iterator netIter;
        for (netIter = net.begin(); netIter != net.end(); ++netIter) {
            if (netIter->x() < xmin) {
                xmin = netIter->x();
            }
            if (netIter->x() > xmax) {
                xmax = netIter->x();
            }
        }
        oaPoint leftEnd(xmin, y_val);
        oaPoint rightEnd(xmax, y_val);
        createWire(leftEnd, rightEnd);
        
        for (netIter = net.begin(); netIter != net.end(); ++netIter) {
            oaPoint steiner(netIter->x(), y_val);
            createWire(*netIter, steiner);
        }
        */
        Net_t::iterator it1 = net.begin();
        Net_t::iterator it2 = net.begin();
        ++it2;
        return routeTwoContacts(*it1, *it2);
    } 

    return true;
}


bool
Router_t::routeIO(Net_t &net)
{
    return true;
}

// Route two contacts using line-probing algorithm as described in
// "A Solution to line-routing problems on the continuous plane"
bool
Router_t::routeTwoContacts(oaPoint lhs, oaPoint rhs)
{
    // two contacts are represented by two leftdown points of their actual
    // box, now we move these two points to the center of contact bounding box
    EndPoint_t A(lhs.x()+_designRule.viaWidth()/2, \
        lhs.y()+_designRule.viaHeight()/2);

    EndPoint_t B(rhs.x()+_designRule.viaWidth()/2, \
        rhs.y()+_designRule.viaHeight()/2);
    
    bool intersect = false;
    EndPoint_t *src = &A;
    EndPoint_t *dst = &B;

    // Algorithm begins
    while (!intersect) {
        if (src->noEscape()) {
            if (dst->noEscape()) {
                return false;
            }
            else {
                // swap src and dst
                EndPoint_t *temp = src;
                src = dst;
                dst = temp;
                // apply escape algorithm
                intersect = escape(*src, *dst);
            }
        }
        else {
            intersect = escape(*src, *dst);
            // swap src and dst
            EndPoint_t *temp = src;
            src = dst;
            dst = temp;
        }
    }
    // apply refinement algorithm
    PointSet_t points1 = src->escapePoints();
    PointSet_t points2 = dst->escapePoints();
    PointSet_t::const_iterator it;
    for (it = points1.begin(); it != points1.end(); ++it) {
        cout << "(" << it->x() << ", " << it->y() << ") ";
    } 
    cout << endl;
    for (it = points2.begin(); it != points2.end(); ++it) {
        cout << "(" << it->x() << ", " << it->y() << ") ";
    }
    cout << endl;
    return true;
}

// escape algorithm
bool
Router_t::escape(EndPoint_t &src, EndPoint_t &dst)
{
    if ((src.orient() == HORIZONTAL) || (src.orient() == BOTH)) {
        line_t escapeLine;
        getEscapeLine(src, HORIZONTAL, escapeLine);
        // add escapeLine
        src.addHline(escapeLine);
        if (dst.isIntersect(escapeLine)) {
            return true;
        }
    }
    if ((src.orient() == VERTICAL) || (src.orient() == BOTH)) {
        line_t escapeLine;
        getEscapeLine(src, VERTICAL, escapeLine);
        // add escapeLine
        src.addVline(escapeLine);
        if (dst.isIntersect(escapeLine)) {
            return true;
        }
    }
    // get escapePoint
    getEscapePoint(src);
    return false;
}


// Examine the orientation flag and construct the appropriate escape line
// through the object point.
void
Router_t::getEscapeLine(const EndPoint_t &src, Orient_t orient, line_t &escapeLine)
{
    line_t leftCover, rightCover, bottomCover, topCover;
    if (orient == HORIZONTAL) {
        line_t leftCover, rightCover;
        getCover(src, LEFT, leftCover);
        getCover(src, RIGHT, rightCover);
        escapeLine.first.y() = escapeLine.second.y() = src.getObjectPoint().y();       
        escapeLine.first.x() = leftCover.first.x();
        escapeLine.second.x() = rightCover.first.x();
    }
    else if (orient == VERTICAL) {
        line_t bottomCover, topCover;
        getCover(src, BOTTOM, bottomCover);
        getCover(src, TOP, topCover);
        escapeLine.first.x() = escapeLine.second.x() = src.getObjectPoint().x();
        escapeLine.first.y() = bottomCover.first.y();
        escapeLine.second.y() = topCover.first.y();
    }
    else {
        cerr << "Invalid orient!" << endl;
        exit(1);
    }
}

// Apply escape point finding algorithm to find escape point of 
// object point. If escapePoint is found, set orientation flag, push
// escapePoint to a list of previous escapePoints and return. Otherwise
// set noEscape flag and return.
void
Router_t::getEscapePoint(EndPoint_t &src)
{
    // get covers
    line_t bottomCover, topCover, leftCover, rightCover;
    oaPoint objectPoint = src.getObjectPoint();

    getCover(src, BOTTOM, bottomCover);
    getCover(src, TOP, topCover);
    getCover(src, LEFT, leftCover);
    getCover(src, RIGHT, rightCover);

    vector<oaPoint> extremeties;
    if (bottomCover.first.y() != _VSSBox.top()) {
        extremeties.push_back(bottomCover.first);
        extremeties.push_back(bottomCover.second);
    } 
    if (topCover.first.y() != _VDDBox.bottom()) {
        extremeties.push_back(topCover.first);
        extremeties.push_back(topCover.second);
    }
    // sort extremeties
    Comparator comp(objectPoint);
    sort(extremeties.begin(), extremeties.end(), comp);

    vector<oaPoint>::const_iterator pIter;
    oaPoint escapePoint;

    // Escape Process I
    for (pIter = extremeties.begin(); pIter != extremeties.end(); ++pIter) {
        if (pIter->x() < objectPoint.x()) {
            if (pIter->x() - leftCover.first.x() >= _designRule.metalWidth()) {
                escapePoint.x() = pIter->x() - _designRule.metalWidth() / 2;
                escapePoint.y() = objectPoint.y();
                if (!src.onEscapeLines(escapePoint, VERTICAL)) {
                    src.setOrient(VERTICAL);
                    src.addEscapePoint(escapePoint);
                    return;
                }
            }
        }
        else {
            if (rightCover.first.x() - pIter->x() >= _designRule.metalWidth()) {
                escapePoint.x() = pIter->x() + _designRule.metalWidth() / 2;
                escapePoint.y() = objectPoint.y();
                if (!src.onEscapeLines(escapePoint, VERTICAL)) {
                    src.setOrient(VERTICAL);
                    src.addEscapePoint(escapePoint);
                    return;
                }
            }
        }
    }

    // sort extremeties
    extremeties.clear();
    if (leftCover.first.x() != _VDDBox.left()) {
        extremeties.push_back(leftCover.first);
        extremeties.push_back(leftCover.second);
    } 
    if (rightCover.first.x() != _VDDBox.right()) {
        extremeties.push_back(rightCover.first);
        extremeties.push_back(rightCover.second);
    }
    sort(extremeties.begin(), extremeties.end(), comp);
    for (pIter = extremeties.begin(); pIter != extremeties.end(); ++pIter) {
        if (pIter->y() < objectPoint.y()) {
            if (pIter->y() - bottomCover.first.y() >= _designRule.metalWidth()) {
                escapePoint.x() = objectPoint.x();
                escapePoint.y() = pIter->y() - _designRule.metalWidth() / 2;
                if (!src.onEscapeLines(escapePoint, HORIZONTAL)) {
                    src.setOrient(HORIZONTAL);
                    src.addEscapePoint(escapePoint);
                    return;
                }
            }
        }
        else {
            if (topCover.first.y() - pIter->y() >= _designRule.metalWidth()) {
                escapePoint.x() = objectPoint.x();
                escapePoint.y() = pIter->y() + _designRule.metalWidth() / 2;
                if (!src.onEscapeLines(escapePoint, HORIZONTAL)) {
                    src.setOrient(HORIZONTAL);
                    src.addEscapePoint(escapePoint);
                    return;
                }
            }
        }
    }
    src.setNoEscape(true);
    // Escape Process II
    // TO BE DONE LATER (if I still have time...)
}

void
Router_t::createWire(const oaPoint &lhs, const oaPoint &rhs)
{
    if (lhs != rhs) {
        if (lhs.x() == rhs.x()) {
            // create wire rect
            oaCoord wireleft = lhs.x();
            oaCoord wireright = lhs.x() + _designRule.viaWidth();
            oaCoord wiretop, wirebottom;
            if (lhs.y() < rhs.y()) {
                wirebottom = lhs.y() - _designRule.viaExtension();
                wiretop = rhs.y() + _designRule.viaHeight() + \
                          _designRule.viaExtension();
            } else {
                wirebottom = rhs.y() - _designRule.viaExtension();
                wiretop = lhs.y() + _designRule.viaHeight() + \
                          _designRule.viaExtension();
            }
            oaBox wirebox(wireleft, wirebottom, wireright, wiretop);
            oaRect::create(_design->getTopBlock(), METAL1, 1, wirebox);
        } else if (lhs.y() == rhs.y()) {
            // create wire rect
            oaCoord wiretop = lhs.y() + _designRule.viaHeight();
            oaCoord wirebottom = lhs.y();
            oaCoord wireleft, wireright;
            if (lhs.x() < rhs.x()) {
                wireleft = lhs.x() - _designRule.viaExtension();
                wireright = rhs.x() + _designRule.viaWidth() + \
                            _designRule.viaExtension();
            }
            oaBox wirebox(wireleft, wirebottom, wireright, wiretop);
            oaRect::create(_design->getTopBlock(), METAL2, 1, wirebox);
            
        }
    }
#ifdef DEBUG
    if ((lhs.x() != rhs.x()) && (lhs.y() != rhs.y())) {
        cerr << "Error:" << endl;
        cerr << "lhs: (" << lhs.x() << " " << lhs.y() << ")" << endl;
        cerr << "rhs: (" << rhs.x() << " " << rhs.y() << ")" << endl;
        exit(1);
    }
#endif
}

void
Router_t::getCover(const EndPoint_t &src, CoverType type, line_t &cover)
{
    LineSet_t::const_iterator lineIter;
    oaPoint objectPoint = src.getObjectPoint();

    switch (type) {
    case LEFT:
        if (src.justStart()) {
            objectPoint.x() -= _designRule.viaWidth() / 2;
        }
        lineIter = _m2Barriers.lower_bound(objectPoint.x());
        do {
            --lineIter;
            oaInt4 ylow = lineIter->second.first.y() - _designRule.metalSpacing();
            oaInt4 yhigh = lineIter->second.second.y() + _designRule.metalSpacing();
            if (ylow <= objectPoint.y() && objectPoint.y() <= yhigh) {
                cover.first.x() = cover.second.x() = lineIter->first;
                cover.first.y() = ylow;
                cover.second.y() = yhigh;
                return;
            }
        } while (lineIter != _m2Barriers.begin());
        break;
    case RIGHT:
        if (src.justStart()) {
            objectPoint.x() += _designRule.viaWidth() / 2;
        }
        lineIter = _m2Barriers.upper_bound(objectPoint.x());
        while (lineIter != _m2Barriers.end()) {
            oaInt4 ylow = lineIter->second.first.y() - _designRule.metalSpacing();
            oaInt4 yhigh = lineIter->second.second.y() + _designRule.metalSpacing();
            if (ylow <= objectPoint.y() && objectPoint.y() <= yhigh) {
                cover.first.x() = cover.second.x() = lineIter->first;
                cover.first.y() = ylow;
                cover.second.y() = yhigh;
                return;
            }
            ++lineIter;
        }
        break;
    case BOTTOM:
        if (src.justStart()) {
            objectPoint.y() -= _designRule.viaWidth() / 2;
        }
        lineIter = _m1Barriers.lower_bound(objectPoint.y());
        do {
            --lineIter;
            oaInt4 xlow = lineIter->second.first.x() - _designRule.metalSpacing();
            oaInt4 xhigh = lineIter->second.second.x() + _designRule.metalSpacing();
            if (xlow <= objectPoint.x() && objectPoint.x() <= xhigh) {
                cover.first.y() = cover.second.y() = lineIter->first;
                cover.first.x() = xlow;
                cover.second.x() = xhigh;
                return;
            }
        } while (lineIter != _m1Barriers.begin());
        break;
    case TOP:
        if (src.justStart()) {
            objectPoint.y() += _designRule.viaWidth() / 2;
        }
        lineIter = _m1Barriers.upper_bound(objectPoint.y());
        while (lineIter != _m1Barriers.end()) {
            oaInt4 xlow = lineIter->second.first.x() - _designRule.metalSpacing();
            oaInt4 xhigh = lineIter->second.second.x() + _designRule.metalSpacing();
            if (xlow <= objectPoint.x() && objectPoint.x() <= xhigh) {
                cover.first.y() = cover.second.y() = lineIter->first;
                cover.first.x() = xlow;
                cover.second.x() = xhigh;
                return;
            }
            ++lineIter;
        }
        break;
    default:
        cerr << "Invalid cover type!" << endl;
        exit(1);
    }
}

void
Router_t::addObstacle(oaLayerNum layer, const oa::oaBox &box)
{
    if (METAL1 == layer) {
        line_t bottomEdge(oaPoint(box.left(), box.bottom()), \
                oaPoint(box.right(), box.bottom()));

        line_t topEdge(oaPoint(box.left(), box.top()), \
                oaPoint(box.right(), box.top()));

        addHline(bottomEdge);
        addHline(topEdge);
    }
    else if (METAL2 == layer) {
        line_t leftEdge(oaPoint(box.left(), box.bottom()), \
                oaPoint(box.left(), box.top()));

        line_t rightEdge(oaPoint(box.right(), box.bottom()), \
                oaPoint(box.right(), box.top()));

        addVline(leftEdge);
        addVline(rightEdge);
    }
    else {
        cerr << "Invalid layer!" << endl;
        exit(1);
    }
}

void
Router_t::addHline(const line_t &line)
{
    if (line.first.y() != line.second.y()) {
        return;
    }
    LineSet_t::iterator it;

    it = _m1Barriers.find(line.first.y()); 
    if (it == _m1Barriers.end()) {
        _m1Barriers[line.first.y()] = line;
    }
    else {
        if (it->second.first.x() > line.first.x()) {
            it->second.first.x() = line.first.x();
        }
        if (it->second.second.x() < line.second.x()) {
            it->second.second.x() = line.second.x();
        }
    }
}

void
Router_t::addVline(const line_t &line)
{
    if (line.first.x() != line.second.x()) {
        return;
    }
    LineSet_t::iterator it;

    it = _m2Barriers.find(line.first.x()); 
    if (it == _m2Barriers.end()) {
        _m2Barriers[line.first.x()] = line;
    }
    else {
        if (it->second.first.y() > line.first.y()) {
            it->second.first.y() = line.first.y();
        }
        if (it->second.second.y() < line.second.y()) {
            it->second.second.y() = line.second.y();
        }
    }
}
