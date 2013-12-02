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

bool compx(const oaPoint &lhs, const oaPoint &rhs) {
    return lhs.x() < rhs.x();
}

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

class ContactsNumComparator {
public:
    ContactsNumComparator(const map<oaInt4, oaInt4> &contactsNum) : _contactsNum(contactsNum) {}
    bool operator()(const Net_t &lhs, const Net_t &rhs) {
        return _contactsNum[lhs.id()] < _contactsNum[rhs.id()];
    }
private:
    map<oaInt4, oaInt4> _contactsNum;
};


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

    addObstacle(METAL1, -1, routeRegionBox);
    addObstacle(METAL2, -1, routeRegionBox);
    NetSet_t::const_iterator netIter;
    // create metal1 for each contact
    for (netIter = _nets.begin(); netIter != _nets.end(); ++netIter) {
        Net_t::const_iterator citer;
        for (citer = netIter->begin(); citer != netIter->end(); ++citer) {
            oaPoint upperRight(citer->x() + _designRule.viaWidth(), \
                    citer->y() + _designRule.viaHeight());

            oaBox m1Box(*citer, upperRight);
            m1Box.bottom() -= _designRule.viaExtension();
            m1Box.top() += _designRule.viaExtension();
            oaRect::create(_design->getTopBlock(), METAL1, 1, m1Box);
            // add all contacts as M1 obstacles
            addObstacle(METAL1, netIter->id(), m1Box);
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
    reorderNets();
    NetSet_t::const_iterator netIter;
    bool result = true;

    for (netIter = _nets.begin(); netIter != _nets.end(); ++netIter) {
        bool oneResult = routeOneNet(*netIter);
        result = oneResult && result;
    }
    return result;
}

void
Router_t::reorderNets()
{
    map<oaInt4, oaInt4> contactsNum;
    NetSet_t::iterator netIter1, netIter2;

    // put VDD, VSS to the first two elements in _nets
    for (netIter1 = _nets.begin(); netIter1 != _nets.end(); ++netIter1) {
        if (netIter1->type() == VDD) {
            // swap with the first element
            Net_t temp = _nets.front();
            _nets.front() = *netIter1;
            *netIter1 = temp;
        }
    }
    for (netIter1 = _nets.begin(); netIter1 != _nets.end(); ++netIter1) {
        if (netIter1->type() == VSS) {
            // swap with the second element
            netIter2 = _nets.begin();
            ++netIter2;
            Net_t temp = *netIter2;
            *netIter2 = *netIter1;
            *netIter1 = temp;
        }
    }

    for (netIter1 = _nets.begin(); netIter1 != _nets.end(); ++netIter1) {
        contactsNum[netIter1->id()] = 0;
    }

    for (netIter1 = _nets.begin(); netIter1 != _nets.end(); ++netIter1) {
        for (netIter2 = _nets.begin(); netIter2 != _nets.end(); ++netIter2) {
            if (netIter2 != netIter1) {
                for (Net_t::const_iterator it = netIter2->begin(); \
                        it != netIter2->end(); ++it) {
                    if (netIter1->contains(*it)) {
                        contactsNum[netIter1->id()] += 1;
                    }
                }
            }
        }
    }
    ContactsNumComparator comp(contactsNum);
    netIter1 = _nets.begin();
    ++netIter1;
    ++netIter1;
    // do not sort _nets[0] and _nets[1]
    sort(netIter1, _nets.end(), comp);

    return;
}


bool
Router_t::routeOneNet(const Net_t &net)
{
    switch (net.type()) {
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
Router_t::routeVDD(const Net_t &net)
{
    oaBoolean noViolation = true;
    set<oaPoint, bool(*)(const oaPoint&, const oaPoint&)> contactPoints(compx);
    set<oaPoint, bool(*)(const oaPoint&, const oaPoint&)>::iterator piter;
    Net_t::const_iterator it;

    for (it = net.begin(); it != net.end(); ++it) {
        piter = contactPoints.find(*it);
        if (piter == contactPoints.end()) {
            // check violations and continue even with violation
            set<oaPoint, bool(*)(const oaPoint&, const oaPoint&)>::iterator iter;
            iter = contactPoints.lower_bound(*it);
            if (iter != contactPoints.end() && iter != contactPoints.begin()) {
                --iter;
                if ((it->x() - iter->x()) < (_designRule.viaWidth() + \
                            _designRule.metalSpacing())) {
                    noViolation = false;
                }
            }
            iter = contactPoints.upper_bound(*it);
            if (iter != contactPoints.end()) {
                if ((iter->x() - it->x()) < (_designRule.viaWidth() + \
                            _designRule.metalSpacing())) {
                    noViolation = false;
                }
            }
            // insert point
            contactPoints.insert(*it);
        }
        else {
            if (it->y() < piter->y()) {
                // if new point is "under" the old one, replace the old one
                // otherwise do nothing
                contactPoints.erase(piter);
                contactPoints.insert(*it);
            } 
        } 
    }

    // connect all points in set to VDD rail using metal1
    for (piter = contactPoints.begin(); piter != contactPoints.end(); ++piter) {
        // since point of each contact is leftdown point of the bounding box
        // we need to shift it to the center of bounding box
        oaPoint A(piter->x() + _designRule.viaWidth() / 2, \
                piter->y() + _designRule.viaHeight() / 2);
        
        oaPoint B(A.x(), _VDDBox.bottom());
        createWire(A, B, net.id());
    } 

    if (noViolation) {
        return true;
    }
    else {
        cout << "DRC violation in routing VDD net." << endl;
        return false;
    }
}


bool
Router_t::routeVSS(const Net_t &net)
{
    oaBoolean noViolation = true;
    set<oaPoint, bool(*)(const oaPoint&, const oaPoint&)> contactPoints(compx);
    set<oaPoint, bool(*)(const oaPoint&, const oaPoint&)>::iterator piter;
    Net_t::const_iterator it;

    for (it = net.begin(); it != net.end(); ++it) {
        piter = contactPoints.find(*it);
        if (piter == contactPoints.end()) {
            // check violations and continue even with violation
            set<oaPoint, bool(*)(const oaPoint&, const oaPoint&)>::iterator iter;
            iter = contactPoints.lower_bound(*it);
            if (iter != contactPoints.end() && iter != contactPoints.begin()) {
                --iter;
                if ((it->x() - iter->x()) < (_designRule.viaWidth() + \
                            _designRule.metalSpacing())) {
                    noViolation = false;
                }
            }
            iter = contactPoints.upper_bound(*it);
            if (iter != contactPoints.end()) {
                if ((iter->x() - it->x()) < (_designRule.viaWidth() + \
                            _designRule.metalSpacing())) {
                    noViolation = false;
                }
            }
            // insert point
            contactPoints.insert(*it);
        }
        else {
            if (it->y() > piter->y()) {
                // if new point is "above" the old one, replace the old one
                // otherwise do nothing
                contactPoints.erase(piter);
                contactPoints.insert(*it);
            } 
        } 
    }

    // connect all points in set to VDD rail using metal1
    for (piter = contactPoints.begin(); piter != contactPoints.end(); ++piter) {
        // since point of each contact is leftdown point of the bounding box
        // we need to shift it to the center of bounding box
        oaPoint A(piter->x() + _designRule.viaWidth() / 2, \
                piter->y() + _designRule.viaHeight() / 2);
        
        oaPoint B(A.x(), _VSSBox.top());
        createWire(A, B, net.id());
    } 

    if (noViolation) {
        return true;
    }
    else {
        cout << "DRC violation in routing VSS net." << endl;
        return false;
    }
}


bool
Router_t::routeSignal(const Net_t &net)
{
    if (net.size() > 1) {
        bool result = true;
        int front = 0;
        int back = net.size() - 1;
        while (1) {
            oaInt4 xdiff = net[front].x() - net[front+1].x();
            if ((xdiff > 0 && xdiff < _designRule.viaWidth()) || \
                    (xdiff < 0 && xdiff > -_designRule.viaWidth())) {
                oaCoord wireLeft = (net[front].x() < net[front+1].x()) ? net[front].x() : net[front+1].x();
                oaCoord wireRight = (net[front].x() > net[front+1].x()) ? net[front].x() : net[front+1].x();
                wireRight += _designRule.viaWidth();
                oaCoord wireBottom = (net[front].y() < net[front+1].y()) ? net[front].y() : net[front+1].y();
                oaCoord wireTop = (net[front].y() > net[front+1].y()) ? net[front].y() : net[front+1].y();
                wireTop += (_designRule.viaHeight() + _designRule.viaExtension());
                wireBottom -= (_designRule.viaExtension());
                oaBox wireBox(wireLeft, wireBottom, wireRight, wireTop);
                oaRect::create(_design->getTopBlock(), METAL1, 1, wireBox);
                addObstacle(METAL1, net.id(), wireBox);
            }
            else {
                EndPoint_t A(net[front].x()+_designRule.viaWidth()/2, \
                    net[front].y()+_designRule.viaHeight()/2, net.id());

                EndPoint_t B(net[front+1].x()+_designRule.viaWidth()/2, \
                    net[front+1].y()+_designRule.viaHeight()/2, net.id());

                result = routeTwoContacts(A, B) && result;
            }
            front++;

            if (front < back) {
                xdiff = net[back].x() - net[back-1].x();
                if ((xdiff > 0 && xdiff < _designRule.viaWidth()) || \
                        (xdiff < 0 && xdiff > -_designRule.viaWidth())) {
                    oaCoord wireLeft = (net[back].x() < net[back-1].x()) ? net[back].x() : net[back-1].x();
                    oaCoord wireRight = (net[back].x() > net[back-1].x()) ? net[back].x() : net[back-1].x();
                    wireRight += _designRule.viaWidth();
                    oaCoord wireBottom = (net[back].y() < net[back-1].y()) ? net[back].y() : net[back-1].y();
                    oaCoord wireTop = (net[back].y() > net[back-1].y()) ? net[back].y() : net[back-1].y();
                    wireTop += (_designRule.viaHeight() + _designRule.viaExtension());
                    wireBottom -= (_designRule.viaExtension());
                    oaBox wireBox(wireLeft, wireBottom, wireRight, wireTop);
                    oaRect::create(_design->getTopBlock(), METAL1, 1, wireBox);
                    addObstacle(METAL1, net.id(), wireBox);
                }
                else {
                    EndPoint_t A1(net[back].x()+_designRule.viaWidth()/2, \
                        net[back].y()+_designRule.viaHeight()/2, net.id());

                    EndPoint_t B1(net[back-1].x()+_designRule.viaWidth()/2, \
                        net[back-1].y()+_designRule.viaHeight()/2, net.id());

                    result = routeTwoContacts(A1, B1) && result;
                }
            }
            
            if (front >= back) {
                break;
            }
        }
        /*
        for (; it2 != net.end(); ++it1, ++it2) {
            oaInt4 xdiff = it1->x() - it2->x();
            if ((xdiff > 0 && xdiff < _designRule.viaWidth()) || \
                    (xdiff < 0 && xdiff > -_designRule.viaWidth())) {
                oaCoord wireLeft = (it1->x() < it2->x()) ? it1->x() : it2->x();
                oaCoord wireRight = (it1->x() > it2->x()) ? it1->x() : it2->x();
                wireRight += _designRule.viaWidth();
                oaCoord wireBottom = (it1->y() < it2->y()) ? it1->y() : it2->y();
                oaCoord wireTop = (it1->y() > it2->y()) ? it1->y() : it2->y();
                wireTop += (_designRule.viaHeight() + _designRule.viaExtension());
                wireBottom -= (_designRule.viaExtension());
                oaBox wireBox(wireLeft, wireBottom, wireRight, wireTop);
                oaRect::create(_design->getTopBlock(), METAL1, 1, wireBox);
                addObstacle(METAL1, net.id(), wireBox);
                continue;
            }
            EndPoint_t A(it1->x()+_designRule.viaWidth()/2, \
                it1->y()+_designRule.viaHeight()/2, net.id());

            EndPoint_t B(it2->x()+_designRule.viaWidth()/2, \
                it2->y()+_designRule.viaHeight()/2, net.id());

            result = routeTwoContacts(A, B) && result;
        }
        */
        return result;
    } 

    return true;
}


bool
Router_t::routeIO(const Net_t &net)
{
    bool result = true;
    Net_t::const_iterator it1 = net.begin();
    Net_t::const_iterator it2 = net.begin();
    ++it2;
    // create a rectangular on this point
    oaCoord wireLeft = it1->x() + _designRule.viaWidth() / 2 - \
                       _designRule.metalWidth() / 2;
    oaCoord wireBottom = it1->y() - _designRule.viaExtension();
    oaCoord wireRight = it1->x() + _designRule.viaWidth() / 2 + \
                        _designRule.metalWidth() / 2;
    oaCoord wireTop = it1->y() + _designRule.viaHeight() + \
                      _designRule.viaExtension();
    oaBox wireBox(wireLeft, wireBottom, wireRight, wireTop);
    oaRect::create(_design->getTopBlock(), METAL1, 1, wireBox);
    addObstacle(METAL1, net.id(), wireBox);
    // create oaText on metal1
    oaText::create(_design->getTopBlock(), METAL1, 1, net.portName(), \
            *it1, oaTextAlign(oacLowerLeftTextAlign), oaOrient(oacR0), \
            oaFont(oacRomanFont), oaDist(1000), false, true, true);

    for (; it2 != net.end(); ++it1, ++it2) {
        oaInt4 xdiff = it1->x() - it2->x();
        if ((xdiff > 0 && xdiff < _designRule.viaWidth()) || \
                (xdiff < 0 && xdiff > -_designRule.viaWidth())) {
            wireLeft = (it1->x() < it2->x()) ? it1->x() : it2->x();
            wireRight = (it1->x() > it2->x()) ? it1->x() : it2->x();
            wireRight += _designRule.viaWidth();
            wireBottom = (it1->y() < it2->y()) ? it1->y() : it2->y();
            wireTop = (it1->y() > it2->y()) ? it1->y() : it2->y();
            wireTop += (_designRule.viaHeight() + _designRule.viaExtension());
            wireBottom -= (_designRule.viaExtension());

            oaBox wireBox(wireLeft, wireBottom, wireRight, wireTop);
            oaRect::create(_design->getTopBlock(), METAL1, 1, wireBox);
            addObstacle(METAL1, net.id(), wireBox);
            continue;
        }
        EndPoint_t A(it1->x()+_designRule.viaWidth()/2, \
            it1->y()+_designRule.viaHeight()/2, net.id());

        EndPoint_t B(it2->x()+_designRule.viaWidth()/2, \
            it2->y()+_designRule.viaHeight()/2, net.id());

        result = routeTwoContacts(A, B) && result;
    }
    return result;
}

// Route two contacts using line-probing algorithm as described in
// "A Solution to line-routing problems on the continuous plane"
bool
Router_t::routeTwoContacts(EndPoint_t &lhs, EndPoint_t &rhs)
{
    // two contacts are represented by two leftdown points of their actual
    // box, now we move these two points to the center of contact bounding box
    
    bool intersect = false;
    EndPoint_t *src = &lhs;
    EndPoint_t *dst = &rhs;
    oaPoint intersectionPoint;

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
                intersect = escape(*src, *dst, intersectionPoint);
            }
        }
        else {
            intersect = escape(*src, *dst, intersectionPoint);
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
    cout << "Escape point of src are: " << endl;
    for (it = points1.begin(); it != points1.end(); ++it) {
        cout << "(" << it->x() << ", " << it->y() << ") ";
    } 
    cout << endl;
    cout << "Cross point is: (" << intersectionPoint.x() << ", ";
    cout << intersectionPoint.y() << ")" << endl;
    // create a metal1 layer around cross point
    //oaRect::create(_design->getTopBlock(), METAL1, 1, oaBox(intersectionPoint, 800));
    cout << "Escape point of dst are: " << endl;
    for (it = points2.begin(); it != points2.end(); ++it) {
        cout << "(" << it->x() << ", " << it->y() << ") ";
    }
    cout << endl;
    
    src->getCornerPoints(intersectionPoint);
    dst->getCornerPoints(intersectionPoint);
    //PointSet_t::const_iterator it;
    cout << "Corner points of src are as follows:" << endl;
    for (it = src->cornerPoints().begin(); it != src->cornerPoints().end(); ++it) {
        cout << "(" << it->x() << ", " << it->y() << ") ";
    }
    cout << endl;
    
    cout << "Corner points of dst are as follows:" << endl;
    for (it = dst->cornerPoints().begin(); it != dst->cornerPoints().end(); ++it) {
        cout << "(" << it->x() << ", " << it->y() << ") ";
    }
    cout << endl;
    cout << endl;

    // connect cornerPoints and intersectionPoint
    // create via for intersectionPoint
    
    if (intersectionPoint != src->cornerPoints().back() && \
            intersectionPoint != dst->cornerPoints().back()) {
        createVia(intersectionPoint);
    }
    else if (intersectionPoint == src->cornerPoints().back()) {
        if (intersectionPoint.y() == dst->cornerPoints().front().y()) {
            createVia(intersectionPoint);
        }
    }
    else {
        if (intersectionPoint.y() == src->cornerPoints().front().y()) {
            createVia(intersectionPoint);
        }
    }
    
    // connect src points
    PointSet_t::const_iterator it1 = src->cornerPoints().begin();
    PointSet_t::const_iterator it2 = src->cornerPoints().begin();
    ++it2;
    createWire(intersectionPoint, *it1, src->netID());
    
    // may need to create via for it1
    if (*it1 == src->cornerPoints().back()) {
        if (intersectionPoint != *it1 && intersectionPoint.y() == it1->y()) {
            createVia(*it1);
        }
    }
    
    for (; it2 != src->cornerPoints().end(); ++it1, ++it2) {
        createWire(*it1, *it2, src->netID());
        createVia(*it1);
        
        // may need to createVia for contact
        if (*it2 == src->cornerPoints().back()) {
            if (it1->y() == it2->y()) {
                // contact is connected to metal2 layer
                createVia(*it2);
            }
        }
        
    }

    // connect dst points
    it1 = dst->cornerPoints().begin();
    it2 = dst->cornerPoints().begin();
    ++it2;
    createWire(intersectionPoint, *it1, dst->netID());
    
    if (*it1 == dst->cornerPoints().back()) {
        if (intersectionPoint != *it1 && intersectionPoint.y() == it1->y()) {
            createVia(*it1);
        }
    }
    
    for (; it2 != dst->cornerPoints().end(); ++it1, ++it2) {
        createWire(*it1, *it2, dst->netID());
        createVia(*it1);
        
        // may need to create Via for contact
        if (*it2 == dst->cornerPoints().back()) {
            if (it1->y() == it2->y()) {
                // contact is connected to metal2 layer
                createVia(*it2);
            }
        }
        
    }
    return true;
}

// escape algorithm
bool
Router_t::escape(EndPoint_t &src, EndPoint_t &dst, oaPoint &intersectionPoint)
{
    if ((src.orient() == HORIZONTAL) || (src.orient() == BOTH)) {
        line_t escapeLine;
        getEscapeLine(src, HORIZONTAL, escapeLine);
        // add escapeLine
        //
        // createWire(escapeLine.first, escapeLine.second, _designRule.metalWidth());

        if (escapeLine.first != escapeLine.second) {
            src.addHline(escapeLine);
            if (dst.isIntersect(escapeLine, intersectionPoint)) {
                return true;
            }
        }
    }
    if ((src.orient() == VERTICAL) || (src.orient() == BOTH)) {
        line_t escapeLine;
        getEscapeLine(src, VERTICAL, escapeLine);
        // add escapeLine
        //
        // createWire(escapeLine.first, escapeLine.second, _designRule.metalWidth());

        if (escapeLine.first != escapeLine.second) {
            src.addVline(escapeLine);
            if (dst.isIntersect(escapeLine, intersectionPoint)) {
                return true;
            }
        }
    }
    // get escapePoint
    if (!getEscapePointI(src)) {
        bool intersectionFlag = false;
        bool escapeII = getEscapePointII(src, dst, intersectionFlag, intersectionPoint);
        if (intersectionFlag) {
            return true;
        }
        else if (!escapeII) {
            src.setNoEscape(true);
        }
    }
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
        if (sameBox(leftCover, rightCover)) {
            cout << "leftcover and right cover are in the same box." << endl;
            escapeLine.first = escapeLine.second = src.getObjectPoint();
            return;
        }
        escapeLine.first.y() = escapeLine.second.y() = src.getObjectPoint().y();       
        if (leftCover.first.x() == _VDDBox.left()) {
            escapeLine.first.x() = leftCover.first.x();
        }
        else {
            escapeLine.first.x() = leftCover.first.x() + \
                                   _designRule.metalSpacing() + \
                                   _designRule.metalWidth() / 2;
        }
        if (rightCover.first.x() == _VDDBox.right()) {
            escapeLine.second.x() = rightCover.first.x();
        }
        else {
            escapeLine.second.x() = rightCover.first.x() - \
                                    _designRule.metalSpacing() - \
                                    _designRule.metalWidth() / 2;
        }
    }
    else if (orient == VERTICAL) {
        line_t bottomCover, topCover;
        getCover(src, BOTTOM, bottomCover);
        getCover(src, TOP, topCover);
        if (sameBox(bottomCover, topCover)) {
            cout << "bottomcover and topcover are in the same box." << endl;
            escapeLine.first = escapeLine.second = src.getObjectPoint();
            return;
        }
        escapeLine.first.x() = escapeLine.second.x() = src.getObjectPoint().x();
        if (bottomCover.first.y() == _VSSBox.top()) {
            escapeLine.first.y() = bottomCover.first.y();
        }
        else {
            escapeLine.first.y() = bottomCover.first.y() + \
                                   _designRule.metalSpacing() + \
                                   _designRule.metalWidth() / 2;
        }
        if (topCover.first.y() == _VDDBox.bottom()) {
            escapeLine.second.y() = topCover.first.y();
        }
        else {
            escapeLine.second.y() = topCover.first.y() - \
                                    _designRule.metalSpacing() - \
                                    _designRule.metalWidth() / 2;
        }
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
bool
Router_t::getEscapePointI(EndPoint_t &src)
{
    // get covers
    line_t bottomCover, topCover, leftCover, rightCover;
    oaPoint objectPoint = src.getObjectPoint();

    getCover(src, BOTTOM, bottomCover);
    getCover(src, TOP, topCover);
    getCover(src, LEFT, leftCover);
    getCover(src, RIGHT, rightCover);

    bool noHorizontalEscape = sameBox(leftCover, rightCover);
    bool noVerticalEscape  = sameBox(bottomCover, topCover);

    oaInt4 movement = _designRule.metalSpacing() + _designRule.metalWidth() / 2;

    if (bottomCover.first.y() != _VSSBox.top()) {
        bottomCover.first.x() -= movement;
        bottomCover.second.x() += movement;
        bottomCover.first.y() += movement;
        bottomCover.second.y() += movement;
    }
    
    if (topCover.first.y() != _VDDBox.bottom()) {
        topCover.first.x() -= movement;
        topCover.second.x() += movement;
        topCover.first.y() -= movement;
        topCover.second.y() -= movement;
    }
    
    if (leftCover.first.x() != _VDDBox.left()) {
        leftCover.first.y() -= movement;
        leftCover.second.y() += movement;
        leftCover.first.x() += movement;
        leftCover.second.x() += movement;
    }
    
    if (rightCover.first.x() != _VDDBox.right()) {
        rightCover.first.y() -= movement;
        rightCover.second.y() += movement;
        rightCover.first.x() -= movement;
        rightCover.second.x() -= movement;
    }

    vector<oaPoint> extremeties;
    Comparator comp(objectPoint);
    vector<oaPoint>::const_iterator pIter;
    oaPoint escapePoint;

    if (!noHorizontalEscape) {
        if (bottomCover.first.y() != _VSSBox.top()) {
            // check if bottomCover reaches VSS rail
            extremeties.push_back(bottomCover.first);
            extremeties.push_back(bottomCover.second);
        } 
        if (topCover.first.y() != _VDDBox.bottom()) {
            // check if topCover reaches VDD rail
            extremeties.push_back(topCover.first);
            extremeties.push_back(topCover.second);
        }
        // sort extremeties
        sort(extremeties.begin(), extremeties.end(), comp);

        // Escape Process I
        for (pIter = extremeties.begin(); pIter != extremeties.end(); ++pIter) {
            if (pIter->x() < objectPoint.x()) {
                if ((pIter->x() - leftCover.first.x()) >= 0) {
                    if (src.orient() == VERTICAL) {
                        // continue on horizontal line, step can be less than minimum step
                        escapePoint.x() = pIter->x();
                    }
                    else {
                        oaInt4 xval = objectPoint.x() - _designRule.minimumStep();
                        if (xval > pIter->x()) {
                            escapePoint.x() = pIter->x();
                        }
                        else if (xval < leftCover.first.x()) {
                            escapePoint.x() = leftCover.first.x();
                        }
                        else {
                            escapePoint.x() = xval;
                        }
                    }
                    // escapePoint.x() = (pIter->x() + leftCover.first.x()) / 2;
                    escapePoint.y() = objectPoint.y();
                    if (!src.onEscapeLines(escapePoint, VERTICAL)) {
                        Orient_t prevOrient;
                        prevOrient = src.orient();
                        src.setOrient(VERTICAL);
                        src.addEscapePoint(escapePoint);
                        // check if escapeLine of new esapePoint is longer than objectPoint
                        line_t escapeVline;
                        getEscapeLine(src, VERTICAL, escapeVline);
                        if ((escapeVline.second.y() - escapeVline.first.y()) < \
                                (topCover.first.y() - bottomCover.first.y())) {
                            src.setOrient(prevOrient);
                            src.removeEscapePoint();
                        }
                        else {
                            return true;
                        }
                    }
                }
            }
            else {
                if ((rightCover.first.x() - pIter->x()) >= 0) {
                    if (src.orient() == VERTICAL) {
                        // continue on horizontal line
                        escapePoint.x() = pIter->x();
                    }
                    else {
                        oaInt4 xval = objectPoint.x() + _designRule.minimumStep();
                        if (xval < pIter->x()) {
                            escapePoint.x() = pIter->x();
                        }
                        else if (xval > rightCover.first.x()) {
                            escapePoint.x() = rightCover.first.x();
                        }
                        else {
                            escapePoint.x() = xval;
                        }
                    }
                    //escapePoint.x() = (pIter->x() + rightCover.first.x()) / 2;
                    escapePoint.y() = objectPoint.y();
                    if (!src.onEscapeLines(escapePoint, VERTICAL)) {
                        Orient_t prevOrient;
                        prevOrient = src.orient();
                        src.setOrient(VERTICAL);
                        src.addEscapePoint(escapePoint);
                        // check if escapeLine of new esapePoint is longer than objectPoint
                        line_t escapeVline;
                        getEscapeLine(src, VERTICAL, escapeVline);
                        if ((escapeVline.second.y() - escapeVline.first.y()) < \
                                (topCover.first.y() - bottomCover.first.y())) {
                            src.setOrient(prevOrient);
                            src.removeEscapePoint();
                        }
                        else {
                            return true;
                        }
                    }
                }
            }
        }
    }

    // sort extremeties
    extremeties.clear();
    if (!noVerticalEscape) {
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
                if ((pIter->y() - bottomCover.first.y()) >= 0) {
                    if (src.orient() == HORIZONTAL) {
                        // continue on horizontal line
                        escapePoint.y() = pIter->y();
                    }
                    else  {
                        oaInt4 yval = objectPoint.y() - _designRule.minimumStep();
                        if (yval > pIter->y()) {
                            escapePoint.y() = pIter->y();
                        }
                        else if (yval < bottomCover.first.y()) {
                            escapePoint.y() = bottomCover.first.y();
                        }
                        else {
                            escapePoint.y() = yval;
                        }
                    }
                    escapePoint.x() = objectPoint.x();
                    //escapePoint.y() = (pIter->y() + bottomCover.first.y()) / 2;
                    if (!src.onEscapeLines(escapePoint, HORIZONTAL)) {
                        Orient_t prevOrient;
                        prevOrient = src.orient();
                        src.setOrient(HORIZONTAL);
                        src.addEscapePoint(escapePoint);
                        // check if escapeLine of new esapePoint is longer than objectPoint
                        line_t escapeHline;
                        getEscapeLine(src, HORIZONTAL, escapeHline);
                        if ((escapeHline.second.x() - escapeHline.first.x()) < \
                                (rightCover.first.x() - leftCover.first.x())) {
                            src.setOrient(prevOrient);
                            src.removeEscapePoint();
                        }
                        else {
                            return true;
                        }
                    }
                }
            }
            else {
                if ((topCover.first.y() - pIter->y()) >= 0) {
                    if (src.orient() == HORIZONTAL) {
                        // continue on horizontal line
                        escapePoint.y() = pIter->y();
                    }
                    else {
                        oaInt4 yval = objectPoint.y() + _designRule.minimumStep();
                        if (yval < pIter->y()) {
                            escapePoint.y() = pIter->y();
                        }
                        else if (yval > topCover.first.y()) {
                            escapePoint.y() = topCover.first.y();
                        }
                        else {
                            escapePoint.y() = yval;
                        }
                    }
                    escapePoint.x() = objectPoint.x();
                    //escapePoint.y() = (pIter->y() + topCover.first.y()) / 2;
                    if (!src.onEscapeLines(escapePoint, HORIZONTAL)) {
                        Orient_t prevOrient;
                        prevOrient = src.orient();
                        src.setOrient(HORIZONTAL);
                        src.addEscapePoint(escapePoint);
                        // check if escapeLine of new esapePoint is longer than objectPoint
                        line_t escapeHline;
                        getEscapeLine(src, HORIZONTAL, escapeHline);
                        if ((escapeHline.second.x() - escapeHline.first.x()) < \
                                (rightCover.first.x() - leftCover.first.x())) {
                            src.setOrient(prevOrient);
                            src.removeEscapePoint();
                        }
                        else {
                            return true;
                        }
                    }
                }
            }
        }
    }

    // Escape Process II
    // TO BE DONE LATER (if I still have time...)
    return false;
}

bool
Router_t::getEscapePointII(EndPoint_t &src, const EndPoint_t &dst, \
        bool &intersectionFlag, oaPoint &intersectionPoint)
{
    vector<oaPoint> r;
    // get covers
    line_t bottomCover, topCover, leftCover, rightCover;
    oaPoint objectPoint = src.getObjectPoint();

    getCover(src, BOTTOM, bottomCover);
    getCover(src, TOP, topCover);
    getCover(src, LEFT, leftCover);
    getCover(src, RIGHT, rightCover);

    bool noHorizontalEscape = sameBox(leftCover, rightCover);
    bool noVerticalEscape  = sameBox(bottomCover, topCover);

    oaInt4 movement = _designRule.metalSpacing() + _designRule.metalWidth() / 2;
    
    oaPoint leftEnd(leftCover.first.x(), objectPoint.y());
    oaPoint rightEnd(rightCover.first.x(), objectPoint.y());
    oaPoint bottomEnd(objectPoint.x(), bottomCover.first.y());
    oaPoint topEnd(objectPoint.x(), topCover.first.y());

    leftEnd += oaPoint(movement, 0);
    rightEnd -= oaPoint(movement, 0);
    bottomEnd += oaPoint(0, movement);
    topEnd -= oaPoint(0, movement);

    r.push_back(topEnd);
    r.push_back(rightEnd);
    r.push_back(bottomEnd);
    r.push_back(leftEnd);
    
    bool r1, r2, r3, r4; 
    r1 = r2 = r3 = r4 = true;
    while (r1 || r2 || r3 || r4) {
        for (int i = 0; i < r.size(); ++i) {
            switch (i) {
            case 0:
                if (noVerticalEscape || (topCover.first.y() == _VDDBox.bottom()) || \
                        r[i].y() <= objectPoint.y()) {
                    r1 = false;
                }
                else {
                    line_t escapeHline;
                    src.addEscapePoint(r[i]);
                    getEscapeLine(src, HORIZONTAL, escapeHline);
                    if (dst.isIntersect(escapeHline, intersectionPoint)) {
                        // add this line
                        src.addHline(escapeHline);
                        intersectionFlag = true;
                        return true;
                    }
                    else {
                        if (getEscapePointI(src)) {
                            intersectionFlag = false;
                            return true;
                        }
                        else {
                            // remove r[i] from escapePoints vector
                            src.removeEscapePoint();
                            r[i] -= oaPoint(0, _designRule.minimumStep());
                        }
                    }
                }
                break;
            case 1:
                if (noHorizontalEscape || (rightCover.first.y() == _VDDBox.right()) || \
                        r[i].x() <= objectPoint.x()) {
                    r2 = false;
                }
                else {
                    line_t escapeVline;
                    src.addEscapePoint(r[i]);
                    getEscapeLine(src, VERTICAL, escapeVline);
                    if (dst.isIntersect(escapeVline, intersectionPoint)) {
                        // add escapeVline
                        src.addVline(escapeVline);
                        intersectionFlag = true;
                        return true;
                    }
                    else {
                        if (getEscapePointI(src)) {
                            intersectionFlag = false;
                            return true;
                        }
                        else {
                            // remove r[i] from escapePoints vector
                            src.removeEscapePoint();
                            r[i] -= oaPoint(_designRule.minimumStep(), 0);
                        }
                    }
                }
                break;
            case 2:
                if (noVerticalEscape || (bottomCover.first.y() == _VSSBox.top()) || \
                        r[i].y() >= objectPoint.y()) {
                    r3 = false;
                }
                else {
                    line_t escapeHline;
                    src.addEscapePoint(r[i]);
                    getEscapeLine(src, HORIZONTAL, escapeHline);
                    if (dst.isIntersect(escapeHline, intersectionPoint)) {
                        src.addHline(escapeHline);
                        intersectionFlag = true;
                        return true;
                    }
                    else {
                        if (getEscapePointI(src)) {
                            intersectionFlag = false;
                            return true;
                        }
                        else {
                            // remove r[i] from escapePoints vector
                            src.removeEscapePoint();
                            r[i] += oaPoint(0, _designRule.minimumStep());
                        }
                    }
                }
                break;
            case 3:
                if (noHorizontalEscape || (leftCover.first.x() == _VDDBox.left()) || \
                        r[i].x() >= objectPoint.x()) {
                    r4 = false;
                }
                else {
                    line_t escapeVline;
                    src.addEscapePoint(r[i]);
                    getEscapeLine(src, VERTICAL, escapeVline);
                    oaPoint intersectionPoint;
                    if (dst.isIntersect(escapeVline, intersectionPoint)) {
                        src.addVline(escapeVline);
                        intersectionFlag = true;
                        return true;
                    }
                    else {
                        if (getEscapePointI(src)) {
                            intersectionFlag = false;
                            return true;
                        }
                        else {
                            // remove r[i] from escapePoints vector
                            src.removeEscapePoint();
                            r[i] -= oaPoint(_designRule.minimumStep(), 0);
                        }
                    }
                }
                break;
            default:
                break;
            } 
        }
    } 
}

void
Router_t::createWire(const oaPoint &lhs, const oaPoint &rhs, oaInt4 netID)
{
    oaInt4 width = _designRule.metalWidth();
    if (lhs != rhs) {
        if (lhs.x() == rhs.x()) {
            // create wire rect
            oaCoord wireleft = lhs.x() - width / 2;
            //oaCoord wireright = lhs.x() + _designRule.viaWidth();
            oaCoord wireright = lhs.x() + width / 2;
            oaCoord wiretop, wirebottom;
            if (lhs.y() < rhs.y()) {
                //wirebottom = lhs.y() - _designRule.viaExtension();
                //wiretop = rhs.y() + _designRule.viaHeight() + \
                          _designRule.viaExtension();
                wirebottom = lhs.y() - _designRule.viaHeight() / 2 - \
                             _designRule.viaExtension();
                wiretop = rhs.y() + _designRule.viaHeight() / 2 + \
                          _designRule.viaExtension();
            } else {
                //wirebottom = rhs.y() - _designRule.viaExtension();
                //wiretop = lhs.y() + _designRule.viaHeight() + \
                          _designRule.viaExtension();
                wirebottom = rhs.y() - _designRule.viaHeight() / 2 - \
                             _designRule.viaExtension();
                wiretop = lhs.y() + _designRule.viaHeight() / 2 + \
                          _designRule.viaExtension();
            }
            oaBox wirebox(wireleft, wirebottom, wireright, wiretop);

            oaRect::create(_design->getTopBlock(), METAL1, 1, wirebox);
            // add wirebox as obstacle
            addObstacle(METAL1, netID, wirebox);

        } else if (lhs.y() == rhs.y()) {
            // create wire rect
            //oaCoord wiretop = lhs.y() + _designRule.viaHeight();
            //oaCoord wirebottom = lhs.y();
            oaCoord wiretop = lhs.y() + width / 2;
            oaCoord wirebottom = lhs.y() - width / 2;
            oaCoord wireleft, wireright;
            if (lhs.x() < rhs.x()) {
                //wireleft = lhs.x() - _designRule.viaExtension();
                //wireright = rhs.x() + _designRule.viaWidth() + \
                            _designRule.viaExtension();
                wireleft = lhs.x() - _designRule.viaWidth() / 2 - \
                           _designRule.viaExtension();
                wireright = rhs.x() + _designRule.viaWidth() / 2 + \
                            _designRule.viaExtension();
            } else {
                wireleft = rhs.x() - _designRule.viaWidth() / 2 - \
                           _designRule.viaExtension();
                wireright = lhs.x() + _designRule.viaWidth() / 2 + \
                            _designRule.viaExtension();
            }
            oaBox wirebox(wireleft, wirebottom, wireright, wiretop);
            oaRect::create(_design->getTopBlock(), METAL2, 1, wirebox);
            // add wirebox as obstacle 
            addObstacle(METAL2, netID, wirebox);
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
Router_t::createVia(const oaPoint &point)
{
    oaCoord left, right, bottom, top;
    left = point.x() - _designRule.viaWidth() / 2;
    right = point.x() + _designRule.viaWidth() / 2;
    bottom = point.y() - _designRule.viaHeight() / 2;
    top = point.y() + _designRule.viaHeight() / 2;

    oaRect::create(_design->getTopBlock(), VIA1, 1, oaBox(left, bottom, right, top));
}

void
Router_t::getCover(const EndPoint_t &src, CoverType type, line_t &cover)
{
    BarrierSet_t::iterator lineIter;
    oaPoint objectPoint = src.getObjectPoint();

    switch (type) {
    case LEFT:
        lineIter = _m2Barriers.lower_bound(objectPoint.x());
        do {
            --lineIter;
            if (netID(lineIter) == src.netID()) {
                continue; 
            }
            oaInt4 ylow = lineSeg(lineIter).first.y() - \
                          _designRule.metalSpacing() - \
                          _designRule.metalWidth() / 2;

            oaInt4 yhigh = lineSeg(lineIter).second.y() + \
                           _designRule.metalSpacing() + \
                           _designRule.metalWidth() / 2;

            if (ylow < objectPoint.y() && objectPoint.y() < yhigh) {
                cover = lineSeg(lineIter);
                return;
            }
        } while (lineIter != _m2Barriers.begin());
        break;
    case RIGHT:
        lineIter = _m2Barriers.upper_bound(objectPoint.x());
        for (; lineIter != _m2Barriers.end(); ++lineIter) {
            if (netID(lineIter) == src.netID()) {
                continue;
            }
            oaInt4 ylow = lineSeg(lineIter).first.y() - \
                          _designRule.metalSpacing() - \
                          _designRule.metalWidth() / 2;

            oaInt4 yhigh = lineSeg(lineIter).second.y() + \
                           _designRule.metalSpacing() + \
                           _designRule.metalWidth() / 2;

            if (ylow < objectPoint.y() && objectPoint.y() < yhigh) {
                cover = lineSeg(lineIter);
                return;
            }
        }
        break;
    case BOTTOM:
        lineIter = _m1Barriers.lower_bound(objectPoint.y());
        do {
            --lineIter;
            if (netID(lineIter) == src.netID()) {
                continue; 
            }
            oaInt4 xlow = lineSeg(lineIter).first.x() - \
                          _designRule.metalSpacing() - \
                          _designRule.metalWidth() / 2;

            oaInt4 xhigh = lineSeg(lineIter).second.x() + \
                           _designRule.metalSpacing() + \
                           _designRule.metalWidth() / 2;

            if (xlow < objectPoint.x() && objectPoint.x() < xhigh) {
                cover = lineSeg(lineIter);
                return;
            }
        } while (lineIter != _m1Barriers.begin());
        break;
    case TOP:
        lineIter = _m1Barriers.upper_bound(objectPoint.y());
        for (; lineIter != _m1Barriers.end(); ++lineIter) {
            if (netID(lineIter) == src.netID()) {
                continue; 
            }
            oaInt4 xlow = lineSeg(lineIter).first.x() - \
                          _designRule.metalSpacing() - \
                          _designRule.metalWidth() / 2;

            oaInt4 xhigh = lineSeg(lineIter).second.x() + \
                           _designRule.metalSpacing() + \
                           _designRule.metalWidth() / 2;
            if (xlow < objectPoint.x() && objectPoint.x() < xhigh) {
                cover = lineSeg(lineIter);
                return;
            }
        }
        break;
    default:
        cerr << "Invalid cover type!" << endl;
        exit(1);
    }
}

void
Router_t::addObstacle(oaLayerNum layer, oaInt4 netID, const oa::oaBox &box)
{
    line_t bottomEdge(oaPoint(box.left(), box.bottom()), \
            oaPoint(box.right(), box.bottom()));

    line_t topEdge(oaPoint(box.left(), box.top()), \
            oaPoint(box.right(), box.top()));

    line_t leftEdge(oaPoint(box.left(), box.bottom()), \
            oaPoint(box.left(), box.top()));

    line_t rightEdge(oaPoint(box.right(), box.bottom()), \
            oaPoint(box.right(), box.top()));


    if (METAL1 == layer) {
        _m1Barriers.insert(make_pair(box.bottom(), make_pair(netID, bottomEdge)));
        _m1Barriers.insert(make_pair(box.top(), make_pair(netID, topEdge)));
        _m1Vlines.insert(make_pair(box.left(), make_pair(netID, leftEdge)));
        _m1Vlines.insert(make_pair(box.right(), make_pair(netID, rightEdge)));
    }
    else if (METAL2 == layer) {
        _m2Barriers.insert(make_pair(box.left(), make_pair(netID, leftEdge)));
        _m2Barriers.insert(make_pair(box.right(), make_pair(netID, rightEdge)));
        _m2Hlines.insert(make_pair(box.bottom(), make_pair(netID, bottomEdge)));
        _m2Hlines.insert(make_pair(box.top(), make_pair(netID, topEdge)));
    }
    else {
        cerr << "Invalid layer!" << endl;
        exit(1);
    }
}

bool
Router_t::sameBox(line_t &lhs, line_t &rhs)
{
    if (lhs.first.x() == rhs.first.x() && lhs.second.x() == rhs.second.x()) {
        // horizontal lines 
        pair<BarrierSet_t::iterator, BarrierSet_t::iterator> ret;
        ret = _m1Vlines.equal_range(lhs.first.x());
        BarrierSet_t::iterator it;
        for (it = ret.first; it != ret.second; ++it) {
            if (netID(it) != -1 && lineSeg(it).first.y() == lhs.first.y() && \
                    lineSeg(it).second.y() == rhs.first.y()) {
                return true;
            }
        }
    } 
    else if (lhs.first.y() == rhs.first.y() && lhs.second.y() == rhs.second.y()) {
        // vertical lines
        pair<BarrierSet_t::iterator, BarrierSet_t::iterator> ret;
        ret = _m2Hlines.equal_range(lhs.first.y());
        BarrierSet_t::iterator it;
        for (it = ret.first; it != ret.second; ++it) {
            if (netID(it) != -1 && lineSeg(it).first.x() == lhs.first.x() && \
                    lineSeg(it).second.x() == rhs.first.x()) {
                return true;
            }
        }
    }
    
    return false;
}
