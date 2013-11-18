#include <vector>
#include <algorithm>
#include <iostream>
#include "Router.h"

using namespace oa;
using namespace std;

const oaLayerNum CONTACT = 7;
const oaLayerNum METAL1 = 8;
const oaLayerNum VIA1 = 11;
const oaLayerNum METAL2 = 12;

static bool compx(const oaPoint &lhs, const oaPoint &rhs)
{
    return (lhs.y() < rhs.y());
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
    } 

    return true;
}


bool
Router_t::routeIO(Net_t &net)
{
    return true;
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
