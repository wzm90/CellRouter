#include <vector>
#include <iostream>
#include "Router.h"

using namespace oa;
using namespace std;

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
}

bool
Router_t::route()
{
    net_iterator netIter;
    bool result;

    // route VDD
    for (netIter = _nets.begin(VDD); netIter != _nets.end(VDD); ++netIter) {
        Net_t net = *netIter;
        bool oneResult = routeOneNet(VDD, net);
        result = oneResult && result;
    }
    // route VSS
    for (netIter = _nets.begin(VSS); netIter != _nets.end(VSS); ++netIter) {
        Net_t net = *netIter;
        bool oneResult = routeOneNet(VSS, net);
        result = oneResult && result;
    }
    // route S
    for (netIter = _nets.begin(S); netIter != _nets.end(S); ++netIter) {
        Net_t net = *netIter;
        bool oneResult = routeOneNet(S, net);
        result = oneResult && result;
    }
    // route IO
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
    return true;
}
