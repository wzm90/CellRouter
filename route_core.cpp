#include <vector>
#include <iostream>
#include "route_core.h"

using namespace oa;
using namespace std;

void
route(oaDesign *design, oaTech *tech, const NetSet_t &nets,\
        const DRC_t &designRule)
{
    oaBlock *block = design->getTopBlock();
    
    oaLayerHeader *m1LayerHeader;
    m1LayerHeader = oaLayerHeader::find(block, 8);
    if (NULL == m1LayerHeader) {
        cerr << "Cannot open metal1 layer.\n";
    }
    oaCoord left, bottom, right, top;
    left = bottom = right = top = 0;

    oaIter<oaLPPHeader> LPPHeaderIter(m1LayerHeader->getLPPHeaders());
    while (oaLPPHeader *LPPHeader = LPPHeaderIter.getNext()) {
        oaIter<oaShape> shapeIter(LPPHeader->getShapes());
        while (oaShape *shape = shapeIter.getNext()) {
            if (shape->getType() == oacRectType) {
                oaBox bbox;
                shape->getBBox(bbox);
                left = bbox.left();
                right = bbox.right();
                if (bbox.bottom() < 0) {
                    // vss rail
                    bottom = bbox.top();
                } else {
                    top = bbox.bottom();
                }
            }
        }
    }

    cout << "The region is: " << left << " " << bottom << " " << right << " " << top << endl;

    // check if metal2 layer is in the database
    if (NULL == oaLayer::find(tech, "metal2")) {
        cout << "Creating metal2 layer\n";
        oaPhysicalLayer::create(tech, "metal2", 12, oacMetalMaterial, 12);
    }

    // draw grid
    for (oaCoord i = bottom; i < top; i += 1200) {
        oaRect::create(block, 8, 1, oaBox(left, i, right, i+20));
    }

    for (oaCoord i = left; i < right; i+= 1200) {
        oaRect::create(block, 12, 1, oaBox(i, bottom, i+20, top));
    }

    net_iterator netIter = nets.begin(VDD); 
    cout << "VDD net: " << endl;
    for (; netIter != nets.end(VDD); ++netIter) {
        Net_t::const_iterator pointIter = netIter->begin();
        for (; pointIter != netIter->end(); ++pointIter) {
            cout << "\t(" << pointIter->x() << " " << pointIter->y() << ")" << endl;
        } 
        cout << endl;
    }
    netIter = nets.begin(VSS); 
    cout << "VSS net: " << endl;
    for (; netIter != nets.end(VSS); ++netIter) {
        Net_t::const_iterator pointIter = netIter->begin();
        for (; pointIter != netIter->end(); ++pointIter) {
            cout << "\t(" << pointIter->x() << " " << pointIter->y() << ")" << endl;
        } 
        cout << endl;
    }
    netIter = nets.begin(S); 
    cout << "S net: " << endl;
    for (; netIter != nets.end(S); ++netIter) {
        Net_t::const_iterator pointIter = netIter->begin();
        for (; pointIter != netIter->end(); ++pointIter) {
            cout << "\t(" << pointIter->x() << " " << pointIter->y() << ")" << endl;
        } 
        cout << endl;
    }
    netIter = nets.begin(IO); 
    cout << "IO net: " << endl;
    for (; netIter != nets.end(IO); ++netIter) {
        Net_t::const_iterator pointIter = netIter->begin();
        for (; pointIter != netIter->end(); ++pointIter) {
            cout << "\t(" << pointIter->x() << " " << pointIter->y() << ")" << endl;
        } 
        cout << endl;
    }
    return;
}
