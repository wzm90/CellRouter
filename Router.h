#ifndef ROUTER_H_
#define ROUTER_H_

#include <vector>
#include "oaDesignDB.h"
#include "Net.h"
#include "NetSet.h"
#include "DRC.h"
#include "EndPoint.h"

class Router_t {
public:
    Router_t(oa::oaDesign *design, oa::oaTech *tech, std::ifstream &file1,\
            std::ifstream &file2);
    bool route();
private:
    typedef enum { LEFT, BOTTOM, RIGHT, TOP } CoverType;
    
    bool routeOneNet(NetType_t type, Net_t &net);
    bool routeVDD(Net_t &net);
    bool routeVSS(Net_t &net);
    bool routeSignal(Net_t &net);
    bool routeIO(Net_t &net);
    void createWire(const oa::oaPoint &lhs, const oa::oaPoint &rhs);
    bool routeTwoContacts(oa::oaPoint lhs, oa::oaPoint rhs);
    // escape: perform escape algorithm
    bool escape(EndPoint_t &src, EndPoint_t &dst);
    void getEscapeLine(const EndPoint_t &src, Orient_t orient, line_t &escapeLine);
    void getEscapePoint(EndPoint_t &src); 
    void getCover(const EndPoint_t &src, CoverType type, line_t &cover);
    void addObstacle(oa::oaLayerNum layer, const oa::oaBox &box);
    void addHline(const line_t &line);
    void addVline(const line_t &line);

    oa::oaDesign *_design;
    oa::oaTech *_tech;
    oa::oaBox _VDDBox;
    oa::oaBox _VSSBox;
    NetSet_t _nets;
    DRC_t _designRule;
    LineSet_t _m1Barriers;
    LineSet_t _m2Barriers;
};
#endif
